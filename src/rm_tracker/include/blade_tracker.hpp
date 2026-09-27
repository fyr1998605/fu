#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>

#include "buff_config.hpp"

// 单个扇叶跟踪器
struct BladeTracker {
    int id;                     // 唯一ID
    cv::Point2f position;       // 当前位置
    cv::Point2f velocity;       // 估计速度
    std::deque<cv::Point2f> history;  // 历史轨迹
    int lost_count;             // 连续丢失帧数
    int hit_count;              // 连续命中帧数
    float confidence;           // 置信度
    float avg_radius;           // 平均半径
    bool is_valid;              // 是否有效
    
    // 默认构造函数
    BladeTracker() : id(-1), lost_count(0), hit_count(0),
                    confidence(0.0f), avg_radius(0.0f), is_valid(false) {}
    
    // 带参数的构造函数
    BladeTracker(int _id) : id(_id), lost_count(0), hit_count(0),
                           confidence(0.0f), avg_radius(0.0f), is_valid(false) {}
    
    // 更新跟踪器
    void update(const cv::Point2f& new_pos, const cv::Point2f& circle_center) {
        // 更新历史轨迹
        history.push_back(new_pos);
        if (history.size() > 10) {
            history.pop_front();
        }
        
        // 更新位置
        cv::Point2f old_pos = position;
        position = new_pos;
        
        // 计算速度
        if (history.size() >= 2) {
            velocity = (position - history.front()) * (1.0f / (history.size() - 1));
        }
        
        // 更新半径
        float radius = cv::norm(new_pos - circle_center);
        if (avg_radius == 0.0f) {
            avg_radius = radius;
        } else {
            avg_radius = avg_radius * 0.9f + radius * 0.1f;  // 指数平滑
        }
        
        // 更新计数
        lost_count = 0;
        hit_count++;
        
        // 更新置信度
        confidence = std::min(1.0f, hit_count / 10.0f);
        is_valid = true;
    }
    
    // 预测下一帧位置
    cv::Point2f predict() const {
        return position + velocity;
    }
    
    // 标记为丢失
    void markLost() {
        lost_count++;
        hit_count = 0;
        confidence *= 0.9f;  // 置信度衰减
        
        if (lost_count > 3) {
            is_valid = false;
            confidence = 0.0f;
        }
    }
};

// Small-energy-mechanism phase tracker adapted from Climber_Vision_26.
// Its Kalman state is [continuous phase, angular velocity]. It stays entirely
// in image-pixel space and does not use camera, IMU, gun, or map coordinates.
class SmallBuffPhaseTracker {
public:
    static constexpr double kBladePeriod = 2.0 * CV_PI / 5.0;
    // 其余常量已参数化 -> Buff.angle.small_buff_* (见 angle_cfg_)

    // 小符相位跟踪参数注入 (Fu.cpp 从 Buff.angle 载入)。默认值 == 迁移前硬编码值。
    void configure(const buff::AngleConfig& config) { angle_cfg_ = config; }

    bool update(const cv::Point2f& r_center,
                const cv::Point2f& blade_center,
                double timestamp_seconds,
                double predict_seconds,
                cv::Point2f& predicted_center) {
        const cv::Point2f offset = blade_center - r_center;
        const double measured_radius = cv::norm(offset);
        if (!isFinitePoint(r_center) || !isFinitePoint(blade_center) ||
            measured_radius < angle_cfg_.small_buff_min_radius_px ||   // 原 1.0
            !std::isfinite(timestamp_seconds) ||
            !std::isfinite(predict_seconds) || predict_seconds < 0.0) {
            return false;
        }

        const double raw_angle = std::atan2(offset.y, offset.x);
        if (!initialized_) {
            initialized_ = true;
            last_timestamp_ = timestamp_seconds;
            last_observed_phase_ = raw_angle;
            radius_ = measured_radius;
            center_ = r_center;
            initializeKalman(raw_angle);
            return false;
        }

        const double dt = timestamp_seconds - last_timestamp_;
        if (dt <= 1e-4 || dt > angle_cfg_.small_buff_max_dt) {  // 原 dt > 0.5
            // Partial reset: when the tracker was already locked, keep
            // direction and angular velocity — only re-initialize phase
            // from the new observation. A full reset would discard omega
            // and force angle_cfg_.small_buff_converge_delay frames of re-convergence, causing
            // prediction jumps of 8-160 px every time the target reappears
            // after a detection gap (e.g. power-rune light off→on).
            if (locked_direction_ != 0) {
                const double saved_omega =
                    kalman_.statePost.at<double>(1, 0);
                saved_direction_ = locked_direction_;
                initializeKalman(raw_angle);
                kalman_.statePost.at<double>(1, 0) =
                    saved_direction_ * std::max(
                        angle_cfg_.small_buff_min_speed,
                        std::min(angle_cfg_.small_buff_max_speed,
                                 std::abs(saved_omega)));
                // Tighten omega covariance — we trust the persisted value
                kalman_.errorCovPost.at<double>(1, 1) = angle_cfg_.small_buff_omega_reset_cov;  // 原 0.02
                direction_votes_ = saved_direction_ * angle_cfg_.small_buff_converge_delay;
                locked_direction_ = saved_direction_;
                accepted_count_ = 0;
                rejected_count_ = 0;
                phase_history_.clear();
                time_history_.clear();
                last_timestamp_ = timestamp_seconds;
                last_observed_phase_ = raw_angle;
                center_ = r_center;
                radius_ = measured_radius;
                return false;
            }
            reset();
            return false;
        }

        configurePrediction(dt);
        const cv::Mat prediction = kalman_.predict();
        const double predicted_phase = prediction.at<double>(0, 0);
        const double observed_phase = predicted_phase +
            wrapBladeDelta(raw_angle - predicted_phase);
        const double innovation = observed_phase - predicted_phase;
        if (std::abs(innovation) > angle_cfg_.small_buff_max_roll_residual) {
            // Keep the predicted state for a short observation dropout.
            kalman_.statePost = kalman_.statePre.clone();
            kalman_.errorCovPost = kalman_.errorCovPre.clone();
            last_timestamp_ = timestamp_seconds;
            ROS_WARN_THROTTLE(0.3,
                "[JITTER-KALMAN] REJECT: innovation=%.2f° > thr=%.2f°  raw=%.2f° pred=%.2f°  rejected=%d/3",
                innovation * 180.0 / CV_PI, angle_cfg_.small_buff_max_roll_residual * 180.0 / CV_PI,
                raw_angle * 180.0 / CV_PI, predicted_phase * 180.0 / CV_PI,
                rejected_count_ + 1);
            if (++rejected_count_ > angle_cfg_.small_buff_reject_reset_count) reset();  // 原 > 3
            return false;
        }

        const double phase_delta = observed_phase - last_observed_phase_;
        if (std::abs(phase_delta) > angle_cfg_.small_buff_vote_min_phase_delta) {  // 原 0.002
            direction_votes_ += phase_delta > 0.0 ? 1 : -1;
            direction_votes_ = std::max(-angle_cfg_.small_buff_vote_saturation,
                                        std::min(angle_cfg_.small_buff_vote_saturation,
                                                 direction_votes_));  // 原 ±20
        }

        const int direction = direction_votes_ >= angle_cfg_.small_buff_converge_delay ? 1 :
                              direction_votes_ <= -angle_cfg_.small_buff_converge_delay ? -1 : 0;

        cv::Mat measurement(1, 1, CV_64F);
        measurement.at<double>(0, 0) = observed_phase;
        kalman_.correct(measurement);

        // Compute angular velocity from observed phase history instead of
        // a hardcoded constant. Use linear fit over recent phase/timestamp pairs.
        if (direction != 0 && locked_direction_ == 0) {
            locked_direction_ = direction;
            double computed_omega = angle_cfg_.small_buff_min_speed;  // floor fallback
            if (phase_history_.size() >=
                static_cast<std::size_t>(angle_cfg_.small_buff_omega_fit_min_samples)) {  // 原 >= 5
                double sum_dphase = 0.0, sum_dt = 0.0;
                for (size_t i = 1; i < phase_history_.size(); i++) {
                    double dp = phase_history_[i] - phase_history_[i - 1];
                    double dt = time_history_[i] - time_history_[i - 1];
                    if (dt > 1e-6) {
                        sum_dphase += dp;
                        sum_dt += dt;
                    }
                }
                if (sum_dt > 1e-6) {
                    computed_omega = std::abs(sum_dphase / sum_dt);
                }
            }
            computed_omega = std::max(angle_cfg_.small_buff_min_speed,
                std::min(angle_cfg_.small_buff_max_speed, computed_omega));
            kalman_.statePost.at<double>(1, 0) = direction * computed_omega;
            kalman_.errorCovPost.at<double>(1, 1) = angle_cfg_.small_buff_omega_locked_cov;  // 原 0.05
        }

        if (locked_direction_ != 0) {
            double omega = kalman_.statePost.at<double>(1, 0);
            if (omega * locked_direction_ <= 0.0) {
                omega = locked_direction_ * angle_cfg_.small_buff_min_speed;
            } else {
                omega = locked_direction_ * std::max(
                    angle_cfg_.small_buff_min_speed,
                    std::min(angle_cfg_.small_buff_max_speed, std::abs(omega)));
            }
            kalman_.statePost.at<double>(1, 0) = omega;
        }

        // 原 0.9*radius_ + 0.1*measured_radius / 0.8f*center_ + 0.2f*r_center
        radius_ = (1.0 - angle_cfg_.small_buff_radius_ema) * radius_ +
                  angle_cfg_.small_buff_radius_ema * measured_radius;
        // R 中心瞬跳时直接吸附, 不要走 EMA。
        // predicted_center = center_ + radius_*unit(phase), 所以
        // predicted_radius = (center_ - r_center) + radius_*unit(phase) ——
        // center_ 落后观测值越远, 第一项越主导方向。上机日志抓到过
        // center=(653,391) r_center=(651,280) radius=110.9px (相差 ~1 个半径),
        // 此时预测点被甩到圆的另一侧, 下游 PnPSolver 以 physical_phase_limit 拒绝
        // (image_phase_deg ≈ ±175°), 整帧作废。
        const double center_shift = cv::norm(r_center - center_);
        if (radius_ > 1e-6 &&
            center_shift > angle_cfg_.small_buff_center_snap_ratio * radius_) {
            center_ = r_center;
        } else {
            center_ = center_ * static_cast<float>(1.0 - angle_cfg_.small_buff_center_ema) +
                      r_center * static_cast<float>(angle_cfg_.small_buff_center_ema);
        }
        last_timestamp_ = timestamp_seconds;
        last_observed_phase_ = observed_phase;
        rejected_count_ = 0;
        ++accepted_count_;

        // Store phase/timestamp history for omega computation
        phase_history_.push_back(observed_phase);
        time_history_.push_back(timestamp_seconds);
        while (phase_history_.size() >
               static_cast<std::size_t>(angle_cfg_.small_buff_phase_history_max)) {  // 原 20
            phase_history_.pop_front();
            time_history_.pop_front();
        }

        // ── 抖动诊断: Kalman内部状态 ──
        {
            double phase_deg = phase_delta * 180.0 / CV_PI;
            double omega_deg = kalman_.statePost.at<double>(1, 0) * 180.0 / CV_PI;
            ROS_INFO_THROTTLE(0.5,
                "[JITTER-KALMAN] phase=%.2f° omega=%.1f°/s innovation=%.3f° radius=%.1fpx dir=%d accepted=%d",
                phase_deg, omega_deg,
                innovation * 180.0 / CV_PI,
                radius_, locked_direction_, accepted_count_);
        }

        // Fast-track: saved direction bypasses converge delay after vision loss.
        if (locked_direction_ == 0) return false;
        if (accepted_count_ < angle_cfg_.small_buff_converge_delay && saved_direction_ == 0) return false;

        const double phase = kalman_.statePost.at<double>(0, 0);
        const double omega = kalman_.statePost.at<double>(1, 0);
        const double future_phase = phase + omega * predict_seconds;
        predicted_center = center_ + cv::Point2f(
            static_cast<float>(radius_ * std::cos(future_phase)),
            static_cast<float>(radius_ * std::sin(future_phase)));
        return isFinitePoint(predicted_center);
    }

    void reset() {
		// Persist rotation direction across resets to prevent
		// direction flips after brief vision loss.
		if (locked_direction_ != 0) saved_direction_ = locked_direction_;
		initialized_ = false;
		last_timestamp_ = 0.0;
		last_observed_phase_ = 0.0;
		radius_ = 0.0;
		center_ = cv::Point2f(0.0f, 0.0f);
		direction_votes_ = 0;
		locked_direction_ = 0;
		accepted_count_ = 0;
		rejected_count_ = 0;
		phase_history_.clear();
		time_history_.clear();
		// Pre-seed direction votes from saved direction to speed up re-convergence.
		if (saved_direction_ != 0) {
			direction_votes_ = saved_direction_ * angle_cfg_.small_buff_converge_delay;

		}
    }
	int lockedDirection() const { return locked_direction_; }

private:
    void initializeKalman(double phase) {
        kalman_.init(2, 1, 0, CV_64F);
        kalman_.transitionMatrix = (cv::Mat_<double>(2, 2) <<
            1.0, 0.0,
            0.0, 1.0);
        kalman_.measurementMatrix = (cv::Mat_<double>(1, 2) << 1.0, 0.0);
        kalman_.statePost = (cv::Mat_<double>(2, 1) << phase, 0.0);
        kalman_.errorCovPost = (cv::Mat_<double>(2, 2) <<
            angle_cfg_.small_buff_init_cov_phase, 0.00,     // 原 0.10
            0.00, angle_cfg_.small_buff_init_cov_omega);    // 原 0.50
        kalman_.measurementNoiseCov = cv::Mat::eye(1, 1, CV_64F) *
            angle_cfg_.small_buff_phase_measurement_noise;
    }

    void configurePrediction(double dt) {
        kalman_.transitionMatrix.at<double>(0, 0) = 1.0;
        kalman_.transitionMatrix.at<double>(0, 1) = dt;
        kalman_.transitionMatrix.at<double>(1, 0) = 0.0;
        kalman_.transitionMatrix.at<double>(1, 1) = 1.0;

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;
        kalman_.processNoiseCov = (cv::Mat_<double>(2, 2) <<
            0.25 * dt4 * angle_cfg_.small_buff_phase_process_noise,
            0.50 * dt3 * angle_cfg_.small_buff_phase_process_noise,
            0.50 * dt3 * angle_cfg_.small_buff_phase_process_noise,
            dt2 * angle_cfg_.small_buff_phase_process_noise);
    }

    static double wrapBladeDelta(double delta) {
        return std::remainder(delta, kBladePeriod);
    }

    static bool isFinitePoint(const cv::Point2f& point) {
        return std::isfinite(point.x) && std::isfinite(point.y);
    }

    bool initialized_ = false;
    double last_timestamp_ = 0.0;
    double last_observed_phase_ = 0.0;
    double radius_ = 0.0;
    cv::Point2f center_{0.0f, 0.0f};
    int direction_votes_ = 0;
    int locked_direction_ = 0;
    int accepted_count_ = 0;
    int rejected_count_ = 0;
	int saved_direction_ = 0;  // persisted across resets
    cv::KalmanFilter kalman_;
    // Phase/timestamp history for computing angular velocity from observations
    std::deque<double> phase_history_;
    std::deque<double> time_history_;
    // 相位历史长度已参数化 -> Buff.angle.small_buff_phase_history_max
    buff::AngleConfig angle_cfg_;   // 小符相位跟踪参数 (默认值=迁移前硬编码值)
};

/**
 * @brief 增强型多扇叶跟踪器
 * 功能：维护最多5个扇叶的跟踪状态
 */
class EnhancedMultiBladeTracker {
public:
    /**
     * @param match_threshold 匹配距离阈值（像素）
     */
    EnhancedMultiBladeTracker(float match_threshold = 100.0f)
        : match_thresh_(match_threshold), last_target_id_(-1) {
        // 初始化5个跟踪器
        for (int i = 0; i < 5; ++i) {
            trackers_[i] = BladeTracker(i);
        }
    }
    
    /**
     * @brief 主更新函数
     * @param detected_centers 当前帧检测到的扇叶中心
     * @param circle_center 圆心
     * @param[out] target_center 输出目标中心
     * @param[out] need_reset_kalman 是否需要重置卡尔曼滤波器
     * @return 是否有有效目标
     */
    bool update(const std::vector<cv::Point2f>& detected_centers,
                const cv::Point2f& circle_center,
                cv::Point2f& target_center,
                bool& need_reset_kalman) {
        need_reset_kalman = false;
        
        // 无检测时，所有跟踪器标记为丢失
        if (detected_centers.empty()) {
            for (auto& tracker : trackers_) {
                if (tracker.is_valid) {
                    tracker.markLost();
                }
            }
            return selectTarget(circle_center, target_center, need_reset_kalman);
        }
        
        // 匹配检测和跟踪器
        std::vector<int> matches = matchDetections(detected_centers, circle_center);
        
        // 更新匹配的跟踪器
        std::vector<bool> detection_matched(detected_centers.size(), false);
        for (int i = 0; i < 5; ++i) {
            if (matches[i] != -1) {
                trackers_[i].update(detected_centers[matches[i]], circle_center);
                detection_matched[matches[i]] = true;
            } else if (trackers_[i].is_valid) {
                trackers_[i].markLost();
            }
        }
        
        // 处理未匹配的检测：尝试分配给无效跟踪器
        for (size_t j = 0; j < detected_centers.size(); ++j) {
            if (!detection_matched[j]) {
                assignDetectionToInactiveTracker(detected_centers[j], circle_center);
            }
        }
        
        return selectTarget(circle_center, target_center, need_reset_kalman);
    }
    
    /** 重置所有跟踪器 */
    void reset() {
        for (int i = 0; i < 5; ++i) {
            trackers_[i] = BladeTracker(i);
        }
        last_target_id_ = -1;
    }
    
    /** 获取活跃跟踪器数量 */
    int getActiveCount() const {
        int count = 0;
        for (const auto& tracker : trackers_) {
            if (tracker.is_valid) count++;
        }
        return count;
    }
    
    /** 获取指定ID的跟踪器 */
    const BladeTracker& getTracker(int id) const {
        return trackers_[id];
    }
    
    /** 获取当前目标ID */
    int getCurrentTargetID() const {
        return last_target_id_;
    }
    
private:
    std::array<BladeTracker, 5> trackers_;  // 固定5个跟踪器
    float match_thresh_;  // 匹配阈值
    int last_target_id_;  // 上一次选择的目标ID
    
    // 计算匹配代价
    float calculateMatchCost(const BladeTracker& tracker,
                            const cv::Point2f& detection,
                            const cv::Point2f& circle_center) const {
        if (!tracker.is_valid) {
            return std::numeric_limits<float>::max();
        }
        
        // 距离代价
        float dist = cv::norm(tracker.position - detection);
        if (dist > match_thresh_) {
            return std::numeric_limits<float>::max();
        }
        float dist_cost = dist / match_thresh_;
        
        // 半径一致性代价
        float radius_cost = 0.0f;
        if (tracker.avg_radius > 0.0f) {
            float current_radius = cv::norm(detection - circle_center);
            radius_cost = std::abs(current_radius - tracker.avg_radius) / tracker.avg_radius;
        }
        
        // 运动预测代价
        float motion_cost = 0.0f;
        if (tracker.hit_count > 1 && tracker.history.size() >= 2) {
            cv::Point2f predicted = tracker.predict();
            motion_cost = cv::norm(detection - predicted) / match_thresh_;
        }
        
        // 加权总代价
        return 0.5f * dist_cost + 0.3f * radius_cost + 0.2f * motion_cost;
    }
    
    // 匹配检测和跟踪器
    std::vector<int> matchDetections(const std::vector<cv::Point2f>& detections,
                                     const cv::Point2f& circle_center) {
        std::vector<int> matches(5, -1);
        std::vector<bool> det_used(detections.size(), false);
        
        // 第一轮：高置信度跟踪器优先匹配
        for (int i = 0; i < 5; ++i) {
            if (!trackers_[i].is_valid || trackers_[i].confidence < 0.5f) {
                continue;
            }
            
            float min_cost = 1.0f;  // 成本阈值
            int best_j = -1;
            
            for (size_t j = 0; j < detections.size(); ++j) {
                if (det_used[j]) continue;
                
                float cost = calculateMatchCost(trackers_[i], detections[j], circle_center);
                if (cost < min_cost) {
                    min_cost = cost;
                    best_j = static_cast<int>(j);
                }
            }
            
            if (best_j != -1 && min_cost < 1.0f) {
                matches[i] = best_j;
                det_used[best_j] = true;
            }
        }
        
        // 第二轮：低置信度跟踪器匹配
        for (int i = 0; i < 5; ++i) {
            if (matches[i] != -1) continue;  // 已匹配
            
            float min_cost = 1.0f;
            int best_j = -1;
            
            for (size_t j = 0; j < detections.size(); ++j) {
                if (det_used[j]) continue;
                
                float cost = calculateMatchCost(trackers_[i], detections[j], circle_center);
                if (cost < min_cost) {
                    min_cost = cost;
                    best_j = static_cast<int>(j);
                }
            }
            
            if (best_j != -1 && min_cost < 1.0f) {
                matches[i] = best_j;
                det_used[best_j] = true;
            }
        }
        
        return matches;
    }
    
    // 将检测分配给无效跟踪器
    void assignDetectionToInactiveTracker(const cv::Point2f& detection,
                                          const cv::Point2f& circle_center) {
        // 寻找最差的跟踪器（置信度最低且有效）
        int worst_idx = -1;
        float worst_confidence = 1.0f;
        
        for (int i = 0; i < 5; ++i) {
            if (trackers_[i].is_valid) {
                if (trackers_[i].confidence < worst_confidence) {
                    worst_confidence = trackers_[i].confidence;
                    worst_idx = i;
                }
            } else {
                // 如果找到无效跟踪器，直接使用
                trackers_[i].update(detection, circle_center);
                return;
            }
        }
        
        // 如果没有无效跟踪器，但最差跟踪器置信度很低，则替换
        if (worst_idx != -1 && worst_confidence < 0.3f) {
            trackers_[worst_idx].update(detection, circle_center);
        }
    }
    
    // 选择打击目标
    bool selectTarget(const cv::Point2f& circle_center,
                     cv::Point2f& target_center,
                     bool& need_reset_kalman) {
        need_reset_kalman = false;
        
        // 收集有效跟踪器
        std::vector<const BladeTracker*> valid_trackers;
        for (const auto& tracker : trackers_) {
            if (tracker.is_valid) {
                valid_trackers.push_back(&tracker);
            }
        }
        
        if (valid_trackers.empty()) {
            target_center = cv::Point2f(-1, -1);
            return false;
        }
        
        // 选择最佳目标
        const BladeTracker* best_tracker = nullptr;
        float best_score = -1.0f;
        
        for (const auto& tracker : valid_trackers) {
            // 计算综合评分
            float score = 0.0f;
            
            // 1. 置信度（40%权重）
            score += tracker->confidence * 0.4f;
            
            // 2. 距离圆心远近（30%权重，越远越容易打击）
            float dist_to_center = cv::norm(tracker->position - circle_center);
            score += (dist_to_center / 500.0f) * 0.3f;  // 假设最大半径约500像素
            
            // 3. 连续命中次数（20%权重）
            score += (std::min(tracker->hit_count, 20) / 20.0f) * 0.2f;
            
            // 4. 距离上次目标的相似度（10%权重，避免频繁切换）
            if (last_target_id_ == tracker->id) {
                score += 0.1f;
            }
            
            if (score > best_score) {
                best_score = score;
                best_tracker = tracker;
            }
        }
        
        if (best_tracker) {
            // ====== 添加速度预测 ======
            // 估计子弹飞行时间（假设距离3米，弹速21m/s）
            double fly_time = 3.0 / 21.0; // 距离/速度
            // 根据扇叶速度预测击中位置
            target_center = best_tracker->position + best_tracker->velocity * fly_time;
            
            need_reset_kalman = (last_target_id_ != best_tracker->id);
            last_target_id_ = best_tracker->id;
            return true;
        }
        
        target_center = cv::Point2f(-1, -1);
        return false;
    }
};

