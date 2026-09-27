#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

#include "buff_config.hpp"

// World-frame observation of one power-rune target. All distances use meters.
struct BuffWorldObservation {
    double stamp_seconds = 0.0;
    Eigen::Vector3d r_center_map = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_center_map = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_normal_map = Eigen::Vector3d::UnitZ();
};

struct BuffWorldTrackerConfig {
    // A positive value uses CAD/measured geometry. Zero learns the radius from
    // the first valid observations and must be replaced by the measured value.
    double radius_m = 0.0;
    double hub_normal_offset_m = 0.0;

    double center_measurement_std_m = 0.015;
    double normal_measurement_std_rad = 0.030;
    double phase_measurement_std_rad = 0.020;
    double center_process_std_mps = 0.010;
    double normal_process_std_radps = 0.010;
    double omega_process_std_radps2 = 0.50;
};

// State: [R center x/y/z, rotor-plane normal yaw/pitch, blade phase, omega].
// The nonlinear image/PnP geometry is converted to a linear measurement before
// the update; phase is unwrapped with the five-blade (72 degree) equivalence.
class BuffWorldTracker {
public:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kBladePeriod = 2.0 * kPi / 5.0;
    // kMinSmallBuffOmega / kMaxSmallBuffOmega 已参数化 -> Buff.world_tracker.min_omega / max_omega

    void configure(const BuffWorldTrackerConfig& config) {
        config_ = config;
        reset();
    }

    // 扩展配置注入 (Fu.cpp 从 Buff.world_tracker 载入内部门限/方向锁/ready 滞回等)。
    // 默认值 == 迁移前硬编码值。
    void configure(const buff::WorldTrackerConfig& extended) {
        extended_ = extended;
    }

    void reset() {
        // locked_direction_ persists across resets — never cleared here.
        initialized_ = false;
        ready_ = false;
        observation_count_ = 0;
        last_stamp_seconds_ = 0.0;
        radius_m_ = config_.radius_m;
        state_.setZero();
        covariance_.setIdentity();
        covariance_ *= extended_.init_covariance_scale;  // 原 1e3
        basis_u_map_ = Eigen::Vector3d::UnitX();
        basis_v_map_ = Eigen::Vector3d::UnitY();
        last_normal_map_ = Eigen::Vector3d::UnitZ();
        direction_lock_counter_ = 0;
        ready_enter_counter_ = 0;
        ready_exit_counter_ = 0;
    }

    bool update(const BuffWorldObservation& observation) {
        if (!isFiniteObservation(observation)) return false;

        Eigen::Vector3d normal = observation.target_normal_map.normalized();
        if (initialized_ && normal.dot(last_normal_map_) < 0.0) normal = -normal;

        if (!initialized_) {
            initialize(observation, normal);
            return false;
        }

        const double dt = observation.stamp_seconds - last_stamp_seconds_;
        if (!std::isfinite(dt) || dt <= extended_.min_dt_seconds ||   // 原 <= 1e-4
            dt > extended_.max_dt_seconds) {                          // 原 > 1.0
            reset();
            initialize(observation, normal);
            return false;
        }

        predictState(dt);
        updateBasis(normal);

        const Eigen::Vector3d rotor_plane_center =
            observation.r_center_map - config_.hub_normal_offset_m * normal;
        const Eigen::Vector3d radial = projectToPlane(
            observation.target_center_map - rotor_plane_center, normal);
        const double observed_radius = radial.norm();
        if (!std::isfinite(observed_radius) || observed_radius < extended_.min_radius_m) return false;  // 原 < 0.05

        const double raw_phase = std::atan2(
            radial.dot(basis_v_map_), radial.dot(basis_u_map_));
        const double observed_phase = state_(5) +
            std::remainder(raw_phase - state_(5), kBladePeriod);

        const Eigen::Vector2d normal_angles = normalToYawPitch(normal);

        Eigen::Matrix<double, 6, 1> measurement;
        measurement << observation.r_center_map.x(), observation.r_center_map.y(),
            observation.r_center_map.z(), normal_angles.x(), normal_angles.y(),
            observed_phase;

        Eigen::Matrix<double, 6, 7> h = Eigen::Matrix<double, 6, 7>::Zero();
        h(0, 0) = 1.0;
        h(1, 1) = 1.0;
        h(2, 2) = 1.0;
        h(3, 3) = 1.0;
        h(4, 4) = 1.0;
        h(5, 5) = 1.0;

        Eigen::Matrix<double, 6, 1> residual = measurement - h * state_;
        residual(3) = wrapNear(residual(3));
        residual(4) = wrapNear(residual(4));
        residual(5) = std::remainder(residual(5), kBladePeriod);

        Eigen::Matrix<double, 6, 6> measurement_noise =
            Eigen::Matrix<double, 6, 6>::Zero();
        const double center_var = square(config_.center_measurement_std_m);
        measurement_noise(0, 0) = center_var;
        measurement_noise(1, 1) = center_var;
        measurement_noise(2, 2) = center_var;
        measurement_noise(3, 3) = square(config_.normal_measurement_std_rad);
        measurement_noise(4, 4) = square(config_.normal_measurement_std_rad);
        measurement_noise(5, 5) = square(config_.phase_measurement_std_rad);

        const Eigen::Matrix<double, 6, 6> innovation_covariance =
            h * covariance_ * h.transpose() + measurement_noise;

        // ── 野值门限 ──
        // 斜视时 PnP 深度可差出米级(z=4.4 vs 9.7m)。一次野值就能把
        // center/omega 状态打飞("发散"), 观测噪声再大也挡不住。
        // 逐维 Mahalanobis 门限: 任一分量超限则整帧丢弃(只推进时间戳)。
        for (int index = 0; index < 6; ++index) {
            const double sigma = std::sqrt(innovation_covariance(index, index));
            if (std::abs(residual(index)) > extended_.residual_gate_sigma * sigma) {  // 原 kResidualGateSigma 3.5
                last_stamp_seconds_ = observation.stamp_seconds;
                last_normal_map_ = normal;
                return ready_;
            }
        }

        // 半径学习(仅 radius_mm=0 时生效)放在门限之后, 避免野值污染半径
        if (config_.radius_m <= 0.0) {
            radius_m_ = radius_m_ <= 0.0 ? observed_radius
                                         : 0.9 * radius_m_ + 0.1 * observed_radius;
        }

        // ── 持续相位差分估计 omega（参照像素卡尔曼blade_tracker）──
        // EKF 的 omega 状态没有直接观测，仅能通过相位残差间接推断，
        // 收敛极慢。用相邻帧 raw_phase 差分直接估计 omega 并注入状态。
        // 前 8 帧使用快速 EMA 初始化，之后用慢速 EMA 持续跟踪。
        // 注意: 必须用与 observed_phase 一致的 72° 等效差分。识别器可能
        // 切换到另一个扇叶(±k·72°), 若用 2π remainder, 别名残差折算出的
        // est_omega 会直接夹到 ±1.571, 叠加 init 期大 alpha 导致重新锁定
        // 后 omega 符号翻转(BUFF_WORLD omega 跳到 +0.5~+0.9 rad/s 的根因)。
        {
            if (!init_phase_valid_) {
                init_raw_phase_ = raw_phase;
                init_stamp_ = observation.stamp_seconds;
                init_phase_valid_ = true;
            } else {
                double phase_diff = std::remainder(raw_phase - init_raw_phase_, kBladePeriod);
                double time_diff = observation.stamp_seconds - init_stamp_;
                if (time_diff > extended_.min_time_diff_seconds && std::isfinite(phase_diff)) {  // 原 > 0.02
                    double est_omega = phase_diff / time_diff;
                    est_omega = std::max(-extended_.max_omega,   // 原 -kMaxSmallBuffOmega
                                  std::min(extended_.max_omega, est_omega));
                    // 方向已锁定且样本明显反向: 判为噪声, 拒绝注入
                    const bool contradicts_lock = locked_direction_ != 0 &&
                        est_omega * locked_direction_ < -extended_.reverse_omega_reject_threshold;  // 原 -0.30
                    if (!contradicts_lock) {
                        // 初始化阶段快速收敛，后续慢速跟踪(≈1.5s时间常数)
                        double alpha = (observation_count_ < extended_.omega_ema_fast_frames)  // 原 < 8
                            ? extended_.omega_ema_fast : extended_.omega_ema_slow;             // 原 0.5 / 0.03
                        state_(6) = (1.0 - alpha) * state_(6) + alpha * est_omega;
                    }
                }
                init_raw_phase_ = raw_phase;
                init_stamp_ = observation.stamp_seconds;
            }
        }

        const Eigen::Matrix<double, 7, 6> gain = covariance_ * h.transpose() *
            innovation_covariance.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
        state_ += gain * residual;
        covariance_ = (Eigen::Matrix<double, 7, 7>::Identity() - gain * h) * covariance_;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());

        state_(3) = wrapNear(state_(3));
        const double normal_pitch_limit = extended_.normal_pitch_limit_deg * kPi / 180.0;  // 原 90°
        state_(4) = std::max(-normal_pitch_limit + 1e-3,
                              std::min(normal_pitch_limit - 1e-3, state_(4)));
        if (observation_count_ >= extended_.omega_clamp_start_frames) {  // 原 >= 4
            state_(6) = std::max(-extended_.max_omega,
                                  std::min(extended_.max_omega, state_(6)));
        }

        last_stamp_seconds_ = observation.stamp_seconds;
        last_normal_map_ = normal;
        ++observation_count_;

        // ── 方向锁：|omega| ≥ direction_lock_omega 连续 direction_lock_frames 帧后锁定 ──
        if (std::abs(state_(6)) >= extended_.direction_lock_omega) {   // 原 0.25
            if (direction_lock_counter_ < extended_.direction_lock_frames)  // 原 15
                ++direction_lock_counter_;
            if (direction_lock_counter_ >= extended_.direction_lock_frames && locked_direction_ == 0)
                locked_direction_ = (state_(6) > 0.0) ? 1 : -1;
        } else {
            direction_lock_counter_ = 0;
        }

        // ── ready 滞回：条件连续满足 ready_enter_frames 帧进入，连续不满足 ready_exit_frames 帧退出 ──
        const bool condition_ok = observation_count_ >= extended_.minimum_observations &&  // 原 >= 5
            std::abs(state_(6)) >= extended_.min_omega;                                    // 原 kMinSmallBuffOmega
        if (condition_ok) {
            ready_exit_counter_ = 0;
            if (ready_enter_counter_ < extended_.ready_enter_frames)   // 原 10
                ++ready_enter_counter_;
        } else {
            ready_enter_counter_ = 0;
            if (ready_exit_counter_ < extended_.ready_exit_frames)     // 原 15
                ++ready_exit_counter_;
        }

        if (!ready_ && ready_enter_counter_ >= extended_.ready_enter_frames)
            ready_ = true;
        else if (ready_ && ready_exit_counter_ >= extended_.ready_exit_frames)
            ready_ = false;

        return ready_;
    }

    bool predictTarget(double future_seconds, Eigen::Vector3d& target_map) const {//预测未来目标位置
        if (!ready_ || !std::isfinite(future_seconds) || future_seconds < 0.0 ||
            future_seconds > extended_.max_prediction_seconds || radius_m_ <= 0.0) {  // 原 > 1.0
            return false;
        }

        const Eigen::Vector3d normal = yawPitchToNormal(state_(3), state_(4));//计算法线向量
        Eigen::Vector3d u = projectToPlane(basis_u_map_, normal);
        if (u.norm() < 1e-6) return false;
        u.normalize();
        const Eigen::Vector3d v = normal.cross(u).normalized();
        const double phase = state_(5) + state_(6) * future_seconds;//计算未来相位
        const Eigen::Vector3d rotor_plane_center = state_.head<3>() -
            config_.hub_normal_offset_m * normal;//计算转子平面中心
        target_map = rotor_plane_center + radius_m_ *
            (std::cos(phase) * u + std::sin(phase) * v);//计算未来目标位置
        return target_map.allFinite();//返回是否成功
    }

    bool isReady() const { return ready_; }
    double radiusMeters() const { return radius_m_; }
    double omegaRadPerSecond() const { return state_(6); }
    int lockedDirection() const { return locked_direction_; }

private:
    static double square(double value) { return value * value; }

    static double wrapNear(double angle) { return std::remainder(angle, 2.0 * kPi); }

    static Eigen::Vector3d projectToPlane(const Eigen::Vector3d& vector,
                                          const Eigen::Vector3d& normal) {
        return vector - vector.dot(normal) * normal;
    }

    static Eigen::Vector2d normalToYawPitch(const Eigen::Vector3d& normal) {
        return Eigen::Vector2d(
            std::atan2(normal.y(), normal.x()),
            std::atan2(normal.z(), std::hypot(normal.x(), normal.y())));
    }

    static Eigen::Vector3d yawPitchToNormal(double yaw, double pitch) {
        const double horizontal = std::cos(pitch);
        return Eigen::Vector3d(horizontal * std::cos(yaw),
                               horizontal * std::sin(yaw),
                               std::sin(pitch));
    }

    static bool isFiniteObservation(const BuffWorldObservation& observation) {
        return std::isfinite(observation.stamp_seconds) &&
            observation.r_center_map.allFinite() &&
            observation.target_center_map.allFinite() &&
            observation.target_normal_map.allFinite() &&
            observation.target_normal_map.norm() > 1e-6;
    }

    void initialize(const BuffWorldObservation& observation,
                    const Eigen::Vector3d& normal) {
        const Eigen::Vector3d rotor_plane_center =
            observation.r_center_map - config_.hub_normal_offset_m * normal;
        Eigen::Vector3d radial = projectToPlane(
            observation.target_center_map - rotor_plane_center, normal);
        if (radial.norm() < extended_.min_radius_m) return;  // 原 < 0.05
        radial.normalize();

        basis_u_map_ = radial;
        basis_v_map_ = normal.cross(basis_u_map_).normalized();
        const Eigen::Vector2d normal_angles = normalToYawPitch(normal);
        state_ << observation.r_center_map.x(), observation.r_center_map.y(),
            observation.r_center_map.z(), normal_angles.x(), normal_angles.y(),
            0.0, 0.0;
        covariance_.setZero();
        covariance_.diagonal() << square(0.10), square(0.10), square(0.10),
            square(0.20), square(0.20), square(0.20), square(0.50);
		// Bias initial omega toward locked direction (实测 0.8~1.3 rad/s)。
		if (locked_direction_ != 0) {
			state_(6) = locked_direction_ * extended_.omega_init_abs;  // 原 * 0.9
		}
        radius_m_ = config_.radius_m > 0.0 ? config_.radius_m :
            projectToPlane(observation.target_center_map - rotor_plane_center, normal).norm();
        last_stamp_seconds_ = observation.stamp_seconds;
        last_normal_map_ = normal;
        initialized_ = true;
        ready_ = false;
        observation_count_ = 1;
        init_phase_valid_ = false;  // 重置相位差分
    }

    void predictState(double dt) {
        Eigen::Matrix<double, 7, 7> transition =
            Eigen::Matrix<double, 7, 7>::Identity();
        transition(5, 6) = dt;
        state_ = transition * state_;
        covariance_ = transition * covariance_ * transition.transpose();

        Eigen::Matrix<double, 7, 7> process_noise =
            Eigen::Matrix<double, 7, 7>::Zero();
        const double center_var = square(config_.center_process_std_mps) * dt;
        process_noise(0, 0) = center_var;
        process_noise(1, 1) = center_var;
        process_noise(2, 2) = center_var;
        const double normal_var = square(config_.normal_process_std_radps) * dt;
        process_noise(3, 3) = normal_var;
        process_noise(4, 4) = normal_var;
        // ── 连续时间 omega 随机游走的标准离散化 ──
        // phase_dot = omega, omega_dot ~ N(0, q)
        // Q = ∫ e^{A·s} G q Gᵀ e^{Aᵀ·s} ds  （A=[0 1;0 0], G=[0;1]）
        //   = [[dt³/3, dt²/2], [dt²/2, dt]] × q
        //
        // 注意：即使 omega_var 很大，Q_phase ∝ dt³ 在 100fps 下也仅 ~2e-8，
        // 不足以阻止相位协方差坍塌。因此额外加入直接相位过程噪声下限，
        // 保证 K_phase 稳态 ≥ 0.10（对应 ~0.03 rad/s 的无观测相位漂移）。
        const double omega_var = square(config_.omega_process_std_radps2);
        const double phase_process_noise_floor = extended_.phase_process_noise_floor;  // 原 1.0e-5 rad²/frame
        process_noise(5, 5) = std::max(
            (1.0 / 3.0) * dt * dt * dt * omega_var,
            phase_process_noise_floor);
        process_noise(5, 6) = 0.5 * dt * dt * omega_var;
        process_noise(6, 5) = process_noise(5, 6);
        process_noise(6, 6) = dt * omega_var;
        covariance_ += process_noise;
    }

    void updateBasis(const Eigen::Vector3d& normal) {
        Eigen::Vector3d projected_u = projectToPlane(basis_u_map_, normal);
        if (projected_u.norm() < 1e-6) {
            projected_u = normal.unitOrthogonal();
        }
        basis_u_map_ = projected_u.normalized();
        basis_v_map_ = normal.cross(basis_u_map_).normalized();
    }

    BuffWorldTrackerConfig config_;
    buff::WorldTrackerConfig extended_;   // 内部门限/方向锁/ready 滞回等 (默认=迁移前硬编码)
    Eigen::Matrix<double, 7, 1> state_ = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 7> covariance_ = Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::Vector3d basis_u_map_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d basis_v_map_ = Eigen::Vector3d::UnitY();
    Eigen::Vector3d last_normal_map_ = Eigen::Vector3d::UnitZ();
    double radius_m_ = 0.0;
    double last_stamp_seconds_ = 0.0;
    int observation_count_ = 0;
    bool initialized_ = false;
    bool ready_ = false;
    // ── 方向锁 / ready 滞回 / 野值门限 已参数化 -> Buff.world_tracker.* ──
	int locked_direction_ = 0;        // -1=CCW, 0=unknown, +1=CW (persists across resets)
	int direction_lock_counter_ = 0;
	int ready_enter_counter_ = 0;
	int ready_exit_counter_ = 0;
	// ── 相位差分估计 omega ──
	double init_raw_phase_ = 0.0;
	double init_stamp_ = 0.0;
	bool   init_phase_valid_ = false;
};
