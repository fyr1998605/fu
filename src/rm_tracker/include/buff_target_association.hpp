#pragma once

#include "buff_detection_stabilizer.hpp"

// Algorithm references: Climber_Vision_26 BigTargetGroup/BigTargetSelector and
// JLU BuffTarget::matchBlades. IDs belong to rotating blades, not screen sectors.
// This association state never generates a firing prediction.
class BuffBladeAssociation {
public:
    static constexpr double kPeriod = 6.283185307179586 / 5.0;

    struct Config {
        double gate_rad = 0.30;
        double min_radius_ratio = 0.65;
        double max_radius_ratio = 1.5;
        double speed_smoothing = 0.35;
        double speed_limit_radps = 3.0;
        int switch_confirm_frames = 3;
    };

    struct Result {
        int index = -1;
        int blade_id = -1;
        bool changed = false;
        std::vector<int> candidate_ids;
    };

    void setTimeoutSeconds(double seconds) {
        timeout_ = std::isfinite(seconds) ? std::max(0.01, seconds) : 0.25;
    }
    void reset() { *this = BuffBladeAssociation(config_, timeout_); }
    explicit BuffBladeAssociation(double timeout = 0.25)
        : BuffBladeAssociation(Config{}, timeout) {}
    explicit BuffBladeAssociation(Config config, double timeout = 0.25)
        : config_(config) { setTimeoutSeconds(timeout); }
    int lockedId() const { return locked_id_; }

    Result update(const std::vector<BuffTemporalCandidate>& candidates,
                  double timestamp, double expected_speed =
                      std::numeric_limits<double>::quiet_NaN(),
                  bool keep_visible_lock = true) {
        Result result;
        result.candidate_ids.assign(candidates.size(), -1);
        if (!std::isfinite(timestamp) ||
            (initialized_ && timestamp <= last_frame_time_)) return result;
        if (initialized_ && timestamp - last_observation_time_ > 0.5) reset();
        last_frame_time_ = timestamp;

        int best = -1;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (!valid(candidates[i])) continue;
            if (best < 0 || candidates[i].confidence > candidates[best].confidence)
                best = static_cast<int>(i);
        }
        if (best < 0) {
            clearPending();
            first_miss_time_ = std::numeric_limits<double>::quiet_NaN();
            return result;
        }
        if (!initialized_) {
            phase_ = angle(candidates[best]);
            radius_ = radius(candidates[best]);
            last_observation_time_ = timestamp;
            initialized_ = true;
            locked_id_ = 0;
            result.changed = true;
        }

        const double dt = timestamp - last_observation_time_;
        const double speed = std::isfinite(expected_speed)
            ? std::clamp(expected_speed, -config_.speed_limit_radps,
                         config_.speed_limit_radps) : speed_;
        const double predicted = phase_ + speed * dt;
        std::array<int, 5> slots{{-1, -1, -1, -1, -1}};
        std::array<double, 5> errors;
        errors.fill(std::numeric_limits<double>::infinity());
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (!valid(candidates[i])) continue;
            const double r = radius(candidates[i]);
            if (r < config_.min_radius_ratio * radius_ ||
                r > config_.max_radius_ratio * radius_) continue;
            const double delta = std::remainder(angle(candidates[i]) - predicted,
                                                6.283185307179586);
            const int steps = static_cast<int>(std::lround(delta / kPeriod));
            const double residual = delta - steps * kPeriod;
            if (std::abs(residual) > config_.gate_rad) continue;
            const int id = (steps % 5 + 5) % 5;
            // One observation per blade; never let duplicate boxes advance a
            // switch confirmation. Prefer confidence for the same physical ID.
            if (slots[id] < 0 || candidates[i].confidence > candidates[slots[id]].confidence) {
                slots[id] = static_cast<int>(i);
                errors[id] = residual;
            }
        }

        double residual_sum = 0.0, radius_sum = 0.0;
        int count = 0, acquisition = -1;
        for (int id = 0; id < 5; ++id) {
            const int index = slots[id];
            if (index < 0) continue;
            result.candidate_ids[index] = id;
            residual_sum += errors[id];
            radius_sum += radius(candidates[index]);
            ++count;
            if (acquisition < 0 || candidates[index].confidence > candidates[slots[acquisition]].confidence)
                acquisition = id;
        }
        if (count == 0) {
            clearPending();
            first_miss_time_ = std::numeric_limits<double>::quiet_NaN();
            return result;
        }

        // Every visible blade measures the SAME wheel phase after subtracting
        // its 72-degree offset, including while the selected blade is absent.
        const double measured_phase = predicted + residual_sum / count;
        if (dt > 1e-4) {
            const double observed_speed = std::clamp(
                (measured_phase - phase_) / dt, -config_.speed_limit_radps,
                config_.speed_limit_radps);
            speed_ += config_.speed_smoothing * (observed_speed - speed_);
        }
        phase_ = measured_phase;
        radius_ = radius_sum / count;
        last_observation_time_ = timestamp;

        if (slots[locked_id_] >= 0 && (keep_visible_lock || acquisition == locked_id_)) {
            clearPending();
            first_miss_time_ = std::numeric_limits<double>::quiet_NaN();
        } else {
            if (!std::isfinite(first_miss_time_)) first_miss_time_ = timestamp;
            if (acquisition != pending_id_) {
                pending_id_ = acquisition;
                pending_count_ = 1;
            } else {
                ++pending_count_;
            }
            // A persistently visible alternative is confirmed in three frames;
            // a one-frame dropout never steals the visible target's identity.
            if (pending_count_ < config_.switch_confirm_frames &&
                timestamp - first_miss_time_ < timeout_) return result;
            result.changed = result.changed || locked_id_ != acquisition;
            locked_id_ = acquisition;
            first_miss_time_ = std::numeric_limits<double>::quiet_NaN();
            clearPending();
        }
        result.index = slots[locked_id_];
        result.blade_id = locked_id_;
        return result;
    }

private:
    static double radius(const BuffTemporalCandidate& c) {
        return std::hypot(c.fan_center.x - c.r_center.x, c.fan_center.y - c.r_center.y);
    }
    static double angle(const BuffTemporalCandidate& c) {
        return std::atan2(c.fan_center.y - c.r_center.y, c.fan_center.x - c.r_center.x);
    }
    static bool valid(const BuffTemporalCandidate& c) {
        return std::isfinite(c.confidence) && std::isfinite(c.r_center.x) &&
               std::isfinite(c.r_center.y) && std::isfinite(c.fan_center.x) &&
               std::isfinite(c.fan_center.y) && radius(c) > 1.0;
    }
    void clearPending() { pending_id_ = -1; pending_count_ = 0; }
    double timeout_ = 0.25;
    double phase_ = 0.0, speed_ = 0.0, radius_ = 0.0;
    double last_observation_time_ = 0.0, last_frame_time_ = 0.0;
    double first_miss_time_ = std::numeric_limits<double>::quiet_NaN();
    int locked_id_ = -1, pending_id_ = -1, pending_count_ = 0;
    bool initialized_ = false;
    Config config_;
};
