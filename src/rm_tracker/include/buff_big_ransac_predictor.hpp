#pragma once

#include <string>

#include "buff_zju_ransac.hpp"

namespace buff_zju {

inline bool is_big_buff_ransac_active(
    const bool enabled, const std::string& move_mode) {
    return enabled && move_mode == "big";
}

struct BigBuffRansacConfig {
    struct RansacConfig {
        int max_iterations = 200;
        double threshold = 0.5;
        double min_omega = 1.884;
        double max_omega = 2.0;
        double min_amplitude = 0.780;
        double max_amplitude = 1.045;
        unsigned int random_seed = 0x5EEDu;
    } ransac;

    int min_inliers = 100;
    double max_abs_speed = 3.2;
    double max_gap_seconds = 5.0;
    // 连续拒绝多少帧后重建极角基线（检测器扇叶/ID 跳变后的恢复机制）
    int max_consecutive_rejections = 30;
};

inline bool operator==(
    const BigBuffRansacConfig::RansacConfig& lhs,
    const BigBuffRansacConfig::RansacConfig& rhs) {
    return lhs.max_iterations == rhs.max_iterations &&
        lhs.threshold == rhs.threshold &&
        lhs.min_omega == rhs.min_omega &&
        lhs.max_omega == rhs.max_omega &&
        lhs.min_amplitude == rhs.min_amplitude &&
        lhs.max_amplitude == rhs.max_amplitude &&
        lhs.random_seed == rhs.random_seed;
}

inline bool operator!=(
    const BigBuffRansacConfig::RansacConfig& lhs,
    const BigBuffRansacConfig::RansacConfig& rhs) {
    return !(lhs == rhs);
}

inline bool operator==(
    const BigBuffRansacConfig& lhs, const BigBuffRansacConfig& rhs) {
    return lhs.ransac == rhs.ransac &&
        lhs.min_inliers == rhs.min_inliers &&
        lhs.max_abs_speed == rhs.max_abs_speed &&
        lhs.max_gap_seconds == rhs.max_gap_seconds &&
        lhs.max_consecutive_rejections == rhs.max_consecutive_rejections;
}

inline bool operator!=(
    const BigBuffRansacConfig& lhs, const BigBuffRansacConfig& rhs) {
    return !(lhs == rhs);
}

// Replace non-finite / out-of-range fields with the BigBuffRansacConfig
// defaults and enforce ordering (min <= max). Applied before the fitter is
// constructed so predictor and fitter share one consistent set of rules.
BigBuffRansacConfig sanitize_big_buff_ransac_config(
    const BigBuffRansacConfig& config);

struct BigBuffPredictionState {
    bool ready = false;
    double roll = 0.0;
    double roll_velocity = 0.0;
    double amplitude = 0.0;
    double omega = 0.0;
    double phase = 0.0;
    double offset = 0.0;
    int inliers = 0;
};

class BigBuffRansacPredictor {
public:
    explicit BigBuffRansacPredictor(BigBuffRansacConfig config = {});

    void reset();

    // timestamp is in seconds and roll is an unwrapped continuous angle.
    // False means the sample was rejected and the caller should use its fallback.
    bool update(double timestamp, double roll);

    // Accept a raw polar angle from the detector and unwrap it internally.
    // The raw angle may cross the 2*pi boundary between adjacent frames.
    bool update_polar(double timestamp, double raw_angle);

    // Optional externally confirmed direction: +1 for increasing polar angle,
    // -1 for decreasing polar angle, 0 to leave direction unconstrained.
    void set_rotation_sign(int sign);

    bool ready() const;
    double predict_roll(double horizon_seconds) const;
    const BigBuffPredictionState& state() const { return state_; }

private:
    void initialize(double timestamp, double roll);
    bool update_continuous(double timestamp, double roll);
    void update_state(double timestamp);
    static double normalize_angle(double angle);

    BigBuffRansacConfig config_;
    RansacSineFitter fitter_;
    BigBuffPredictionState state_;
    double last_timestamp_ = 0.0;
    double current_roll_ = 0.0;
    double current_velocity_ = 0.0;
    double last_polar_angle_ = 0.0;
    int consecutive_rejections_ = 0;
    bool initialized_ = false;
    bool has_polar_angle_ = false;
    bool last_update_valid_ = false;
    int rotation_sign_ = 0;
};

}  // namespace buff_zju
