#pragma once

#include <cmath>
#include <limits>

namespace buff_zju {

constexpr double kBigPredictionMaxDisagreementRadians = 0.5235987755982988;
constexpr int kBigPredictionAcquireFrames = 3;
constexpr double kBigPredictionMaxLeadRadians = 1.5;
constexpr double kBigPredictionMaxLeadStepRadians = 0.35;

struct BigBuffPredictionGateConfig {
    double max_disagreement_rad = kBigPredictionMaxDisagreementRadians;
    int acquire_frames = kBigPredictionAcquireFrames;
    double max_lead_rad = kBigPredictionMaxLeadRadians;
    double max_lead_step_rad = kBigPredictionMaxLeadStepRadians;
};

inline double bigBuffPredictionAngle(const double bright_blade_angle,
                                     const double lead_angle) {
    if (!std::isfinite(bright_blade_angle) || !std::isfinite(lead_angle)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::remainder(bright_blade_angle + lead_angle,
                          2.0 * 3.14159265358979323846);
}

inline bool bigPredictionAgreesWithPrimary(const double primary_angle,
                                           const double big_angle,
                                           BigBuffPredictionGateConfig config = BigBuffPredictionGateConfig{}) {
    if (!std::isfinite(primary_angle) || !std::isfinite(big_angle)) return false;
    return std::abs(std::remainder(big_angle - primary_angle, 6.283185307179586)) <=
        config.max_disagreement_rad;
}

// Require consecutive valid predictions before accepting the model output.
// A bad update or invalid projected point resets the acquisition streak.
class BigBuffPredictionGate {
public:
    BigBuffPredictionGate() = default;
    explicit BigBuffPredictionGate(BigBuffPredictionGateConfig config)
        : config_(config) {}

    bool update(const bool prediction_valid) {
        if (!prediction_valid) {
            reject();
            return false;
        }
        if (good_frames_ < config_.acquire_frames) {
            ++good_frames_;
        }
        active_ = good_frames_ >= config_.acquire_frames;
        return active_;
    }

    // Validate the selected model's lead before it is added to the
    // current bright-blade angle. The lead must remain physically bounded,
    // keep the established rotation direction, and change continuously.
    bool updateLead(const bool prediction_valid,
                    const double lead_angle,
                    const int rotation_sign,
                    double& stable_lead) {
        if (!prediction_valid || !std::isfinite(lead_angle) ||
            std::abs(lead_angle) > config_.max_lead_rad ||
            (rotation_sign != 0 &&
             lead_angle * static_cast<double>(rotation_sign) < -0.05)) {
            reject();
            return false;
        }
        if (has_last_lead_ &&
            std::abs(lead_angle - last_lead_) >
                config_.max_lead_step_rad) {
            reject();
            return false;
        }
        last_lead_ = lead_angle;
        has_last_lead_ = true;
        if (good_frames_ < config_.acquire_frames) {
            ++good_frames_;
        }
        active_ = good_frames_ >= config_.acquire_frames;
        stable_lead = last_lead_;
        return active_;
    }

    void reject() {
        good_frames_ = 0;
        active_ = false;
        has_last_lead_ = false;
        last_lead_ = 0.0;
    }

    bool active() const { return active_; }

private:
    int good_frames_ = 0;
    bool active_ = false;
    bool has_last_lead_ = false;
    double last_lead_ = 0.0;
    BigBuffPredictionGateConfig config_;
};

}  // namespace buff_zju
