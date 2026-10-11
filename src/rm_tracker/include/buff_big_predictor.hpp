#pragma once

#include <cmath>
#include <limits>

#include "buff_big_prediction_gate.hpp"
#include "buff_big_ransac_predictor.hpp"

namespace buff_zju {

// One big-buff prediction path: motion estimate -> lead validation -> current
// bright blade + lead. Invalid observations never select another motion model.
class BigBuffPredictor {
public:
    explicit BigBuffPredictor(BigBuffRansacConfig config = {}) : motion_(config) {}
    void configure(const BigBuffRansacConfig& config) {
        motion_ = BigBuffRansacPredictor(config);
        gate_.reject();
        lead_ = 0.0;
    }

    bool update(double timestamp, double bright_blade_angle, double horizon,
                double& predicted_angle) {
        predicted_angle = std::numeric_limits<double>::quiet_NaN();
        if (!std::isfinite(horizon) || horizon < 0.0 || horizon > 2.0) {
            gate_.reject();
            return false;
        }
        const bool valid = motion_.update_polar(timestamp, bright_blade_angle) &&
            motion_.ready();
        // Consume the fitted sine model only when ready. predict_roll() also
        // supports linear startup extrapolation, which this path never uses.
        const double candidate_lead = valid ? std::remainder(
            motion_.predict_roll(horizon) - motion_.state().roll,
            6.283185307179586) : std::numeric_limits<double>::quiet_NaN();
        const double speed = motion_.state().roll_velocity;
        const int direction = speed > 0.05 ? 1 : speed < -0.05 ? -1 : 0;
        if (!gate_.updateLead(valid, candidate_lead, direction, lead_)) {
            return false;
        }
        predicted_angle = bigBuffPredictionAngle(bright_blade_angle, lead_);
        return std::isfinite(predicted_angle);
    }

    void reject() { gate_.reject(); }
    void reset() {
        motion_.reset();
        gate_.reject();
        lead_ = 0.0;
    }
    double lead() const { return lead_; }
    double angular_speed() const { return motion_.state().roll_velocity; }
    int inliers() const { return motion_.state().inliers; }

private:
    BigBuffRansacPredictor motion_;
    BigBuffPredictionGate gate_;
    double lead_ = 0.0;
};

}  // namespace buff_zju
