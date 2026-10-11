#include "buff_big_variable_speed_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace buff_zju {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kBladePeriod = kTwoPi / 5.0;
constexpr double kMinDeltaSeconds = 1e-4;
constexpr double kMaxGapSeconds = 0.5;
constexpr double kMaxPredictionSeconds = 2.0;
constexpr double kMaxAngularSpeed = 3.2;
constexpr double kMaxAngularAcceleration = 8.0;
constexpr double kMaxPhaseInnovation = 0.35;

}  // namespace

void BigBuffVariableSpeedPredictor::reset() {
    last_timestamp_ = 0.0;
    last_raw_angle_ = 0.0;
    phase_ = 0.0;
    angular_speed_ = 0.0;
    angular_acceleration_ = 0.0;
    accepted_samples_ = 0;
    initialized_ = false;
}

double BigBuffVariableSpeedPredictor::prediction_lead(
    const double horizon_seconds) const {
    if (!std::isfinite(horizon_seconds) || horizon_seconds < 0.0 ||
        horizon_seconds > kMaxPredictionSeconds) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return angular_speed_ * horizon_seconds +
        0.5 * angular_acceleration_ * horizon_seconds * horizon_seconds;
}

bool BigBuffVariableSpeedPredictor::initialize(
    const double timestamp_seconds, const double raw_angle) {
    if (!std::isfinite(timestamp_seconds) || !std::isfinite(raw_angle)) {
        return false;
    }
    initialized_ = true;
    last_timestamp_ = timestamp_seconds;
    last_raw_angle_ = raw_angle;
    phase_ = raw_angle;
    angular_speed_ = 0.0;
    angular_acceleration_ = 0.0;
    accepted_samples_ = 1;
    return true;
}

bool BigBuffVariableSpeedPredictor::update(
    const double timestamp_seconds,
    const double raw_angle,
    const double horizon_seconds,
    double& predicted_angle) {
    if (!std::isfinite(timestamp_seconds) || !std::isfinite(raw_angle) ||
        !std::isfinite(horizon_seconds) || horizon_seconds < 0.0 ||
        horizon_seconds > kMaxPredictionSeconds) {
        return false;
    }

    if (!initialized_) {
        initialize(timestamp_seconds, raw_angle);
        return false;
    }

    const double dt = timestamp_seconds - last_timestamp_;
    if (!std::isfinite(dt) || dt <= kMinDeltaSeconds) {
        return false;
    }
    if (dt > kMaxGapSeconds) {
        initialize(timestamp_seconds, raw_angle);
        return false;
    }

    const double wrapped_delta =
        std::remainder(raw_angle - last_raw_angle_, kTwoPi);
    const double expected_delta = angular_speed_ * dt +
        0.5 * angular_acceleration_ * dt * dt;
    const int max_aliases = std::clamp(
        static_cast<int>(std::ceil(kMaxAngularSpeed * dt / kBladePeriod)) + 2,
        2, 40);

    double accepted_delta = 0.0;
    double best_score = std::numeric_limits<double>::infinity();
    bool found_candidate = false;
    for (int alias = -max_aliases; alias <= max_aliases; ++alias) {
        const double candidate = wrapped_delta + alias * kBladePeriod;
        const double speed = candidate / dt;
        if (!std::isfinite(speed) || std::abs(speed) > kMaxAngularSpeed) {
            continue;
        }
        const double score = accepted_samples_ >= 2
            ? std::abs(candidate - expected_delta)
            : std::abs(candidate);
        if (score < best_score) {
            best_score = score;
            accepted_delta = candidate;
            found_candidate = true;
        }
    }
    if (!found_candidate) {
        return false;
    }
    if (accepted_samples_ >= 3 && best_score > kMaxPhaseInnovation) {
        return false;
    }

    const double measured_interval_speed = accepted_delta / dt;
    if (accepted_samples_ >= 2) {
        // Each interval speed is the average speed over that interval; the
        // difference of adjacent interval averages divided by dt estimates
        // angular acceleration directly.
        const double measured_acceleration = std::clamp(
            (measured_interval_speed - angular_speed_) / dt,
            -kMaxAngularAcceleration, kMaxAngularAcceleration);
        angular_acceleration_ =
            0.55 * measured_acceleration + 0.45 * angular_acceleration_;
        const double measured_end_speed = measured_interval_speed +
            0.5 * angular_acceleration_ * dt;
        angular_speed_ = 0.65 * measured_end_speed +
            0.35 * angular_speed_;
    } else {
        angular_speed_ = measured_interval_speed;
    }
    angular_speed_ = std::clamp(
        angular_speed_, -kMaxAngularSpeed, kMaxAngularSpeed);

    phase_ += accepted_delta;
    last_raw_angle_ = raw_angle;
    last_timestamp_ = timestamp_seconds;
    ++accepted_samples_;
    if (!ready()) {
        return false;
    }

    predicted_angle = normalize_angle(phase_ + prediction_lead(horizon_seconds));
    return std::isfinite(predicted_angle);
}

double BigBuffVariableSpeedPredictor::normalize_angle(const double angle) {
    return std::remainder(angle, kTwoPi);
}

}  // namespace buff_zju
