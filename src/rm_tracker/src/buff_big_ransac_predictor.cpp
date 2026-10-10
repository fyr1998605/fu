#include "buff_big_ransac_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace buff_zju {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kMinThreshold = 1e-4;
constexpr int kMaxRansacIterations = 1000;
constexpr int kMaxInliers = 300;
constexpr int kMaxConsecutiveRejections = 300;
constexpr int kMaxFitIntervalSamples = 100;
constexpr double kMaxAbsSpeed = 20.0;
constexpr double kMaxGapSeconds = 30.0;

// 非有限或为负的值回落为 fallback（避免 NaN 传入 uniform_real_distribution 等）
double nonnegative_or_default(const double value, const double fallback) {
    return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

double strictly_positive_or_default(const double value, const double fallback) {
    return std::isfinite(value) && value >= kMinThreshold ? value : fallback;
}

}  // namespace

BigBuffRansacConfig sanitize_big_buff_ransac_config(
    const BigBuffRansacConfig& config) {
    const BigBuffRansacConfig dflt;
    BigBuffRansacConfig out;
    out.ransac.max_iterations = config.ransac.max_iterations > 0
        ? std::clamp(config.ransac.max_iterations, 1, kMaxRansacIterations)
        : dflt.ransac.max_iterations;
    out.ransac.threshold =
        strictly_positive_or_default(config.ransac.threshold, dflt.ransac.threshold);
    out.ransac.min_omega =
        strictly_positive_or_default(config.ransac.min_omega, dflt.ransac.min_omega);
    out.ransac.max_omega =
        strictly_positive_or_default(config.ransac.max_omega, dflt.ransac.max_omega);
    out.ransac.max_omega = std::max(out.ransac.min_omega, out.ransac.max_omega);
    out.ransac.min_amplitude = nonnegative_or_default(
        config.ransac.min_amplitude, dflt.ransac.min_amplitude);
    out.ransac.max_amplitude = nonnegative_or_default(
        config.ransac.max_amplitude, dflt.ransac.max_amplitude);
    out.ransac.max_amplitude =
        std::max(out.ransac.min_amplitude, out.ransac.max_amplitude);
    out.ransac.random_seed = config.ransac.random_seed;
    out.min_inliers = std::clamp(config.min_inliers, 3, kMaxInliers);
    out.max_abs_speed = std::clamp(
        strictly_positive_or_default(config.max_abs_speed, dflt.max_abs_speed),
        kMinThreshold, kMaxAbsSpeed);
    out.max_gap_seconds = std::clamp(
        strictly_positive_or_default(config.max_gap_seconds, dflt.max_gap_seconds),
        kMinThreshold, kMaxGapSeconds);
    out.fit_every_n_samples = std::clamp(
        config.fit_every_n_samples, 1, kMaxFitIntervalSamples);
    out.max_consecutive_rejections = std::clamp(
        config.max_consecutive_rejections, 1, kMaxConsecutiveRejections);
    return out;
}

BigBuffRansacPredictor::BigBuffRansacPredictor(const BigBuffRansacConfig config)
: config_(sanitize_big_buff_ransac_config(config)),
  fitter_(
      config_.ransac.max_iterations,
      config_.ransac.threshold,
      config_.ransac.min_omega,
      config_.ransac.max_omega,
      config_.ransac.min_amplitude,
      config_.ransac.max_amplitude,
      config_.ransac.random_seed,
      config_.max_gap_seconds) {
}

void BigBuffRansacPredictor::reset() {
    fitter_.reset();
    state_ = BigBuffPredictionState{};
    last_timestamp_ = 0.0;
    current_roll_ = 0.0;
    current_velocity_ = 0.0;
    last_polar_angle_ = 0.0;
    consecutive_rejections_ = 0;
    fit_samples_since_last_fit_ = 0;
    initialized_ = false;
    has_polar_angle_ = false;
    last_update_valid_ = false;
}

void BigBuffRansacPredictor::set_rotation_sign(const int sign) {
    rotation_sign_ = sign > 0 ? 1 : (sign < 0 ? -1 : 0);
}

void BigBuffRansacPredictor::initialize(const double timestamp, const double roll) {
    initialized_ = true;
    last_update_valid_ = true;
    last_timestamp_ = timestamp;
    current_roll_ = roll;
    current_velocity_ = 0.0;
    state_ = BigBuffPredictionState{};
    state_.roll = roll;
}

bool BigBuffRansacPredictor::update(const double timestamp, const double roll) {
    return update_continuous(timestamp, roll);
}

bool BigBuffRansacPredictor::update_polar(
    const double timestamp, const double raw_angle) {
    if (!std::isfinite(timestamp) || !std::isfinite(raw_angle)) {
        last_update_valid_ = false;
        return false;
    }

    if (!has_polar_angle_) {
        has_polar_angle_ = true;
        last_polar_angle_ = raw_angle;
        const bool accepted = update_continuous(timestamp, raw_angle);
        if (!accepted) {
            has_polar_angle_ = false;
        }
        return accepted;
    }

    constexpr double two_pi = 2.0 * 3.14159265358979323846;
    // The legacy angle processor treats the five physical blades as
    // equivalent observations and corrects ID switches by multiples of
    // 2*pi/5.  Generate those aliases here as well, so a 72-degree detector
    // jump is not mistaken for an impossible angular velocity.
    constexpr double blade_period = two_pi / 5.0;
    const double polar_delta = std::remainder(raw_angle - last_polar_angle_, two_pi);
    if (!std::isfinite(polar_delta) || !initialized_) {
        last_update_valid_ = false;
        return false;
    }

    const double delta_time = timestamp - last_timestamp_;
    if (std::isfinite(delta_time) &&
        delta_time > config_.max_gap_seconds) {
        // After a long loss of signal, the old unwrapped roll is no longer a
        // reliable reference. Re-anchor to the fresh detector angle and keep
        // the polar baseline initialized for the following frame.
        const bool accepted = update_continuous(timestamp, raw_angle);
        if (accepted) {
            last_polar_angle_ = raw_angle;
            has_polar_angle_ = true;
            consecutive_rejections_ = 0;
        }
        return accepted;
    }

    // Direction-aware unwrap: the shortest arc is ambiguous once the wheel
    // has rotated more than pi over the gap, and a five-blade ID switch adds
    // an apparent 2*pi/5 jump. Pick the alias whose implied velocity
    // continues the last accepted velocity. Without prior velocity/direction,
    // retain the raw shortest arc unless it violates the speed gate.
    // Candidates that violate the speed gate are dropped; if none qualifies,
    // keep the base delta and let
    // update_continuous reject it (feeding the rejection counter below).
    // Known limit: when the prior velocity is near zero the ambiguity cannot
    // be resolved; a real direction reversal is handled outside via
    // g_persistent_rotation_dir, not here.
    double delta = polar_delta;
    if (std::isfinite(delta_time) && delta_time > kEpsilon &&
        delta_time <= config_.max_gap_seconds) {
        const bool have_prior =
            std::isfinite(current_velocity_) &&
            std::abs(current_velocity_) > kEpsilon;
        double best_delta = polar_delta;
        double best_score = std::numeric_limits<double>::infinity();
        const int max_aliases = std::min(
            40, std::max(2, static_cast<int>(std::ceil(
                    config_.max_abs_speed * delta_time / blade_period)) + 2));
        for (int k = -max_aliases; k <= max_aliases; ++k) {
            const double candidate = polar_delta + k * blade_period;
            const double velocity = candidate / delta_time;
            if (!std::isfinite(velocity) ||
                std::abs(velocity) > config_.max_abs_speed) {
                continue;
            }
            if (rotation_sign_ != 0 &&
                velocity * static_cast<double>(rotation_sign_) < -kEpsilon) {
                continue;
            }
            double score = 0.0;
            if (have_prior) {
                score = std::abs(velocity - current_velocity_);
            } else if (rotation_sign_ != 0) {
                // With a known direction, choose the smallest-speed alias
                // consistent with that direction. This handles a first-frame
                // five-blade ID switch without changing the sign.
                score = std::abs(velocity);
            } else {
                // Without either prior velocity or direction, the raw
                // shortest arc is the least-assumptive interpretation. Only
                // fall back to a blade alias when k=0 violates the speed gate.
                score = (k == 0) ? 0.0 : 1.0e6 + std::abs(candidate);
            }
            if (score < best_score) {
                best_score = score;
                best_delta = candidate;
            }
        }
        delta = best_delta;
    }

    const bool direction_ok =
        rotation_sign_ == 0 || !std::isfinite(delta_time) ||
        delta_time <= kEpsilon ||
        (delta / delta_time) * static_cast<double>(rotation_sign_) >= -kEpsilon;
    const bool accepted = direction_ok &&
        update_continuous(timestamp, current_roll_ + delta);
    if (accepted) {
        last_polar_angle_ = raw_angle;
        has_polar_angle_ = true;
        consecutive_rejections_ = 0;
        return true;
    }
    // Persistent rejection streak: the detector likely switched blade or ID,
    // so re-anchor the polar baseline to the current raw angle. Keep
    // initialized_/current_roll_/last_timestamp_ untouched (the frozen
    // last_timestamp_ must keep driving the >max_gap_seconds loss-of-signal
    // reset).
    if (++consecutive_rejections_ >= config_.max_consecutive_rejections) {
        last_polar_angle_ = raw_angle;
        consecutive_rejections_ = 0;
    }
    return false;
}

bool BigBuffRansacPredictor::update_continuous(
    const double timestamp, const double roll) {
    if (!std::isfinite(timestamp) || !std::isfinite(roll)) {
        last_update_valid_ = false;
        return false;
    }
    if (!initialized_) {
        initialize(timestamp, roll);
        return true;
    }

    const double delta_time = timestamp - last_timestamp_;
    if (!std::isfinite(delta_time) || delta_time <= kEpsilon) {
        last_update_valid_ = false;
        return false;
    }

    if (delta_time > config_.max_gap_seconds) {
        reset();
        initialize(timestamp, roll);
        return true;
    }

    const double velocity = (roll - current_roll_) / delta_time;
    if (!std::isfinite(velocity) || std::abs(velocity) > config_.max_abs_speed) {
        // Keep the valid fitting window. A single bad detection is skipped;
        // the caller's fallback remains active until a later sample is accepted.
        last_update_valid_ = false;
        return false;
    }

    last_timestamp_ = timestamp;
    current_roll_ = roll;
    state_.roll = roll;
    current_velocity_ = velocity;
    fitter_.add_data(timestamp, velocity);
    if (++fit_samples_since_last_fit_ >= config_.fit_every_n_samples) {
        fitter_.fit();
        fit_samples_since_last_fit_ = 0;
    }
    update_state(timestamp);
    last_update_valid_ = true;
    return true;
}

bool BigBuffRansacPredictor::ready() const {
    return last_update_valid_ && state_.ready;
}

double BigBuffRansacPredictor::predict_roll(const double horizon_seconds) const {
    if (!initialized_ || !std::isfinite(horizon_seconds)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double horizon = std::max(0.0, horizon_seconds);
    double predicted_roll = current_roll_ + current_velocity_ * horizon;
    if (ready() && std::abs(state_.omega) > kEpsilon) {
        predicted_roll = current_roll_ +
            state_.offset * horizon +
            state_.amplitude *
                (std::cos(state_.phase) -
                 std::cos(state_.phase + state_.omega * horizon)) /
                state_.omega;
    }
    return normalize_angle(predicted_roll);
}

void BigBuffRansacPredictor::update_state(const double timestamp) {
    const auto& result = fitter_.best_result();
    state_.roll = current_roll_;
    state_.roll_velocity = current_velocity_;
    state_.inliers = result.inliers;
    state_.ready = result.inliers >= config_.min_inliers &&
        std::isfinite(result.omega) && std::abs(result.omega) > kEpsilon;

    if (!state_.ready) {
        state_.amplitude = 0.0;
        state_.omega = 0.0;
        state_.phase = 0.0;
        state_.offset = 0.0;
        return;
    }

    state_.amplitude = result.A;
    state_.omega = result.omega;
    state_.offset = result.C;
    state_.phase = result.omega * fitter_.relative_time(timestamp) + result.phi;
    state_.roll_velocity = result.A * std::sin(state_.phase) + result.C;
}

double BigBuffRansacPredictor::normalize_angle(const double angle) {
    constexpr double two_pi = 2.0 * 3.14159265358979323846;
    return std::remainder(angle, two_pi);
}

}  // namespace buff_zju
