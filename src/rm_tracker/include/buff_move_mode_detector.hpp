#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "buff_config.hpp"

namespace buff {

enum class MoveMode {
    Small,
    Big,
};

struct MoveModeObservation {
    bool changed = false;
    bool evaluated = false;
    MoveMode mode = MoveMode::Small;
    double amplitude = 0.0;
};

// Low-cost classifier for the legacy angle-difference predictor. It does not
// allocate, fit, or run an optimizer: every ten valid samples it scans only
// the newest 60 samples and applies hysteresis plus three confirmations.
class MoveModeDetector {
public:
    // 判别参数注入 (Fu.cpp 从 Buff.legacy_predictor 载入)。默认值 == 迁移前硬编码值。
    void configure(const LegacyPredictorConfig& config) { legacy_cfg_ = config; }

    MoveModeObservation observe(
        const std::vector<double>& samples, const double delta_t) {
        MoveModeObservation result;
        result.mode = mode_;
        if (samples.empty() || !std::isfinite(delta_t) || delta_t <= 0.0) {
            return result;
        }

        ++sample_count_;
        if (sample_count_ % static_cast<std::size_t>(legacy_cfg_.mode_detector_evaluate_every) != 0) {
            return result;
        }

        const std::size_t window_samples =
            static_cast<std::size_t>(legacy_cfg_.mode_detector_window_samples);
        const std::size_t begin = samples.size() > window_samples
            ? samples.size() - window_samples : 0;
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        double previous = std::numeric_limits<double>::quiet_NaN();
        std::size_t valid_count = 0;
        const double max_sample_jump =
            legacy_cfg_.mode_detector_max_sample_jump_ratio * 3.14159265358979323846;
        for (std::size_t index = begin; index < samples.size(); ++index) {
            const double value = samples[index];
            if (!std::isfinite(value)) {
                continue;
            }
            if (std::isfinite(previous) &&
                std::abs(value - previous) > max_sample_jump) {
                continue;
            }
            previous = value;
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
            ++valid_count;
        }
        if (valid_count <
            static_cast<std::size_t>(legacy_cfg_.mode_detector_minimum_samples)) {
            return result;
        }

        const double expected_amplitude = expected_big_amplitude(delta_t);
        if (!std::isfinite(expected_amplitude) || expected_amplitude <= kEpsilon) {
            return result;
        }

        result.evaluated = true;
        result.amplitude = 0.5 * (maximum - minimum);
        const MoveMode candidate =
            result.amplitude >= expected_amplitude * legacy_cfg_.mode_detector_big_threshold_ratio
                ? MoveMode::Big
                : result.amplitude <= expected_amplitude * legacy_cfg_.mode_detector_small_threshold_ratio
                    ? MoveMode::Small
                    : mode_;
        if (candidate == mode_) {
            candidate_ = mode_;
            candidate_count_ = 0;
            result.mode = mode_;
            return result;
        }

        if (candidate == candidate_) {
            ++candidate_count_;
        } else {
            candidate_ = candidate;
            candidate_count_ = 1;
        }
        if (candidate_count_ >= legacy_cfg_.mode_detector_confirmations) {
            mode_ = candidate_;
            candidate_count_ = 0;
            result.changed = true;
        }
        result.mode = mode_;
        return result;
    }

    void reset() {
        mode_ = MoveMode::Small;
        candidate_ = MoveMode::Small;
        candidate_count_ = 0;
        sample_count_ = 0;
    }

    MoveMode mode() const { return mode_; }

private:
    double expected_big_amplitude(const double delta_t) const {
        // 原硬编码 omega=1.924 / level_ratio=1.1775 / angular_speed=2.090
        const double omega = legacy_cfg_.big_const_omega;
        const double level_ratio = legacy_cfg_.big_const_level;
        const double angular_speed = legacy_cfg_.big_angular_speed;
        const double amplitude =
            2.0 * (angular_speed - level_ratio) *
            std::sin(omega * delta_t * 0.5) / omega;
        return std::abs(amplitude);
    }

    static constexpr double kEpsilon = 1e-9;

    LegacyPredictorConfig legacy_cfg_;   // 判别参数 (默认值=迁移前硬编码值)
    MoveMode mode_ = MoveMode::Small;
    MoveMode candidate_ = MoveMode::Small;
    int candidate_count_ = 0;
    std::size_t sample_count_ = 0;
};

}  // namespace buff
