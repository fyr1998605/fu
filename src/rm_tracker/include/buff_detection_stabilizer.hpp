#ifndef BUFF_DETECTION_STABILIZER_HPP
#define BUFF_DETECTION_STABILIZER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

struct BuffPoint {
    float x = 0.0f;
    float y = 0.0f;
};

struct BuffTemporalCandidate {
    BuffPoint r_center;
    BuffPoint fan_center;
    float confidence = 0.0f;
};

struct BuffStabilizedGeometry {
    BuffPoint r_center;
    BuffPoint fan_center;
    std::array<BuffPoint, 4> corners;
};

class BuffDetectionStabilizer {
public:
    explicit BuffDetectionStabilizer(float alpha = 0.9f)
        : alpha_(std::max(0.0f, std::min(1.0f, alpha))) {}

    BuffStabilizedGeometry update(const BuffTemporalCandidate& measurement,
                                  const std::array<BuffPoint, 4>& corners,
                                  bool identity_changed = false) {
        if (identity_changed) reset();
        if (!has_state_) {
            smoothed_r_center_   = measurement.r_center;
            smoothed_fan_center_ = measurement.fan_center;
            smoothed_corners_    = corners;
            has_state_ = true;
            return BuffStabilizedGeometry{smoothed_r_center_, smoothed_fan_center_,
                                          smoothed_corners_};
        }

        const BuffPoint previous_fan = smoothed_fan_center_;
        // EMA-smooth centers
        smoothed_r_center_   = interpolate(smoothed_r_center_, measurement.r_center);
        smoothed_fan_center_ = interpolate(smoothed_fan_center_, measurement.fan_center);

        // Smooth plate shape relative to its own center. Absolute corner EMA
        // would lag a second time behind a translating/rotating plate.
        for (std::size_t i = 0; i < 4; ++i) {
            const BuffPoint previous_relative{smoothed_corners_[i].x - previous_fan.x,
                                              smoothed_corners_[i].y - previous_fan.y};
            const BuffPoint measured_relative{corners[i].x - measurement.fan_center.x,
                                              corners[i].y - measurement.fan_center.y};
            const BuffPoint shape = interpolate(previous_relative, measured_relative);
            smoothed_corners_[i] = {smoothed_fan_center_.x + shape.x,
                                    smoothed_fan_center_.y + shape.y};
        }

        return BuffStabilizedGeometry{smoothed_r_center_, smoothed_fan_center_,
                                      smoothed_corners_};
    }

    void reset() {
        has_state_ = false;
    }

private:
    BuffPoint interpolate(const BuffPoint& previous, const BuffPoint& measurement) const {
        return {
            previous.x + alpha_ * (measurement.x - previous.x),
            previous.y + alpha_ * (measurement.y - previous.y)};
    }

    float alpha_;
    bool has_state_ = false;
    BuffPoint smoothed_r_center_;
    BuffPoint smoothed_fan_center_;
    std::array<BuffPoint, 4> smoothed_corners_;
};

#endif  // BUFF_DETECTION_STABILIZER_HPP
