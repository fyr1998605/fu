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

    // Returns the index of the best associated candidate, or -1 if no candidate
    // passes the spatial association gate. When has_previous is true and a gate
    // is set, a failed association is treated as a tracking loss — it does NOT
    // silently fall back to the highest-confidence candidate (see Section 4.3).
    static int selectCandidate(const std::vector<BuffTemporalCandidate>& candidates,
                               const BuffPoint& previous_fan_center,
                               bool has_previous,
                               float association_gate_px) {
        if (candidates.empty()) return -1;

        int highest_confidence_index = 0;
        for (std::size_t i = 1; i < candidates.size(); ++i) {
            if (candidates[i].confidence > candidates[highest_confidence_index].confidence) {
                highest_confidence_index = static_cast<int>(i);
            }
        }
        if (!has_previous || association_gate_px <= 0.0f) {
            return highest_confidence_index;
        }

        int nearest_index = -1;
        float nearest_distance_squared = std::numeric_limits<float>::infinity();
        const float gate_squared = association_gate_px * association_gate_px;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            const float dx = candidates[i].fan_center.x - previous_fan_center.x;
            const float dy = candidates[i].fan_center.y - previous_fan_center.y;
            const float distance_squared = dx * dx + dy * dy;
            if (distance_squared <= gate_squared && distance_squared < nearest_distance_squared) {
                nearest_distance_squared = distance_squared;
                nearest_index = static_cast<int>(i);
            }
        }
        // Fix: When association fails, return -1 to signal tracking loss
        // instead of silently picking the highest-confidence candidate.
        // A distant high-confidence detection is likely a different blade or a
        // false positive — accepting it causes the "box flies away" cascade.
        return nearest_index;
    }

    static float associationGateForRadius(float radius_px) {
        // 1.2×radius covers adjacent-blade chord length (~1.18×radius).
        // The old 0.45×radius gate rejected legitimate blade switches
        // (108-130 px jumps) as false positives, causing temporal
        // association failure → reacquisition stall → World EKF starved
        // of observations after every power-rune light on.
        return std::max(20.0f, radius_px * 1.2f);
    }

    // Confirm a persistent jump only after three consecutive consistent frames
    // (the initial pending frame + two confirmations that are spatially near it).
    // The old two-frame logic accepted error candidates that appeared for just two
    // frames, causing 373px → 264px → 489px jumps to pass (see Section 4.4).
    bool confirmPersistentJump(const BuffTemporalCandidate& measurement,
                               float association_gate_px) {
        if (!has_pending_jump_) {
            pending_jump_ = measurement;
            has_pending_jump_ = true;
            pending_jump_confirmations_ = 0;
            return false;
        }

        if (candidatesAreNear(measurement, pending_jump_, association_gate_px)) {
            pending_jump_confirmations_++;
            // Need 3 total consistent frames: 1 initial + 2 confirming matches
            if (pending_jump_confirmations_ >= 2) {
                has_pending_jump_ = false;
                pending_jump_confirmations_ = 0;
                return true;
            }
            // Update pending to latest for the next comparison
            pending_jump_ = measurement;
            return false;
        }

        // New measurement not near pending — reset to this new measurement
        pending_jump_ = measurement;
        pending_jump_confirmations_ = 0;
        return false;
    }

    void clearPendingJump() {
        has_pending_jump_ = false;
        pending_jump_confirmations_ = 0;
    }

    void beginReacquisition() {
        awaiting_reacquisition_ = true;
        clearPendingJump();
    }

    bool isAwaitingReacquisition() const { return awaiting_reacquisition_; }

    bool confirmReacquisition(const BuffTemporalCandidate& measurement,
                              float association_gate_px) {
        if (!awaiting_reacquisition_) return true;
        if (!confirmPersistentJump(measurement, association_gate_px)) return false;
        awaiting_reacquisition_ = false;
        return true;
    }

    BuffStabilizedGeometry update(const BuffTemporalCandidate& measurement,
                                  const std::array<BuffPoint, 4>& corners) {
        if (!has_state_) {
            smoothed_r_center_   = measurement.r_center;
            smoothed_fan_center_ = measurement.fan_center;
            smoothed_corners_    = corners;
            has_state_ = true;
            return BuffStabilizedGeometry{smoothed_r_center_, smoothed_fan_center_,
                                          smoothed_corners_};
        }

        // EMA-smooth centers
        smoothed_r_center_   = interpolate(smoothed_r_center_, measurement.r_center);
        smoothed_fan_center_ = interpolate(smoothed_fan_center_, measurement.fan_center);

        // Fan-center correction: align raw measurement to smoothed frame
        const BuffPoint fan_correction{
            smoothed_fan_center_.x - measurement.fan_center.x,
            smoothed_fan_center_.y - measurement.fan_center.y};

        // EMA-smooth each corner individually:
        //   1) translate raw corner into smoothed frame via fan_correction
        //   2) EMA with previous smoothed corner to suppress shape jitter
        for (std::size_t i = 0; i < 4; ++i) {
            const BuffPoint aligned{
                corners[i].x + fan_correction.x,
                corners[i].y + fan_correction.y};
            smoothed_corners_[i] = interpolate(smoothed_corners_[i], aligned);
        }

        return BuffStabilizedGeometry{smoothed_r_center_, smoothed_fan_center_,
                                      smoothed_corners_};
    }

    void reset() {
        has_state_ = false;
        clearPendingJump();
        awaiting_reacquisition_ = false;
    }

private:
    static bool candidatesAreNear(const BuffTemporalCandidate& first,
                                  const BuffTemporalCandidate& second,
                                  float association_gate_px) {
        if (association_gate_px <= 0.0f) return false;
        const float gate_squared = association_gate_px * association_gate_px;
        const float r_dx = first.r_center.x - second.r_center.x;
        const float r_dy = first.r_center.y - second.r_center.y;
        const float fan_dx = first.fan_center.x - second.fan_center.x;
        const float fan_dy = first.fan_center.y - second.fan_center.y;
        return r_dx * r_dx + r_dy * r_dy <= gate_squared &&
               fan_dx * fan_dx + fan_dy * fan_dy <= gate_squared;
    }

    BuffPoint interpolate(const BuffPoint& previous, const BuffPoint& measurement) const {
        return {
            previous.x + alpha_ * (measurement.x - previous.x),
            previous.y + alpha_ * (measurement.y - previous.y)};
    }

    float alpha_;
    bool has_state_ = false;
    bool has_pending_jump_ = false;
    int  pending_jump_confirmations_ = 0;  // frames confirming the pending jump
    bool awaiting_reacquisition_ = false;
    BuffPoint smoothed_r_center_;
    BuffPoint smoothed_fan_center_;
    std::array<BuffPoint, 4> smoothed_corners_;
    BuffTemporalCandidate pending_jump_;
};

#endif  // BUFF_DETECTION_STABILIZER_HPP
