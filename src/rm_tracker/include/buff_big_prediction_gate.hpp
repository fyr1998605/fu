#pragma once

#include <cmath>

namespace buff_zju {

constexpr double kBigPredictionMaxDisagreementRadians = 0.5235987755982988;
constexpr int kBigPredictionAcquireFrames = 3;

inline bool bigPredictionAgreesWithPrimary(const double primary_angle,
                                           const double big_angle) {
    if (!std::isfinite(primary_angle) || !std::isfinite(big_angle)) return false;
    return std::abs(std::remainder(big_angle - primary_angle, 6.283185307179586)) <=
        kBigPredictionMaxDisagreementRadians;
}

// Require consecutive valid predictions before the variable-speed model can
// take over. A bad update or invalid projected point resets the streak and
// immediately returns control to the stable primary path.
class BigBuffPredictionGate {
public:
    bool update(const bool prediction_valid) {
        if (!prediction_valid) {
            reject();
            return false;
        }
        if (good_frames_ < kBigPredictionAcquireFrames) {
            ++good_frames_;
        }
        active_ = good_frames_ >= kBigPredictionAcquireFrames;
        return active_;
    }

    void reject() {
        good_frames_ = 0;
        active_ = false;
    }

    bool active() const { return active_; }

private:
    int good_frames_ = 0;
    bool active_ = false;
};

}  // namespace buff_zju
