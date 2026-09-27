#include "buff_zju_ransac.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

namespace {

void require(const bool condition) {
    if (!condition) {
        std::abort();
    }
}

}  // namespace

int main() {
    constexpr double amplitude = 0.90;
    constexpr double omega = 1.94;
    constexpr double phase = 0.35;
    constexpr double offset = 0.02;

    buff_zju::RansacSineFitter fitter(
        300, 0.08, 1.80, 2.05, 0.70, 1.10);

    for (int index = 0; index < 60; ++index) {
        const double timestamp = 0.02 * index;
        double velocity = amplitude * std::sin(omega * timestamp + phase) + offset;
        if (index == 11) velocity += 0.75;
        if (index == 37) velocity -= 0.70;
        if (index == 52) velocity += 0.90;
        fitter.add_data(timestamp, velocity);
    }

    fitter.fit();
    const auto& result = fitter.best_result();
    require(result.inliers >= 50);
    require(std::abs(result.omega - omega) < 0.08);
    require(std::abs(result.A - amplitude) < 0.15);

    buff_zju::RansacSineFitter gap_fitter(
        20, 0.08, 1.80, 2.05, 0.70, 1.10, 0x5EEDu, 0.5);
    gap_fitter.add_data(0.0, 0.0);
    gap_fitter.add_data(0.6, 0.0);
    require(gap_fitter.sample_count() == 1);

    // Non-finite bounds must fall back to defaults: no UB in the omega
    // distribution and no crash in fit(). With amplitudes clamped to
    // [3.0, 3.0] every candidate model is rejected, so inliers == 0.
    buff_zju::RansacSineFitter nan_fitter(
        -5,
        std::numeric_limits<double>::quiet_NaN(),   // threshold -> 0.5
        std::numeric_limits<double>::quiet_NaN(),   // min_omega -> 1.884
        2.5,                                        // max_omega stays 2.5
        3.0,                                        // min_amplitude stays 3.0
        std::numeric_limits<double>::quiet_NaN());  // max_amplitude -> 1.045 -> clamped to 3.0
    for (int index = 0; index < 8; ++index) {
        const double timestamp = 0.02 * index;
        const double velocity = 1.94 + 0.8 * std::sin(1.94 * timestamp + 0.35);
        nan_fitter.add_data(timestamp, velocity);
    }
    nan_fitter.fit();
    require(nan_fitter.best_result().inliers == 0);

    return 0;
}
