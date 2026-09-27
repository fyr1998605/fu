#include "buff_move_mode_detector.hpp"

#include <cassert>
#include <cmath>
#include <string>
#include <vector>

int main() {
    buff::MoveModeDetector detector;
    std::vector<double> small_samples(60, 0.18);

    for (int update = 0; update < 29; ++update) {
        const auto result = detector.observe(small_samples, 0.30);
        assert(!result.changed);
    }

    // A large amplitude must not switch before three independent checks.
    std::vector<double> large_samples;
    large_samples.reserve(60);
    for (int index = 0; index < 60; ++index) {
        large_samples.push_back(0.35 + 0.40 * std::sin(index * 0.08));
    }
    bool switched = false;
    int large_evaluations = 0;
    for (int update = 0; update < 30; ++update) {
        const auto result = detector.observe(large_samples, 0.30);
        if (result.evaluated) {
            ++large_evaluations;
            if (large_evaluations < 3) {
                assert(!result.changed);
            }
        }
        switched = switched || result.changed;
        if (switched) {
            assert(result.mode == buff::MoveMode::Big);
            break;
        }
    }
    assert(switched);

    // The same recent-window logic must be able to demote back to small buff
    // after the signal remains low for three independent checks.
    std::vector<double> low_samples(60, 0.0);
    bool switched_back = false;
    int small_evaluations = 0;
    for (int update = 0; update < 30; ++update) {
        const auto result = detector.observe(low_samples, 0.30);
        if (result.evaluated) {
            ++small_evaluations;
            if (small_evaluations < 3) {
                assert(!result.changed);
            }
        }
        switched_back = switched_back || result.changed;
        if (switched_back) {
            assert(result.mode == buff::MoveMode::Small);
            break;
        }
    }
    assert(switched_back);

    detector.reset();
    assert(detector.mode() == buff::MoveMode::Small);
    return 0;
}
