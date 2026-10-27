#include "buff_big_variable_speed_predictor.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

namespace {

void require(const bool condition) {
    if (!condition) std::abort();
}

double wrap(const double angle) {
    constexpr double two_pi = 2.0 * 3.14159265358979323846;
    return std::remainder(angle, two_pi);
}

}  // namespace

int main() {
    buff_zju::BigBuffVariableSpeedPredictor predictor;
    constexpr double dt = 0.02;
    constexpr double horizon = 0.16;
    constexpr double initial_speed = 0.82;
    constexpr double angular_acceleration = 0.65;
    constexpr double initial_phase = 0.4;

    for (int index = 0; index <= 100; ++index) {
        const double timestamp = index * dt;
        const double phase = initial_phase + initial_speed * timestamp +
            0.5 * angular_acceleration * timestamp * timestamp;
        double predicted_phase = 0.0;
        const bool ready = predictor.update(
            timestamp, wrap(phase), horizon, predicted_phase);
        if (index < buff_zju::BigBuffVariableSpeedPredictor::kWarmupSamples - 1) {
            require(!ready);
            continue;
        }

        require(ready);
        const double speed_now = initial_speed +
            angular_acceleration * timestamp;
        const double expected = phase + speed_now * horizon +
            0.5 * angular_acceleration * horizon * horizon;
        require(std::abs(wrap(predicted_phase - expected)) < 0.04);
    }

    predictor.reset();
    constexpr double blade_period = 2.0 * 3.14159265358979323846 / 5.0;
    for (int index = 0; index <= 40; ++index) {
        const double timestamp = index * dt;
        const double phase = 0.2 + 1.1 * timestamp +
            0.5 * 0.3 * timestamp * timestamp;
        const double blade_switch = (index / 7 % 3) * blade_period;
        double predicted_phase = 0.0;
        const bool ready = predictor.update(
            timestamp, wrap(phase + blade_switch), 0.1, predicted_phase);
        if (index >= buff_zju::BigBuffVariableSpeedPredictor::kWarmupSamples - 1) {
            require(ready);
        }
    }

    predictor.reset();
    for (int index = 0; index < buff_zju::BigBuffVariableSpeedPredictor::kWarmupSamples;
         ++index) {
        double ignored = 0.0;
        predictor.update(index * dt, wrap(0.8 * index * dt), horizon, ignored);
    }
    double ignored = 0.0;
    const double speed_before_outlier = predictor.angular_speed();
    // A physically plausible blade-period alias exists for this bad sample,
    // but its phase innovation contradicts the established motion history.
    require(!predictor.update(0.38, wrap(0.8 * 0.38 + 0.8), horizon, ignored));
    require(std::abs(predictor.angular_speed() - speed_before_outlier) < 1e-9);
    require(predictor.update(0.40, wrap(0.8 * 0.40), horizon, ignored));

    double predicted_phase = 0.0;
    require(!predictor.update(0.0, 0.0, 0.1, predicted_phase));
    require(!predictor.update(0.02, std::numeric_limits<double>::quiet_NaN(),
                             0.1, predicted_phase));
    return 0;
}
