#include "buff_big_predictor.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <algorithm>
#include <random>
#include <vector>

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::abort();
    }
}
double wrap(double angle) { return std::remainder(angle, 6.283185307179586); }
double phase(double t) {
    return 0.2 + 1.19 * t + 0.9 / 1.94 *
        (std::cos(0.4) - std::cos(1.94 * t + 0.4));
}
}

int main() {
    constexpr double dt = 0.01;
    constexpr double horizon = 0.35;
    constexpr double blade_period = 6.283185307179586 / 5.0;
    buff_zju::BigBuffRansacConfig config;
    config.ransac.threshold = 0.03;
    config.min_inliers = 80;
    // Sine motion exceeds the small-buff speed ceiling. The output must
    // remain anchored to the current blade in both rotation directions.
    for (const int direction : {-1, 1}) {
        buff_zju::BigBuffPredictor predictor(config);
        for (int i = 0; i < 600; ++i) {
            const double t = i * dt;
            const double alias = (i / 37 % 5) * blade_period;
            const double observed = wrap(direction * phase(t) + alias);
            double predicted = 0.0;
            const bool valid = predictor.update(t, observed, horizon, predicted);
            if (i < 80) require(!valid, "warm-up must not invent an aim point");
            if (i >= 400) {
                require(valid, "converged sine prediction rejected");
                require(std::abs(wrap(predicted - direction * phase(t + horizon) - alias)) < 0.04,
                        "prediction lost the current bright blade or sine lead");
            }
        }
        double predicted = 0.0;
        require(!predictor.update(6.0, std::numeric_limits<double>::quiet_NaN(),
                                 horizon, predicted), "invalid input accepted");
        require(!std::isfinite(predicted), "invalid input reused a stale aim point");
        // The same model reacquires after three valid observations.
        for (int i = 601; i < 604; ++i) {
            const bool valid = predictor.update(i * dt,
                wrap(direction * phase(i * dt)), horizon, predicted);
            require(valid == (i == 603), "unexpected reacquisition behavior");
        }
        predictor.reset();
        require(!predictor.update(8.0, 0.5, horizon, predicted),
                "reset must restart warm-up");
    }

    // Regression for noisy finite differences with a 350ms horizon.
    buff_zju::BigBuffPredictor noisy;
    std::mt19937 generator(20261011);
    std::normal_distribution<double> noise(0.0, 0.004);
    std::vector<double> errors;
    for (int i = 0; i < 1000; ++i) {
        const double t = i * dt;
        double observed = phase(t) + noise(generator);
        if (i > 0 && i % 97 == 0) observed += 0.16;
        double predicted = 0.0;
        const bool valid = noisy.update(t, wrap(observed), horizon, predicted);
        if (i >= 400 && valid) errors.push_back(std::abs(wrap(predicted - phase(t + horizon))));
    }
    require(errors.size() > 400, "noisy sine model lost too many predictions");
    std::sort(errors.begin(), errors.end());
    require(errors[errors.size() * 95 / 100] < 0.16, "noisy sine prediction error too large");
    std::cout << "buff_big_predictor_test passed\n";
}
