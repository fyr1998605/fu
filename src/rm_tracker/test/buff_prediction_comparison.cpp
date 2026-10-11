#include "buff_big_predictor.hpp"
#include "buff_big_variable_speed_predictor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

double true_angle(double t) {
    constexpr double amplitude = 0.9, frequency = 1.94, offset = 1.19, phase = 0.4;
    return 0.2 + offset * t + amplitude / frequency *
        (std::cos(phase) - std::cos(frequency * t + phase));
}
double wrap(double a) { return std::remainder(a, 6.283185307179586); }
double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    return values[static_cast<std::size_t>(std::ceil(values.size() * fraction)) - 1];
}
struct Metrics {
    int evaluated = 0;
    std::vector<double> errors, update_ms;
    void add(bool valid, double prediction, double truth, double elapsed_ms, bool evaluate) {
        update_ms.push_back(elapsed_ms);
        if (evaluate) {
            ++evaluated;
            if (valid) errors.push_back(std::abs(wrap(prediction - truth)) * 180.0 / 3.14159265358979323846);
        }
    }
    void print(const char* input, const char* model, double horizon) const {
        double sum = 0;
        for (double error : errors) sum += error;
        std::cout << input << ',' << model << ',' << horizon << ','
                  << errors.size() << '/' << evaluated << ','
                  << (errors.empty() ? std::numeric_limits<double>::quiet_NaN()
                                     : sum / errors.size()) << ','
                  << percentile(errors, 0.95) << ',' << percentile(errors, 1.0) << ','
                  << percentile(update_ms, 0.99) << ',' << percentile(update_ms, 1.0) << '\n';
    }
};
class VariableComparisonPredictor {
public:
    bool update(double timestamp, double angle, double horizon, double& predicted) {
        predicted = std::numeric_limits<double>::quiet_NaN();
        double unused = 0.0, lead = 0.0;
        const bool valid = motion.update(timestamp, angle, horizon, unused);
        const double speed = motion.angular_speed();
        const int direction = speed > 0.05 ? 1 : speed < -0.05 ? -1 : 0;
        if (!gate.updateLead(valid, motion.prediction_lead(horizon), direction, lead)) return false;
        predicted = buff_zju::bigBuffPredictionAngle(angle, lead);
        return std::isfinite(predicted);
    }
    buff_zju::BigBuffVariableSpeedPredictor motion;
    buff_zju::BigBuffPredictionGate gate;
};
int main() {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "input,model,horizon_s,valid_after_4s,mean_error_deg,p95_error_deg,max_error_deg,p99_update_ms,max_update_ms\n";
    for (const double horizon : {0.15, 0.35, 0.50}) {
        for (const bool noisy : {false, true}) {
            VariableComparisonPredictor variable;
            buff_zju::BigBuffPredictor ransac;
            std::mt19937 generator(20261011);
            std::normal_distribution<double> noise(0.0, 0.004);
            Metrics variable_metrics, ransac_metrics;
            for (int i = 0; i < 1000; ++i) {
                const double t = i * 0.01;
                double observed = true_angle(t);
                if (noisy) {
                    observed += noise(generator);
                    if (i > 0 && i % 97 == 0) observed += 0.16;
                }
                observed = wrap(observed);
                const double truth = true_angle(t + horizon);
                double prediction = 0.0;
                auto start = std::chrono::steady_clock::now();
                const bool variable_ok = variable.update(t, observed, horizon, prediction);
                double elapsed = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - start).count();
                variable_metrics.add(variable_ok, prediction, truth, elapsed, t >= 4.0);
                start = std::chrono::steady_clock::now();
                const bool ransac_ok = ransac.update(t, observed, horizon, prediction);
                elapsed = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - start).count();
                ransac_metrics.add(ransac_ok, prediction, truth, elapsed, t >= 4.0);
            }
            variable_metrics.print(noisy ? "noise_and_outliers" : "clean", "variable", horizon);
            ransac_metrics.print(noisy ? "noise_and_outliers" : "clean", "ransac", horizon);
        }
    }
}
