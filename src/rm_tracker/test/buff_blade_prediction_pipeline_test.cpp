#include "buff_target_association.hpp"
#include "buff_big_predictor.hpp"
#include <iostream>
#include <random>
#include <cstdlib>

void check(bool condition, const char* reason) {
    if (!condition) { std::cerr << reason << '\n'; std::abort(); }
}

int main() {
    for (int sign : {-1, 1}) {
        BuffBladeAssociation association;
        BuffDetectionStabilizer geometry;
        buff_zju::BigBuffPredictor predictor;
        std::mt19937 random(42);
        std::normal_distribution<double> noise(0, .002);
        int accepted = 0, predictions = 0, switches = 0;
        for (int frame = 0; frame < 1000; ++frame) {
            const double t = frame / 25.0;
            const double phase = sign * (1.19 * t + .9 / 1.94 *
                (std::cos(.4) - std::cos(1.94 * t + .4)));
            // Deliberately stress faster-than-rule rounds: two lit blades per
            // one-second cycle; first leaves at .40s, second leaves at 1.00s.
            const int cycle = frame / 25;
            const int first_id = cycle % 5;
            const int second_id = (first_id + 2) % 5;
            const BuffPoint center{320 + 60 * static_cast<float>(std::sin(t)),
                                   240 + 40 * static_cast<float>(std::cos(t))};
            std::vector<BuffTemporalCandidate> candidates;
            for (int id : {first_id, second_id}) {
                if (id == first_id && frame % 25 >= 10) continue;
                const double angle = phase + id * association.kPeriod + noise(random);
                candidates.push_back({center,
                    {center.x + 100 * static_cast<float>(std::cos(angle)),
                     center.y + 100 * static_cast<float>(std::sin(angle))},
                    frame % 2 == 0 && id == first_id ? .95f : .6f});
            }
            const double prior_speed = frame > 1 ? predictor.angular_speed() :
                std::numeric_limits<double>::quiet_NaN();
            const auto selection = association.update(candidates, t, prior_speed);
            if (selection.index < 0) {
                const bool wheel_observed = std::any_of(selection.candidate_ids.begin(),
                    selection.candidate_ids.end(), [](int id) { return id >= 0; });
                if (!wheel_observed) predictor.reject();
                continue;
            }
            ++accepted;
            switches += selection.changed;
            const auto& observation = candidates[selection.index];
            std::array<BuffPoint, 4> corners{{
                {observation.fan_center.x - 8, observation.fan_center.y - 8},
                {observation.fan_center.x + 8, observation.fan_center.y - 8},
                {observation.fan_center.x + 8, observation.fan_center.y + 8},
                {observation.fan_center.x - 8, observation.fan_center.y + 8}}};
            const auto shape = geometry.update(observation, corners, selection.changed);
            if (selection.changed) check(std::hypot(shape.fan_center.x - observation.fan_center.x,
                shape.fan_center.y - observation.fan_center.y) < .001,
                "switch must not interpolate geometry across blades");
            const double observed_angle = std::atan2(shape.fan_center.y - shape.r_center.y,
                                                      shape.fan_center.x - shape.r_center.x);
            double predicted_angle = 0;
            if (!predictor.update(t, observed_angle, .35, predicted_angle)) continue;
            ++predictions;
            const double measured = std::atan2(observation.fan_center.y - center.y,
                                                observation.fan_center.x - center.x);
            const double lead = std::remainder(predicted_angle - measured, 6.283185307179586);
            check(std::isfinite(predicted_angle) && sign * lead >= -.05 &&
                  std::abs(lead) < .9, "wrong-blade or reversed lead");
        }
        check(accepted > 750 && switches > 50, "one-second rounds must keep producing observations");
        check(predictions > 500, "blade changes must retain the learned sine model");
        std::cout << "sign=" << sign << " accepted=" << accepted
                  << " switches=" << switches << " predictions=" << predictions << '\n';
    }
}
