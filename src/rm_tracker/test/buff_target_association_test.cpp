#include "buff_target_association.hpp"

#include <cstdlib>
#include <iostream>

namespace {
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::abort(); }
}
BuffTemporalCandidate blade(double phase, int id, float confidence,
                            float cx = 320, float cy = 240, float radius = 100) {
    const double angle = phase + id * BuffBladeAssociation::kPeriod;
    return {{cx, cy}, {cx + radius * static_cast<float>(std::cos(angle)),
                      cy + radius * static_cast<float>(std::sin(angle))}, confidence};
}
}

int main() {
    for (int sign : {-1, 1}) {
        BuffBladeAssociation tracker;
        auto first = tracker.update({blade(0, 0, .9f), blade(0, 1, .5f)}, 0);
        require(first.index == 0 && first.changed, "initial acquisition");
        for (int i = 1; i <= 600; ++i) {
            const double t = i * .04;
            const double phase = sign * (1.19 * t + .9 / 1.94 * (1 - std::cos(1.94 * t)));
            const double speed = sign * (1.19 + .9 * std::sin(1.94 * (t - .04)));
            // Camera translation far beyond the old pixel gate and crossing
            // every screen quadrant must not alter a physical blade ID.
            const float cx = 320 + 140 * std::sin(i * .17);
            const float cy = 240 + 100 * std::cos(i * .13);
            std::vector<BuffTemporalCandidate> observations{
                blade(phase, 1, .99f, cx, cy), blade(phase, 0, .3f, cx, cy)};
            if (i % 97 == 0) observations.resize(1); // one-frame locked-blade miss
            const auto result = tracker.update(observations, t, speed);
            if (i % 97 == 0) require(result.index < 0, "miss must produce no observation");
            else require(result.index == 1 && result.blade_id == 0 && !result.changed,
                         "confidence/translation/rotation must not steal lock");
        }
    }

    BuffBladeAssociation tracker;
    tracker.update({blade(0, 0, .9f), blade(0, 1, .5f)}, 0);
    require(tracker.update({blade(.04, 1, .9f)}, .04, 1).index < 0, "switch frame 1");
    require(tracker.update({blade(.08, 1, .9f)}, .08, 1).index < 0, "switch frame 2");
    auto switched = tracker.update({blade(.12, 1, .9f)}, .12, 1);
    require(switched.index == 0 && switched.blade_id == 1 && switched.changed,
            "persistent alternative must switch without 250ms stall");
    require(tracker.update({blade(.16, 0, .99f), blade(.16, 1, .3f)}, .16, 1).index == 1,
            "returning old blade must not steal new lock");
    require(tracker.update({blade(.2, 1, .9f, 320, 240, 250)}, .2, 1).index < 0,
            "wrong radius must not update");
    require(tracker.update({blade(.6, 1, .9f)}, .6, 1).index == 0,
            "prediction must bridge a bounded missing interval");
    require(tracker.update({blade(.6, 1, .9f)}, .6, 1).index < 0,
            "duplicate timestamps are not fresh observations");

    // Two detector duplicates cannot count as consecutive switch frames.
    BuffBladeAssociation duplicates;
    duplicates.update({blade(0, 0, .9f)}, 0);
    const auto duplicate = duplicates.update({blade(.04, 1, .9f), blade(.04, 1, .8f)}, .04, 1);
    require(duplicate.index < 0 && duplicate.candidate_ids[0] == 1 &&
            duplicate.candidate_ids[1] == -1, "one observation per ID");
    duplicates.update({}, .08, 1);
    require(duplicates.update({blade(.12, 1, .9f)}, .12, 1).index < 0,
            "missing frame must break consecutive confirmation");
    auto invalid = blade(.16, 1, .9f);
    invalid.fan_center.x = std::numeric_limits<float>::quiet_NaN();
    require(duplicates.update({invalid}, .16, 1).index < 0, "nonfinite candidate");
    const auto reacquired = duplicates.update({blade(2, 3, .9f)}, 1.0, 1);
    require(reacquired.index == 0 && reacquired.changed, "long absence resets identity");

    // Explicit legacy confidence priority must accept an unchanged best blade
    // every frame, instead of restarting three-frame confirmation repeatedly.
    BuffBladeAssociation confidence;
    require(confidence.update({blade(0, 0, .9f)}, 0, 1, false).index == 0,
            "confidence acquisition must produce a fresh observation");
    require(confidence.update({blade(.04, 0, .9f), blade(.04, 1, .5f)}, .04, 1, false).index == 0,
            "unchanged highest-confidence blade must not wait again");
    require(confidence.update({blade(.08, 0, .3f), blade(.08, 1, .9f)}, .08, 1, false).index < 0,
            "explicit confidence change still needs confirmation");

    std::cout << "polar association regressions passed\n";
}
