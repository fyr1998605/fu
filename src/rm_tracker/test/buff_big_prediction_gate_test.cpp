#include "buff_big_prediction_gate.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

namespace {

void require(const bool condition) {
    if (!condition) std::abort();
}

}  // namespace

int main() {
    constexpr double kPi = 3.14159265358979323846;

    require(buff_zju::bigPredictionAgreesWithPrimary(0.20, 0.20));
    require(buff_zju::bigPredictionAgreesWithPrimary(0.20, 0.20 + kPi / 9.0));
    require(buff_zju::bigPredictionAgreesWithPrimary(2.0 * kPi - 0.05, 0.05));

    // Adjacent rune blades are 72 degrees apart. A result this far from the
    // shared small-buff tracker must not take over the command output.
    require(!buff_zju::bigPredictionAgreesWithPrimary(0.20, 0.20 + 2.0 * kPi / 5.0));
    require(!buff_zju::bigPredictionAgreesWithPrimary(
        0.20, 0.20 + buff_zju::kBigPredictionMaxDisagreementRadians + 0.01));
    require(!buff_zju::bigPredictionAgreesWithPrimary(
        std::numeric_limits<double>::quiet_NaN(), 0.20));

    buff_zju::BigBuffPredictionGate gate;
    // Valid variable-speed predictions can differ substantially from a
    // constant-speed primary during acceleration and must still be admitted.
    require(!gate.update(true));
    require(!gate.update(true));
    require(gate.update(true));

    // An invalid prediction immediately returns control to the primary path.
    require(!gate.update(false));
    require(!gate.update(true));
    require(!gate.update(true));
    require(gate.update(true));

    // Large-buff output is anchored to the currently detected bright blade.
    // Changing the detected blade by one 72-degree alias must not preserve
    // the previous absolute phase.
    const double lead = 0.18;
    const double first_blade = 0.20;
    const double next_blade = first_blade + 2.0 * kPi / 5.0;
    require(std::abs(buff_zju::bigBuffPredictionAngle(first_blade, lead) -
                     (first_blade + lead)) < 1e-9);
    require(std::abs(buff_zju::bigBuffPredictionAngle(next_blade, lead) -
                     (next_blade + lead)) < 1e-9);

    // A sudden lead jump is rejected instead of moving the output across a
    // blade alias.
    double stable_lead = 0.0;
    buff_zju::BigBuffPredictionGate lead_gate;
    require(!lead_gate.updateLead(true, 0.18, 1, stable_lead));
    require(!lead_gate.updateLead(true, 0.19, 1, stable_lead));
    require(lead_gate.updateLead(true, 0.20, 1, stable_lead));
    require(std::abs(stable_lead - 0.20) < 1e-9);
    require(!lead_gate.updateLead(true, 1.20, 1, stable_lead));
    require(!lead_gate.active());
    return 0;
}
