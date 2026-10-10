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
    return 0;
}
