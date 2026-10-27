#include "buff_control_state.hpp"
#include <cstdlib>
#include <iostream>

void check(bool value, const char* reason) {
    if (!value) { std::cerr << reason << '\n'; std::exit(1); }
}

int main() {
    check(BuffControlState::code(false, false) == 0, "no target");
    check(BuffControlState::code(false, true) == 0, "prediction cannot replace an observation");
    check(BuffControlState::code(true, false) == 1, "observed model acquisition must not aim");
    check(BuffControlState::code(true, true) == 3, "valid observed prediction enables aiming");

    BuffControlState state;
    check(!state.expired(100.0, .05), "no initial command to expire");
    state.published(3, 100.0);
    check(!state.expired(100.049, .05), "allow configured wait for next frame");
    check(state.expired(100.051, .05), "camera silence must stop an aiming command");
    check(state.expired(100.2, .05), "unprocessed callbacks must not renew the deadline");
    state.published(0, 100.2);
    check(!state.expired(101.0, .05), "stop command need not be repeated in every loop");
    state.published(1, 102.0);
    check(state.expired(102.051, .05), "a disappeared observation must become no target");
    state.published(3, 103.0);
    state.reset();
    check(!state.expired(1.0, .05), "clock/mode reset must clear the old deadline");
    std::cout << "Control states and source-loss deadline passed\n";
}
