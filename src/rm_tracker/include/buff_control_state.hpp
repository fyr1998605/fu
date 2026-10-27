#pragma once

#include <cmath>

// MCU state values: 0=no target, 1=observed without aiming, 3=aiming.
// Only a completed source frame renews the control deadline; callbacks and
// waiting images do not keep a previous aiming command alive.
class BuffControlState {
public:
    static int code(bool observed, bool aim_ready) {
        return !observed ? 0 : aim_ready ? 3 : 1;
    }

    void published(int code, double wall_seconds) {
        code_ = code;
        last_output_seconds_ = wall_seconds;
    }

    bool expired(double wall_seconds, double max_wait_seconds) const {
        return code_ != 0 && std::isfinite(last_output_seconds_) &&
            wall_seconds - last_output_seconds_ >= max_wait_seconds;
    }

    void reset() { code_ = 0; last_output_seconds_ = NAN; }

private:
    int code_ = 0;
    double last_output_seconds_ = NAN;
};
