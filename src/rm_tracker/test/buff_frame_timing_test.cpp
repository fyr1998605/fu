#include "buff_frame_timing.hpp"

#include <cstdlib>
#include <iostream>

void require(bool condition) { if (!condition) std::abort(); }

int main() {
    BuffTimingWindow window(50.0);
    for (int i = 1; i <= 100; ++i) {
        BuffFrameTiming frame;
        frame.sequence = i;
        frame.total_ms = i == 37 ? 180.0 : 10.0;
        frame.ms[static_cast<std::size_t>(BuffTimingStage::Display)] =
            i == 37 ? 160.0 : 1.0;
        frame.arrival_gap_ms = i == 38 ? 185.0 : 10.0;
        frame.capture_gap_ms = 10.0;
        window.add(frame);
    }
    const auto summary = window.take();
    require(summary.frames == 100 && summary.slow_frames == 1);
    // p99 can hide a single outlier: max and its complete breakdown must survive.
    require(summary.p99_ms == 10.0 && summary.worst.total_ms == 180.0);
    require(summary.worst.sequence == 37);
    require(summary.worst.stage(BuffTimingStage::Display) == 160.0);
    require(summary.max_arrival_gap_ms == 185.0);
    require(summary.max_capture_gap_ms == 10.0);
    require(window.take().frames == 0);

    // The rolling percentile buffer stays bounded without losing the window max.
    for (std::size_t i = 0; i < BuffTimingWindow::kCapacity + 20; ++i) {
        BuffFrameTiming frame;
        frame.total_ms = i == 0 ? 250.0 : 8.0;
        window.add(frame);
    }
    const auto overflow = window.take();
    require(overflow.frames == BuffTimingWindow::kCapacity + 20);
    require(overflow.p99_ms == 8.0 && overflow.worst.total_ms == 250.0);
    std::cout << "buff_frame_timing_test passed\n";
}
