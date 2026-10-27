#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

enum class BuffTimingStage : std::size_t {
    Callbacks, Copy, Initialize, Identify, Observe, Predict, Solve, Output,
    Draw, Display, Diagnostics, Count
};

struct BuffFrameTiming {
    static constexpr std::size_t kStageCount =
        static_cast<std::size_t>(BuffTimingStage::Count);
    std::array<double, kStageCount> ms{};
    std::uint64_t sequence = 0;
    double total_ms = 0.0;
    double arrival_gap_ms = 0.0;
    double capture_gap_ms = 0.0;
    double image_age_ms = std::numeric_limits<double>::quiet_NaN();
    double stage(BuffTimingStage which) const {
        return ms[static_cast<std::size_t>(which)];
    }
};

class BuffStageTimer {
public:
    BuffStageTimer(BuffFrameTiming& frame, BuffTimingStage stage)
        : frame_(frame), stage_(stage), start_(Clock::now()) {}
    ~BuffStageTimer() {
        frame_.ms[static_cast<std::size_t>(stage_)] +=
            std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    }
    BuffStageTimer(const BuffStageTimer&) = delete;
    BuffStageTimer& operator=(const BuffStageTimer&) = delete;
private:
    using Clock = std::chrono::steady_clock;
    BuffFrameTiming& frame_;
    BuffTimingStage stage_;
    Clock::time_point start_;
};

struct BuffTimingSummary {
    std::size_t frames = 0;
    std::size_t slow_frames = 0;
    double p99_ms = 0.0;
    double max_arrival_gap_ms = 0.0;
    double max_capture_gap_ms = 0.0;
    BuffFrameTiming worst;
};

// Capture every frame before rate-limiting the report. A throttled instantaneous
// measurement can miss the one slow frame that froze the display.
class BuffTimingWindow {
public:
    static constexpr std::size_t kCapacity = 512;
    explicit BuffTimingWindow(double slow_frame_ms = 50.0)
        : slow_frame_ms_(slow_frame_ms) {}

    void add(const BuffFrameTiming& frame) {
        totals_[next_] = frame.total_ms;
        next_ = (next_ + 1) % kCapacity;
        stored_ = std::min(stored_ + 1, kCapacity);
        if (summary_.frames == 0 || frame.total_ms > summary_.worst.total_ms) {
            summary_.worst = frame;
        }
        ++summary_.frames;
        if (frame.total_ms >= slow_frame_ms_) ++summary_.slow_frames;
        summary_.max_arrival_gap_ms = std::max(
            summary_.max_arrival_gap_ms, frame.arrival_gap_ms);
        summary_.max_capture_gap_ms = std::max(
            summary_.max_capture_gap_ms, frame.capture_gap_ms);
    }

    BuffTimingSummary take() {
        if (stored_ != 0) {
            auto sorted = totals_;
            std::sort(sorted.begin(), sorted.begin() + stored_);
            const auto rank = static_cast<std::size_t>(std::ceil(stored_ * 0.99));
            summary_.p99_ms = sorted[rank - 1];
        }
        const auto result = summary_;
        summary_ = BuffTimingSummary{};
        stored_ = 0;
        next_ = 0;
        return result;
    }
private:
    std::array<double, kCapacity> totals_{};
    std::size_t stored_ = 0;
    std::size_t next_ = 0;
    double slow_frame_ms_;
    BuffTimingSummary summary_;
};
