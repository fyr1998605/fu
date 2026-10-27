#pragma once

#include <opencv2/core.hpp>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <deque>
#include <mutex>
#include <utility>

struct BuffDisplayFrame {
    cv::Mat image;
    std::uint64_t sequence = 0;
};

// A small bounded FIFO used only by the debug display path.  The aiming path
// keeps its latest-frame semantics; this queue prevents the UI from being
// blocked by inference and preserves frame order while the display catches up.
class BuffDisplayFrameQueue {
public:
    explicit BuffDisplayFrameQueue(std::size_t capacity = 8)
        : capacity_(capacity == 0 ? 1 : capacity) {}

    bool push(const cv::Mat& image, std::uint64_t sequence) {
        if (image.empty()) return false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) return false;
            if (frames_.size() >= capacity_) frames_.pop_front();
            frames_.push_back({image.clone(), sequence});
        }
        condition_.notify_one();
        return true;
    }

    bool tryPop(cv::Mat& image, std::uint64_t& sequence) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frames_.empty()) return false;
        BuffDisplayFrame frame = std::move(frames_.front());
        frames_.pop_front();
        image = std::move(frame.image);
        sequence = frame.sequence;
        return true;
    }

    bool waitPop(cv::Mat& image, std::uint64_t& sequence) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!condition_.wait_for(lock, std::chrono::milliseconds(10), [this] {
            return stopped_ || !frames_.empty();
        })) return false;
        if (stopped_) return false;
        if (frames_.empty()) return false;
        BuffDisplayFrame frame = std::move(frames_.front());
        frames_.pop_front();
        image = std::move(frame.image);
        sequence = frame.sequence;
        return true;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        condition_.notify_all();
    }

private:
    const std::size_t capacity_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<BuffDisplayFrame> frames_;
    bool stopped_ = false;
};
