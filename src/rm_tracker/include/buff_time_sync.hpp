#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <stdexcept>

// Image and pose stamps use the same ROS clock. Wall time only bounds waiting.
// Reference: the validated armor serialAt()/synchronizeCurrentFramePose() path.
struct BuffPoseSample {
    double stamp_seconds = 0.0;
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Quaterniond imu_to_map = Eigen::Quaterniond::Identity();
};

struct BuffPoseMatch {
    BuffPoseSample pose;
    double lower_stamp_seconds = 0.0;
    double upper_stamp_seconds = 0.0;
    std::uint64_t image_sequence = 0;
};

struct BuffTimeSyncConfig {
    double history_seconds = 0.5;
    double max_bracket_gap_seconds = 0.03;
    double max_wait_seconds = 0.05;
    double imu_stamp_offset_seconds = 0.0;
};

class BuffTimeSynchronizer {
public:
    enum class Status { NoFrame, Waiting, Ready, Expired };
    explicit BuffTimeSynchronizer(BuffTimeSyncConfig config = {}) : config_(config) {
        if (!std::isfinite(config.history_seconds) || config.history_seconds <= 0.0 ||
            !std::isfinite(config.max_bracket_gap_seconds) ||
            config.max_bracket_gap_seconds <= 0.0 ||
            config.max_bracket_gap_seconds > config.history_seconds ||
            !std::isfinite(config.max_wait_seconds) || config.max_wait_seconds <= 0.0 ||
            !std::isfinite(config.imu_stamp_offset_seconds))
            throw std::invalid_argument("Invalid Buff time synchronization parameters");
    }

    // A /clock reset is distinct from an out-of-order sensor packet. The caller
    // resets its predictors when this returns true; late packets are rejected.
    bool observeClock(double ros_seconds) {
        if (!std::isfinite(ros_seconds) || ros_seconds < 0.0) return false;
        const bool reset = std::isfinite(last_ros_seconds_) && ros_seconds < last_ros_seconds_;
        if (reset) {
            history_.clear();
            images_.clear();
            last_image_seconds_ = 0.0;
        }
        last_ros_seconds_ = ros_seconds;
        return reset;
    }

    bool pushPose(BuffPoseSample pose) {
        pose.stamp_seconds += config_.imu_stamp_offset_seconds;
        const double norm = pose.imu_to_map.norm();
        if (!validStamp(pose.stamp_seconds) || !pose.rpy.allFinite() ||
            !pose.imu_to_map.coeffs().allFinite() || norm < 0.9 || norm > 1.1)
            return false;
        pose.imu_to_map.normalize();
        if (!history_.empty() && pose.stamp_seconds < history_.back().stamp_seconds)
            return false;
        if (!history_.empty() && pose.stamp_seconds == history_.back().stamp_seconds)
            history_.back() = pose;
        else
            history_.push_back(pose);
        const double keep_after = pose.stamp_seconds - config_.history_seconds;
        while (history_.size() > 2 && history_[1].stamp_seconds < keep_after)
            history_.pop_front();
        while (history_.size() > kCapacity) history_.pop_front();
        return true;
    }

    bool offerImage(double stamp_seconds, double arrival_wall_seconds,
                    std::uint64_t sequence = 0) {
        if (!validStamp(stamp_seconds) || !std::isfinite(arrival_wall_seconds) ||
            stamp_seconds <= last_image_seconds_) return false;
        // Keep a bounded opportunity for the IMU to catch up. Replacing the
        // sole pending timestamp with each camera frame can starve processing.
        images_.push_back({stamp_seconds, arrival_wall_seconds, sequence});
        while (images_.size() > kImageCapacity) images_.pop_front();
        last_image_seconds_ = stamp_seconds;
        return true;
    }

    Status poll(double wall_seconds, BuffPoseMatch& matched) {
        if (images_.empty()) return Status::NoFrame;
        if (!std::isfinite(wall_seconds)) return Status::Waiting;
        // A late upper pose cannot revive an expired frame. Each image keeps
        // its own arrival/deadline, including during continuous camera input.
        PendingImage expired;
        bool had_expired = false;
        while (!images_.empty() &&
               wall_seconds - images_.front().arrival_wall_seconds >= config_.max_wait_seconds) {
            expired = images_.front();
            images_.pop_front();
            had_expired = true;
        }
        // Process the newest matchable image, never a newer unbracketed image.
        // Drop earlier images together so observation time stays monotonic.
        for (std::size_t count = images_.size(); count > 0; --count) {
            const std::size_t index = count - 1;
            if (!poseAt(images_[index].stamp_seconds, matched)) continue;
            matched.image_sequence = images_[index].sequence;
            images_.erase(images_.begin(), images_.begin() + count);
            return Status::Ready;
        }
        if (had_expired) {
            matched = {};
            matched.pose.stamp_seconds = expired.stamp_seconds;
            matched.image_sequence = expired.sequence;
            return Status::Expired;
        }
        return Status::Waiting;
    }

    bool poseAt(double stamp_seconds, BuffPoseMatch& matched) const {
        if (!validStamp(stamp_seconds) || history_.empty()) return false;
        const auto upper = std::lower_bound(history_.begin(), history_.end(), stamp_seconds,
            [](const BuffPoseSample& p, double t) { return p.stamp_seconds < t; });
        if (upper != history_.end() && upper->stamp_seconds == stamp_seconds) {
            matched = {*upper, stamp_seconds, stamp_seconds};
            return true;
        }
        if (upper == history_.begin() || upper == history_.end()) return false;
        const auto lower = std::prev(upper);
        const double span = upper->stamp_seconds - lower->stamp_seconds;
        if (span <= 0.0 || span > config_.max_bracket_gap_seconds + 1e-9) return false;
        const double ratio = (stamp_seconds - lower->stamp_seconds) / span;
        matched.pose.stamp_seconds = stamp_seconds;
        for (int axis = 0; axis < 3; ++axis) {
            matched.pose.rpy[axis] = lower->rpy[axis] + ratio * std::remainder(
                upper->rpy[axis] - lower->rpy[axis], 6.283185307179586);
        }
        // Interpolate the supplied orientation, retaining its original frame
        // convention independently of any install offsets in the RPY fields.
        matched.pose.imu_to_map = lower->imu_to_map.slerp(ratio, upper->imu_to_map).normalized();
        matched.lower_stamp_seconds = lower->stamp_seconds;
        matched.upper_stamp_seconds = upper->stamp_seconds;
        return true;
    }

    std::size_t historySize() const { return history_.size(); }
    std::size_t pendingImageCount() const { return images_.size(); }
    std::uint64_t oldestImageSequence() const {
        return images_.empty() ? 0 : images_.front().sequence;
    }
    static constexpr std::size_t kCapacity = 512;
    // The production camera publisher is capped at 120 Hz: eight slots cover
    // the default 50 ms wait without accumulating a long image latency queue.
    static constexpr std::size_t kImageCapacity = 8;

private:
    struct PendingImage {
        double stamp_seconds = 0.0;
        double arrival_wall_seconds = 0.0;
        std::uint64_t sequence = 0;
    };
    bool validStamp(double stamp) const {
        return std::isfinite(stamp) && stamp > 0.0 &&
            std::isfinite(last_ros_seconds_) &&
            stamp <= last_ros_seconds_ + config_.max_bracket_gap_seconds &&
            stamp >= last_ros_seconds_ - config_.history_seconds;
    }
    BuffTimeSyncConfig config_;
    std::deque<BuffPoseSample> history_;
    std::deque<PendingImage> images_;
    double last_ros_seconds_ = std::numeric_limits<double>::quiet_NaN();
    double last_image_seconds_ = 0.0;
};
