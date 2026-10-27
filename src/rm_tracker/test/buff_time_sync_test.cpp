#include "buff_time_sync.hpp"
#include <cstdlib>
#include <iostream>
#include <array>

void check(bool value, const char* reason) {
    if (!value) { std::cerr << reason << '\n'; std::abort(); }
}
void near(double actual, double expected, const char* reason) {
    check(std::abs(actual - expected) < 1e-8, reason);
}
BuffPoseSample pose(double stamp, double yaw = 0.0) {
    return {stamp, {0.0, 0.0, yaw}, Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))};
}

int main() {
    using Status = BuffTimeSynchronizer::Status;
    BuffTimeSynchronizer sync;
    BuffPoseMatch match;
    sync.observeClock(10.02);
    check(sync.pushPose(pose(10.0, 3.10)), "lower pose");
    check(sync.offerImage(10.01, 100.0), "image before upper IMU arrives");
    check(sync.poll(100.002, match) == Status::Waiting, "nearest pose must not substitute for bracket");
    check(sync.pushPose(pose(10.02, -3.10)), "upper pose");
    check(sync.poll(100.005, match) == Status::Ready, "late upper pose resolves pending image");
    near(match.pose.stamp_seconds, 10.01, "interpolation uses source time");
    near(match.pose.rpy.z(), 3.141592653589793, "yaw shortest path across pi");
    near((match.pose.imu_to_map * Eigen::Vector3d::UnitX()).x(), -1.0, "SLERP shortest path across pi");
    near(match.pose.imu_to_map.norm(), 1.0, "normalized interpolation quaternion");
    near(match.upper_stamp_seconds - match.lower_stamp_seconds, .02, "real bracket span");
    check(sync.poll(100.006, match) == Status::NoFrame, "consume each source frame once");
    check(!sync.offerImage(10.01, 100.01), "duplicate image stamp");
    check(!sync.offerImage(10.005, 100.01), "late image stamp");
    check(!sync.pushPose(pose(10.005)), "out-of-order pose does not reset the epoch");
    check(sync.pushPose(pose(10.02, -.4)), "duplicate pose replacement");
    check(sync.poseAt(10.02, match), "exact sample");
    near(match.pose.rpy.z(), -.4, "new duplicate replaces old data");

    BuffTimeSynchronizer gap;
    gap.observeClock(20.1);
    gap.pushPose(pose(20.0));
    gap.pushPose(pose(20.08));
    gap.offerImage(20.04, 200.0);
    check(gap.poll(200.01, match) == Status::Waiting, "reject overly wide bracket");
    check(gap.poll(200.051, match) == Status::Expired, "wall timeout while sim clock is paused");
    check(gap.poll(201.0, match) == Status::NoFrame, "expired frames are not republished");

    BuffTimeSynchronizer latest;
    latest.observeClock(30.1);
    latest.pushPose(pose(30.0));
    latest.offerImage(30.01, 300.0);
    latest.offerImage(30.02, 300.02);
    latest.pushPose(pose(30.03));
    check(latest.poll(300.021, match) == Status::Ready, "latest-frame replacement");
    near(match.pose.stamp_seconds, 30.02, "older pending frame must not reappear");
    latest.offerImage(30.04, 301.0);
    latest.offerImage(30.05, 301.04);
    check(latest.poll(301.051, match) == Status::Expired, "new images cannot extend an IMU outage forever");

    struct StreamCase { double epoch, lag; int minimum_aligned; };
    // Exact decimal endpoint grids can skip a frame from double rounding.
    // Keep strict brackets, and verify bounded progress plus Unix-epoch input.
    const std::array<StreamCase, 4> streams{{
        {10.0, .02, 180}, {50.0, .02, 90},
        {50.0, .025, 180}, {1700000000.0, .025, 180}}};
    for (const auto& input : streams) {
        BuffTimeSynchronizer stream;
        int aligned = 0;
        double previous_match = 0.0;
        for (int i = 0; i < 200; ++i) {
            const double now = input.epoch + i * .01;
            stream.observeClock(now);
            check(stream.pushPose(pose(now - input.lag)), "continuous delayed IMU");
            check(stream.offerImage(now, 500.0 + i * .01, i + 1), "continuous image");
            if (stream.poll(500.0 + i * .01, match) == Status::Ready) {
                ++aligned;
                check(match.pose.stamp_seconds > previous_match, "matched image time stays monotonic");
                check(match.pose.stamp_seconds <= now - input.lag + 1e-8,
                      "never use an unbracketed latest image");
                near(match.pose.stamp_seconds, input.epoch + (match.image_sequence - 1) * .01,
                     "match returns the selected source image identity");
                previous_match = match.pose.stamp_seconds;
            }
            check(stream.pendingImageCount() <= stream.kImageCapacity, "bounded stream image cache");
        }
        check(aligned > input.minimum_aligned, "continuous delayed IMU must not starve image processing");
    }

    BuffTimeSynchronizer deadline;
    deadline.observeClock(60.02);
    deadline.pushPose(pose(60.0));
    deadline.offerImage(60.01, 600.0, 1);
    deadline.observeClock(60.12);
    deadline.pushPose(pose(60.02));
    check(deadline.poll(600.1, match) == Status::Expired,
          "a late complete bracket must not revive a frame past its 50ms deadline");

    BuffTimeSynchronizer independent;
    independent.observeClock(70.03);
    independent.pushPose(pose(70.0));
    independent.offerImage(70.01, 700.0, 1);
    independent.offerImage(70.02, 700.04, 2);
    independent.pushPose(pose(70.03));
    check(independent.poll(700.051, match) == Status::Ready,
          "an older expired frame must not expire a newer valid frame");
    check(match.image_sequence == 2, "select the latest valid source payload");
    check(independent.pendingImageCount() == 0, "drop processed and older queued images");

    BuffTimeSynchronizer capacity;
    for (int i = 0; i < 100; ++i) {
        capacity.observeClock(80.0 + i * .001);
        capacity.offerImage(80.0 + i * .001, 800.0 + i * .001, i + 1);
    }
    check(capacity.pendingImageCount() == capacity.kImageCapacity, "bounded image memory during outage");
    check(capacity.oldestImageSequence() == 101 - capacity.kImageCapacity,
          "payload cache can prune to the same retained source sequence");

    BuffTimeSynchronizer offset({.5, .03, .05, -.005});
    offset.observeClock(40.02);
    offset.pushPose(pose(40.01, .1));
    offset.pushPose(pose(40.02, .3));
    check(offset.poseAt(40.01, match), "corrected IMU brackets the image");
    near(match.pose.rpy.z(), .2, "offset applied only once");

    check(latest.observeClock(1.0), "explicit clock rollback starts a new epoch");
    check(latest.historySize() == 0, "rollback clears old IMU history");
    check(latest.pendingImageCount() == 0, "rollback clears all buffered images");
    check(latest.poll(400.0, match) == Status::NoFrame, "rollback discards pending old frame");
    check(!latest.pushPose(pose(30.0)), "reject old-epoch future packet");
    check(!latest.offerImage(30.0, 400.0), "reject old-epoch image");
    check(latest.pushPose(pose(1.0)), "new-epoch pose");
    check(latest.offerImage(1.0, 400.0), "new epoch permits lower source stamp");
    check(latest.poll(400.0, match) == Status::Ready, "new epoch proceeds");
    check(!latest.offerImage(0.0, 400.1), "zero image stamp");
    auto invalid = pose(1.01);
    invalid.imu_to_map.coeffs().setZero();
    check(!latest.pushPose(invalid), "zero quaternion");
    invalid = pose(1.01);
    invalid.rpy.x() = std::numeric_limits<double>::quiet_NaN();
    check(!latest.pushPose(invalid), "nonfinite pose");

    BuffTimeSynchronizer bounded;
    for (int i = 0; i < 10000; ++i) {
        const double t = 100 + i * .0001;
        bounded.observeClock(t);
        bounded.pushPose(pose(t));
    }
    check(bounded.historySize() <= BuffTimeSynchronizer::kCapacity, "bounded IMU memory");
    bool rejected = false;
    try { BuffTimeSynchronizer bad({.5, .03, -.1, 0}); }
    catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "reject invalid synchronization parameters");
    std::cout << "Buff time-sync regressions passed\n";
}
