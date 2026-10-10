// SmallBuffPhaseTracker: 靶子切换 (five-blade ID switch) regression test.
//
// Field report: "打中能量机关后切换靶子预测点会反转". The detector reports only
// the active/lit blade, so a hit moves the measured polar angle by one blade
// period (2*pi/5 = 72 deg). The phase filter folds every measurement by that
// period on purpose - a blade switch must not look like a 72 deg velocity
// spike - but the predicted point used to be placed at the folded phase too,
// which left it on whichever blade the tracker first locked onto. After each
// switch the point fell one more blade behind the lit target (0, -72, -144 deg),
// so a few hits put the aim on the far side of the rune.
//
// The position is now re-anchored to the blade the detector reports while the
// velocity model, direction votes and rejection logic stay blade-invariant.

#include <cmath>
#include <cstdlib>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "blade_tracker.hpp"

namespace {

constexpr double kBladePeriod = 2.0 * M_PI / 5.0;
constexpr double kCenterX = 650.0;
constexpr double kCenterY = 502.0;
constexpr double kRadius = 160.0;
constexpr double kFps = 50.0;
constexpr double kDt = 1.0 / kFps;
constexpr double kOmega = -M_PI / 3.0;  // -60 deg/s
constexpr double kLeadSeconds = 0.10;

int g_failures = 0;

void require(const bool condition, const char* what) {
    if (!condition) {
        std::cerr << "FAIL: " << what << std::endl;
        ++g_failures;
    }
}

double pointPhase(const cv::Point2f& point) {
    return std::atan2(point.y - kCenterY, point.x - kCenterX);
}

double wrapToPi(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

// Blade centre of the lit blade, in image coordinates.
cv::Point2f litBladeCenter(const double time_seconds, const int lit_index) {
    const double theta = kOmega * time_seconds + lit_index * kBladePeriod;
    return cv::Point2f(
        static_cast<float>(kCenterX + kRadius * std::cos(theta)),
        static_cast<float>(kCenterY + kRadius * std::sin(theta)));
}

// Angular error (deg) of the predicted point against the lit blade's position
// at the prediction horizon. Zero means the point leads the live target exactly.
double predictionErrorDeg(const cv::Point2f& point, const double time_seconds,
                          const int lit_index) {
    const double theta = kOmega * time_seconds + lit_index * kBladePeriod;
    const double future = std::atan2(
        std::sin(theta + kOmega * kLeadSeconds),
        std::cos(theta + kOmega * kLeadSeconds));
    return std::abs(wrapToPi(pointPhase(point) - future) * 180.0 / M_PI);
}

struct RuneResult {
    double worst_steady_error_deg = 0.0;
    int anchor_before_first_switch = 0;
    int anchor_at_end = 0;
};

int anchorStepsOf(const SmallBuffPhaseTracker& tracker) {
    return static_cast<int>(
        std::lround(tracker.bladeAnchorRadians() / kBladePeriod));
}

// Runs the tracker over a five-blade rune whose lit blade changes at each entry
// of `switch_frames`. The error is measured against the lit blade's position at
// the prediction horizon, so it also validates the lead (angular velocity).
RuneResult runRune(SmallBuffPhaseTracker& tracker,
                   const std::vector<int>& switch_frames,
                   const std::vector<int>& switch_steps,
                   const int total_frames,
                   const int start_frame = 120,
                   const int guard_frames =
                       SmallBuffPhaseTracker::kAnchorConfirmFrames + 1) {
    const cv::Point2f center(static_cast<float>(kCenterX),
                             static_cast<float>(kCenterY));
    RuneResult result;
    result.anchor_before_first_switch = anchorStepsOf(tracker);
    int lit_index = 0;
    for (int frame = 0; frame < total_frames; ++frame) {
        for (std::size_t i = 0; i < switch_frames.size(); ++i) {
            if (frame == switch_frames[i]) {
                lit_index = (lit_index + switch_steps[i]) % 5;
                if (lit_index < 0) lit_index += 5;
            }
        }
        const double time_seconds = frame / kFps;
        const cv::Point2f blade = litBladeCenter(time_seconds, lit_index);
        cv::Point2f predicted;
        if (!tracker.update(center, blade, time_seconds, kLeadSeconds,
                            predicted)) {
            continue;
        }
        if (frame < start_frame) continue;
        bool guarded = false;
        for (const int switch_frame : switch_frames) {
            if (frame >= switch_frame && frame <= switch_frame + guard_frames) {
                guarded = true;
            }
        }
        if (guarded) continue;
        result.worst_steady_error_deg = std::max(
            result.worst_steady_error_deg,
            predictionErrorDeg(predicted, time_seconds, lit_index));
    }
    result.anchor_at_end = anchorStepsOf(tracker);
    return result;
}

// The anchor selects which blade the folded phase belongs to, so it must stay
// within one whole turn of the measurement: it is applied as a multiple of
// 2*pi/5, which is enough to reach the neighbouring blade at most.
void requireBoundedAnchor(const RuneResult& result) {
    require(std::abs(result.anchor_at_end) <= 2,
            "anchor must stay bounded (it selects the adjacent blade only)");
}

// The predicted point must sit on the blade the detector reports, also after a
// target switch, and the switch must not disturb the blade-invariant velocity.
void testFollowsTargetSwitch() {
    SmallBuffPhaseTracker tracker;
    const RuneResult result = runRune(
        tracker, {250, 500, 750}, {1, 1, 1}, 1000);

    std::cout << "[blade_tracker] follow switch: worst steady error = "
              << result.worst_steady_error_deg << " deg, anchor "
              << result.anchor_before_first_switch << " -> "
              << result.anchor_at_end << " blade(s)" << std::endl;
    require(result.worst_steady_error_deg < 5.0,
            "predicted point must stay on the reported blade after a switch");
    require(result.anchor_at_end != result.anchor_before_first_switch,
            "the switch must re-anchor the point onto the new blade");
    requireBoundedAnchor(result);
}

void testReversedSwitch() {
    SmallBuffPhaseTracker tracker;
    const RuneResult result = runRune(
        tracker, {250, 500, 750}, {-1, -1, -1}, 1000);

    std::cout << "[blade_tracker] reversed switch: worst steady error = "
              << result.worst_steady_error_deg << " deg, anchor "
              << result.anchor_before_first_switch << " -> "
              << result.anchor_at_end << " blade(s)" << std::endl;
    require(result.worst_steady_error_deg < 5.0,
            "predicted point must follow a switch in the other direction too");
    require(result.anchor_at_end != result.anchor_before_first_switch,
            "the switch must re-anchor the point onto the new blade");
    requireBoundedAnchor(result);
}

// A one-frame detector excursion must not re-anchor the aim point.
void testOutlierDoesNotReanchor() {
    SmallBuffPhaseTracker tracker;
    const cv::Point2f center(static_cast<float>(kCenterX),
                             static_cast<float>(kCenterY));
    const int outlier_frame = 400;
    double error_after_outlier = 0.0;
    int anchor_before_outlier = 0;
    for (int frame = 0; frame <= outlier_frame + 2; ++frame) {
        const double time_seconds = frame / kFps;
        cv::Point2f blade = litBladeCenter(time_seconds, 0);
        if (frame == outlier_frame) {
            // 40 deg displaced 靶心 for a single frame.
            const float angle = static_cast<float>(
                0.70 + std::atan2(blade.y - kCenterY, blade.x - kCenterX));
            blade = cv::Point2f(
                static_cast<float>(kCenterX + kRadius * std::cos(angle)),
                static_cast<float>(kCenterY + kRadius * std::sin(angle)));
        }
        if (frame == outlier_frame) {
            anchor_before_outlier = anchorStepsOf(tracker);
        }
        cv::Point2f predicted;
        if (!tracker.update(center, blade, time_seconds, kLeadSeconds,
                            predicted)) {
            continue;
        }
        if (frame > outlier_frame) {
            error_after_outlier = predictionErrorDeg(predicted, time_seconds, 0);
        }
    }
    const int anchor_steps = anchorStepsOf(tracker);
    std::cout << "[blade_tracker] outlier: error after = " << error_after_outlier
              << " deg, anchor " << anchor_before_outlier << " -> "
              << anchor_steps << " blade(s)" << std::endl;
    require(error_after_outlier < 5.0,
            "a one-frame outlier must not move the aim point off the blade");
    require(anchor_steps == anchor_before_outlier,
            "a one-frame outlier must not re-anchor the tracked blade");
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "buff_blade_tracker_test");
    // The tracker logs through ROS_INFO_THROTTLE / ROS_WARN_THROTTLE, which
    // need a valid ROS time even though this test never talks to a master.
    ros::Time::init();
    testFollowsTargetSwitch();
    testReversedSwitch();
    testOutlierDoesNotReanchor();
    if (g_failures != 0) {
        std::cerr << g_failures << " check(s) failed" << std::endl;
        return 1;
    }
    std::cout << "all blade tracker checks passed" << std::endl;
    return 0;
}
