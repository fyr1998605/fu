// Fu_angleprocess (legacy polar-angle processor) unit tests.
//
// NOTE: the include order below provides cv::Point2f, PI, Eigen and M_PI.

#include <cmath>
#include <cstdlib>
#include <limits>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include "GimbalPos.hpp"  // defines PI + using namespace std/cv
#include "buff_angle_processor.hpp"

int g_persistent_rotation_dir = 0;

namespace {

void require(const bool condition) {
    if (!condition) {
        std::abort();
    }
}

constexpr float kTwoPiF = 2.0f * 3.1415926f;

}  // namespace

int main() {
    {
        Fu_angleprocess ap;
        ap.set_image_size(640, 480);
        const double result = ap.pre_predict_angle_process(
            cv::Point2f(1300.0f, 240.0f), cv::Point2f(600.0f, 240.0f),
            0.0, 0.1, 120.0);
        require(result == -1.0);
        require(!ap.last_frame_polar_valid);
        require(!std::isfinite(ap.polar_angle(
            cv::Point2f(10.0f, 10.0f), cv::Point2f(10.0f, 10.0f))));
    }

    {
        Fu_angleprocess ap;
        for (int index = 0; index < 3; ++index) {
            ap.angle_date.push_back(static_cast<float>(index));
            ap.angle_time.push_back(static_cast<double>(index));
        }
        require(ap.angle_diff_fun(0.01, 120.0) == -50.0);
    }

    // Public legacy buffers must be repaired to a paired, bounded window
    // before they are consumed by angle_diff_fun.
    {
        Fu_angleprocess ap;
        for (int index = 0; index < 305; ++index) {
            ap.angle_date.push_back(static_cast<float>(index));
        }
        for (int index = 0; index < 300; ++index) {
            ap.angle_time.push_back(static_cast<double>(index));
        }
        const double result = ap.pre_predict_angle_process(
            cv::Point2f(320.0f, 240.0f), cv::Point2f(420.0f, 240.0f),
            1000.0, 0.10, 120.0);
        (void)result;
        require(ap.angle_date.size() == ap.angle_time.size());
        require(ap.angle_date.size() <= 300);
    }
    // --- angle2kf: counterclockwise (正转) continuity across the 2*pi wrap ---
    {
        Fu_angleprocess ap;
        ap.anticlockwise = true;
        ap.anticlockwise_flag = true;
        require(std::abs(ap.angle2kf(0.05f) - 0.05f) < 1e-4f);
        require(std::abs(ap.angle2kf(0.15f) - 0.15f) < 1e-4f);
        require(std::abs(ap.angle2kf(0.5f) - 0.5f) < 1e-4f);
        require(std::abs(ap.angle2kf(1.0f) - 1.0f) < 1e-4f);
        require(std::abs(ap.angle2kf(3.0f) - 3.0f) < 1e-4f);
        require(std::abs(ap.angle2kf(6.2f) - 6.2f) < 1e-4f);
        // After the 2*pi wrap the accumulated angle keeps increasing.
        require(std::abs(ap.angle2kf(0.05f) - (0.05f + kTwoPiF)) < 1e-4f);
        require(std::abs(ap.angle2kf(0.15f) - (0.15f + kTwoPiF)) < 1e-4f);
        require(std::abs(ap.angle2kf(1.0f) - (1.0f + kTwoPiF)) < 1e-4f);
    }

    // --- angle2kf: clockwise (反转) decreasing accumulation ---
    {
        Fu_angleprocess ap;
        ap.anticlockwise = false;
        ap.anticlockwise_flag = true;
        require(std::abs(ap.angle2kf(6.2f) - (6.2f - kTwoPiF)) < 1e-4f);
        require(std::abs(ap.angle2kf(5.0f) - (5.0f - kTwoPiF)) < 1e-4f);
        require(std::abs(ap.angle2kf(0.05f) - (0.05f - kTwoPiF)) < 1e-4f);
        // Wrap back around: one more revolution subtracted.
        require(std::abs(ap.angle2kf(6.2f) - (6.2f - 2.0f * kTwoPiF)) < 1e-4f);
    }

    // --- pre_predict_angle_process end-to-end: diff ~= omega * delta_t ---
    {
        Fu_angleprocess ap;
        const double cx = 640.0;
        const double cy = 512.0;
        const double radius = 100.0;
        const double omega = 1.9;
        const double fps = 50.0;
        const double delta_t = 0.15;
        double sum = 0.0;
        int count = 0;
        for (int k = 0; k < 60; ++k) {
            const double time_ms = k * (1000.0 / fps);
            const double theta = omega * time_ms * 0.001;
            const cv::Point2f center(
                static_cast<float>(cx), static_cast<float>(cy));
            const cv::Point2f armor(
                static_cast<float>(cx + radius * std::cos(theta)),
                static_cast<float>(cy + radius * std::sin(theta)));
            const double diff = ap.pre_predict_angle_process(
                center, armor, time_ms, delta_t, fps);
            // Collect after turn() has locked the direction (21 samples).
            if (k >= 30 && diff != -1.0) {
                sum += diff;
                ++count;
            }
        }
        require(count >= 20);
        const double mean_diff = sum / count;
        // Expected omega * delta_t = 1.9 * 0.15 = 0.285 rad (positive).
        require(mean_diff > 0.15 && mean_diff < 0.45);
    }

    return 0;
}
