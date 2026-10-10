#include "buff_big_ransac_predictor.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

namespace {

void require(const bool condition) {
    if (!condition) {
        std::abort();
    }
}

}  // namespace

int main() {
    require(buff_zju::is_big_buff_ransac_active(true, "big"));
    require(!buff_zju::is_big_buff_ransac_active(false, "big"));
    require(!buff_zju::is_big_buff_ransac_active(true, "small"));
    require(std::abs(buff_zju::BigBuffRansacConfig{}.max_abs_speed - 3.2) < 1e-12);

    buff_zju::BigBuffRansacConfig config;
    config.min_inliers = 20;
    config.max_abs_speed = 3.0;
    config.max_gap_seconds = 5.0;
    config.ransac.max_iterations = 300;
    config.ransac.threshold = 0.10;
    config.ransac.min_omega = 1.80;
    config.ransac.max_omega = 2.05;
    config.ransac.min_amplitude = 0.70;
    config.ransac.max_amplitude = 1.10;

    buff_zju::BigBuffRansacPredictor predictor(config);
    require(predictor.update(0.0, 0.30));
    require(!predictor.ready());
    require(!predictor.update(0.0, 0.31));
    require(!predictor.update(0.01, 0.34));
    predictor.reset();
    require(predictor.update(0.0, 0.30));

    constexpr double amplitude = 0.90;
    constexpr double omega = 1.94;
    constexpr double phase = 0.35;
    constexpr double offset = 0.02;
    double roll = 0.30;
    for (int index = 1; index <= 80; ++index) {
        const double timestamp = index * 0.02;
        const double velocity = amplitude * std::sin(omega * timestamp + phase) + offset;
        roll += velocity * 0.02;
        require(predictor.update(timestamp, roll));
    }

    require(predictor.ready());
    const auto state = predictor.state();
    const double horizon = 0.20;
    const double expected = state.roll + state.offset * horizon +
        state.amplitude *
            (std::cos(state.phase) -
             std::cos(state.phase + state.omega * horizon)) /
            state.omega;
    constexpr double pi = 3.14159265358979323846;
    const double predicted_roll = predictor.predict_roll(horizon);
    require(std::abs(predicted_roll - std::remainder(expected, 2.0 * pi)) < 1e-6);

    const double radius = 120.0;
    const double point_x = 640.0 + radius * std::cos(predicted_roll);
    const double point_y = 512.0 + radius * std::sin(predicted_roll);
    require(std::isfinite(point_x) && std::isfinite(point_y));

    require(predictor.update(7.0, roll + 1.0));
    require(!predictor.ready());
    require(std::isfinite(predictor.predict_roll(0.10)));

    // The runtime supplies the measured polar angle, which wraps at 2*pi.
    predictor.reset();
    require(predictor.update_polar(0.0, 6.20));
    require(predictor.update_polar(0.05, 0.02));
    require(std::abs(predictor.state().roll - (6.20 + 0.103185307179586)) < 1e-3);

    // A physically impossible jump is rejected, while the next valid sample
    // can establish a new baseline for the legacy fallback path.
    predictor.reset();
    require(predictor.update_polar(0.0, 0.0));
    const double baseline_roll = predictor.state().roll;
    require(!predictor.update_polar(0.02, 3.0));
    require(!predictor.ready());
    require(std::abs(predictor.state().roll - baseline_roll) < 1e-12);
    require(predictor.update_polar(0.04, 0.02));
    require(std::abs(predictor.state().roll - 0.02) < 1e-12);

    predictor.reset();
    require(predictor.update_polar(0.0, 6.20));
    require(predictor.update_polar(6.0, 0.10));
    const double roll_after_gap = predictor.state().roll;
    require(predictor.update_polar(6.02, 0.12));
    require(std::abs(predictor.state().roll - roll_after_gap - 0.02) < 1e-9);

    // An externally locked direction rejects the opposite unwrap candidate.
    predictor.reset();
    predictor.set_rotation_sign(-1);
    require(predictor.update_polar(0.0, 1.0));
    require(!predictor.update_polar(0.05, 1.10));
    predictor.set_rotation_sign(1);
    require(predictor.update_polar(0.05, 1.10));

    buff_zju::BigBuffRansacConfig rejection_config = config;
    rejection_config.max_abs_speed = 3.2;
    rejection_config.min_inliers = 20;
    buff_zju::BigBuffRansacPredictor rejection_predictor(rejection_config);
    double rejection_roll = 0.30;
    for (int index = 0; index <= 80; ++index) {
        const double timestamp = index * 0.02;
        const double velocity = amplitude * std::sin(omega * timestamp + phase) + offset;
        if (index > 0) {
            rejection_roll += velocity * 0.02;
        }
        require(rejection_predictor.update(timestamp, rejection_roll));
    }
    require(rejection_predictor.ready());
    const double roll_before_rejection = rejection_roll;
    require(!rejection_predictor.update(1.62, roll_before_rejection + 0.10));
    const double v162 = amplitude * std::sin(omega * 1.62 + phase) + offset;
    const double v164 = amplitude * std::sin(omega * 1.64 + phase) + offset;
    rejection_roll += v162 * 0.02 + v164 * 0.02;
    require(rejection_predictor.update(1.64, rejection_roll));
    require(rejection_predictor.ready());

    predictor.reset();
    require(predictor.update_polar(0.0, 1.0));
    const double timestamp_baseline_roll = predictor.state().roll;
    require(!predictor.update_polar(-0.01, 1.01));
    require(std::abs(predictor.state().roll - timestamp_baseline_roll) < 1e-12);
    require(predictor.update_polar(0.01, 1.02));
    require(std::abs(predictor.state().roll - 1.02) < 1e-12);

    // Re-fitting must not keep a previously ready sine model alive after the
    // sliding window has been replaced by incompatible motion.
    buff_zju::BigBuffRansacConfig stale_config = config;
    stale_config.ransac.max_iterations = 20;
    stale_config.min_inliers = 20;
    buff_zju::BigBuffRansacPredictor stale_predictor(stale_config);
    double stale_roll = 0.30;
    for (int index = 0; index <= 80; ++index) {
        const double timestamp = index * 0.02;
        const double velocity = amplitude * std::sin(omega * timestamp + phase) + offset;
        if (index > 0) {
            stale_roll += velocity * 0.02;
        }
        require(stale_predictor.update(timestamp, stale_roll));
    }
    require(stale_predictor.ready());
    for (int index = 1; index <= 360; ++index) {
        stale_roll += 0.0;
        require(stale_predictor.update(1.60 + index * 0.02, stale_roll));
    }
    require(!stale_predictor.ready());

    predictor.reset();
    require(!std::isfinite(predictor.predict_roll(0.10)));

    // Direction-aware unwrap: after a 2 s detection gap the wheel has turned
    // ~3.8 rad; the shortest-arc remainder would read it as -2.48 rad and the
    // sample would be accepted with an inverted velocity. The unwrap must
    // pick the wrap candidate that continues the previous velocity.
    predictor.reset();
    constexpr double two_pi_test = 2.0 * 3.14159265358979323846;
    const double mean_v = 1.94;
    const double amp = 0.80;
    const double w = 1.94;
    const double ph = 0.35;
    double acc = 0.05;
    require(predictor.update_polar(0.0, std::fmod(acc, two_pi_test)));
    for (int idx = 1; idx <= 99; ++idx) {
        const double t = idx * 0.02;
        acc += (mean_v + amp * std::sin(w * t + ph)) * 0.02;
        require(predictor.update_polar(t, std::fmod(acc, two_pi_test)));
    }
    require(predictor.state().roll_velocity > 1.0);
    const double roll_before = predictor.state().roll;
    acc += 3.8;  // 2 s gap at ~1.9 rad/s
    require(predictor.update_polar(3.98, std::fmod(acc, two_pi_test)));
    require(std::abs(predictor.state().roll - roll_before - 3.8) < 1e-6);
    require(predictor.state().roll_velocity > 0.0);

    // A five-blade ID switch is an equivalent 2*pi/5 observation and should
    // be corrected immediately instead of poisoning the velocity sample.
    buff_zju::BigBuffRansacConfig jump_config = config;
    jump_config.max_consecutive_rejections = 5;
    buff_zju::BigBuffRansacPredictor jump_predictor(jump_config);
    double jump_acc = 0.05;
    require(jump_predictor.update_polar(0.0, jump_acc));
    for (int idx = 1; idx <= 30; ++idx) {
        jump_acc += 1.94 / 60.0;
        require(jump_predictor.update_polar(
            idx / 60.0, std::fmod(jump_acc, two_pi_test)));
    }
    const double jump_roll_before = jump_predictor.state().roll;
    jump_acc += 1.257;  // one-time blade/ID switch: constant 72-deg offset
    jump_acc += 1.94 / 60.0;
    require(jump_predictor.update_polar(
        31.0 / 60.0, std::fmod(jump_acc, two_pi_test)));
    require(jump_predictor.state().roll > jump_roll_before);

    // A genuinely impossible jump must still be rejected.
    require(!jump_predictor.update_polar(
        32.0 / 60.0, std::fmod(jump_acc + 3.0, two_pi_test)));

    // Param sanitization: non-finite / out-of-range values fall back to the
    // BigBuffRansacConfig defaults and the predictor stays usable.
    buff_zju::BigBuffRansacConfig bad_config = config;
    bad_config.ransac.min_omega = std::numeric_limits<double>::quiet_NaN();
    bad_config.ransac.max_amplitude = 0.1;  // below min_amplitude 0.70
    bad_config.max_gap_seconds = -3.0;
    bad_config.max_consecutive_rejections = 0;
    const buff_zju::BigBuffRansacConfig clean =
        buff_zju::sanitize_big_buff_ransac_config(bad_config);
    require(clean != bad_config);
    require(clean.ransac.min_omega == 1.884);  // default, not the test's 1.80
    require(clean.ransac.max_amplitude >= clean.ransac.min_amplitude);
    require(clean.max_gap_seconds > 0.0);
    require(clean.max_consecutive_rejections >= 1);
    require(buff_zju::sanitize_big_buff_ransac_config(config) == config);

    buff_zju::BigBuffRansacConfig boundary_config = config;
    boundary_config.ransac.max_iterations = 5000;
    boundary_config.ransac.threshold = 0.0;
    boundary_config.ransac.min_omega = 0.0;
    boundary_config.ransac.max_omega = 0.0;
    boundary_config.min_inliers = 1000;
    boundary_config.max_consecutive_rejections = 1000;
    const auto bounded =
        buff_zju::sanitize_big_buff_ransac_config(boundary_config);
    require(bounded.ransac.max_iterations <= 1000);
    require(bounded.ransac.threshold > 0.0);
    require(bounded.ransac.min_omega > 0.0);
    require(bounded.ransac.max_omega >= bounded.ransac.min_omega);
    require(bounded.min_inliers >= 3 && bounded.min_inliers <= 300);
    require(bounded.max_consecutive_rejections <= 300);
    boundary_config.max_abs_speed = 1000.0;
    boundary_config.max_gap_seconds = 1000.0;
    const auto bounded_runtime =
        buff_zju::sanitize_big_buff_ransac_config(boundary_config);
    require(bounded_runtime.max_abs_speed <= 20.0);
    require(bounded_runtime.max_gap_seconds <= 30.0);
    require(bounded_runtime.fit_every_n_samples <= 100);
    buff_zju::BigBuffRansacPredictor bad_predictor(bad_config);
    for (int idx = 0; idx < 5; ++idx) {
        const double t = 0.02 * idx;
        require(bad_predictor.update_polar(
            t, std::fmod(1.94 * t, two_pi_test)));
    }
    require(!bad_predictor.ready());
    require(std::isfinite(bad_predictor.state().roll));

    // The large-buff model should retain the previous fit between scheduled
    // refits, then become ready only after a fresh fit reaches its inlier gate.
    buff_zju::BigBuffRansacConfig cadence_config = config;
    cadence_config.fit_every_n_samples = 5;
    cadence_config.min_inliers = 7;
    buff_zju::BigBuffRansacPredictor cadence_predictor(cadence_config);
    double cadence_roll = 0.30;
    require(cadence_predictor.update(0.0, cadence_roll));
    for (int index = 1; index <= 9; ++index) {
        const double timestamp = index * 0.02;
        const double velocity = amplitude *
            std::sin(omega * timestamp + phase) + offset;
        cadence_roll += velocity * 0.02;
        require(cadence_predictor.update(timestamp, cadence_roll));
        require(!cadence_predictor.ready());
    }
    const double tenth_timestamp = 0.20;
    cadence_roll += (amplitude *
        std::sin(omega * tenth_timestamp + phase) + offset) * 0.02;
    require(cadence_predictor.update(tenth_timestamp, cadence_roll));
    require(cadence_predictor.ready());
    require(cadence_predictor.state().inliers >= cadence_config.min_inliers);

    return 0;
}
