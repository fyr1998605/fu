// 能量机关参数加载器单元测试 (纯 C++, 只依赖 yaml-cpp)
//
// 覆盖:
//   1. 文件不存在 / 路径为空  -> 返回 false 且 error 非空
//   2. 没有 Buff: 节          -> 全部取默认值 + 警告
//   3. 显式键                  -> 正确解析
//   4. 顶层旧扁平键            -> 过渡兼容 + deprecated 警告
//   5. 越界值                  -> 夹取到边界并记录警告
//   6. 非法类型                -> 回落默认值并记录警告
//   7. 默认值 == 迁移前的硬编码值 (行为不变的护栏)

#include "buff_config.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

const char* kTempPath = "buff_config_test.yaml";

void writeFile(const std::string& text) {
    std::ofstream out(kTempPath, std::ios::trunc);
    assert(out.is_open());
    out << text;
    out.close();
}

bool contains(const std::vector<std::string>& lines, const std::string& needle) {
    for (const std::string& line : lines) {
        if (line.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool nearlyEqual(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-9;
}

}  // namespace

int main() {
    // ---------------- 7. 默认值护栏 ----------------
    // 这些数值必须与迁移前的硬编码值完全一致, 否则默认行为会变。
    {
        const buff::BuffConfig defaults;
        assert(defaults.sync.imu_pose_history_capacity == 200);
        assert(nearlyEqual(defaults.sync.max_imu_image_skew_seconds, 0.05));
        assert(nearlyEqual(defaults.sync.imu_stamp_offset_seconds, 0.0));
        assert(nearlyEqual(defaults.sync.quaternion_norm_min, 0.9));
        assert(nearlyEqual(defaults.sync.quaternion_norm_max, 1.1));

        assert(nearlyEqual(defaults.timing.delta_t_init, 0.3));            // Fu.cpp
        assert(nearlyEqual(defaults.timing.delta_t_compensation, 0.15));   // Fu.cpp
        assert(nearlyEqual(defaults.timing.delta_t_min, 0.01));
        assert(nearlyEqual(defaults.timing.delta_t_max, 1.0));
        assert(defaults.timing.flight_time_iterations == 2);
        assert(nearlyEqual(defaults.timing.flight_time_valid_max, 2.0));
        assert(nearlyEqual(defaults.timing.lead_ema_old, 0.8));
        assert(nearlyEqual(defaults.timing.lead_ema_new, 0.2));
        assert(nearlyEqual(defaults.timing.fallback_fps, 120.0));
        assert(defaults.timing.angle_cache_len == 300);
        assert(nearlyEqual(defaults.timing.default_omega, 1.047));
        assert(defaults.timing.kalman_bad_frames == 20);
        assert(nearlyEqual(defaults.timing.divergence_radius_ratio_min, 0.3));
        assert(nearlyEqual(defaults.timing.divergence_radius_ratio_max, 3.5));

        assert(nearlyEqual(defaults.control.pitch_compensation, 0.0));
        assert(nearlyEqual(defaults.control.yaw_compensation, 0.0));
        assert(nearlyEqual(defaults.control.gimbal_pose_alpha, 0.30));
        assert(nearlyEqual(defaults.control.pitch_max_deg, 22.0));
        assert(nearlyEqual(defaults.control.pitch_min_deg, -15.0));
        assert(nearlyEqual(defaults.control.stuck_threshold_deg, 19.0));
        assert(defaults.control.stuck_frames == 30);
        assert(nearlyEqual(defaults.control.recovery_pitch_deg, -10.0));
        assert(defaults.control.recovery_frames == 15);

        assert(nearlyEqual(defaults.ballistic.default_velocity, 24.0));
        assert(nearlyEqual(defaults.ballistic.velocity_min_valid, 10.0));
        assert(nearlyEqual(defaults.ballistic.gravity, 9.8));
        assert(defaults.ballistic.iteration_times == 30);
        assert(nearlyEqual(defaults.ballistic.converge_epsilon, 0.0001));
        assert(nearlyEqual(defaults.ballistic.height_tolerance, 0.01));
        assert(nearlyEqual(defaults.ballistic.angle_limit_deg, 29.7));
        assert(nearlyEqual(defaults.ballistic.phase_probe_rad, 0.30));
        assert(nearlyEqual(defaults.ballistic.probe_valid_rad, 0.015));
        assert(nearlyEqual(defaults.ballistic.physical_phase_limit_rad, 2.5));
        assert(nearlyEqual(defaults.ballistic.max_reprojection_error_px, 12.0));
        assert(defaults.ballistic.reprojection_check_enabled == false);
        assert(nearlyEqual(defaults.ballistic.optical_center_offset_x, -23.0));
        assert(nearlyEqual(defaults.ballistic.optical_center_offset_y, 24.0));

        assert(defaults.model.move_mode == "small");
        assert(defaults.model.run_without_serial == false);
        assert(defaults.big_ransac.enabled == true);
        assert(nearlyEqual(defaults.recognition.conf_threshold, 0.2));
        assert(nearlyEqual(defaults.recognition.nms_threshold, 0.2));
        assert(defaults.recognition.binary_threshold == 120);
        assert(defaults.recognition.dilate_kernel_size == 5);
        assert(nearlyEqual(defaults.recognition.r_center_mask_ratio, 0.8));
        assert(nearlyEqual(defaults.recognition.r_center_contour_div, 3.0));
        assert(nearlyEqual(defaults.recognition.r_center_refine_max_shift, 5.0));
        assert(nearlyEqual(defaults.recognition.box_alpha, 0.3));
        assert(nearlyEqual(defaults.recognition.stabilizer_alpha, 0.9));
        assert(nearlyEqual(defaults.recognition.jump_fan_radius_ratio, 0.55));
        assert(nearlyEqual(defaults.recognition.jump_r_radius_ratio, 0.35));
        assert(nearlyEqual(defaults.recognition.radius_change_reject_ratio, 0.50));
        assert(nearlyEqual(defaults.recognition.side_change_reject_ratio, 0.50));
        assert(nearlyEqual(defaults.recognition.ego_motion_max_delta_rad, 0.262));
        assert(nearlyEqual(defaults.geometry.target_width_mm, 300.0));
        assert(nearlyEqual(defaults.geometry.target_height_mm, 300.0));
        assert(defaults.geometry.camera_matrix.size() == 9);
        assert(defaults.geometry.dist_coeffs.size() == 5);
        assert(nearlyEqual(defaults.geometry.radius_mm, 0.0));  // 代码默认: 让 EKF 自学习
        assert(nearlyEqual(defaults.geometry.hub_normal_offset_mm, 0.0));
        assert(defaults.angle.blade_count == 5);
        assert(defaults.angle.direction_judge_samples == 21);
        assert(nearlyEqual(defaults.angle.direction_filter_ratio, 0.028));
        assert(defaults.angle.angle_buffer_len == 300);
        assert(nearlyEqual(defaults.angle.small_buff_max_roll_residual, 0.55));
        assert(nearlyEqual(defaults.legacy_predictor.big_const_omega, 1.924));
        assert(defaults.legacy_predictor.big_min_judge_samples == 21);
        assert(nearlyEqual(defaults.legacy_predictor.big_smooth_lookback_seconds, 0.10));
        assert(defaults.legacy_predictor.mode_detector_window_samples == 60);
        assert(defaults.legacy_predictor.mode_detector_minimum_samples == 12);
        assert(defaults.legacy_predictor.mode_detector_evaluate_every == 10);
        assert(defaults.legacy_predictor.mode_detector_confirmations == 3);
        assert(nearlyEqual(defaults.legacy_predictor.mode_detector_max_sample_jump_ratio, 0.014));
        assert(nearlyEqual(defaults.legacy_predictor.mode_detector_big_threshold_ratio, 0.60));
        assert(nearlyEqual(defaults.legacy_predictor.mode_detector_small_threshold_ratio, 0.40));
        assert(nearlyEqual(defaults.legacy_predictor.small_default_omega,
                           3.14159265358979323846 / 3.0));
        assert(defaults.world_tracker.ekf_enabled == false);
        assert(defaults.world_tracker.ekf_control_enabled == false);
        assert(defaults.world_tracker.apply_gimbal_pose == true);
        assert(nearlyEqual(defaults.world_tracker.center_measurement_std_m, 0.015));
        assert(nearlyEqual(defaults.world_tracker.direction_lock_omega, 0.25));
        assert(defaults.world_tracker.ready_enter_frames == 10);
        assert(defaults.world_tracker.ready_exit_frames == 15);
        assert(nearlyEqual(defaults.world_tracker.residual_gate_sigma, 3.5));
        assert(nearlyEqual(defaults.world_tracker.phase_process_noise_floor, 1.0e-5));
        assert(nearlyEqual(defaults.world_tracker.omega_init_abs, 0.9));
        assert(nearlyEqual(defaults.world_tracker.omega_ema_fast, 0.5));
        assert(nearlyEqual(defaults.world_tracker.omega_ema_slow, 0.03));
        assert(nearlyEqual(defaults.world_tracker.min_radius_m, 0.05));
        assert(nearlyEqual(defaults.world_tracker.max_dt_seconds, 1.0));
        assert(defaults.big_ransac.max_iterations == 200);
        assert(nearlyEqual(defaults.big_ransac.threshold, 0.5));
        assert(defaults.big_ransac.min_inliers == 100);
        assert(defaults.big_ransac.random_seed == 0x5EEDu);
    }

    // ---------------- 1. 路径为空 / 文件不存在 ----------------
    {
        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;

        assert(!buff::loadBuffConfig("", config, warnings, error));
        assert(!error.empty());

        assert(!buff::loadBuffConfig("this_file_does_not_exist_12345.yaml",
                                     config, warnings, error));
        assert(!error.empty());
    }

    // ---------------- 2. 没有 Buff: 节 -> 全部默认 ----------------
    {
        writeFile("---\nbuff_target_width_mm: 300.0\n");
        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;

        assert(buff::loadBuffConfig(kTempPath, config, warnings, error));
        assert(error.empty());
        assert(contains(warnings, "no 'Buff:' section"));
        assert(nearlyEqual(config.timing.delta_t_compensation, 0.15));
        assert(nearlyEqual(config.control.pitch_max_deg, 22.0));
        assert(nearlyEqual(config.ballistic.default_velocity, 24.0));
    }

    // ---------------- 3. 显式键解析 ----------------
    {
        writeFile(
            "Buff:\n"
            "  sync:\n"
            "    imu_pose_history_capacity: 400\n"
            "    max_imu_image_skew_seconds: 0.02\n"
            "  timing:\n"
            "    delta_t_init: 0.25\n"
            "    delta_t_compensation: 0.22\n"
            "    kalman_bad_frames: 12\n"
            "    default_omega: 1.2\n"
            "  control:\n"
            "    pitch_max_deg: 30.0\n"
            "    pitch_min_deg: -20.0\n"
            "    recovery_frames: 8\n"
            "  ballistic:\n"
            "    gravity: 9.81\n"
            "    default_velocity: 21.0\n"
            "    reprojection_check_enabled: true\n"
            "  model:\n"
            "    move_mode: auto\n"
            "  recognition:\n"
            "    conf_threshold: 0.35\n"
            "    binary_threshold: 135\n"
            "  geometry:\n"
            "    target_width_mm: 320.0\n"
            "    radius_mm: 700.0\n"
            "    camera_matrix: [1, 2, 3, 4, 5, 6, 7, 8, 9]\n"
            "    dist_coeffs: [1, 2, 3, 4, 5]\n"
            "  world_tracker:\n"
            "    ekf_enabled: true\n"
            "    residual_gate_sigma: 4.0\n"
            "    ready_enter_frames: 7\n"
            "  big_ransac:\n"
            "    enabled: false\n"
            "    max_iterations: 320\n"
            "    random_seed: 1234\n");

        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;

        assert(buff::loadBuffConfig(kTempPath, config, warnings, error));
        assert(warnings.empty());
        assert(config.sync.imu_pose_history_capacity == 400);
        assert(nearlyEqual(config.sync.max_imu_image_skew_seconds, 0.02));
        assert(nearlyEqual(config.timing.delta_t_init, 0.25));
        assert(nearlyEqual(config.timing.delta_t_compensation, 0.22));
        assert(config.timing.kalman_bad_frames == 12);
        assert(nearlyEqual(config.timing.default_omega, 1.2));
        assert(nearlyEqual(config.control.pitch_max_deg, 30.0));
        assert(nearlyEqual(config.control.pitch_min_deg, -20.0));
        assert(config.control.recovery_frames == 8);
        assert(nearlyEqual(config.ballistic.gravity, 9.81));
        assert(nearlyEqual(config.ballistic.default_velocity, 21.0));
        assert(config.ballistic.reprojection_check_enabled == true);
        assert(config.model.move_mode == "auto");
        assert(config.big_ransac.enabled == false);
        assert(nearlyEqual(config.recognition.conf_threshold, 0.35));
        assert(config.recognition.binary_threshold == 135);
        assert(nearlyEqual(config.geometry.target_width_mm, 320.0));
        assert(nearlyEqual(config.geometry.radius_mm, 700.0));
        assert(config.geometry.camera_matrix[8] == 9.0);
        assert(config.world_tracker.ekf_enabled == true);
        assert(nearlyEqual(config.world_tracker.residual_gate_sigma, 4.0));
        assert(config.world_tracker.ready_enter_frames == 7);
        assert(config.big_ransac.max_iterations == 320);
        assert(config.big_ransac.random_seed == 1234u);
        // 未指定的键仍取默认值
        assert(nearlyEqual(config.timing.fallback_fps, 120.0));
        assert(config.ballistic.iteration_times == 30);
    }

    // ---------------- 4. 顶层旧扁平键 -> 过渡兼容 ----------------
    {
        writeFile("---\nbuff_imu_stamp_offset_seconds: 0.03\n");
        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;

        assert(buff::loadBuffConfig(kTempPath, config, warnings, error));
        assert(nearlyEqual(config.sync.imu_stamp_offset_seconds, 0.03));
        assert(contains(warnings, "deprecated top-level key"));
    }

    // ---------------- 5. 越界值 -> 夹取 ----------------
    {
        writeFile(
            "Buff:\n"
            "  timing:\n"
            "    lead_ema_old: 1.7\n"
            "    angle_cache_len: 0\n"
            "    kalman_bad_frames: 0\n"
            "  control:\n"
            "    pitch_min_deg: 40.0\n"
            "  ballistic:\n"
            "    angle_limit_deg: 200.0\n"
            "    gravity: -5.0\n");

        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;

        assert(buff::loadBuffConfig(kTempPath, config, warnings, error));
        assert(nearlyEqual(config.timing.lead_ema_old, 1.0));
        assert(config.timing.angle_cache_len == 1);
        assert(config.timing.kalman_bad_frames == 1);
        // pitch_min >= pitch_max -> 恢复默认对
        assert(nearlyEqual(config.control.pitch_min_deg, -15.0));
        assert(nearlyEqual(config.control.pitch_max_deg, 22.0));
        assert(nearlyEqual(config.ballistic.angle_limit_deg, 89.0));
        assert(nearlyEqual(config.ballistic.gravity, 9.8));  // 负重力回落默认值
        assert(contains(warnings, "clamped to"));
        assert(contains(warnings, "must be > 0"));
        assert(contains(warnings, "restoring defaults"));
    }

    // ---------------- 6. 非法类型 -> 回落默认值 ----------------
    {
        writeFile(
            "Buff:\n"
            "  ballistic:\n"
            "    gravity: \"not_a_number\"\n"
            "  timing:\n"
            "    delta_t_compensation: [1, 2, 3]\n");

        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;

        assert(buff::loadBuffConfig(kTempPath, config, warnings, error));
        assert(nearlyEqual(config.ballistic.gravity, 9.8));
        assert(nearlyEqual(config.timing.delta_t_compensation, 0.15));
        assert(contains(warnings, "invalid value"));
    }

    // ---------------- 启动打印 ----------------
    {
        writeFile(
            "Buff:\n"
            "  control:\n"
            "    pitch_max_deg: 25.0\n");
        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;
        assert(buff::loadBuffConfig(kTempPath, config, warnings, error));

        const std::vector<std::string> lines = buff::buffConfigToLines(config);
        assert(!lines.empty());
        assert(contains(lines, "Buff.control.pitch_max_deg = 25"));
        assert(contains(lines, "Buff.timing.delta_t_compensation = 0.15"));
        assert(contains(lines, "Buff.ballistic.default_velocity = 24"));
        assert(contains(lines, "Buff.model.move_mode = small"));
        assert(contains(lines, "Buff.recognition.conf_threshold = 0.2"));
        assert(contains(lines, "Buff.geometry.target_width_mm = 300"));
        assert(contains(lines, "Buff.world_tracker.ekf_enabled = false"));
        assert(contains(lines, "Buff.big_ransac.max_iterations = 200"));
    }

    std::remove(kTempPath);

    // ---------------- 真实配置文件: 必须零警告 ----------------
    // 仓库里的 rm_fu.yaml 是调参的唯一入口。它必须与校验规则完全一致:
    // 任何超范围/拼错类型的值都会让 loadBuffConfig 产生警告, 这里直接判失败,
    // 避免"改了 yaml 但启动时被静默回落"的情况被带到车上。
    // 路径由 CMake 的 BUFF_FU_CONFIG_PATH 宏注入 (手动编译时该测试自动跳过)。
#ifdef BUFF_FU_CONFIG_PATH
    {
        buff::BuffConfig config;
        std::vector<std::string> warnings;
        std::string error;

        const bool loaded =
            buff::loadBuffConfig(BUFF_FU_CONFIG_PATH, config, warnings, error);
        assert(loaded);
        assert(error.empty());
        if (!loaded) {
            return 1;
        }
        for (const std::string& warning : warnings) {
            std::fprintf(stderr, "unexpected config warning: %s\n", warning.c_str());
        }
        assert(warnings.empty());

        // 过渡期重复键: Buff.sync.imu_stamp_offset_seconds 必须与顶层旧键一致,
        // 否则两处会悄悄漂移 (P3 删除旧键后该检查随之失效)。
        const YAML::Node legacy = config.raw["buff_imu_stamp_offset_seconds"];
        if (legacy && !legacy.IsNull()) {
            assert(nearlyEqual(config.sync.imu_stamp_offset_seconds,
                               legacy.as<double>()));
        }

        // 打印必须覆盖到真实文件里的每一节
        const std::vector<std::string> lines = buff::buffConfigToLines(config);
        assert(contains(lines, "Buff.sync.imu_pose_history_capacity"));
        assert(contains(lines, "Buff.timing.delta_t_compensation"));
        assert(contains(lines, "Buff.control.pitch_max_deg"));
        assert(contains(lines, "Buff.ballistic.angle_limit_deg"));
    }
#endif

    return 0;
}
