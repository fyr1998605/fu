#pragma once

// ============================================================================
//  能量机关 (Buff) 参数加载器 —— 参照自瞄(装甲板)的配置方式
//
//  自瞄的做法 (RmTracker::setParam / ModelObserver::setParam):
//    一个 YAML 文件 + 结构化分节 + 逐键 .as<T>(默认值) + 解析错误处理 + 启动打印
//
//  本文件与之保持一致, 唯一的差别是: 刻意不调用任何 ROS API,
//  警告以字符串返回, 由调用方 (Fu.cpp) 打印。
//  => 可以被纯 C++ 单测覆盖, 不需要 catkin / roscore 运行环境。
//
//  取值优先级 (高 -> 低):
//    1. launch <param>            (由 Fu.cpp 用 private_nh.hasParam() 判断, 本文件不涉及)
//    2. YAML  Buff.<section>.<key>
//    3. YAML  顶层旧扁平键         (过渡期兼容, 命中会记录 deprecated 警告)
//    4. 本文件的默认值             (== 迁移前的硬编码值, 保证行为不变)
//
//  用法:
//    buff::BuffConfig cfg;
//    std::vector<std::string> warnings;
//    std::string error;
//    if (!buff::loadBuffConfig(path, cfg, warnings, error)) { /* fatal */ }
//    for (const auto& line : buff::buffConfigToLines(cfg)) ROS_INFO("%s", line.c_str());
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace buff {

// ---------------------------------------------------------------------------
//  IMU / 图像 时间同步
// ---------------------------------------------------------------------------
struct SyncConfig {
    // 原 Fu.cpp constexpr kImuPoseHistoryCapacity
    std::size_t imu_pose_history_capacity = 200;
    // 原 Fu.cpp constexpr kMaximumImuImageSkewSeconds (图-IMU 最大时差, 秒)
    double max_imu_image_skew_seconds = 0.05;
    // 原 buff_imu_stamp_offset_seconds (rm_fu.yaml 顶层旧键) -> Buff.sync 节
    double imu_stamp_offset_seconds = 0.0;
    // 原 buff_node_runtime.hpp 四元数模长有效窗口
    double quaternion_norm_min = 0.9;
    double quaternion_norm_max = 1.1;
};

// ---------------------------------------------------------------------------
//  时延 / 提前量 / 兜底策略
// ---------------------------------------------------------------------------
struct TimingConfig {
    // 原 Fu.cpp: double delta_t_ = 0.3;
    double delta_t_init = 0.3;
    // 原 Fu.cpp: double delta_t_compensation = 0.15;  (程序耗时 + 发弹时间)
    double delta_t_compensation = 0.15;
    // 原 buff_node_runtime.hpp: clamp(delta_t_, 0.01, 1.0)
    double delta_t_min = 0.01;
    double delta_t_max = 1.0;
    // 原 buff_node_runtime.hpp: for (iteration = 0; iteration < 2; ++iteration)
    int flight_time_iterations = 2;
    // 原 buff_node_runtime.hpp:481-495 命中时间/首帧飞行时间有效区间
    double flight_time_valid_min = 0.01;
    double flight_time_valid_max = 2.0;
    // 原 buff_node_runtime.hpp:528 delta_t_ = 0.8*x + 0.2*y
    double lead_ema_old = 0.8;
    double lead_ema_new = 0.2;
    // 检测丢失时的外推续跟上限(帧): 连续丢失不超过此帧数时, 用上一帧几何 + 当前
    // 时间戳让相位跟踪器继续外推并下发, 云台不停转; 超过才回保底冻结。
    // 约 50fps 下一帧 20ms, 12 帧 ≈ 0.24s。0 = 关闭续跟(退回旧行为)。
    int max_coast_frames = 12;
    // 原 buff_node_runtime.hpp:318/326 兜底帧率 EMA
    double fallback_fps = 120.0;
    double fallback_fps_ema_old = 0.9;
    double fallback_fps_ema_new = 0.1;
    // 原 buff_node_runtime.hpp:325 帧间隔有效上限 (ms)
    double fps_interval_max_ms = 500.0;
    // 原 buff_node_runtime.hpp:341 角度缓存上限
    int angle_cache_len = 300;
    // 原 buff_node_runtime.hpp:456 最后兜底角速度 (60 deg/s)
    double default_omega = 1.047;
    // 原 buff_node_runtime.hpp:423 卡尔曼连续坏帧切兜底阈值
    int kalman_bad_frames = 20;
    // 原 buff_node_runtime.hpp:390-393 发散检测
    double divergence_min_radius_px = 1.0;
    double divergence_radius_ratio_min = 0.3;
    double divergence_radius_ratio_max = 3.5;
};

// ---------------------------------------------------------------------------
//  云台下发 / 限幅 / 卡死恢复
// ---------------------------------------------------------------------------
struct ControlConfig {
    // 原 Fu.cpp: double pitch_compensation = 0.0; / yaw_compensation = 0.0;  (rad)
    double pitch_compensation = 0.0;
    double yaw_compensation = 0.0;
    // 原 buff_pnp_solver.hpp:39 云台位姿 EMA α = 0.30
    double gimbal_pose_alpha = 0.30;
    // 原 buff_node_runtime.hpp:541-542 pitch 软限幅
    double pitch_max_deg = 22.0;
    double pitch_min_deg = -15.0;
    // 原 buff_node_runtime.hpp:551-566 卡死检测与恢复
    double stuck_threshold_deg = 19.0;
    int stuck_frames = 30;
    double recovery_pitch_deg = -10.0;
    int recovery_frames = 15;
    // 原 buff_node_runtime.hpp:577 ROS_INFO_THROTTLE
    double log_throttle_s = 0.2;
};

// ---------------------------------------------------------------------------
//  PnP / 弹道 / 相位
// ---------------------------------------------------------------------------
struct BallisticConfig {
    // 原 buff_pnp_solver.hpp:361 串口无有效弹速时的兜底弹速 (m/s)
    double default_velocity = 24.0;
    // 原 buff_pnp_solver.hpp:362 串口弹速有效门限
    double velocity_min_valid = 10.0;
    // 原 buff_pnp_solver.hpp:365
    double gravity = 9.8;
    // 原 buff_pnp_solver.hpp:360
    int iteration_times = 30;
    // 原 buff_pnp_solver.hpp:403 弹道迭代收敛阈值
    double converge_epsilon = 0.0001;
    // 原 buff_pnp_solver.hpp:407 高度差失败门限 (m)
    double height_tolerance = 0.01;
    // 原 buff_pnp_solver.hpp 中 4 处 M_PI*0.165 (补偿角/枪管角上限)
    double angle_limit_deg = 29.7;
    // 原 buff_pnp_solver.hpp:538 相位符号探针角 (rad)
    double phase_probe_rad = 0.30;
    // 原 buff_pnp_solver.hpp:570 探针信号有效性下限 (rad)
    double probe_valid_rad = 0.015;
    // 原 buff_pnp_solver.hpp:593 单帧物理相位上限 (rad)
    double physical_phase_limit_rad = 2.5;
    // 原 buff_pnp_solver.hpp:730 (校验当前被注释, 由下面的开关控制)
    double max_reprojection_error_px = 12.0;
    bool reprojection_check_enabled = false;
    // 原 buff_pnp_solver.hpp:322 主点人为偏移 (px)
    double optical_center_offset_x = -23.0;
    double optical_center_offset_y = 24.0;
};

// ---------------------------------------------------------------------------
//  模型 / 模式 / 推理
// ---------------------------------------------------------------------------
struct ModelConfig {
    // 原 launch <param> buff_model_path (默认 Model/fu_moxi/buff_repvgg.xml)
    std::string model_path = "Model/fu_moxi/buff_repvgg.xml";
    // 原 launch <param> enemy_color (默认 "blue")
    std::string enemy_color = "blue";
    // 原 buff_move_mode: small | big | auto
    std::string move_mode = "small";
    // 原 run_buff_without_serial (无串口也跑打符, 调试用)
    bool run_without_serial = false;
    // 原 Fu.cpp: bool functional = true (串口未下发 ShootFlag 前默认打符)
    bool functional_default = true;
    // 原 buff_recognizer.cpp fillTensor 输入尺寸 640
    int input_size = 640;
    // 原 buff_recognizer.cpp compile_model(..., "CPU")
    std::string model_device = "CPU";
};

// ---------------------------------------------------------------------------
//  识别 (BuffRecognizer / BuffDetectionStabilizer)
// ---------------------------------------------------------------------------
struct RecognitionConfig {
    // 原 buff_recognizer.hpp/cpp conf/nms 0.2 / 0.2
    double conf_threshold = 0.2;
    double nms_threshold = 0.2;
    // 原 buff_recognizer.hpp R中心精化
    int binary_threshold = 120;
    int dilate_kernel_size = 5;
    double r_center_mask_ratio = 0.8;
    double r_center_contour_div = 3.0;
    double r_center_refine_max_shift = 5.0;
    // 原 buff_recognizer.hpp 检测框 EMA 0.3 / 几何 EMA 0.9
    double box_alpha = 0.3;
    double stabilizer_alpha = 0.9;
    // 原 buff_detection_stabilizer.hpp 时序关联门限
    double assoc_gate_min_px = 20.0;
    double assoc_gate_radius_ratio = 1.2;
    // 原 buff_recognizer.cpp 跳变判定比例
    double jump_fan_radius_ratio = 0.55;
    double jump_r_radius_ratio = 0.35;
    double radius_change_reject_ratio = 0.50;
    double side_change_reject_ratio = 0.50;
    // 原 buff_detection_stabilizer.hpp 连续帧确认 (pending>=2 -> 共 3 帧)
    int jump_confirm_frames = 3;
    int reacquire_confirm_frames = 3;
    // 丢帧滞回: 连续多少帧空结果/关联失败才清 has_prev_。
    // 单帧 miss 不清 -> 下一帧检出可直接按原关联门限接上, 不必走 3 帧重捕获确认;
    // 清状态会把每次丢检的代价从 1 帧抬到 3 帧, 丢帧率高时会显著滑坡。
    int miss_reset_frames = 3;
    // 原 buff_recognizer.cpp 有效性保护阈值
    double min_valid_radius_px = 1.0;
    // 原 buff_recognizer.cpp 自运动补偿云台跳变上限 15°(0.262 rad)
    double ego_motion_max_delta_rad = 0.262;
};

// ---------------------------------------------------------------------------
//  靶面几何 / 相机标定
// ---------------------------------------------------------------------------
struct GeometryConfig {
    // 原 buff_target_width_mm / buff_target_height_mm
    double target_width_mm = 300.0;
    double target_height_mm = 300.0;
    // 原 buff_camera_matrix / buff_dist_coeffs
    std::vector<double> camera_matrix{
        1329.24429634650, 0.518156995670085, 661.300343956123,
        0.0, 1325.89618530323, 452.645884191168,
        0.0, 0.0, 1.0};
    std::vector<double> dist_coeffs{
        -0.0442540638030886, 0.0902922934048078,
        -0.00375815319429763, 0.00269367029458212, 0.0308390738152262};
    // 原 buff_radius_mm / buff_hub_normal_offset_mm (Fu.cpp 参数默认值 0.0 -> 让 EKF 自学习半径;
    // rm_fu.yaml 实际提供 650.0)
    double radius_mm = 0.0;
    double hub_normal_offset_mm = 0.0;
};

// ---------------------------------------------------------------------------
//  角度处理 (Fu_angleprocess) + 小符像素卡尔曼 (SmallBuffPhaseTracker)
// ---------------------------------------------------------------------------
struct AngleConfig {
    // ---- Fu_angleprocess (极坐标 / 差分 / 方向 / 五叶片跳变修正) ----
    // 原 2*j*PI/5 与 kBladePeriod = 2*PI/5
    int blade_count = 5;
    // 原 buff_angle_processor.hpp trun_count.size() == 21
    int direction_judge_samples = 21;
    // 原 6.283185 / -6.283185 圈数修正
    double wrap_period = 6.283185;
    // 原 2*PI/6.55 补角容差
    double unwrap_tolerance_divisor = 6.55;
    // 原 差分搜索起点 0.5*fps*delta_t
    double diff_search_start_ratio = 0.5;
    // 原 差分超时容差 +80ms
    double diff_timeout_margin_ms = 80.0;
    // 原 差分角异常门限 1.169 rad (67°)
    double diff_angle_limit_rad = 1.169;
    // 原 turn() 方向判定滤波: 相邻差分角 PI*0.028 (≈5°)
    double direction_filter_ratio = 0.028;
    // 原 angle_date/angle_time 缓存上限 300
    int angle_buffer_len = 300;
    // 原 补角搜索范围 ±6 (header 版; Fu_yuan 旧版为 ±5)
    int sector_search_range = 6;
    // 原 buff_angle_processor.hpp image_width_/image_height_ 默认
    int image_width = 1280;
    int image_height = 1024;

    // ---- SmallBuffPhaseTracker (blade_tracker.hpp, 小符像素卡尔曼) ----
    int small_buff_converge_delay = 5;           // kConvergeDelay
    double small_buff_max_roll_residual = 0.55;  // kMaxRollResidual
    double small_buff_min_speed = 0.052;         // kMinSmallBuffSpeed
    double small_buff_max_speed = 1.571;         // kMaxSmallBuffSpeed
    double small_buff_phase_process_noise = 0.005;      // kPhaseProcessNoise
    double small_buff_phase_measurement_noise = 0.004;  // kPhaseMeasurementNoise
    double small_buff_min_radius_px = 1.0;       // measured_radius < 1.0 门限
    double small_buff_max_dt = 0.5;              // dt > 0.5 门限
    int small_buff_reject_reset_count = 3;       // 连续拒绝 3 次 reset
    double small_buff_vote_min_phase_delta = 0.002;  // 投票生效最小相位差
    int small_buff_vote_saturation = 20;         // 方向投票饱和 ±20
    int small_buff_omega_fit_min_samples = 5;    // ω线性拟合最少样本
    double small_buff_radius_ema = 0.1;          // 半径 EMA 新值权重
    double small_buff_center_ema = 0.2;          // 圆心 EMA 新值权重
    // R 中心瞬跳吸附阈值(×半径): 观测到的 r_center 与跟踪器 center_ 相差超过此比例时
    // 直接吸附而不是走 EMA。R 中心跳变(重捕获/检出换叶)后 EMA 要好几帧才追上,
    // 这期间 predicted_center 会跑出圆外, 下游以 physical_phase_limit 拒绝。
    double small_buff_center_snap_ratio = 0.3;
    double small_buff_omega_reset_cov = 0.02;    // 重置后 ω 协方差
    double small_buff_omega_locked_cov = 0.05;   // 锁定后 ω 协方差
    double small_buff_init_cov_phase = 0.10;     // 初始相位协方差
    double small_buff_init_cov_omega = 0.50;     // 初始 ω 协方差
    int small_buff_phase_history_max = 20;       // kPhaseHistoryMaxSize
};

// ---------------------------------------------------------------------------
//  旧 FFT/Ceres 大符预测器 (buff_predicted) + 小符速度估计
// ---------------------------------------------------------------------------
struct LegacyPredictorConfig {
    // ---- 大符 FFT/Ceres ----
    int big_min_init_samples = 6;           // delta_angle_data.size() < 6
    double big_const_level = 1.1775;        // 大符基线均值 (×delta_t)
    double big_const_omega = 1.924;         // 大符角速度定值 rad/s
    double big_angular_speed = 2.090;       // 大符振幅公式中的 2.090
    int ceres_trigger_factor = 2;           // ceres_data_count = fps*PI*2
    double big_jump_ratio = 0.014;          // PI*0.014 突变差分角门限
    int big_moving_average_window = 7;      // moving_average(..., 7)
    int big_min_judge_samples = 21;         // 升降判定最少样本 (原 21)
    double big_smooth_lookback_seconds = 0.10;  // 极值附近平滑回看 (原 fps*0.10)
    double big_initial_phase = 0.75;        // phi 初相位估计
    int ceres_max_num_iterations = 500;
    double ceres_function_tolerance = 1.0e-15;
    double ceres_gradient_tolerance = 1.0e-15;
    double ceres_parameter_tolerance = 1.0e-15;
    int ceres_num_threads = 2;
    double big_fit_max_level = 1.310;       // 拟合最大基线 (×delta_t)
    double big_fit_max_omega = 2.000;       // 拟合最大角速度
    double big_fit_min_level = 1.045;       // 拟合最小基线 (×delta_t)
    double big_fit_min_omega = 1.884;       // 拟合最小角速度
    double big_forced_omega = 2.0;          // 原 omega = 2 强制覆盖
    double big_extreme_wait_seconds = 0.2;  // 极值后冻结时长 (fps*0.2)
    int big_vote_lookback_near = 5;         // 上升/下降表决回看 -5
    int big_vote_lookback_mid = 10;         // -10
    int big_vote_lookback_far = 15;         // -15
    int big_smooth_window = 5;              // 0.10s×5 点平滑

    // ---- 小符速度估计 (buff_predicted 内) ----
    int small_min_init_samples = 10;        // delta_angle_data.size() >= 10
    double small_jump_ratio = 0.028;        // PI*0.028 突变门限 (5.04°)
    int small_init_timeout_frames = 60;     // 初始化超时 60 帧
    // 原 PI/3 * delta_t -> 用 π/3 精确值, 避免 1.047 引入的 0.02% 偏差
    double small_default_omega = 3.14159265358979323846 / 3.0;  // 保底 60°/s
    int small_ema_update_period = 30;       // 每 30 帧 EMA 更新
    int small_recent_window = 60;           // 最近 60 帧窗口
    double small_speed_ema_alpha = 0.3;     // 小符速度 EMA 系数
    double small_smooth_ema_old = 0.75;     // 输出 EMA: 旧值权重
    double small_smooth_ema_new = 0.25;     // 输出 EMA: 新值权重

    // ---- 大小符自动判别 (MoveModeDetector) ----
    int mode_detector_window_samples = 60;          // 扫描窗口样本数
    int mode_detector_minimum_samples = 12;         // 最小有效样本数
    int mode_detector_evaluate_every = 10;          // 每 N 样本评估一次
    int mode_detector_confirmations = 3;            // 模式切换确认次数
    double mode_detector_max_sample_jump_ratio = 0.014;  // 突变剔除门限 (×π)
    double mode_detector_big_threshold_ratio = 0.60;     // 大符振幅比阈值
    double mode_detector_small_threshold_ratio = 0.40;   // 小符振幅比阈值
};

// ---------------------------------------------------------------------------
//  世界系 EKF (BuffWorldTracker) 可调参数
// ---------------------------------------------------------------------------
struct WorldTrackerConfig {
    // 原 buff_world_* 开关组
    bool ekf_enabled = false;
    bool ekf_control_enabled = false;
    bool apply_gimbal_pose = true;
    bool quaternion_is_imu_to_map = true;
    // 观测噪声 R (Fu.cpp param 默认值, 与 BuffWorldTrackerConfig 结构体默认不同)
    double center_measurement_std_m = 0.015;
    double normal_measurement_std_rad = 0.030;
    double phase_measurement_std_rad = 0.040;
    // 过程噪声 Q
    double center_process_std_mps = 0.010;
    double normal_process_std_radps = 0.010;
    double omega_process_std_radps2 = 0.250;
    // 原 buff_world_tracker.hpp 内部硬编码门限
    double min_omega = 0.15;            // kMinSmallBuffOmega
    double max_omega = 1.571;           // kMaxSmallBuffOmega
    double direction_lock_omega = 0.25; // kDirectionLockOmega
    int direction_lock_frames = 15;     // kDirectionLockFrames
    int ready_enter_frames = 10;        // kReadyEnterFrames
    int ready_exit_frames = 15;         // kReadyExitFrames
    double residual_gate_sigma = 3.5;   // kResidualGateSigma (Mahalanobis 野值门限)
    double phase_process_noise_floor = 1.0e-5;  // 相位过程噪声下限 rad²/帧
    double omega_init_abs = 0.9;        // 锁定方向后 ω 偏置
    double omega_ema_fast = 0.5;        // 前几帧 ω 注入 EMA
    double omega_ema_slow = 0.03;       // 之后 ω 注入 EMA
    int omega_ema_fast_frames = 8;      // 快 EMA 帧数分界
    double reverse_omega_reject_threshold = 0.30;  // 反向 ω 拒绝阈值
    double min_time_diff_seconds = 0.02;           // 差分最小间隔
    int omega_clamp_start_frames = 4;   // ω 限幅启动帧数
    double min_radius_m = 0.05;         // 半径有效性 5cm
    double min_dt_seconds = 1.0e-4;     // dt 下限
    double max_dt_seconds = 1.0;        // dt 上限
    int minimum_observations = 5;       // ready 最小帧数
    double max_prediction_seconds = 1.0;   // 预测时域上限
    double normal_pitch_limit_deg = 90.0;  // 法线 pitch 限幅 ±90°
    double init_covariance_scale = 1.0e3;  // 初始协方差放大倍数
};

// ---------------------------------------------------------------------------
//  大符 ZJU RANSAC 正弦模型
// ---------------------------------------------------------------------------
struct BigRansacConfig {
    // 原 Fu.cpp 参数默认 g_buff_big_ransac_enabled = true (rm_fu.yaml 里覆盖为 false)
    bool enabled = true;
    int max_iterations = 200;
    double threshold = 0.5;
    double min_omega = 1.884;
    double max_omega = 2.0;
    double min_amplitude = 0.780;
    double max_amplitude = 1.045;
    int min_inliers = 100;
    double max_abs_speed = 3.2;
    double max_gap_seconds = 5.0;
    int max_consecutive_rejections = 30;
    unsigned int random_seed = 0x5EEDu;
};

// ---------------------------------------------------------------------------
//  聚合
// ---------------------------------------------------------------------------
struct BuffConfig {
    // 原始节点, 供后续阶段按需读取尚未结构化的键
    YAML::Node raw;

    SyncConfig sync;
    TimingConfig timing;
    ControlConfig control;
    BallisticConfig ballistic;
    ModelConfig model;
    RecognitionConfig recognition;
    GeometryConfig geometry;
    AngleConfig angle;
    LegacyPredictorConfig legacy_predictor;
    WorldTrackerConfig world_tracker;
    BigRansacConfig big_ransac;
};

namespace detail {

// 按 "a.b.c" 形式取节点; 任一层缺失返回 Undefined
//
// 注意: 不能写 `YAML::Node current = root; ... current = current[key];` ——
// yaml-cpp 0.6 的 operator= 会把子节点的数据写进共享的内部节点: 实测第一次
// 查询成功后 Buff map 就被篡改成子节点的内容, 导致除第一个键之外的全部参数
// 静默回落到代码默认值 (现象: 改了 rm_fu.yaml 完全没效果, 且不报任何警告)。
// 每层先 Clone 出私有副本再取子键, 保证不触碰调用方持有的节点。
inline YAML::Node lookup(const YAML::Node& root, const std::string& dotted) {
    YAML::Node current = YAML::Clone(root);
    std::size_t start = 0;
    while (true) {
        if (!current || !current.IsMap()) {
            return YAML::Node(YAML::NodeType::Undefined);
        }
        const std::size_t dot = dotted.find('.', start);
        const std::string key = (dot == std::string::npos)
            ? dotted.substr(start)
            : dotted.substr(start, dot - start);
        current = YAML::Clone(current)[key];
        if (dot == std::string::npos) {
            return current;
        }
        start = dot + 1;
    }
}

inline std::string formatNumber(double value) {
    std::ostringstream out;
    out.precision(10);
    out << value;
    return out.str();
}

inline std::string formatVector(const std::vector<double>& values) {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << ", ";
        out << values[i];
    }
    out << "]";
    return out.str();
}

// 单键解析失败时回落到默认值并记录警告
template <typename T>
T parseOr(const YAML::Node& node, const std::string& name, const T& fallback,
          std::vector<std::string>& warnings) {
    try {
        return node.as<T>();
    } catch (const YAML::Exception& error) {
        warnings.push_back("Buff." + name + ": invalid value (" +
                           std::string(error.what()) + "), using default");
        return fallback;
    }
}

// 非有限值 -> 默认值
inline void requireFinite(double& value, double fallback, const std::string& name,
                          std::vector<std::string>& warnings) {
    if (!std::isfinite(value)) {
        warnings.push_back("Buff." + name + ": non-finite value, using default " +
                           formatNumber(fallback));
        value = fallback;
    }
}

// 越界 -> 夹取 (记录警告)
inline void clampTo(double& value, double low, double high, const std::string& name,
                    std::vector<std::string>& warnings) {
    if (value < low || value > high) {
        const double clamped = std::min(std::max(value, low), high);
        warnings.push_back("Buff." + name + ": " + formatNumber(value) +
                           " out of range [" + formatNumber(low) + ", " +
                           formatNumber(high) + "], clamped to " +
                           formatNumber(clamped));
        value = clamped;
    }
}

// 只夹上/下界 (下界为 0 的参数用 clampMin, 上界用 clampMax)
inline void clampMin(double& value, double low, const std::string& name,
                     std::vector<std::string>& warnings) {
    clampTo(value, low, std::numeric_limits<double>::max(), name, warnings);
}

inline void clampMax(double& value, double high, const std::string& name,
                     std::vector<std::string>& warnings) {
    clampTo(value, -std::numeric_limits<double>::max(), high, name, warnings);
}

// 必须为正: 非有限或 <= 0 一律回落到默认值
// (物理量为负时夹到 1e-3 只会静默失效, 回落默认值更安全也更好排查)
inline void requirePositive(double& value, double fallback, const std::string& name,
                            std::vector<std::string>& warnings) {
    if (!std::isfinite(value) || value <= 0.0) {
        warnings.push_back("Buff." + name +
                           ": must be > 0 and finite, using default " +
                           formatNumber(fallback));
        value = fallback;
    }
}

// 整数/计数的上下界 (避免每个字段都写一段 if)
template <typename T>
inline void requireAtLeast(T& value, T minimum, const std::string& name,
                           std::vector<std::string>& warnings) {
    if (value < minimum) {
        warnings.push_back("Buff." + name + ": must be >= " +
                           formatNumber(static_cast<double>(minimum)) + ", using " +
                           formatNumber(static_cast<double>(minimum)));
        value = minimum;
    }
}

template <typename T>
inline void requireAtMost(T& value, T maximum, const std::string& name,
                          std::vector<std::string>& warnings) {
    if (value > maximum) {
        warnings.push_back("Buff." + name + ": must be <= " +
                           formatNumber(static_cast<double>(maximum)) + ", using " +
                           formatNumber(static_cast<double>(maximum)));
        value = maximum;
    }
}

}  // namespace detail

// ---------------------------------------------------------------------------
//  加载
// ---------------------------------------------------------------------------
inline bool loadBuffConfig(const std::string& path, BuffConfig& out,
                           std::vector<std::string>& warnings, std::string& error) {
    warnings.clear();
    error.clear();

    if (path.empty()) {
        error = "empty Buff configuration path";
        return false;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::BadFile& bad_file) {
        error = "cannot open Buff configuration '" + path + "': " + bad_file.what();
        return false;
    } catch (const YAML::ParserException& parse_error) {
        error = "Buff configuration '" + path + "' parse error at line " +
                std::to_string(parse_error.mark.line) + ": " + parse_error.what();
        return false;
    } catch (const std::exception& other) {
        error = "Buff configuration '" + path + "' load failed: " + other.what();
        return false;
    }

    out = BuffConfig{};
    out.raw = root;

    const YAML::Node buff = root["Buff"];
    if (!buff || !buff.IsMap()) {
        warnings.push_back(
            "no 'Buff:' section in the configuration; every parameter uses its "
            "built-in default (same values as the pre-migration hardcoded ones)");
    }

    // 读一个键: 先 Buff 节, 再顶层旧扁平键 (过渡兼容), 最后默认值
    const auto read = [&](const std::string& key, const std::string& legacy_key,
                          const auto& fallback) {
        using T = typename std::decay<decltype(fallback)>::type;
        const YAML::Node primary = detail::lookup(buff, key);
        if (primary && !primary.IsNull()) {
            return detail::parseOr<T>(primary, key, fallback, warnings);
        }
        if (!legacy_key.empty()) {
            const YAML::Node legacy = detail::lookup(root, legacy_key);
            if (legacy && !legacy.IsNull()) {
                warnings.push_back("deprecated top-level key '" + legacy_key +
                                   "'; move it to Buff." + key);
                return detail::parseOr<T>(legacy, legacy_key, fallback, warnings);
            }
        }
        return static_cast<T>(fallback);
    };

    // ------------------------- sync -------------------------
    {
        SyncConfig& c = out.sync;
        c.imu_pose_history_capacity =
            read("sync.imu_pose_history_capacity", "",
                 c.imu_pose_history_capacity);
        c.max_imu_image_skew_seconds =
            read("sync.max_imu_image_skew_seconds", "", c.max_imu_image_skew_seconds);
        c.imu_stamp_offset_seconds =
            read("sync.imu_stamp_offset_seconds", "buff_imu_stamp_offset_seconds",
                 c.imu_stamp_offset_seconds);
        c.quaternion_norm_min =
            read("sync.quaternion_norm_min", "", c.quaternion_norm_min);
        c.quaternion_norm_max =
            read("sync.quaternion_norm_max", "", c.quaternion_norm_max);
        detail::requirePositive(c.max_imu_image_skew_seconds, 0.05,
                                "sync.max_imu_image_skew_seconds", warnings);
        detail::requireAtLeast(c.imu_pose_history_capacity,
                               static_cast<std::size_t>(1),
                               "sync.imu_pose_history_capacity", warnings);
        detail::requireFinite(c.imu_stamp_offset_seconds, 0.0,
                              "sync.imu_stamp_offset_seconds", warnings);
        detail::requireFinite(c.quaternion_norm_min, 0.9,
                              "sync.quaternion_norm_min", warnings);
        detail::requireFinite(c.quaternion_norm_max, 1.1,
                              "sync.quaternion_norm_max", warnings);
        detail::clampMax(c.max_imu_image_skew_seconds, 5.0,
                         "sync.max_imu_image_skew_seconds", warnings);
        detail::clampTo(c.imu_stamp_offset_seconds, -1.0, 1.0,
                        "sync.imu_stamp_offset_seconds", warnings);
        detail::clampTo(c.quaternion_norm_min, 1.0e-6, 1.0,
                        "sync.quaternion_norm_min", warnings);
        detail::clampTo(c.quaternion_norm_max, 1.0, 1.0e3,
                        "sync.quaternion_norm_max", warnings);
        if (c.quaternion_norm_min >= c.quaternion_norm_max) {
            warnings.push_back(
                "Buff.sync: quaternion_norm_min >= quaternion_norm_max, "
                "restoring defaults 0.9 / 1.1");
            c.quaternion_norm_min = 0.9;
            c.quaternion_norm_max = 1.1;
        }
    }

    // ------------------------- timing -------------------------
    {
        TimingConfig& c = out.timing;
        c.delta_t_init = read("timing.delta_t_init", "", c.delta_t_init);
        c.delta_t_compensation =
            read("timing.delta_t_compensation", "", c.delta_t_compensation);
        c.delta_t_min = read("timing.delta_t_min", "", c.delta_t_min);
        c.delta_t_max = read("timing.delta_t_max", "", c.delta_t_max);
        c.flight_time_iterations =
            read("timing.flight_time_iterations", "", c.flight_time_iterations);
        c.flight_time_valid_min =
            read("timing.flight_time_valid_min", "", c.flight_time_valid_min);
        c.flight_time_valid_max =
            read("timing.flight_time_valid_max", "", c.flight_time_valid_max);
        c.lead_ema_old = read("timing.lead_ema_old", "", c.lead_ema_old);
        c.lead_ema_new = read("timing.lead_ema_new", "", c.lead_ema_new);
        c.max_coast_frames =
            read("timing.max_coast_frames", "", c.max_coast_frames);
        c.fallback_fps = read("timing.fallback_fps", "", c.fallback_fps);
        c.fallback_fps_ema_old =
            read("timing.fallback_fps_ema_old", "", c.fallback_fps_ema_old);
        c.fallback_fps_ema_new =
            read("timing.fallback_fps_ema_new", "", c.fallback_fps_ema_new);
        c.fps_interval_max_ms =
            read("timing.fps_interval_max_ms", "", c.fps_interval_max_ms);
        c.angle_cache_len = read("timing.angle_cache_len", "", c.angle_cache_len);
        c.default_omega = read("timing.default_omega", "", c.default_omega);
        c.kalman_bad_frames =
            read("timing.kalman_bad_frames", "", c.kalman_bad_frames);
        c.divergence_min_radius_px =
            read("timing.divergence_min_radius_px", "", c.divergence_min_radius_px);
        c.divergence_radius_ratio_min = read("timing.divergence_radius_ratio_min", "",
                                             c.divergence_radius_ratio_min);
        c.divergence_radius_ratio_max = read("timing.divergence_radius_ratio_max", "",
                                             c.divergence_radius_ratio_max);

        detail::requireFinite(c.delta_t_init, 0.3, "timing.delta_t_init", warnings);
        detail::requireFinite(c.delta_t_compensation, 0.15,
                              "timing.delta_t_compensation", warnings);
        detail::requirePositive(c.delta_t_min, 0.01, "timing.delta_t_min", warnings);
        detail::requirePositive(c.delta_t_max, 1.0, "timing.delta_t_max", warnings);
        detail::requireFinite(c.flight_time_valid_min, 0.01,
                              "timing.flight_time_valid_min", warnings);
        detail::requireFinite(c.flight_time_valid_max, 2.0,
                              "timing.flight_time_valid_max", warnings);
        detail::requireFinite(c.lead_ema_old, 0.8, "timing.lead_ema_old", warnings);
        detail::requireFinite(c.lead_ema_new, 0.2, "timing.lead_ema_new", warnings);
        detail::requireAtLeast(c.max_coast_frames, 0, "timing.max_coast_frames",
                               warnings);
        detail::requirePositive(c.fallback_fps, 120.0, "timing.fallback_fps", warnings);
        detail::requireFinite(c.fallback_fps_ema_old, 0.9,
                              "timing.fallback_fps_ema_old", warnings);
        detail::requireFinite(c.fallback_fps_ema_new, 0.1,
                              "timing.fallback_fps_ema_new", warnings);
        detail::requirePositive(c.fps_interval_max_ms, 500.0,
                                "timing.fps_interval_max_ms", warnings);
        detail::requirePositive(c.default_omega, 1.047, "timing.default_omega",
                                warnings);
        detail::requireFinite(c.divergence_min_radius_px, 1.0,
                              "timing.divergence_min_radius_px", warnings);
        detail::requirePositive(c.divergence_radius_ratio_min, 0.3,
                                "timing.divergence_radius_ratio_min", warnings);
        detail::requirePositive(c.divergence_radius_ratio_max, 3.5,
                                "timing.divergence_radius_ratio_max", warnings);

        detail::clampMax(c.delta_t_min, 10.0, "timing.delta_t_min", warnings);
        detail::clampMax(c.delta_t_max, 10.0, "timing.delta_t_max", warnings);
        if (c.delta_t_max <= c.delta_t_min) {
            warnings.push_back(
                "Buff.timing: delta_t_max <= delta_t_min, restoring defaults "
                "0.01 / 1.0");
            c.delta_t_min = 0.01;
            c.delta_t_max = 1.0;
        }
        detail::clampTo(c.delta_t_init, c.delta_t_min, c.delta_t_max,
                        "timing.delta_t_init", warnings);
        detail::clampTo(c.delta_t_compensation, 0.0, c.delta_t_max,
                        "timing.delta_t_compensation", warnings);
        detail::requireAtLeast(c.flight_time_iterations, 1,
                               "timing.flight_time_iterations", warnings);
        detail::requireAtMost(c.flight_time_iterations, 10,
                              "timing.flight_time_iterations", warnings);
        detail::clampTo(c.flight_time_valid_min, 0.0, c.flight_time_valid_max,
                        "timing.flight_time_valid_min", warnings);
        detail::clampMin(c.flight_time_valid_max, 1.0e-3,
                         "timing.flight_time_valid_max", warnings);
        detail::clampMax(c.flight_time_valid_max, 10.0,
                         "timing.flight_time_valid_max", warnings);
        detail::clampTo(c.lead_ema_old, 0.0, 1.0, "timing.lead_ema_old", warnings);
        detail::clampTo(c.lead_ema_new, 0.0, 1.0, "timing.lead_ema_new", warnings);
        detail::clampMax(c.fallback_fps, 1000.0, "timing.fallback_fps", warnings);
        detail::clampTo(c.fallback_fps_ema_old, 0.0, 1.0,
                        "timing.fallback_fps_ema_old", warnings);
        detail::clampTo(c.fallback_fps_ema_new, 0.0, 1.0,
                        "timing.fallback_fps_ema_new", warnings);
        detail::clampMax(c.fps_interval_max_ms, 10000.0,
                         "timing.fps_interval_max_ms", warnings);
        detail::requireAtLeast(c.angle_cache_len, 1, "timing.angle_cache_len",
                               warnings);
        detail::clampMax(c.default_omega, 10.0, "timing.default_omega", warnings);
        detail::requireAtLeast(c.kalman_bad_frames, 1,
                               "timing.kalman_bad_frames", warnings);
        detail::clampTo(c.divergence_min_radius_px, 0.0, 1.0e3,
                        "timing.divergence_min_radius_px", warnings);
        detail::clampMax(c.divergence_radius_ratio_min, 1.0e3,
                         "timing.divergence_radius_ratio_min", warnings);
        detail::clampMax(c.divergence_radius_ratio_max, 1.0e3,
                         "timing.divergence_radius_ratio_max", warnings);
        if (c.divergence_radius_ratio_max <= c.divergence_radius_ratio_min) {
            warnings.push_back(
                "Buff.timing: divergence_radius_ratio_max <= "
                "divergence_radius_ratio_min, restoring defaults 0.3 / 3.5");
            c.divergence_radius_ratio_min = 0.3;
            c.divergence_radius_ratio_max = 3.5;
        }
    }

    // ------------------------- control -------------------------
    {
        ControlConfig& c = out.control;
        c.pitch_compensation =
            read("control.pitch_compensation", "", c.pitch_compensation);
        c.yaw_compensation = read("control.yaw_compensation", "", c.yaw_compensation);
        c.gimbal_pose_alpha =
            read("control.gimbal_pose_alpha", "", c.gimbal_pose_alpha);
        c.pitch_max_deg = read("control.pitch_max_deg", "", c.pitch_max_deg);
        c.pitch_min_deg = read("control.pitch_min_deg", "", c.pitch_min_deg);
        c.stuck_threshold_deg =
            read("control.stuck_threshold_deg", "", c.stuck_threshold_deg);
        c.stuck_frames = read("control.stuck_frames", "", c.stuck_frames);
        c.recovery_pitch_deg =
            read("control.recovery_pitch_deg", "", c.recovery_pitch_deg);
        c.recovery_frames = read("control.recovery_frames", "", c.recovery_frames);
        c.log_throttle_s = read("control.log_throttle_s", "", c.log_throttle_s);

        detail::requireFinite(c.pitch_compensation, 0.0,
                              "control.pitch_compensation", warnings);
        detail::requireFinite(c.yaw_compensation, 0.0, "control.yaw_compensation",
                              warnings);
        detail::clampTo(c.gimbal_pose_alpha, 0.0, 1.0, "control.gimbal_pose_alpha",
                        warnings);
        detail::requireFinite(c.pitch_max_deg, 22.0, "control.pitch_max_deg", warnings);
        detail::requireFinite(c.pitch_min_deg, -15.0, "control.pitch_min_deg", warnings);
        detail::requireFinite(c.stuck_threshold_deg, 19.0,
                              "control.stuck_threshold_deg", warnings);
        detail::requireFinite(c.recovery_pitch_deg, -10.0,
                              "control.recovery_pitch_deg", warnings);
        detail::requireFinite(c.log_throttle_s, 0.2, "control.log_throttle_s", warnings);

        detail::clampTo(c.pitch_compensation, -1.0, 1.0, "control.pitch_compensation",
                        warnings);
        detail::clampTo(c.yaw_compensation, -1.0, 1.0, "control.yaw_compensation",
                        warnings);
        detail::clampTo(c.pitch_max_deg, -89.0, 89.0, "control.pitch_max_deg", warnings);
        detail::clampTo(c.pitch_min_deg, -89.0, 89.0, "control.pitch_min_deg", warnings);
        if (c.pitch_min_deg >= c.pitch_max_deg) {
            warnings.push_back(
                "Buff.control: pitch_min_deg >= pitch_max_deg, restoring defaults "
                "-15 / 22");
            c.pitch_min_deg = -15.0;
            c.pitch_max_deg = 22.0;
        }
        detail::clampTo(c.stuck_threshold_deg, -89.0, 89.0,
                        "control.stuck_threshold_deg", warnings);
        detail::clampTo(c.recovery_pitch_deg, -89.0, 89.0,
                        "control.recovery_pitch_deg", warnings);
        detail::requireAtLeast(c.stuck_frames, 1, "control.stuck_frames", warnings);
        detail::requireAtLeast(c.recovery_frames, 1, "control.recovery_frames",
                               warnings);
        detail::clampTo(c.log_throttle_s, 0.0, 60.0, "control.log_throttle_s", warnings);
    }

    // ------------------------- ballistic -------------------------
    {
        BallisticConfig& c = out.ballistic;
        c.default_velocity =
            read("ballistic.default_velocity", "", c.default_velocity);
        c.velocity_min_valid =
            read("ballistic.velocity_min_valid", "", c.velocity_min_valid);
        c.gravity = read("ballistic.gravity", "", c.gravity);
        c.iteration_times = read("ballistic.iteration_times", "", c.iteration_times);
        c.converge_epsilon =
            read("ballistic.converge_epsilon", "", c.converge_epsilon);
        c.height_tolerance =
            read("ballistic.height_tolerance", "", c.height_tolerance);
        c.angle_limit_deg = read("ballistic.angle_limit_deg", "", c.angle_limit_deg);
        c.phase_probe_rad = read("ballistic.phase_probe_rad", "", c.phase_probe_rad);
        c.probe_valid_rad = read("ballistic.probe_valid_rad", "", c.probe_valid_rad);
        c.physical_phase_limit_rad = read("ballistic.physical_phase_limit_rad", "",
                                          c.physical_phase_limit_rad);
        c.max_reprojection_error_px = read("ballistic.max_reprojection_error_px", "",
                                           c.max_reprojection_error_px);
        c.reprojection_check_enabled = read("ballistic.reprojection_check_enabled", "",
                                            c.reprojection_check_enabled);
        c.optical_center_offset_x = read("ballistic.optical_center_offset_x", "",
                                         c.optical_center_offset_x);
        c.optical_center_offset_y = read("ballistic.optical_center_offset_y", "",
                                         c.optical_center_offset_y);

        detail::requirePositive(c.default_velocity, 24.0,
                                "ballistic.default_velocity", warnings);
        detail::requireFinite(c.velocity_min_valid, 10.0,
                              "ballistic.velocity_min_valid", warnings);
        detail::requirePositive(c.gravity, 9.8, "ballistic.gravity", warnings);
        detail::requirePositive(c.converge_epsilon, 0.0001,
                                "ballistic.converge_epsilon", warnings);
        detail::requirePositive(c.height_tolerance, 0.01,
                                "ballistic.height_tolerance", warnings);
        detail::requirePositive(c.angle_limit_deg, 29.7, "ballistic.angle_limit_deg",
                                warnings);
        detail::requirePositive(c.phase_probe_rad, 0.30, "ballistic.phase_probe_rad",
                                warnings);
        detail::requireFinite(c.probe_valid_rad, 0.015, "ballistic.probe_valid_rad",
                              warnings);
        detail::requirePositive(c.physical_phase_limit_rad, 2.5,
                                "ballistic.physical_phase_limit_rad", warnings);
        detail::requirePositive(c.max_reprojection_error_px, 12.0,
                                "ballistic.max_reprojection_error_px", warnings);
        detail::requireFinite(c.optical_center_offset_x, -23.0,
                              "ballistic.optical_center_offset_x", warnings);
        detail::requireFinite(c.optical_center_offset_y, 24.0,
                              "ballistic.optical_center_offset_y", warnings);

        detail::clampMax(c.default_velocity, 200.0, "ballistic.default_velocity",
                         warnings);
        detail::clampTo(c.velocity_min_valid, 0.0, c.default_velocity,
                        "ballistic.velocity_min_valid", warnings);
        detail::clampMax(c.gravity, 100.0, "ballistic.gravity", warnings);
        detail::requireAtLeast(c.iteration_times, 1, "ballistic.iteration_times",
                               warnings);
        detail::requireAtMost(c.iteration_times, 1000, "ballistic.iteration_times",
                              warnings);
        detail::clampMax(c.converge_epsilon, 1.0, "ballistic.converge_epsilon",
                         warnings);
        detail::clampMax(c.height_tolerance, 10.0, "ballistic.height_tolerance",
                         warnings);
        detail::clampMax(c.angle_limit_deg, 89.0, "ballistic.angle_limit_deg",
                         warnings);
        detail::clampMax(c.phase_probe_rad, 3.14159265358979323846,
                         "ballistic.phase_probe_rad", warnings);
        detail::clampTo(c.probe_valid_rad, 0.0, 3.14159265358979323846,
                        "ballistic.probe_valid_rad", warnings);
        detail::clampMax(c.physical_phase_limit_rad, 1.0e3,
                         "ballistic.physical_phase_limit_rad", warnings);
        detail::clampMax(c.max_reprojection_error_px, 1.0e4,
                         "ballistic.max_reprojection_error_px", warnings);
        detail::clampTo(c.optical_center_offset_x, -1.0e4, 1.0e4,
                        "ballistic.optical_center_offset_x", warnings);
        detail::clampTo(c.optical_center_offset_y, -1.0e4, 1.0e4,
                        "ballistic.optical_center_offset_y", warnings);
    }

    // ------------------------- model -------------------------
    {
        ModelConfig& c = out.model;
        c.model_path = read("model.model_path", "", c.model_path);
        c.enemy_color = read("model.enemy_color", "", c.enemy_color);
        c.move_mode = read("model.move_mode", "buff_move_mode", c.move_mode);
        c.run_without_serial =
            read("model.run_without_serial", "", c.run_without_serial);
        c.functional_default =
            read("model.functional_default", "", c.functional_default);
        c.input_size = read("model.input_size", "", c.input_size);
        c.model_device = read("model.model_device", "", c.model_device);

        detail::requireAtLeast(c.input_size, 64, "model.input_size", warnings);
        detail::requireAtMost(c.input_size, 4096, "model.input_size", warnings);
        if (c.move_mode != "small" && c.move_mode != "big" && c.move_mode != "auto") {
            warnings.push_back("Buff.model.move_mode: unsupported '" + c.move_mode +
                               "', using 'small'");
            c.move_mode = "small";
        }
        if (c.enemy_color != "blue" && c.enemy_color != "red") {
            warnings.push_back("Buff.model.enemy_color: unsupported '" +
                               c.enemy_color + "', using 'blue'");
            c.enemy_color = "blue";
        }
    }

    // ------------------------- recognition -------------------------
    {
        RecognitionConfig& c = out.recognition;
        c.conf_threshold = read("recognition.conf_threshold", "", c.conf_threshold);
        c.nms_threshold = read("recognition.nms_threshold", "", c.nms_threshold);
        c.binary_threshold = read("recognition.binary_threshold", "", c.binary_threshold);
        c.dilate_kernel_size =
            read("recognition.dilate_kernel_size", "", c.dilate_kernel_size);
        c.r_center_mask_ratio =
            read("recognition.r_center_mask_ratio", "", c.r_center_mask_ratio);
        c.r_center_contour_div =
            read("recognition.r_center_contour_div", "", c.r_center_contour_div);
        c.r_center_refine_max_shift =
            read("recognition.r_center_refine_max_shift", "", c.r_center_refine_max_shift);
        c.box_alpha = read("recognition.box_alpha", "", c.box_alpha);
        c.stabilizer_alpha =
            read("recognition.stabilizer_alpha", "", c.stabilizer_alpha);
        c.assoc_gate_min_px =
            read("recognition.assoc_gate_min_px", "", c.assoc_gate_min_px);
        c.assoc_gate_radius_ratio =
            read("recognition.assoc_gate_radius_ratio", "", c.assoc_gate_radius_ratio);
        c.jump_fan_radius_ratio =
            read("recognition.jump_fan_radius_ratio", "", c.jump_fan_radius_ratio);
        c.jump_r_radius_ratio =
            read("recognition.jump_r_radius_ratio", "", c.jump_r_radius_ratio);
        c.radius_change_reject_ratio =
            read("recognition.radius_change_reject_ratio", "",
                 c.radius_change_reject_ratio);
        c.side_change_reject_ratio =
            read("recognition.side_change_reject_ratio", "", c.side_change_reject_ratio);
        c.jump_confirm_frames =
            read("recognition.jump_confirm_frames", "", c.jump_confirm_frames);
        c.reacquire_confirm_frames =
            read("recognition.reacquire_confirm_frames", "", c.reacquire_confirm_frames);
        c.miss_reset_frames =
            read("recognition.miss_reset_frames", "", c.miss_reset_frames);
        c.min_valid_radius_px =
            read("recognition.min_valid_radius_px", "", c.min_valid_radius_px);
        c.ego_motion_max_delta_rad =
            read("recognition.ego_motion_max_delta_rad", "", c.ego_motion_max_delta_rad);

        detail::clampTo(c.conf_threshold, 0.0, 1.0, "recognition.conf_threshold", warnings);
        detail::clampTo(c.nms_threshold, 0.0, 1.0, "recognition.nms_threshold", warnings);
        detail::requireAtLeast(c.binary_threshold, 0, "recognition.binary_threshold", warnings);
        detail::requireAtMost(c.binary_threshold, 255, "recognition.binary_threshold", warnings);
        detail::requireAtLeast(c.dilate_kernel_size, 1, "recognition.dilate_kernel_size", warnings);
        detail::requireAtMost(c.dilate_kernel_size, 31, "recognition.dilate_kernel_size", warnings);
        detail::clampTo(c.r_center_mask_ratio, 0.0, 2.0, "recognition.r_center_mask_ratio", warnings);
        detail::clampTo(c.r_center_contour_div, 0.01, 100.0, "recognition.r_center_contour_div", warnings);
        detail::clampTo(c.r_center_refine_max_shift, 0.0, 1000.0, "recognition.r_center_refine_max_shift", warnings);
        detail::clampTo(c.box_alpha, 0.0, 1.0, "recognition.box_alpha", warnings);
        detail::clampTo(c.stabilizer_alpha, 0.0, 1.0, "recognition.stabilizer_alpha", warnings);
        detail::clampTo(c.assoc_gate_min_px, 0.0, 10000.0, "recognition.assoc_gate_min_px", warnings);
        detail::clampTo(c.assoc_gate_radius_ratio, 0.0, 10.0, "recognition.assoc_gate_radius_ratio", warnings);
        detail::clampTo(c.jump_fan_radius_ratio, 0.0, 5.0, "recognition.jump_fan_radius_ratio", warnings);
        detail::clampTo(c.jump_r_radius_ratio, 0.0, 5.0, "recognition.jump_r_radius_ratio", warnings);
        detail::clampTo(c.radius_change_reject_ratio, 0.0, 5.0, "recognition.radius_change_reject_ratio", warnings);
        detail::clampTo(c.side_change_reject_ratio, 0.0, 5.0, "recognition.side_change_reject_ratio", warnings);
        detail::requireAtLeast(c.jump_confirm_frames, 1, "recognition.jump_confirm_frames", warnings);
        detail::requireAtLeast(c.reacquire_confirm_frames, 1, "recognition.reacquire_confirm_frames", warnings);
        detail::requireAtLeast(c.miss_reset_frames, 1, "recognition.miss_reset_frames", warnings);
        detail::clampTo(c.min_valid_radius_px, 0.0, 10000.0, "recognition.min_valid_radius_px", warnings);
        detail::clampTo(c.ego_motion_max_delta_rad, 0.0, 1.57079632679489661923, "recognition.ego_motion_max_delta_rad", warnings);
    }

    // ------------------------- geometry -------------------------
    {
        GeometryConfig& c = out.geometry;
        c.target_width_mm =
            read("geometry.target_width_mm", "buff_target_width_mm", c.target_width_mm);
        c.target_height_mm =
            read("geometry.target_height_mm", "buff_target_height_mm", c.target_height_mm);
        c.camera_matrix =
            read("geometry.camera_matrix", "buff_camera_matrix", c.camera_matrix);
        c.dist_coeffs =
            read("geometry.dist_coeffs", "buff_dist_coeffs", c.dist_coeffs);
        c.radius_mm = read("geometry.radius_mm", "buff_radius_mm", c.radius_mm);
        c.hub_normal_offset_mm = read("geometry.hub_normal_offset_mm",
                                      "buff_hub_normal_offset_mm",
                                      c.hub_normal_offset_mm);

        if (c.camera_matrix.size() != 9) {
            error = "Buff.geometry.camera_matrix must have exactly 9 values";
            return false;
        }
        if (c.dist_coeffs.size() != 5) {
            error = "Buff.geometry.dist_coeffs must have exactly 5 values";
            return false;
        }
        detail::requirePositive(c.target_width_mm, 300.0, "geometry.target_width_mm", warnings);
        detail::requirePositive(c.target_height_mm, 300.0, "geometry.target_height_mm", warnings);
        detail::clampTo(c.radius_mm, 0.0, 100000.0, "geometry.radius_mm", warnings);
        detail::requireFinite(c.hub_normal_offset_mm, 0.0, "geometry.hub_normal_offset_mm", warnings);
    }

    // ------------------------- angle -------------------------
    {
        AngleConfig& c = out.angle;
        c.blade_count = read("angle.blade_count", "", c.blade_count);
        c.direction_judge_samples =
            read("angle.direction_judge_samples", "", c.direction_judge_samples);
        c.wrap_period = read("angle.wrap_period", "", c.wrap_period);
        c.unwrap_tolerance_divisor =
            read("angle.unwrap_tolerance_divisor", "", c.unwrap_tolerance_divisor);
        c.diff_search_start_ratio =
            read("angle.diff_search_start_ratio", "", c.diff_search_start_ratio);
        c.diff_timeout_margin_ms =
            read("angle.diff_timeout_margin_ms", "", c.diff_timeout_margin_ms);
        c.diff_angle_limit_rad =
            read("angle.diff_angle_limit_rad", "", c.diff_angle_limit_rad);
        c.direction_filter_ratio =
            read("angle.direction_filter_ratio", "", c.direction_filter_ratio);
        c.angle_buffer_len =
            read("angle.angle_buffer_len", "", c.angle_buffer_len);
        c.sector_search_range =
            read("angle.sector_search_range", "", c.sector_search_range);
        c.image_width = read("angle.image_width", "", c.image_width);
        c.image_height = read("angle.image_height", "", c.image_height);
        c.small_buff_converge_delay =
            read("angle.small_buff_converge_delay", "", c.small_buff_converge_delay);
        c.small_buff_max_roll_residual =
            read("angle.small_buff_max_roll_residual", "", c.small_buff_max_roll_residual);
        c.small_buff_min_speed =
            read("angle.small_buff_min_speed", "", c.small_buff_min_speed);
        c.small_buff_max_speed =
            read("angle.small_buff_max_speed", "", c.small_buff_max_speed);
        c.small_buff_phase_process_noise =
            read("angle.small_buff_phase_process_noise", "", c.small_buff_phase_process_noise);
        c.small_buff_phase_measurement_noise =
            read("angle.small_buff_phase_measurement_noise", "", c.small_buff_phase_measurement_noise);
        c.small_buff_min_radius_px =
            read("angle.small_buff_min_radius_px", "", c.small_buff_min_radius_px);
        c.small_buff_max_dt =
            read("angle.small_buff_max_dt", "", c.small_buff_max_dt);
        c.small_buff_reject_reset_count =
            read("angle.small_buff_reject_reset_count", "", c.small_buff_reject_reset_count);
        c.small_buff_vote_min_phase_delta =
            read("angle.small_buff_vote_min_phase_delta", "", c.small_buff_vote_min_phase_delta);
        c.small_buff_vote_saturation =
            read("angle.small_buff_vote_saturation", "", c.small_buff_vote_saturation);
        c.small_buff_omega_fit_min_samples =
            read("angle.small_buff_omega_fit_min_samples", "", c.small_buff_omega_fit_min_samples);
        c.small_buff_radius_ema =
            read("angle.small_buff_radius_ema", "", c.small_buff_radius_ema);
        c.small_buff_center_ema =
            read("angle.small_buff_center_ema", "", c.small_buff_center_ema);
        c.small_buff_center_snap_ratio =
            read("angle.small_buff_center_snap_ratio", "",
                 c.small_buff_center_snap_ratio);
        c.small_buff_omega_reset_cov =
            read("angle.small_buff_omega_reset_cov", "", c.small_buff_omega_reset_cov);
        c.small_buff_omega_locked_cov =
            read("angle.small_buff_omega_locked_cov", "", c.small_buff_omega_locked_cov);
        c.small_buff_init_cov_phase =
            read("angle.small_buff_init_cov_phase", "", c.small_buff_init_cov_phase);
        c.small_buff_init_cov_omega =
            read("angle.small_buff_init_cov_omega", "", c.small_buff_init_cov_omega);
        c.small_buff_phase_history_max =
            read("angle.small_buff_phase_history_max", "", c.small_buff_phase_history_max);

        detail::requireAtLeast(c.blade_count, 1, "angle.blade_count", warnings);
        detail::requireAtMost(c.blade_count, 16, "angle.blade_count", warnings);
        detail::requireAtLeast(c.direction_judge_samples, 1, "angle.direction_judge_samples", warnings);
        detail::requirePositive(c.wrap_period, 6.283185, "angle.wrap_period", warnings);
        detail::requirePositive(c.unwrap_tolerance_divisor, 6.55, "angle.unwrap_tolerance_divisor", warnings);
        detail::clampTo(c.diff_search_start_ratio, 0.0, 1.0, "angle.diff_search_start_ratio", warnings);
        detail::clampTo(c.diff_timeout_margin_ms, 0.0, 10000.0, "angle.diff_timeout_margin_ms", warnings);
        detail::requirePositive(c.diff_angle_limit_rad, 1.169, "angle.diff_angle_limit_rad", warnings);
        detail::requirePositive(c.direction_filter_ratio, 0.028, "angle.direction_filter_ratio", warnings);
        detail::requireAtLeast(c.angle_buffer_len, 1, "angle.angle_buffer_len", warnings);
        detail::requireAtLeast(c.sector_search_range, 0, "angle.sector_search_range", warnings);
        detail::requireAtLeast(c.image_width, 1, "angle.image_width", warnings);
        detail::requireAtLeast(c.image_height, 1, "angle.image_height", warnings);
        detail::requireAtLeast(c.small_buff_converge_delay, 1, "angle.small_buff_converge_delay", warnings);
        detail::requirePositive(c.small_buff_max_roll_residual, 0.55, "angle.small_buff_max_roll_residual", warnings);
        detail::requirePositive(c.small_buff_min_speed, 0.052, "angle.small_buff_min_speed", warnings);
        detail::requirePositive(c.small_buff_max_speed, 1.571, "angle.small_buff_max_speed", warnings);
        detail::requirePositive(c.small_buff_phase_process_noise, 0.005, "angle.small_buff_phase_process_noise", warnings);
        detail::requirePositive(c.small_buff_phase_measurement_noise, 0.004, "angle.small_buff_phase_measurement_noise", warnings);
        detail::clampTo(c.small_buff_min_radius_px, 0.0, 10000.0, "angle.small_buff_min_radius_px", warnings);
        detail::requirePositive(c.small_buff_max_dt, 0.5, "angle.small_buff_max_dt", warnings);
        detail::requireAtLeast(c.small_buff_reject_reset_count, 1, "angle.small_buff_reject_reset_count", warnings);
        detail::clampTo(c.small_buff_vote_min_phase_delta, 0.0, 3.14159265358979323846, "angle.small_buff_vote_min_phase_delta", warnings);
        detail::requireAtLeast(c.small_buff_vote_saturation, 1, "angle.small_buff_vote_saturation", warnings);
        detail::requireAtLeast(c.small_buff_omega_fit_min_samples, 2, "angle.small_buff_omega_fit_min_samples", warnings);
        detail::clampTo(c.small_buff_radius_ema, 0.0, 1.0, "angle.small_buff_radius_ema", warnings);
        detail::clampTo(c.small_buff_center_ema, 0.0, 1.0, "angle.small_buff_center_ema", warnings);
        detail::clampTo(c.small_buff_center_snap_ratio, 0.0, 10.0,
                        "angle.small_buff_center_snap_ratio", warnings);
        detail::requirePositive(c.small_buff_omega_reset_cov, 0.02, "angle.small_buff_omega_reset_cov", warnings);
        detail::requirePositive(c.small_buff_omega_locked_cov, 0.05, "angle.small_buff_omega_locked_cov", warnings);
        detail::requirePositive(c.small_buff_init_cov_phase, 0.10, "angle.small_buff_init_cov_phase", warnings);
        detail::requirePositive(c.small_buff_init_cov_omega, 0.50, "angle.small_buff_init_cov_omega", warnings);
        detail::requireAtLeast(c.small_buff_phase_history_max, 1, "angle.small_buff_phase_history_max", warnings);
    }

    // ------------------------- legacy_predictor -------------------------
    {
        LegacyPredictorConfig& c = out.legacy_predictor;
        c.big_min_init_samples =
            read("legacy_predictor.big_min_init_samples", "", c.big_min_init_samples);
        c.big_const_level =
            read("legacy_predictor.big_const_level", "", c.big_const_level);
        c.big_const_omega =
            read("legacy_predictor.big_const_omega", "", c.big_const_omega);
        c.big_angular_speed =
            read("legacy_predictor.big_angular_speed", "", c.big_angular_speed);
        c.ceres_trigger_factor =
            read("legacy_predictor.ceres_trigger_factor", "", c.ceres_trigger_factor);
        c.big_jump_ratio =
            read("legacy_predictor.big_jump_ratio", "", c.big_jump_ratio);
        c.big_moving_average_window =
            read("legacy_predictor.big_moving_average_window", "", c.big_moving_average_window);
        c.big_min_judge_samples =
            read("legacy_predictor.big_min_judge_samples", "", c.big_min_judge_samples);
        c.big_smooth_lookback_seconds =
            read("legacy_predictor.big_smooth_lookback_seconds", "", c.big_smooth_lookback_seconds);
        c.big_initial_phase =
            read("legacy_predictor.big_initial_phase", "", c.big_initial_phase);
        c.ceres_max_num_iterations =
            read("legacy_predictor.ceres_max_num_iterations", "", c.ceres_max_num_iterations);
        c.ceres_function_tolerance =
            read("legacy_predictor.ceres_function_tolerance", "", c.ceres_function_tolerance);
        c.ceres_gradient_tolerance =
            read("legacy_predictor.ceres_gradient_tolerance", "", c.ceres_gradient_tolerance);
        c.ceres_parameter_tolerance =
            read("legacy_predictor.ceres_parameter_tolerance", "", c.ceres_parameter_tolerance);
        c.ceres_num_threads =
            read("legacy_predictor.ceres_num_threads", "", c.ceres_num_threads);
        c.big_fit_max_level =
            read("legacy_predictor.big_fit_max_level", "", c.big_fit_max_level);
        c.big_fit_max_omega =
            read("legacy_predictor.big_fit_max_omega", "", c.big_fit_max_omega);
        c.big_fit_min_level =
            read("legacy_predictor.big_fit_min_level", "", c.big_fit_min_level);
        c.big_fit_min_omega =
            read("legacy_predictor.big_fit_min_omega", "", c.big_fit_min_omega);
        c.big_forced_omega =
            read("legacy_predictor.big_forced_omega", "", c.big_forced_omega);
        c.big_extreme_wait_seconds =
            read("legacy_predictor.big_extreme_wait_seconds", "", c.big_extreme_wait_seconds);
        c.big_vote_lookback_near =
            read("legacy_predictor.big_vote_lookback_near", "", c.big_vote_lookback_near);
        c.big_vote_lookback_mid =
            read("legacy_predictor.big_vote_lookback_mid", "", c.big_vote_lookback_mid);
        c.big_vote_lookback_far =
            read("legacy_predictor.big_vote_lookback_far", "", c.big_vote_lookback_far);
        c.big_smooth_window =
            read("legacy_predictor.big_smooth_window", "", c.big_smooth_window);
        c.small_min_init_samples =
            read("legacy_predictor.small_min_init_samples", "", c.small_min_init_samples);
        c.small_jump_ratio =
            read("legacy_predictor.small_jump_ratio", "", c.small_jump_ratio);
        c.small_init_timeout_frames =
            read("legacy_predictor.small_init_timeout_frames", "", c.small_init_timeout_frames);
        c.small_default_omega =
            read("legacy_predictor.small_default_omega", "", c.small_default_omega);
        c.small_ema_update_period =
            read("legacy_predictor.small_ema_update_period", "", c.small_ema_update_period);
        c.small_recent_window =
            read("legacy_predictor.small_recent_window", "", c.small_recent_window);
        c.small_speed_ema_alpha =
            read("legacy_predictor.small_speed_ema_alpha", "", c.small_speed_ema_alpha);
        c.small_smooth_ema_old =
            read("legacy_predictor.small_smooth_ema_old", "", c.small_smooth_ema_old);
        c.small_smooth_ema_new =
            read("legacy_predictor.small_smooth_ema_new", "", c.small_smooth_ema_new);
        c.mode_detector_window_samples =
            read("legacy_predictor.mode_detector_window_samples", "", c.mode_detector_window_samples);
        c.mode_detector_minimum_samples =
            read("legacy_predictor.mode_detector_minimum_samples", "", c.mode_detector_minimum_samples);
        c.mode_detector_evaluate_every =
            read("legacy_predictor.mode_detector_evaluate_every", "", c.mode_detector_evaluate_every);
        c.mode_detector_confirmations =
            read("legacy_predictor.mode_detector_confirmations", "", c.mode_detector_confirmations);
        c.mode_detector_max_sample_jump_ratio =
            read("legacy_predictor.mode_detector_max_sample_jump_ratio", "", c.mode_detector_max_sample_jump_ratio);
        c.mode_detector_big_threshold_ratio =
            read("legacy_predictor.mode_detector_big_threshold_ratio", "", c.mode_detector_big_threshold_ratio);
        c.mode_detector_small_threshold_ratio =
            read("legacy_predictor.mode_detector_small_threshold_ratio", "", c.mode_detector_small_threshold_ratio);

        detail::requireAtLeast(c.big_min_init_samples, 2, "legacy_predictor.big_min_init_samples", warnings);
        detail::requirePositive(c.big_const_level, 1.1775, "legacy_predictor.big_const_level", warnings);
        detail::requirePositive(c.big_const_omega, 1.924, "legacy_predictor.big_const_omega", warnings);
        detail::requirePositive(c.big_angular_speed, 2.090, "legacy_predictor.big_angular_speed", warnings);
        detail::requireAtLeast(c.ceres_trigger_factor, 1, "legacy_predictor.ceres_trigger_factor", warnings);
        detail::requirePositive(c.big_jump_ratio, 0.014, "legacy_predictor.big_jump_ratio", warnings);
        detail::requireAtLeast(c.big_moving_average_window, 1, "legacy_predictor.big_moving_average_window", warnings);
        detail::requireAtLeast(c.big_min_judge_samples, 2, "legacy_predictor.big_min_judge_samples", warnings);
        detail::clampTo(c.big_smooth_lookback_seconds, 0.0, 10.0, "legacy_predictor.big_smooth_lookback_seconds", warnings);
        detail::requireFinite(c.big_initial_phase, 0.75, "legacy_predictor.big_initial_phase", warnings);
        detail::requireAtLeast(c.ceres_max_num_iterations, 1, "legacy_predictor.ceres_max_num_iterations", warnings);
        detail::requirePositive(c.ceres_function_tolerance, 1.0e-15, "legacy_predictor.ceres_function_tolerance", warnings);
        detail::requirePositive(c.ceres_gradient_tolerance, 1.0e-15, "legacy_predictor.ceres_gradient_tolerance", warnings);
        detail::requirePositive(c.ceres_parameter_tolerance, 1.0e-15, "legacy_predictor.ceres_parameter_tolerance", warnings);
        detail::requireAtLeast(c.ceres_num_threads, 1, "legacy_predictor.ceres_num_threads", warnings);
        detail::requirePositive(c.big_fit_max_level, 1.310, "legacy_predictor.big_fit_max_level", warnings);
        detail::requirePositive(c.big_fit_max_omega, 2.000, "legacy_predictor.big_fit_max_omega", warnings);
        detail::requirePositive(c.big_fit_min_level, 1.045, "legacy_predictor.big_fit_min_level", warnings);
        detail::requirePositive(c.big_fit_min_omega, 1.884, "legacy_predictor.big_fit_min_omega", warnings);
        detail::requirePositive(c.big_forced_omega, 2.0, "legacy_predictor.big_forced_omega", warnings);
        detail::clampTo(c.big_extreme_wait_seconds, 0.0, 10.0, "legacy_predictor.big_extreme_wait_seconds", warnings);
        detail::requireAtLeast(c.big_vote_lookback_near, 1, "legacy_predictor.big_vote_lookback_near", warnings);
        detail::requireAtLeast(c.big_vote_lookback_mid, 1, "legacy_predictor.big_vote_lookback_mid", warnings);
        detail::requireAtLeast(c.big_vote_lookback_far, 1, "legacy_predictor.big_vote_lookback_far", warnings);
        detail::requireAtLeast(c.big_smooth_window, 1, "legacy_predictor.big_smooth_window", warnings);
        detail::requireAtLeast(c.small_min_init_samples, 2, "legacy_predictor.small_min_init_samples", warnings);
        detail::requirePositive(c.small_jump_ratio, 0.028, "legacy_predictor.small_jump_ratio", warnings);
        detail::requireAtLeast(c.small_init_timeout_frames, 1, "legacy_predictor.small_init_timeout_frames", warnings);
        detail::requirePositive(c.small_default_omega, 1.0471975511965976, "legacy_predictor.small_default_omega", warnings);
        detail::requireAtLeast(c.small_ema_update_period, 1, "legacy_predictor.small_ema_update_period", warnings);
        detail::requireAtLeast(c.small_recent_window, 1, "legacy_predictor.small_recent_window", warnings);
        detail::clampTo(c.small_speed_ema_alpha, 0.0, 1.0, "legacy_predictor.small_speed_ema_alpha", warnings);
        detail::clampTo(c.small_smooth_ema_old, 0.0, 1.0, "legacy_predictor.small_smooth_ema_old", warnings);
        detail::clampTo(c.small_smooth_ema_new, 0.0, 1.0, "legacy_predictor.small_smooth_ema_new", warnings);
        detail::requireAtLeast(c.mode_detector_window_samples, 1, "legacy_predictor.mode_detector_window_samples", warnings);
        detail::requireAtLeast(c.mode_detector_minimum_samples, 1, "legacy_predictor.mode_detector_minimum_samples", warnings);
        detail::requireAtLeast(c.mode_detector_evaluate_every, 1, "legacy_predictor.mode_detector_evaluate_every", warnings);
        detail::requireAtLeast(c.mode_detector_confirmations, 1, "legacy_predictor.mode_detector_confirmations", warnings);
        detail::requirePositive(c.mode_detector_max_sample_jump_ratio, 0.014, "legacy_predictor.mode_detector_max_sample_jump_ratio", warnings);
        detail::clampTo(c.mode_detector_big_threshold_ratio, 0.0, 10.0, "legacy_predictor.mode_detector_big_threshold_ratio", warnings);
        detail::clampTo(c.mode_detector_small_threshold_ratio, 0.0, 10.0, "legacy_predictor.mode_detector_small_threshold_ratio", warnings);
    }

    // ------------------------- big_ransac -------------------------
    {
        BigRansacConfig& c = out.big_ransac;
        c.enabled = read("big_ransac.enabled", "buff_big_ransac_enabled", c.enabled);
        c.max_iterations =
            read("big_ransac.max_iterations", "buff_big_ransac_max_iterations", c.max_iterations);
        c.threshold = read("big_ransac.threshold", "buff_big_ransac_threshold", c.threshold);
        c.min_omega = read("big_ransac.min_omega", "buff_big_ransac_min_omega", c.min_omega);
        c.max_omega = read("big_ransac.max_omega", "buff_big_ransac_max_omega", c.max_omega);
        c.min_amplitude =
            read("big_ransac.min_amplitude", "buff_big_ransac_min_amplitude", c.min_amplitude);
        c.max_amplitude =
            read("big_ransac.max_amplitude", "buff_big_ransac_max_amplitude", c.max_amplitude);
        c.min_inliers =
            read("big_ransac.min_inliers", "buff_big_ransac_min_inliers", c.min_inliers);
        c.max_abs_speed =
            read("big_ransac.max_abs_speed", "buff_big_ransac_max_abs_speed", c.max_abs_speed);
        c.max_gap_seconds =
            read("big_ransac.max_gap_seconds", "buff_big_ransac_max_gap_seconds", c.max_gap_seconds);
        c.max_consecutive_rejections =
            read("big_ransac.max_consecutive_rejections", "buff_big_ransac_max_consecutive_rejections", c.max_consecutive_rejections);
        c.random_seed = read("big_ransac.random_seed", "", c.random_seed);

        detail::requireAtLeast(c.max_iterations, 1, "big_ransac.max_iterations", warnings);
        detail::requirePositive(c.threshold, 0.5, "big_ransac.threshold", warnings);
        detail::requirePositive(c.min_omega, 1.884, "big_ransac.min_omega", warnings);
        detail::requirePositive(c.max_omega, 2.0, "big_ransac.max_omega", warnings);
        detail::requirePositive(c.min_amplitude, 0.780, "big_ransac.min_amplitude", warnings);
        detail::requirePositive(c.max_amplitude, 1.045, "big_ransac.max_amplitude", warnings);
        detail::requireAtLeast(c.min_inliers, 3, "big_ransac.min_inliers", warnings);
        detail::requirePositive(c.max_abs_speed, 3.2, "big_ransac.max_abs_speed", warnings);
        detail::requirePositive(c.max_gap_seconds, 5.0, "big_ransac.max_gap_seconds", warnings);
        detail::requireAtLeast(c.max_consecutive_rejections, 1, "big_ransac.max_consecutive_rejections", warnings);
    }

    // ------------------------- world_tracker -------------------------
    {
        WorldTrackerConfig& c = out.world_tracker;
        c.ekf_enabled =
            read("world_tracker.ekf_enabled", "buff_world_ekf_enabled", c.ekf_enabled);
        c.ekf_control_enabled =
            read("world_tracker.ekf_control_enabled", "buff_world_ekf_control_enabled", c.ekf_control_enabled);
        c.apply_gimbal_pose =
            read("world_tracker.apply_gimbal_pose", "buff_world_apply_gimbal_pose", c.apply_gimbal_pose);
        c.quaternion_is_imu_to_map =
            read("world_tracker.quaternion_is_imu_to_map", "buff_quaternion_is_imu_to_map", c.quaternion_is_imu_to_map);
        c.center_measurement_std_m =
            read("world_tracker.center_measurement_std_m", "buff_world_center_measurement_std_m", c.center_measurement_std_m);
        c.normal_measurement_std_rad =
            read("world_tracker.normal_measurement_std_rad", "buff_world_normal_measurement_std_rad", c.normal_measurement_std_rad);
        c.phase_measurement_std_rad =
            read("world_tracker.phase_measurement_std_rad", "buff_world_phase_measurement_std_rad", c.phase_measurement_std_rad);
        c.center_process_std_mps =
            read("world_tracker.center_process_std_mps", "buff_world_center_process_std_mps", c.center_process_std_mps);
        c.normal_process_std_radps =
            read("world_tracker.normal_process_std_radps", "buff_world_normal_process_std_radps", c.normal_process_std_radps);
        c.omega_process_std_radps2 =
            read("world_tracker.omega_process_std_radps2", "buff_world_omega_process_std_radps2", c.omega_process_std_radps2);
        c.min_omega = read("world_tracker.min_omega", "", c.min_omega);
        c.max_omega = read("world_tracker.max_omega", "", c.max_omega);
        c.direction_lock_omega =
            read("world_tracker.direction_lock_omega", "", c.direction_lock_omega);
        c.direction_lock_frames =
            read("world_tracker.direction_lock_frames", "", c.direction_lock_frames);
        c.ready_enter_frames =
            read("world_tracker.ready_enter_frames", "", c.ready_enter_frames);
        c.ready_exit_frames =
            read("world_tracker.ready_exit_frames", "", c.ready_exit_frames);
        c.residual_gate_sigma =
            read("world_tracker.residual_gate_sigma", "", c.residual_gate_sigma);
        c.phase_process_noise_floor =
            read("world_tracker.phase_process_noise_floor", "", c.phase_process_noise_floor);
        c.omega_init_abs = read("world_tracker.omega_init_abs", "", c.omega_init_abs);
        c.omega_ema_fast = read("world_tracker.omega_ema_fast", "", c.omega_ema_fast);
        c.omega_ema_slow = read("world_tracker.omega_ema_slow", "", c.omega_ema_slow);
        c.omega_ema_fast_frames =
            read("world_tracker.omega_ema_fast_frames", "", c.omega_ema_fast_frames);
        c.reverse_omega_reject_threshold =
            read("world_tracker.reverse_omega_reject_threshold", "", c.reverse_omega_reject_threshold);
        c.min_time_diff_seconds =
            read("world_tracker.min_time_diff_seconds", "", c.min_time_diff_seconds);
        c.omega_clamp_start_frames =
            read("world_tracker.omega_clamp_start_frames", "", c.omega_clamp_start_frames);
        c.min_radius_m = read("world_tracker.min_radius_m", "", c.min_radius_m);
        c.min_dt_seconds = read("world_tracker.min_dt_seconds", "", c.min_dt_seconds);
        c.max_dt_seconds = read("world_tracker.max_dt_seconds", "", c.max_dt_seconds);
        c.minimum_observations =
            read("world_tracker.minimum_observations", "", c.minimum_observations);
        c.max_prediction_seconds =
            read("world_tracker.max_prediction_seconds", "", c.max_prediction_seconds);
        c.normal_pitch_limit_deg =
            read("world_tracker.normal_pitch_limit_deg", "", c.normal_pitch_limit_deg);
        c.init_covariance_scale =
            read("world_tracker.init_covariance_scale", "", c.init_covariance_scale);

        detail::requirePositive(c.center_measurement_std_m, 0.015, "world_tracker.center_measurement_std_m", warnings);
        detail::requirePositive(c.normal_measurement_std_rad, 0.030, "world_tracker.normal_measurement_std_rad", warnings);
        detail::requirePositive(c.phase_measurement_std_rad, 0.040, "world_tracker.phase_measurement_std_rad", warnings);
        detail::requirePositive(c.center_process_std_mps, 0.010, "world_tracker.center_process_std_mps", warnings);
        detail::requirePositive(c.normal_process_std_radps, 0.010, "world_tracker.normal_process_std_radps", warnings);
        detail::requirePositive(c.omega_process_std_radps2, 0.250, "world_tracker.omega_process_std_radps2", warnings);
        detail::requirePositive(c.min_omega, 0.15, "world_tracker.min_omega", warnings);
        detail::requirePositive(c.max_omega, 1.571, "world_tracker.max_omega", warnings);
        detail::requirePositive(c.direction_lock_omega, 0.25, "world_tracker.direction_lock_omega", warnings);
        detail::requireAtLeast(c.direction_lock_frames, 1, "world_tracker.direction_lock_frames", warnings);
        detail::requireAtLeast(c.ready_enter_frames, 1, "world_tracker.ready_enter_frames", warnings);
        detail::requireAtLeast(c.ready_exit_frames, 1, "world_tracker.ready_exit_frames", warnings);
        detail::requirePositive(c.residual_gate_sigma, 3.5, "world_tracker.residual_gate_sigma", warnings);
        detail::requirePositive(c.phase_process_noise_floor, 1.0e-5, "world_tracker.phase_process_noise_floor", warnings);
        detail::requirePositive(c.omega_init_abs, 0.9, "world_tracker.omega_init_abs", warnings);
        detail::clampTo(c.omega_ema_fast, 0.0, 1.0, "world_tracker.omega_ema_fast", warnings);
        detail::clampTo(c.omega_ema_slow, 0.0, 1.0, "world_tracker.omega_ema_slow", warnings);
        detail::requireAtLeast(c.omega_ema_fast_frames, 1, "world_tracker.omega_ema_fast_frames", warnings);
        detail::requirePositive(c.reverse_omega_reject_threshold, 0.30, "world_tracker.reverse_omega_reject_threshold", warnings);
        detail::requirePositive(c.min_time_diff_seconds, 0.02, "world_tracker.min_time_diff_seconds", warnings);
        detail::requireAtLeast(c.omega_clamp_start_frames, 1, "world_tracker.omega_clamp_start_frames", warnings);
        detail::requirePositive(c.min_radius_m, 0.05, "world_tracker.min_radius_m", warnings);
        detail::requirePositive(c.min_dt_seconds, 1.0e-4, "world_tracker.min_dt_seconds", warnings);
        detail::requirePositive(c.max_dt_seconds, 1.0, "world_tracker.max_dt_seconds", warnings);
        detail::requireAtLeast(c.minimum_observations, 1, "world_tracker.minimum_observations", warnings);
        detail::requirePositive(c.max_prediction_seconds, 1.0, "world_tracker.max_prediction_seconds", warnings);
        detail::clampTo(c.normal_pitch_limit_deg, 0.0, 90.0, "world_tracker.normal_pitch_limit_deg", warnings);
        detail::requirePositive(c.init_covariance_scale, 1.0e3, "world_tracker.init_covariance_scale", warnings);
    }

    return true;
}

// ---------------------------------------------------------------------------
//  启动打印: 返回 "Buff.<section>.<key> = <value>" 逐行文本
//  调用方决定用什么方式输出 (Fu.cpp 用 ROS_INFO), 本函数不依赖 ROS。
// ---------------------------------------------------------------------------
inline std::vector<std::string> buffConfigToLines(const BuffConfig& cfg) {
    std::vector<std::string> lines;
    const auto add = [&lines](const std::string& key, const auto& value) {
        std::ostringstream out;
        out << "Buff." << key << " = " << value;
        lines.push_back(out.str());
    };

    add("sync.imu_pose_history_capacity", cfg.sync.imu_pose_history_capacity);
    add("sync.max_imu_image_skew_seconds",
        detail::formatNumber(cfg.sync.max_imu_image_skew_seconds));
    add("sync.imu_stamp_offset_seconds",
        detail::formatNumber(cfg.sync.imu_stamp_offset_seconds));
    add("sync.quaternion_norm_min", detail::formatNumber(cfg.sync.quaternion_norm_min));
    add("sync.quaternion_norm_max", detail::formatNumber(cfg.sync.quaternion_norm_max));

    add("timing.delta_t_init", detail::formatNumber(cfg.timing.delta_t_init));
    add("timing.delta_t_compensation",
        detail::formatNumber(cfg.timing.delta_t_compensation));
    add("timing.delta_t_min", detail::formatNumber(cfg.timing.delta_t_min));
    add("timing.delta_t_max", detail::formatNumber(cfg.timing.delta_t_max));
    add("timing.flight_time_iterations", cfg.timing.flight_time_iterations);
    add("timing.flight_time_valid_min",
        detail::formatNumber(cfg.timing.flight_time_valid_min));
    add("timing.flight_time_valid_max",
        detail::formatNumber(cfg.timing.flight_time_valid_max));
    add("timing.lead_ema_old", detail::formatNumber(cfg.timing.lead_ema_old));
    add("timing.lead_ema_new", detail::formatNumber(cfg.timing.lead_ema_new));
    add("timing.max_coast_frames", cfg.timing.max_coast_frames);
    add("timing.fallback_fps", detail::formatNumber(cfg.timing.fallback_fps));
    add("timing.fallback_fps_ema_old",
        detail::formatNumber(cfg.timing.fallback_fps_ema_old));
    add("timing.fallback_fps_ema_new",
        detail::formatNumber(cfg.timing.fallback_fps_ema_new));
    add("timing.fps_interval_max_ms",
        detail::formatNumber(cfg.timing.fps_interval_max_ms));
    add("timing.angle_cache_len", cfg.timing.angle_cache_len);
    add("timing.default_omega", detail::formatNumber(cfg.timing.default_omega));
    add("timing.kalman_bad_frames", cfg.timing.kalman_bad_frames);
    add("timing.divergence_min_radius_px",
        detail::formatNumber(cfg.timing.divergence_min_radius_px));
    add("timing.divergence_radius_ratio_min",
        detail::formatNumber(cfg.timing.divergence_radius_ratio_min));
    add("timing.divergence_radius_ratio_max",
        detail::formatNumber(cfg.timing.divergence_radius_ratio_max));

    add("control.pitch_compensation",
        detail::formatNumber(cfg.control.pitch_compensation));
    add("control.yaw_compensation", detail::formatNumber(cfg.control.yaw_compensation));
    add("control.gimbal_pose_alpha", detail::formatNumber(cfg.control.gimbal_pose_alpha));
    add("control.pitch_max_deg", detail::formatNumber(cfg.control.pitch_max_deg));
    add("control.pitch_min_deg", detail::formatNumber(cfg.control.pitch_min_deg));
    add("control.stuck_threshold_deg",
        detail::formatNumber(cfg.control.stuck_threshold_deg));
    add("control.stuck_frames", cfg.control.stuck_frames);
    add("control.recovery_pitch_deg",
        detail::formatNumber(cfg.control.recovery_pitch_deg));
    add("control.recovery_frames", cfg.control.recovery_frames);
    add("control.log_throttle_s", detail::formatNumber(cfg.control.log_throttle_s));

    add("ballistic.default_velocity",
        detail::formatNumber(cfg.ballistic.default_velocity));
    add("ballistic.velocity_min_valid",
        detail::formatNumber(cfg.ballistic.velocity_min_valid));
    add("ballistic.gravity", detail::formatNumber(cfg.ballistic.gravity));
    add("ballistic.iteration_times", cfg.ballistic.iteration_times);
    add("ballistic.converge_epsilon",
        detail::formatNumber(cfg.ballistic.converge_epsilon));
    add("ballistic.height_tolerance",
        detail::formatNumber(cfg.ballistic.height_tolerance));
    add("ballistic.angle_limit_deg",
        detail::formatNumber(cfg.ballistic.angle_limit_deg));
    add("ballistic.phase_probe_rad",
        detail::formatNumber(cfg.ballistic.phase_probe_rad));
    add("ballistic.probe_valid_rad",
        detail::formatNumber(cfg.ballistic.probe_valid_rad));
    add("ballistic.physical_phase_limit_rad",
        detail::formatNumber(cfg.ballistic.physical_phase_limit_rad));
    add("ballistic.max_reprojection_error_px",
        detail::formatNumber(cfg.ballistic.max_reprojection_error_px));
    add("ballistic.reprojection_check_enabled",
        cfg.ballistic.reprojection_check_enabled ? "true" : "false");
    add("ballistic.optical_center_offset_x",
        detail::formatNumber(cfg.ballistic.optical_center_offset_x));
    add("ballistic.optical_center_offset_y",
        detail::formatNumber(cfg.ballistic.optical_center_offset_y));

    add("model.model_path", cfg.model.model_path);
    add("model.enemy_color", cfg.model.enemy_color);
    add("model.move_mode", cfg.model.move_mode);
    add("model.run_without_serial", cfg.model.run_without_serial ? "true" : "false");
    add("model.functional_default", cfg.model.functional_default ? "true" : "false");
    add("model.input_size", cfg.model.input_size);
    add("model.model_device", cfg.model.model_device);

    add("recognition.conf_threshold", detail::formatNumber(cfg.recognition.conf_threshold));
    add("recognition.nms_threshold", detail::formatNumber(cfg.recognition.nms_threshold));
    add("recognition.binary_threshold", cfg.recognition.binary_threshold);
    add("recognition.dilate_kernel_size", cfg.recognition.dilate_kernel_size);
    add("recognition.r_center_mask_ratio", detail::formatNumber(cfg.recognition.r_center_mask_ratio));
    add("recognition.r_center_contour_div", detail::formatNumber(cfg.recognition.r_center_contour_div));
    add("recognition.r_center_refine_max_shift", detail::formatNumber(cfg.recognition.r_center_refine_max_shift));
    add("recognition.box_alpha", detail::formatNumber(cfg.recognition.box_alpha));
    add("recognition.stabilizer_alpha", detail::formatNumber(cfg.recognition.stabilizer_alpha));
    add("recognition.assoc_gate_min_px", detail::formatNumber(cfg.recognition.assoc_gate_min_px));
    add("recognition.assoc_gate_radius_ratio", detail::formatNumber(cfg.recognition.assoc_gate_radius_ratio));
    add("recognition.jump_fan_radius_ratio", detail::formatNumber(cfg.recognition.jump_fan_radius_ratio));
    add("recognition.jump_r_radius_ratio", detail::formatNumber(cfg.recognition.jump_r_radius_ratio));
    add("recognition.radius_change_reject_ratio", detail::formatNumber(cfg.recognition.radius_change_reject_ratio));
    add("recognition.side_change_reject_ratio", detail::formatNumber(cfg.recognition.side_change_reject_ratio));
    add("recognition.jump_confirm_frames", cfg.recognition.jump_confirm_frames);
    add("recognition.reacquire_confirm_frames", cfg.recognition.reacquire_confirm_frames);
    add("recognition.miss_reset_frames", cfg.recognition.miss_reset_frames);
    add("recognition.min_valid_radius_px", detail::formatNumber(cfg.recognition.min_valid_radius_px));
    add("recognition.ego_motion_max_delta_rad", detail::formatNumber(cfg.recognition.ego_motion_max_delta_rad));

    add("geometry.target_width_mm", detail::formatNumber(cfg.geometry.target_width_mm));
    add("geometry.target_height_mm", detail::formatNumber(cfg.geometry.target_height_mm));
    add("geometry.camera_matrix", detail::formatVector(cfg.geometry.camera_matrix));
    add("geometry.dist_coeffs", detail::formatVector(cfg.geometry.dist_coeffs));
    add("geometry.radius_mm", detail::formatNumber(cfg.geometry.radius_mm));
    add("geometry.hub_normal_offset_mm", detail::formatNumber(cfg.geometry.hub_normal_offset_mm));

    add("angle.blade_count", cfg.angle.blade_count);
    add("angle.direction_judge_samples", cfg.angle.direction_judge_samples);
    add("angle.wrap_period", detail::formatNumber(cfg.angle.wrap_period));
    add("angle.unwrap_tolerance_divisor", detail::formatNumber(cfg.angle.unwrap_tolerance_divisor));
    add("angle.diff_search_start_ratio", detail::formatNumber(cfg.angle.diff_search_start_ratio));
    add("angle.diff_timeout_margin_ms", detail::formatNumber(cfg.angle.diff_timeout_margin_ms));
    add("angle.diff_angle_limit_rad", detail::formatNumber(cfg.angle.diff_angle_limit_rad));
    add("angle.direction_filter_ratio", detail::formatNumber(cfg.angle.direction_filter_ratio));
    add("angle.angle_buffer_len", cfg.angle.angle_buffer_len);
    add("angle.sector_search_range", cfg.angle.sector_search_range);
    add("angle.image_width", cfg.angle.image_width);
    add("angle.image_height", cfg.angle.image_height);
    add("angle.small_buff_converge_delay", cfg.angle.small_buff_converge_delay);
    add("angle.small_buff_max_roll_residual", detail::formatNumber(cfg.angle.small_buff_max_roll_residual));
    add("angle.small_buff_min_speed", detail::formatNumber(cfg.angle.small_buff_min_speed));
    add("angle.small_buff_max_speed", detail::formatNumber(cfg.angle.small_buff_max_speed));
    add("angle.small_buff_phase_process_noise", detail::formatNumber(cfg.angle.small_buff_phase_process_noise));
    add("angle.small_buff_phase_measurement_noise", detail::formatNumber(cfg.angle.small_buff_phase_measurement_noise));
    add("angle.small_buff_min_radius_px", detail::formatNumber(cfg.angle.small_buff_min_radius_px));
    add("angle.small_buff_max_dt", detail::formatNumber(cfg.angle.small_buff_max_dt));
    add("angle.small_buff_reject_reset_count", cfg.angle.small_buff_reject_reset_count);
    add("angle.small_buff_vote_min_phase_delta", detail::formatNumber(cfg.angle.small_buff_vote_min_phase_delta));
    add("angle.small_buff_vote_saturation", cfg.angle.small_buff_vote_saturation);
    add("angle.small_buff_omega_fit_min_samples", cfg.angle.small_buff_omega_fit_min_samples);
    add("angle.small_buff_radius_ema", detail::formatNumber(cfg.angle.small_buff_radius_ema));
    add("angle.small_buff_center_ema", detail::formatNumber(cfg.angle.small_buff_center_ema));
    add("angle.small_buff_center_snap_ratio",
        detail::formatNumber(cfg.angle.small_buff_center_snap_ratio));
    add("angle.small_buff_omega_reset_cov", detail::formatNumber(cfg.angle.small_buff_omega_reset_cov));
    add("angle.small_buff_omega_locked_cov", detail::formatNumber(cfg.angle.small_buff_omega_locked_cov));
    add("angle.small_buff_init_cov_phase", detail::formatNumber(cfg.angle.small_buff_init_cov_phase));
    add("angle.small_buff_init_cov_omega", detail::formatNumber(cfg.angle.small_buff_init_cov_omega));
    add("angle.small_buff_phase_history_max", cfg.angle.small_buff_phase_history_max);

    add("legacy_predictor.big_min_init_samples", cfg.legacy_predictor.big_min_init_samples);
    add("legacy_predictor.big_const_level", detail::formatNumber(cfg.legacy_predictor.big_const_level));
    add("legacy_predictor.big_const_omega", detail::formatNumber(cfg.legacy_predictor.big_const_omega));
    add("legacy_predictor.big_angular_speed", detail::formatNumber(cfg.legacy_predictor.big_angular_speed));
    add("legacy_predictor.ceres_trigger_factor", cfg.legacy_predictor.ceres_trigger_factor);
    add("legacy_predictor.big_jump_ratio", detail::formatNumber(cfg.legacy_predictor.big_jump_ratio));
    add("legacy_predictor.big_moving_average_window", cfg.legacy_predictor.big_moving_average_window);
    add("legacy_predictor.big_min_judge_samples", cfg.legacy_predictor.big_min_judge_samples);
    add("legacy_predictor.big_smooth_lookback_seconds", detail::formatNumber(cfg.legacy_predictor.big_smooth_lookback_seconds));
    add("legacy_predictor.big_initial_phase", detail::formatNumber(cfg.legacy_predictor.big_initial_phase));
    add("legacy_predictor.ceres_max_num_iterations", cfg.legacy_predictor.ceres_max_num_iterations);
    add("legacy_predictor.ceres_function_tolerance", detail::formatNumber(cfg.legacy_predictor.ceres_function_tolerance));
    add("legacy_predictor.ceres_gradient_tolerance", detail::formatNumber(cfg.legacy_predictor.ceres_gradient_tolerance));
    add("legacy_predictor.ceres_parameter_tolerance", detail::formatNumber(cfg.legacy_predictor.ceres_parameter_tolerance));
    add("legacy_predictor.ceres_num_threads", cfg.legacy_predictor.ceres_num_threads);
    add("legacy_predictor.big_fit_max_level", detail::formatNumber(cfg.legacy_predictor.big_fit_max_level));
    add("legacy_predictor.big_fit_max_omega", detail::formatNumber(cfg.legacy_predictor.big_fit_max_omega));
    add("legacy_predictor.big_fit_min_level", detail::formatNumber(cfg.legacy_predictor.big_fit_min_level));
    add("legacy_predictor.big_fit_min_omega", detail::formatNumber(cfg.legacy_predictor.big_fit_min_omega));
    add("legacy_predictor.big_forced_omega", detail::formatNumber(cfg.legacy_predictor.big_forced_omega));
    add("legacy_predictor.big_extreme_wait_seconds", detail::formatNumber(cfg.legacy_predictor.big_extreme_wait_seconds));
    add("legacy_predictor.big_vote_lookback_near", cfg.legacy_predictor.big_vote_lookback_near);
    add("legacy_predictor.big_vote_lookback_mid", cfg.legacy_predictor.big_vote_lookback_mid);
    add("legacy_predictor.big_vote_lookback_far", cfg.legacy_predictor.big_vote_lookback_far);
    add("legacy_predictor.big_smooth_window", cfg.legacy_predictor.big_smooth_window);
    add("legacy_predictor.small_min_init_samples", cfg.legacy_predictor.small_min_init_samples);
    add("legacy_predictor.small_jump_ratio", detail::formatNumber(cfg.legacy_predictor.small_jump_ratio));
    add("legacy_predictor.small_init_timeout_frames", cfg.legacy_predictor.small_init_timeout_frames);
    add("legacy_predictor.small_default_omega", detail::formatNumber(cfg.legacy_predictor.small_default_omega));
    add("legacy_predictor.small_ema_update_period", cfg.legacy_predictor.small_ema_update_period);
    add("legacy_predictor.small_recent_window", cfg.legacy_predictor.small_recent_window);
    add("legacy_predictor.small_speed_ema_alpha", detail::formatNumber(cfg.legacy_predictor.small_speed_ema_alpha));
    add("legacy_predictor.small_smooth_ema_old", detail::formatNumber(cfg.legacy_predictor.small_smooth_ema_old));
    add("legacy_predictor.small_smooth_ema_new", detail::formatNumber(cfg.legacy_predictor.small_smooth_ema_new));
    add("legacy_predictor.mode_detector_window_samples", cfg.legacy_predictor.mode_detector_window_samples);
    add("legacy_predictor.mode_detector_minimum_samples", cfg.legacy_predictor.mode_detector_minimum_samples);
    add("legacy_predictor.mode_detector_evaluate_every", cfg.legacy_predictor.mode_detector_evaluate_every);
    add("legacy_predictor.mode_detector_confirmations", cfg.legacy_predictor.mode_detector_confirmations);
    add("legacy_predictor.mode_detector_max_sample_jump_ratio", detail::formatNumber(cfg.legacy_predictor.mode_detector_max_sample_jump_ratio));
    add("legacy_predictor.mode_detector_big_threshold_ratio", detail::formatNumber(cfg.legacy_predictor.mode_detector_big_threshold_ratio));
    add("legacy_predictor.mode_detector_small_threshold_ratio", detail::formatNumber(cfg.legacy_predictor.mode_detector_small_threshold_ratio));

    add("big_ransac.enabled", cfg.big_ransac.enabled ? "true" : "false");
    add("big_ransac.max_iterations", cfg.big_ransac.max_iterations);
    add("big_ransac.threshold", detail::formatNumber(cfg.big_ransac.threshold));
    add("big_ransac.min_omega", detail::formatNumber(cfg.big_ransac.min_omega));
    add("big_ransac.max_omega", detail::formatNumber(cfg.big_ransac.max_omega));
    add("big_ransac.min_amplitude", detail::formatNumber(cfg.big_ransac.min_amplitude));
    add("big_ransac.max_amplitude", detail::formatNumber(cfg.big_ransac.max_amplitude));
    add("big_ransac.min_inliers", cfg.big_ransac.min_inliers);
    add("big_ransac.max_abs_speed", detail::formatNumber(cfg.big_ransac.max_abs_speed));
    add("big_ransac.max_gap_seconds", detail::formatNumber(cfg.big_ransac.max_gap_seconds));
    add("big_ransac.max_consecutive_rejections", cfg.big_ransac.max_consecutive_rejections);
    add("big_ransac.random_seed", cfg.big_ransac.random_seed);

    add("world_tracker.ekf_enabled", cfg.world_tracker.ekf_enabled ? "true" : "false");
    add("world_tracker.ekf_control_enabled", cfg.world_tracker.ekf_control_enabled ? "true" : "false");
    add("world_tracker.apply_gimbal_pose", cfg.world_tracker.apply_gimbal_pose ? "true" : "false");
    add("world_tracker.quaternion_is_imu_to_map", cfg.world_tracker.quaternion_is_imu_to_map ? "true" : "false");
    add("world_tracker.center_measurement_std_m", detail::formatNumber(cfg.world_tracker.center_measurement_std_m));
    add("world_tracker.normal_measurement_std_rad", detail::formatNumber(cfg.world_tracker.normal_measurement_std_rad));
    add("world_tracker.phase_measurement_std_rad", detail::formatNumber(cfg.world_tracker.phase_measurement_std_rad));
    add("world_tracker.center_process_std_mps", detail::formatNumber(cfg.world_tracker.center_process_std_mps));
    add("world_tracker.normal_process_std_radps", detail::formatNumber(cfg.world_tracker.normal_process_std_radps));
    add("world_tracker.omega_process_std_radps2", detail::formatNumber(cfg.world_tracker.omega_process_std_radps2));
    add("world_tracker.min_omega", detail::formatNumber(cfg.world_tracker.min_omega));
    add("world_tracker.max_omega", detail::formatNumber(cfg.world_tracker.max_omega));
    add("world_tracker.direction_lock_omega", detail::formatNumber(cfg.world_tracker.direction_lock_omega));
    add("world_tracker.direction_lock_frames", cfg.world_tracker.direction_lock_frames);
    add("world_tracker.ready_enter_frames", cfg.world_tracker.ready_enter_frames);
    add("world_tracker.ready_exit_frames", cfg.world_tracker.ready_exit_frames);
    add("world_tracker.residual_gate_sigma", detail::formatNumber(cfg.world_tracker.residual_gate_sigma));
    add("world_tracker.phase_process_noise_floor", detail::formatNumber(cfg.world_tracker.phase_process_noise_floor));
    add("world_tracker.omega_init_abs", detail::formatNumber(cfg.world_tracker.omega_init_abs));
    add("world_tracker.omega_ema_fast", detail::formatNumber(cfg.world_tracker.omega_ema_fast));
    add("world_tracker.omega_ema_slow", detail::formatNumber(cfg.world_tracker.omega_ema_slow));
    add("world_tracker.omega_ema_fast_frames", cfg.world_tracker.omega_ema_fast_frames);
    add("world_tracker.reverse_omega_reject_threshold", detail::formatNumber(cfg.world_tracker.reverse_omega_reject_threshold));
    add("world_tracker.min_time_diff_seconds", detail::formatNumber(cfg.world_tracker.min_time_diff_seconds));
    add("world_tracker.omega_clamp_start_frames", cfg.world_tracker.omega_clamp_start_frames);
    add("world_tracker.min_radius_m", detail::formatNumber(cfg.world_tracker.min_radius_m));
    add("world_tracker.min_dt_seconds", detail::formatNumber(cfg.world_tracker.min_dt_seconds));
    add("world_tracker.max_dt_seconds", detail::formatNumber(cfg.world_tracker.max_dt_seconds));
    add("world_tracker.minimum_observations", cfg.world_tracker.minimum_observations);
    add("world_tracker.max_prediction_seconds", detail::formatNumber(cfg.world_tracker.max_prediction_seconds));
    add("world_tracker.normal_pitch_limit_deg", detail::formatNumber(cfg.world_tracker.normal_pitch_limit_deg));
    add("world_tracker.init_covariance_scale", detail::formatNumber(cfg.world_tracker.init_covariance_scale));

    return lines;
}

}  // namespace buff
