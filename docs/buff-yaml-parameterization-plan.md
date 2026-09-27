# 能量机关（Buff）参数 YAML 化改造方案

> 状态：**P1–P6 已全部实现并本地验证通过（11 节参数全部落地 + 全部模块接线完成）**
> 目标：借鉴自瞄（装甲板）的 YAML 配置方式，把能量机关**几乎所有可调参数**集中到 YAML，调到参数只改文件、不翻代码。
>
> **已确认的三项决策**
> 1. 配置文件形态：**扩展 `rm_fu.yaml`，新增结构化 `Buff:` 节**（旧扁平 `buff_*` 键作为过渡覆盖层）。
> 2. 加载机制：**`yaml-cpp` 直读 `fu_config_path`，与自瞄 `setParam()` 同风格**（不再逐键 `private_nh.param`）。
> 3. 落地范围：**全部 11 节 + 全部模块接线**（超出最初 P1–P4 的范围，按"一起完整参数化"的要求扩展到
>    识别器/角度/小符卡尔曼/旧 FFT-Ceres 预测器/大小符判别/世界 EKF）。
>
> **交付物一览**
> | 文件 | 内容 |
> |---|---|
> | `src/rm_tracker/include/buff_config.hpp` | 11 节结构体 + ROS 无关的加载/校验/打印（~1580 行） |
> | `src/rm_tracker/config/rm_fu.yaml` | `Buff:` 节全部参数（~200 键，旧扁平键保留作过渡层） |
> | `src/rm_tracker/src/Fu.cpp` | 删掉 ~90 行 `private_nh.param`，改为 cfg 派生 + 分发到各模块 |
> | `src/rm_tracker/include/buff_node_runtime.hpp` | sync/timing/control 常量 → `g_buff_config` |
> | `src/rm_tracker/include/buff_pnp_solver.hpp` | ballistic + 云台位姿 EMA + 光心偏移 |
> | `src/rm_tracker/{include,src}/buff_recognizer.*`、`buff_detection_stabilizer.hpp` | recognition（阈值/精化/跳变/关联） |
> | `src/rm_tracker/include/buff_angle_processor.hpp`、`blade_tracker.hpp` | angle（极坐标/差分/方向 + 小符像素卡尔曼） |
> | `src/rm_tracker/include/buff_motion_predictor.hpp`、`buff_move_mode_detector.hpp` | legacy_predictor + 大小符判别 |
> | `src/rm_tracker/include/buff_world_tracker.hpp` | world_tracker（野值 σ/方向锁/ready 滞回/ω 注入等） |
> | 测试 | `buff_config_test.cpp`（默认值护栏 + 真实 yaml 零警告）、`test_buff_config_consistency.py`（结构 + yaml 键对齐守卫） |
>
> **本地验证结果**：`g++ -std=c++17 -Wall -Wextra` 语法检查零警告；一致性守卫 8 项全过（含
> yaml 键与 loader 的 `read()` 完全对齐、无重复/无拼写错误）；`buff_detection_stabilizer_test`、
> `buff_move_mode_detector_test` 实编实跑通过。
> ⚠️ 依赖 ROS/OpenCV/Eigen 的模块（Fu/PnP/角度/预测/世界EKF）**本机无法编译**，改动是"常量 → 配置项"的
> 机械替换且默认值严格一致，但务必在 Ubuntu 上 `catkin_make --pkg rm_tracker` + `ctest` 过一遍再上车。
> ⚠️ 顺带修好了本就失败的过期测试 `buff_detection_stabilizer_test.cpp`（4 处断言描述的是修复前的旧行为）。

---

## 0. 结论速览

| 项目 | 现状 | 改造后 |
|---|---|---|
| 配置文件 | `src/rm_tracker/config/rm_fu.yaml`（只有 ~25 个扁平 `buff_*` 键，覆盖 EKF / RANSAC / PnP 标定） | 同一个文件，新增结构化 `Buff:` 大节，**~180 个参数**全部在内 |
| 加载方式 | `private_nh.param<T>()` 逐键读取（约 90 行散在 `Fu.cpp` 里） | 新增 `buff_config.hpp` + `loadBuffConfig()`，`yaml-cpp` 解析 + 校验 + 启动打印（**与自瞄 `setParam()` 同风格**） |
| 硬编码参数 | 分散在 8 个头文件与 `Fu.cpp` / `buff_recognizer.cpp`，其中约 150 处是纯字面量，改一次要重编译 | 全部进 YAML；代码里只保留"默认值 + 取值域 clamp" |
| 行为 | — | **不改算法、不改默认值**（默认值 = 迁移前的硬编码值），纯搬运 |
| 编译验证 | 本机是 Windows，无 ROS/catkin | 需在 Ubuntu 上 `catkin_make --pkg rm_tracker` + `ctest`；本机可跑 Python 静态守卫测试 |

**先做这一步的原因**：`Fu_yuan.cpp`（105 KB 老代码）**是死代码**，没有进 `CMakeLists.txt`、没有任何 `#include`、build 树里零引用。真正要改的是 `src/Fu.cpp` + 6 个头文件。见 §6.1。

---

## 2. 参照对象：自瞄是怎麼做的

自瞄链路（装甲板）的配置方式已经成熟，三要素如下：

1. **一个 YAML 文件 + 结构化分节**
   `src/rm_tracker/config/rm_tracker.yaml`：`Tracker:` / `ModelObserver:`（下挂 `Standard` / `Balance` / `Outpost`）/ `MPC:` / `Planner:` / `Camera:` / `InstallOffset:`。
2. **一个 `setParam(path)` 入口，用 yaml-cpp 直接解析**
   `RmTracker::setParam()`（`include/RmTracker.hpp:1504`）、`ModelObserver::setParam()`（`include/ModelObserver.hpp:323`）、`CoordinateTransformer::setparam()`（`include/CoorConverter.cpp:106`）：
   ```cpp
   YAML::Node config = YAML::LoadFile(file_path);
   YAML::Node track = config["Tracker"];
   COMMAND_TIMESPAN = track["COMMAND_TIMESPAN"].as<double>(0.11);
   pitch_compensation = track["pitch_compensation"].as<double>(0);
   ```
   特点：**键不存在就用默认值**（第二参数），解析异常有完整错误处理，加载后 `ROS_WARN` 打印关键值，方便确认"参数到底生效没有"。
3. **launch 只传路径**
   `<param name="armor_tracker_config" value="$(find rm_tracker)/config/rm_tracker.yaml"/>`，节点自己解析文件。

**能量机关这一侧现状**：`rm_fu.launch` 已经做了第 3 步（`fu_config_path` + `<rosparam command="load">`），`Fu.cpp` 也已经读了 ~25 个键（第 1 步做了一半），但**没有统一的 Config 结构和统一入口**，剩下的 150 处参数还散在代码里（第 2 步缺失）。所以本次改造 = **补齐第 2 步 + 把剩下的参数搬进第 1 步的文件**。

---

## 3. 参数分布现状（改动范围盘点）

| 模块 | 文件 | 参数大致数量 | 现在能不能调 |
|---|---|---|---|
| 节点/运行时（时延、云台限幅、卡死恢复、兜底策略） | `include/buff_node_runtime.hpp`、`src/Fu.cpp` | ~25 | ❌ 全靠重编译 |
| 识别（YOLO 阈值、R 中心精化、时序关联/跳变） | `src/buff_recognizer.cpp`、`include/buff_recognizer.hpp`、`include/buff_detection_stabilizer.hpp` | ~35 | ❌ 0 个 ROS 参数 |
| 极坐标/角度差分（`Fu_angleprocess`） | `include/buff_angle_processor.hpp` | ~20 | ❌ |
| 小符像素卡尔曼（`SmallBuffPhaseTracker`） | `include/blade_tracker.hpp` | ~20 | ❌ 全是 `static constexpr` |
| 大符预测（`buff_predicted`，FFT+Ceres） | `include/buff_motion_predictor.hpp` | ~45 | ❌ |
| 大小符自动判别（`MoveModeDetector`） | `include/buff_move_mode_detector.hpp` | ~10 | ❌ 7 个 `static constexpr` |
| PnP/弹道/重力补偿 | `include/buff_pnp_solver.hpp` | ~20 | ❌（弹速除外，可被串口覆盖） |
| 世界系 EKF | `include/buff_world_tracker.hpp` | ~30 | ⚠️ 只有 8 个噪声/尺寸参数可调，**野值门限 3.5σ、方向锁、ready 滞回、EMA 全写死** |
| 大符 RANSAC | `include/buff_big_ransac_predictor.hpp`、`buff_zju_ransac.*` | 11 | ✅ 10/11 已可调（但 `buff_big_ransac_enabled: false` 把整条链关了） |
| 检测阶段模型（`Yolov8_Fan.h` / `Yolov8_R.h`） | 未编译进 Fu（见 §6.2） | ~15 | — |
| 死代码 | `src/Fu_yuan.cpp`、`src/tracker_yuan.cpp` | ~150 | 不迁移 |

---

## 4. 方案设计

### 4.1 加载架构（新增 1 个文件，不动算法）

新建 `src/rm_tracker/include/buff_config.hpp`：

```cpp
namespace buff {

// ---- 每个模块一个结构体，字段默认值 == 迁移前的硬编码值 ----
struct DetectConfig  { float conf_threshold = 0.2f; ... };   // 识别
struct AngleConfig   { int   direction_judge_samples = 21; ... };
struct SmallBuffConfig { ... };                              // 小符卡尔曼
struct BigBuffConfig   { ... };                              // 大符 FFT/Ceres + RANSAC
struct BallisticConfig { ... };                              // 弹道/PnP
struct TimingConfig    { ... };                              // 时延/提前量
struct ControlConfig   { ... };                              // 云台限幅/补偿/恢复
struct DebugConfig     { ... };

struct BuffConfig {
    YAML::Node raw;                    // 保留原始节点，一次解析多次取用
    DetectConfig    detect;
    AngleConfig     angle;
    SmallBuffConfig small_buff;
    BigBuffConfig   big_buff;
    BallisticConfig ballistic;
    TimingConfig    timing;
    ControlConfig   control;
    DebugConfig     debug;
    BuffWorldTrackerConfig            world;   // 复用已有结构体
    buff_zju::BigBuffRansacConfig     ransac;  // 复用已有结构体
};

// 唯一入口：与自瞄 setParam 同风格
bool loadBuffConfig(const std::string& path, BuffConfig& out, std::string& error);
void logBuffConfig(const BuffConfig& cfg);   // 启动打印全量参数快照
}  // namespace buff
```

`loadBuffConfig()` 职责（照抄自瞄的成熟做法）：
1. `YAML::LoadFile` + `BadFile` / `ParserException` / `YAML::Exception` 三层错误处理；
2. 取 `config["Buff"]` 各子节点，逐键 `as<T>(默认值)` —— **默认值就是现在的硬编码值**；
3. 取值域校验（clamp + 非法值回退 + `ROS_WARN`），复用已有的 `sanitize_big_buff_ransac_config` 思路；
4. `logBuffConfig()` 分节打印全部参数（自瞄 `setParam` 里也是这么干的），调参时能直接在终端确认"我改的值生效了吗"。

`Fu.cpp` 里：
```cpp
buff::BuffConfig g_buff_config;
// ...
if (!buff::loadBuffConfig(fu_config_path, g_buff_config, err)) { ROS_FATAL(...); return 1; }
buff::logBuffConfig(g_buff_config);
// 分发到各模块
g_recognizer->configure(g_buff_config.detect);
Angleprocess->configure(g_buff_config.angle);
pnpSolver_gravity->configure(g_buff_config.ballistic);
g_small_buff_tracker.configure(g_buff_config.small_buff);
Predicted->configure(g_buff_config.big_buff);
g_buff_move_mode_detector.configure(g_buff_config.big_buff.mode_detector);
```

### 4.2 三种代码改造模式（按类型选，避免大改）

| 模式 | 适用 | 做法 |
|---|---|---|
| **A. 新建 Config 结构体 + `configure()`** | 有类实例的模块（识别器、`Fu_angleprocess`、`PnPSolver_gravity`、`buff_predicted`） | 加 `void configure(const XxxConfig&)`，把原来写在构造函数/成员默认值里的常量改成从 Config 拷进来 |
| **B. 常量成员化** | `static constexpr` 类（`SmallBuffPhaseTracker`、`MoveModeDetector`） | `static constexpr double kX = 0.55;` → `double kX = 0.55;` + setter。**默认值必须一字不改**，保证现有单元测试继续绿 |
| **C. 运行时全局配置** | `buff_node_runtime.hpp` 里 `Fu_Task()` 内的局部常量 | 读全局 `g_buff_config.control.*` / `.timing.*`（该文件已经在用 `g_buff_*` 全局变量，风格一致） |

### 4.3 YAML 结构草案（写入 `rm_fu.yaml`，替换现有扁平键）

```yaml
# ============================================================
#  能量机关 全部可调参数 —— 唯一真源
#  加载: Fu 节点通过 fu_config_path 用 yaml-cpp 解析 (同自瞄 setParam)
#  约定: 默认值 == 迁移前硬编码值; 只改本文件即可调参, 无需重编译
# ============================================================
Buff:
  # ---------- 0. 开关 / 顶层 ----------
  move_mode: auto                 # auto | small | big
  enemy_color: blue               # blue | red
  run_without_serial: false
  model_path: Model/fu_moxi/buff_repvgg.xml
  assumed_fps: 30.0               # 原 Fu.cpp: fps = 30
  image_width: 1280               # 原 buff_angle_processor: image_width_
  image_height: 1024

  # ---------- 1. 靶面几何 / 相机标定 (原 buff_* 扁平键) ----------
  target:
    width_mm: 300.0
    height_mm: 300.0
    radius_mm: 650.0
    hub_normal_offset_mm: 0.0
  calibration:
    camera_matrix: [1329.24429634650, 0.518156995670085, 661.300343956123,
                    0, 1325.89618530323, 452.645884191168, 0, 0, 1]
    dist_coeffs: [-0.0442540638030886, 0.0902922934048078,
                  -0.00375815319429763, 0.00269367029458212, 0.0308390738152262]

  # ---------- 2. 识别 (buff_recognizer / stabilizer) ----------
  detect:
    conf_threshold: 0.2
    nms_threshold: 0.2
    binary_threshold: 120           # R中心精化二值化
    dilate_kernel_size: 5
    r_center_mask_ratio: 0.8
    r_center_contour_div: 3.0
    r_center_refine_max_shift: 5.0
    box_alpha: 0.3                  # 检测框 EMA
    stabilizer_alpha: 0.9           # 几何 EMA
    assoc_gate_min_px: 20.0         # 时序关联门限下限
    assoc_gate_radius_ratio: 1.2
    jump_fan_radius_ratio: 0.55     # 扇叶跳变判定
    jump_r_radius_ratio: 0.35       # R中心跳变判定
    jump_confirm_frames: 3          # 连续帧确认 (现 pending>=2)
    reacquire_confirm_frames: 3
    radius_change_reject_ratio: 0.50
    side_change_reject_ratio: 0.50
    min_valid_radius_px: 1.0
    gimbal_jump_limit_rad: 0.262    # 自运动补偿 15°
    fallback_after_fail_frames: 5

  # ---------- 3. 极坐标 / 角度差分 (Fu_angleprocess) ----------
  angle:
    direction_judge_samples: 21     # 转向判定样本数
    sector_count: 5                 # 五扇叶 2π/5
    unwrap_tolerance_divisor: 6.55  # 2π/6.55 补角容差
    wrap_period: 6.283185
    diff_search_start_ratio: 0.5
    diff_timeout_margin_ms: 80
    diff_angle_limit_rad: 1.169
    sector_search_range: 6          # 原 ±6 (注意不是 ±5)
    min_radius_px: 1.0
    angle_cache_len: 300

  # ---------- 4. 小符像素卡尔曼 (SmallBuffPhaseTracker) ----------
  small_buff:
    blade_count: 5
    converge_delay: 5
    max_roll_residual: 0.55
    min_speed: 0.052
    max_speed: 1.571
    phase_process_noise: 0.005
    phase_measurement_noise: 0.004
    min_radius_px: 1.0
    max_dt: 0.5
    reject_reset_count: 3
    phase_history_max: 20
    radius_ema: 0.1
    center_ema: 0.2

  # ---------- 5. 大符预测 (buff_predicted + RANSAC) ----------
  big_buff:
    enabled_ransac: false
    speed_model:                    # 原硬编码速度表
      const_level: 1.1775
      const_omega: 1.924
      angular_speed: 2.090
      forced_omega: 2.0             # 原 omega = 2 强制覆盖
    fit_bounds:
      min_omega: 1.884
      max_omega: 2.000
      min_level: 1.045
      max_level: 1.310
    windows:
      min_samples: 6
      init_samples_factor: 3.14159265358979   # 原 fps*PI
      ceres_trigger_factor: 6.28318530717959  # 原 2*fps*PI
      moving_average: 7
      vote_lookback: [5, 10, 15]
    thresholds:
      jump_ratio: 0.014             # 原 PI*0.014
      amplitude_classify_ratio: 0.5
    ceres:
      max_num_iterations: 500
      function_tolerance: 1.0e-15
      gradient_tolerance: 1.0e-15
      parameter_tolerance: 1.0e-15
      num_threads: 2
      initial_phase: 0.75
    ransac:                         # 原 buff_big_ransac_* (10 个键)
      max_iterations: 200
      threshold: 0.5
      min_omega: 1.884
      max_omega: 2.0
      min_amplitude: 0.780
      max_amplitude: 1.045
      min_inliers: 100
      max_abs_speed: 3.2
      max_gap_seconds: 5.0
      max_consecutive_rejections: 30
    mode_detector:                  # 大小符自动判别
      window_samples: 60
      minimum_samples: 12
      evaluate_every: 10
      confirmations: 3
      max_sample_jump: 0.04398      # 原 0.014*PI
      big_threshold_ratio: 0.60
      small_threshold_ratio: 0.40

  # ---------- 6. 世界系 EKF (原 buff_world_*) ----------
  world_ekf:
    enabled: false
    control_enabled: false
    apply_gimbal_pose: true
    quaternion_is_imu_to_map: true
    imu_stamp_offset_seconds: 0.0
    min_omega: 0.15
    max_omega: 1.571
    direction_lock_omega: 0.25
    direction_lock_frames: 15
    ready_enter_frames: 10
    ready_exit_frames: 15
    residual_gate_sigma: 3.5        # ★ 现在写死, 最该开放的野值门限
    omega_injection_alpha_fast: 0.5
    omega_injection_alpha_slow: 0.03
    omega_injection_fast_frames: 8
    radius_ema: 0.1
    min_radius_m: 0.05
    phase_process_noise_floor: 1.0e-5
    min_dt: 1.0e-4
    max_dt: 1.0
    max_predict_horizon_s: 1.0
    init_covariance_diag: [0.10, 0.10, 0.10, 0.20, 0.20, 0.20, 0.50]
    center_measurement_std_m: 0.05
    normal_measurement_std_rad: 0.05
    phase_measurement_std_rad: 0.05
    center_process_std_mps: 0.015
    normal_process_std_radps: 0.05
    omega_process_std_radps2: 0.40
    radius_mm: 650.0
    hub_normal_offset_mm: 0.0

  # ---------- 7. 弹道 / PnP ----------
  ballistic:
    default_velocity: 24.0          # 串口无有效弹速时的兜底
    velocity_min_valid: 10.0        # 串口弹速有效门限
    gravity: 9.8
    iteration_times: 30
    converge_epsilon: 0.0001
    height_tolerance: 0.01
    angle_limit_deg: 29.7           # 原 M_PI*0.165
    phase_probe_rad: 0.30
    probe_valid_rad: 0.015
    physical_phase_limit_rad: 2.5
    max_reprojection_error_px: 12.0
    reprojection_check_enabled: false   # 现校验被注释, 显式暴露成开关
    optical_center_offset_x: -23.0      # ★ 现写死的主点人为偏移
    optical_center_offset_y: 24.0

  # ---------- 8. 时延 / 提前量 ----------
  timing:
    delta_t_init: 0.3
    delta_t_compensation: 0.15
    delta_t_min: 0.01
    delta_t_max: 1.0
    lead_ema_old: 0.8
    lead_ema_new: 0.2
    flight_time_valid_max: 2.0
    fallback_fps: 120.0
    fallback_fps_ema: [0.9, 0.1]
    default_omega: 1.047            # 最后兜底 60°/s
    kalman_bad_frames: 20
    divergence_radius_ratio_min: 0.3
    divergence_radius_ratio_max: 3.5
    center_smooth_alpha: 0.3

  # ---------- 9. 云台下发 / 保护 ----------
  control:
    pitch_compensation: 0.0
    yaw_compensation: 0.0
    pitch_max_deg: 22.0
    pitch_min_deg: -15.0
    stuck_threshold_deg: 19.0
    stuck_frames: 30
    recovery_pitch_deg: -10.0
    recovery_frames: 15
    log_throttle_s: 0.2
```

> 键名可按你们习惯再调；**结构与分节方式与自瞄一致**（`模块 → 子节 → 键`），调参时一眼能定位。

### 4.4 向后兼容（关键，保证"现在能跑的不坏"）

1. `Fu.cpp` 里先读 `Buff:` 节；若某键缺失，回落到**当前硬编码默认值**（而不是报错）。
2. 过渡期保留旧扁平键（`buff_world_*` / `buff_big_ransac_*` / `buff_move_mode` / `buff_*_mm` / `buff_camera_matrix`）作为**覆盖层**：`loadBuffConfig` 里检测到旧键就用它并 `ROS_WARN` 提示"已迁移到 Buff.xxx，建议更新 yaml"。这样即使有人用旧 yaml 或 `rosparam set` 也照样生效。
3. 迁移完成后从 `rm_fu.yaml` 删除旧扁平键，只留 `Buff:` 节。
4. `rm_fu.launch` 的 `<rosparam command="load">` 保留（零成本）；关键路径改为 yaml-cpp 直读，**避免"改了 yaml 但节点其实读的是参数服务器"这类调参陷阱**。launch 里顺手补上现在漏掉的 `run_buff_without_serial` 传参（`rm_fu.launch:6` 声明了 arg 但没写成 `<param>`，节点永远读到默认 false）。

### 4.5 可选增强（不必须，评审时决定）

- **热重载**：加 `std_srvs/Trigger` 服务 `/buff/reload_config`，调参不用重启节点（只对运行时读取的参数有效，构造函数期的参数仍需重启）。
- **参数快照发布**：把生效后的参数以 YAML 文本发到 `/buff/config`（`std_msgs/String`），方便录包后核对。

---

## 5. 分阶段实施计划

每个阶段都满足：**默认值不变 → 行为不变 → 可单独编译验证 → 可单独回滚**。

| 阶段 | 内容 | 改动文件 | 验收 |
|---|---|---|---|
| **P0 基线** | 确认现有测试状态。**`src/rm_tracker/test/test_fu_config_layout.py` 当前就是红色的**，而且它其实是"一次未完成重构的规格书"：它要求 `functional = false`（实际 `Fu.cpp:180` 是 `true`）、要求 `track_ok = false`（实际 `buff_node_runtime.hpp:223` 是 `true`）、要求 runtime 里出现 `raw_angle_x` / `limited_angle_x` / `prediction_fail_reason` / `[BUFF_PRED_FAIL]` / `[BUFF_WORLD_FAIL]` / `beginPredictionFrame()` 且删掉裸 `cout`（这些**在 runtime 里全都没有**，但 `buff_pnp_solver.hpp` 侧已经实现了对应 API），还要求 `start_fan.launch` 有 `shootflag_test` arg（实际是 `shootflag`）。→ 必须先决定：**先补完这次重构，还是先刷新测试**。否则后续每一步都没有可信的"绿"判据 | 无 | 在 Ubuntu 上记录 `ctest` 基线（哪些本来就红），并明确测试修订策略 |
| **P1 骨架** | 新建 `buff_config.hpp`（结构体 + `loadBuffConfig` + `logBuffConfig`），**只加不改**；`Fu.cpp` 调用它并打印，但参数还没被使用 | `include/buff_config.hpp`(新)、`src/Fu.cpp` | Fu 可编译；启动能打印全量参数；行为与改前逐帧一致 |
| **P2 YAML 结构** | `rm_fu.yaml` 增加 `Buff:` 节（默认值 = 现状），旧扁平键保留为覆盖层 | `config/rm_fu.yaml` | `loadBuffConfig` 读到的值 == 旧键值（打印比对） |
| **P3 时延/云台链**（收益最高，全写死） | `delta_t_` / `delta_t_compensation` / `pitch_compensation` / `yaw_compensation` / `fps` / pitch 限幅 / 卡死恢复 / 兜底 fps / `default_omega` / 坏帧阈值 | `src/Fu.cpp`、`include/buff_node_runtime.hpp` | 打印值 == yaml 值；改 yaml 立即生效 |
| **P4 PnP/弹道** | 弹速兜底、重力、迭代次数、收敛阈值、角度限、相位探针、物理相位限、重投影门限（含暴露被注释的校验开关） | `include/buff_pnp_solver.hpp` | 现有 `buff_prediction_geometry_test` 绿 |
| **P5 识别** | conf/NMS/二值化/膨胀核/mask 比例/距离惩罚/限幅/EMA/关联门限/跳变判定/重捕获确认/几何一致性拒绝 | `src/buff_recognizer.cpp`、`include/buff_recognizer.hpp`、`include/buff_detection_stabilizer.hpp` | `buff_detection_stabilizer_test` 绿；`configure()` 缺省时行为不变 |
| **P6 角度+预测** | `Fu_angleprocess`（转向判定/补角/差分窗口）、`SmallBuffPhaseTracker`（常量成员化）、`buff_predicted`（~45 个）、`MoveModeDetector`（7 个）、世界 EKF 剩余门限 | `include/buff_angle_processor.hpp`、`blade_tracker.hpp`、`buff_motion_predictor.hpp`、`buff_move_mode_detector.hpp`、`buff_world_tracker.hpp` | `buff_angle_processor_test` / `buff_move_mode_detector_test` / `buff_big_ransac_predictor_test` 全绿 |
| **P7 收尾** | 删除旧扁平键与兼容层、补 `run_buff_without_serial` 传参、参数手册文档、Python 覆盖守卫测试 | `rm_fu.yaml`、`rm_fu.launch`、`docs/` | 全量 `ctest` 绿 |

**优先级建议**：P3 > P4 > P5 > P6 > P7。P3/P4 是"现在想调但调不了"的重灾区（时延 0.15、弹速 24、云台限幅 22°/-15°），改动集中在一个文件里，风险最低、收益最直接。

---

## 6. 风险与边界

### 6.1 死代码不要碰
`src/Fu_yuan.cpp`（105 KB）、`src/tracker_yuan.cpp`：不在 `CMakeLists.txt`、无 `#include`、build 树零引用。其中的 ~150 个常量可作**默认值参考**，但不要直接改它。另注意它与现役 header 版已有分歧（相机内参、畸变、半径 700 vs `buff_radius_mm: 650`、补角搜索 ±5 vs ±6、`1280/1024` 字面量 vs `image_width_`），**以 header 版为准**。

### 6.2 `Yolov8_Fan.h` / `Yolov8_R.h` 不在 Fu 编译链里
`CMakeLists.txt` 的 `Fu` 只编译 `Fu.cpp + buff_recognizer.cpp + buff_zju_ransac.cpp + buff_big_ransac_predictor.cpp + CoorConverter.cpp`。这两个头文件属于旧 R/Fan 双模型路径，其 conf/NMS/图像边界（1280×1024）**改了也不生效**。建议本次不迁移，只在文档里标注"废弃"。

### 6.3 `blade_tracker.hpp` 是自瞄/打符共享头
`RmTracker.hpp:37` 也 include 它（`EnhancedMultiBladeTracker` 被装甲路径使用）。因此：
- 只允许"常量成员化 + 默认值不变"，**绝不改默认值**；
- 只有 Fu 会调用新增的 `configure()`，装甲节点不调用 → 装甲行为零影响；
- 改完必须回归验证装甲链路（至少确认 `tracker` 可编译、`EnhancedMultiBladeTracker(100.0f)` 语义不变）。

### 6.4 两套默认值 / 三套阈值的既有坑（顺手清理，但需确认）
- `BuffWorldTrackerConfig` 的结构体默认值与 `Fu.cpp` 的 param 默认值不一致（phase 0.020 vs 0.040；omega 0.50 vs 0.250）→ 结构体默认值是死代码。YAML 化后应统一到一处。
- conf/NMS 有 4 套语义：`buff_recognizer.hpp` 0.2/0.2、`buff_recognizer.cpp` 显式 0.2/0.2、旧 `Fu_yuan.cpp` 0.2/0.3、`Yolov8_*.h` 实际 `score > 0`（等于不过滤）。
- `MAX_REPROJECTION_ERROR_PX = 12.0` 定义了但校验被注释掉 → **当前重投影门限形同虚设**，需确认是否有意为之。

### 6.5 不该进 YAML 的
`PI`、单位换算 `*0.001`、ros 话题名、`KPT_START/NUM_RAW_KPTS` 等模型输出布局、可视化颜色/坐标、ROS 队列长度。这类改了只会让人误以为能调，反而增加风险。

### 6.6 launch / 命名空间（改了会"静默失效"的坑）

- 现在 `<rosparam command="load">` 和 `<param>` 都写在 `<node name="Fu">` **内部**，所以键落在 `/Fu/*`，而 `Fu.cpp:223` 的 `private_nh("~")` 正是 `/Fu`，参数目前是**真的生效**的。
- ⚠️ **一旦重构时把这个 rosparam 提到 `<launch>` 顶层/全局，所有 `private_nh` 查询都会"静默"回落到硬编码默认值**（ROS 缺键不报错），现象就是"我改了 YAML 但没反应"。所以：保持 `<node>` 内加载，并且**启动时打印实际生效值**（本方案的 `logBuffConfig()` 正好解决这个问题）。
- `rm_fu.launch:6` 的 `run_buff_without_serial` **是死 arg**：声明了但没写成 `<param>`，节点永远读到 `false`，launch 打不开。
- `rm_fu.launch:2-3,9-10` 的 `r_model_path` / `fan_model_path` 传了但 `Fu.cpp` 从不读取（旧双模型残留）。
- `tracker.cpp:16-17` 读 `force_energy_fan`，但**没有任何 launch 传它**；`start.launch` 注释里说的"tracker 转 EnergyFanMode"实际未接线。
- `start.launch:18` / `start_fan.launch:13` 以全局方式加载 `rm_node/config/camera.yaml`（`FrameRate: 120`），这就是代码里 `120.0` 帧率假设的来源 —— 这两处是全局参数，别和 Buff 的私有参数混淆。

### 6.7 顺手发现的既有问题（建议记录，是否一并修由你定）

| 类型 | 位置 | 内容 |
|---|---|---|
| 注释与代码不一致 | `buff_node_runtime.hpp:392-393` | 发散检测实际是 `0.3R ~ 3.5R`，上一行注释写的是 `0.4~2.5` |
| 注释与代码不一致 | `buff_node_runtime.hpp:423` | 切兜底阈值实际 `kalman_bad_frames >= 20`，注释多处写"连续 3 帧" |
| 注释与代码不一致 | `buff_node_runtime.hpp:222-223` | 注释写"下游失败时 `track_ok = false`"，代码是 `bool track_ok = true;`（也正是测试红的原因） |
| 死变量 | `Fu.cpp:147` `camp`、`Fu.cpp:166` `Qtn[4]`、`Fu.cpp:150` `fps=30` | 只写不读 |
| 死代码 | `buff_node_runtime.hpp:118-122` | `smooth_R/smooth_Fan/alpha=0.3` 算完从未使用（原本想平滑圆心，实际没接） |
| 死变量 | `buff_node_runtime.hpp:388/401`、`412-413` | `kalman_ever_healthy`、`fallback_fail_cnt` 只写不读 → "启动收敛期不计发散"的意图已失效 |
| 死常量 | `buff_recognizer.hpp:87`、`210-212` | `NUM_CLASSES`、`last_quadrant_`/`quadrant_stable_cnt_`/`active_quadrant_` 零引用 |
| 浮点等值判失败 | `buff_node_runtime.hpp:443`、`493`、`512` | `predicted_diff_angle != 0.0`、`euler_angles.x == 0.0` 用等值判断"失败" |
| 与规格冲突 | `buff_pnp_solver.hpp` vs `buff_node_runtime.hpp` | PnP 侧已有 `lastReprojectionError()` / `beginPredictionFrame()` / `lastPredictionFailureReason()` / `lastPredictedTargetGun()` 诊断 API，runtime 侧一行没接；同时 runtime 里还留着 `cout<<"2222222222222"` 等 4 处裸调试输出（测试要求删除） |
| 可清理文件 | `src/rm_tracker/src/Fu.cpp.bak_visual`（87 KB）、`src/Fu_yuan.cpp`、`src/tracker_yuan.cpp`（与 `tracker.cpp` 内容相同） | 死文件/备份 |
| 值不一致 | `buff_radius_mm: 650`（yaml）vs `Fu_yuan.cpp:1744` `BLADE_RADIUS = 700` | 需确认哪个是实测值（现役走 yaml 的 650） |

### 6.8 验证环境
本机 Windows 无 ROS/catkin，**无法本地编译 C++**。验证路径：
- 本机：Python 静态守卫测试（yaml 键完整性、源码残留字面量扫描）+ 人工 diff 审查；
- Ubuntu 机器人/开发机：`catkin_make --pkg rm_tracker -j2`、`ctest --test-dir build --output-on-failure`，以及打符 bag 回放（`roslaunch rm_node start_fan_bag.launch`）逐帧比对 `/debugpub`、`/buff/track_ok`、`/auto_angle`。

---

## 7. 测试与回归守卫

1. **扩展 `test/test_fu_config_layout.py`**：断言 `rm_fu.yaml` 存在 `Buff:` 节且关键键齐全；断言 `Fu.cpp` 里不再出现 `delta_t_compensation = 0.15` 这类字面量（`assertNotIn`）。注意该文件已有失效断言（§5 P0），需先修。
2. **新增 `test/test_buff_yaml_coverage.py`**：扫描 `buff_*.hpp/cpp` + `Fu.cpp`，对"可疑数值字面量"报警，白名单外不允许新增魔数 —— 这是防止以后又写回硬编码的护栏。
3. **新增 `test/buff_config_test.cpp`**（只依赖 yaml-cpp，可独立编译）：验证
   - 空/缺键 yaml → 全部取默认值，且默认值 == 迁移前硬编码值；
   - 越界值 → 被 clamp 并打印警告；
   - 非法 yaml → `loadBuffConfig` 返回 false 且 `error` 非空。
4. **既有 C++ 单测必须全绿**：`buff_detection_stabilizer_test`、`buff_prediction_geometry_test`、`buff_zju_ransac_test`、`buff_big_ransac_predictor_test`、`buff_angle_processor_test`、`buff_move_mode_detector_test`（它们直接构造模块、依赖默认值）。

---

## 8. 交付物

| 交付物 | 说明 |
|---|---|
| `docs/buff-参数手册.md` | 按 YAML 分节，逐参数写：含义 / 默认值 / 建议范围 / 调整症状（"滞后→调小 X""抖动→调大 Y"）。现有 `README_buff_upgrade.md` 的"参数总表"可合并进来 |
| `src/rm_tracker/include/buff_config.hpp` | 结构体 + 加载器 + 校验 + 打印 |
| `src/rm_tracker/config/rm_fu.yaml` | 唯一真源，`Buff:` 节 ~180 键 |
| 6 个模块头文件 + `Fu.cpp` | 常量 → 配置 |
| 测试 | 上述 1~4 |

---

## 9. P1–P4 执行清单（本次落地范围）

### P1 — 骨架：新增 `include/buff_config.hpp`（只加不改）

**关键设计**：loader 做成**纯 C++，不调用任何 ROS API**（警告以 `std::vector<std::string>` 返回，由 `Fu.cpp` 打印）。这样它能被独立的单元测试覆盖，不需要 ROS/catkin 环境。

```cpp
namespace buff {

struct SyncConfig {              // IMU/图像同步
    std::size_t imu_pose_history_capacity = 200;
    double max_imu_image_skew_seconds = 0.05;
    double quaternion_norm_min = 0.9;
    double quaternion_norm_max = 1.1;
};
struct TimingConfig {            // 时延/提前量/兜底
    double delta_t_init = 0.3;
    double delta_t_compensation = 0.15;
    double delta_t_min = 0.01;
    double delta_t_max = 1.0;
    int    flight_time_iterations = 2;
    double flight_time_valid_max = 2.0;
    double lead_ema_old = 0.8;
    double lead_ema_new = 0.2;
    double fallback_fps = 120.0;
    double fallback_fps_ema_old = 0.9;
    double fallback_fps_ema_new = 0.1;
    double fps_interval_max_ms = 500.0;
    int    angle_cache_len = 300;
    double default_omega = 1.047;
    int    kalman_bad_frames = 20;
    double divergence_min_radius_px = 1.0;
    double divergence_radius_ratio_min = 0.3;
    double divergence_radius_ratio_max = 3.5;
};
struct ControlConfig {           // 云台下发/限幅/卡死恢复
    double pitch_compensation = 0.0;
    double yaw_compensation = 0.0;
    double pitch_max_deg = 22.0;
    double pitch_min_deg = -15.0;
    double stuck_threshold_deg = 19.0;
    int    stuck_frames = 30;
    double recovery_pitch_deg = -10.0;
    int    recovery_frames = 15;
    double log_throttle_s = 0.2;
};
struct BallisticConfig {         // PnP/弹道/相位
    double default_velocity = 24.0;
    double velocity_min_valid = 10.0;
    double gravity = 9.8;
    int    iteration_times = 30;
    double converge_epsilon = 0.0001;
    double height_tolerance = 0.01;
    double angle_limit_deg = 29.7;          // 原 M_PI*0.165
    double phase_probe_rad = 0.30;
    double probe_valid_rad = 0.015;
    double physical_phase_limit_rad = 2.5;
    double max_reprojection_error_px = 12.0;
    bool   reprojection_check_enabled = false;
    double optical_center_offset_x = -23.0;
    double optical_center_offset_y = 24.0;
};

struct BuffConfig {
    YAML::Node     raw;
    SyncConfig      sync;
    TimingConfig    timing;
    ControlConfig   control;
    BallisticConfig ballistic;
    // 复用现有结构体，P2 阶段一并从 Buff: 节读取
    BuffWorldTrackerConfig        world;
    buff_zju::BigBuffRansacConfig ransac;
    // 顶层开关/标定（原扁平 buff_* 键）
    std::string move_mode = "small";
    std::string enemy_color = "blue";
    std::string model_path = "Model/fu_moxi/buff_repvgg.xml";
    bool   run_without_serial = false;
    double target_width_mm = 300.0;
    double target_height_mm = 300.0;
    double radius_mm = 650.0;
    double hub_normal_offset_mm = 0.0;
    std::vector<double> camera_matrix{/* 9 个 */};
    std::vector<double> dist_coeffs{/* 5 个 */};
};

// 纯函数：不调用 ROS，便于单测
bool loadBuffConfig(const std::string& path,
                    BuffConfig& out,
                    std::vector<std::string>& warnings,
                    std::string& error);
std::vector<std::string> buffConfigToLines(const BuffConfig& cfg);  // Fu.cpp 用 ROS_INFO 逐行打印
}  // namespace buff
```

**优先级规则（显式三层，避免"静默回落"）**：
1. launch 里的 `<param>`（ROS 参数服务器，仅 `enemy_color` / `buff_model_path` / `run_buff_without_serial` 这类，`Fu.cpp` 用 `private_nh.hasParam()` 判断是否显式设置）；
2. YAML `Buff:` 节；
3. YAML 顶层旧扁平键（过渡覆盖层，读到就打 `deprecated` 警告）；
4. 代码默认值（= 迁移前硬编码值）。

**P1 改动文件**：`include/buff_config.hpp`（新）、`src/Fu.cpp`（调用 + 打印，参数暂时还没被使用）、`CMakeLists.txt`（新增 `buff_config_test` 目标，位于 `CATKIN_ENABLE_TESTING` 块内，只链 `yaml-cpp`）。
**P1 验收**：Fu 可编译；启动打印全量参数；行为与改前逐帧一致（参数尚未生效，纯骨架）。

### P2 — `rm_fu.yaml` 增加 `Buff:` 节

新增 `Buff.timing` / `Buff.control` / `Buff.ballistic` / `Buff.sync` 四节（键见上），并把现有顶层键**原样保留**作为过渡层。同时把 `buff_world_*` / `buff_big_ransac_*` 纳入 `Buff.world_ekf` / `Buff.big_buff.ransac`（两处值必须一致，loader 优先 `Buff:` 节）。
**P2 验收**：`buff_config_test` 断言"从新键读到的值 == 从旧键读到的值"；`buffConfigToLines()` 打印值 == 旧行为。

### P3 — 时延 / 云台链参数化（收益最高）

| 现在的硬编码 | 位置 | YAML 键 |
|---|---|---|
| `double delta_t_ = 0.3;` | `Fu.cpp:181` | `Buff.timing.delta_t_init` |
| `double delta_t_compensation = 0.15;` | `Fu.cpp:182` | `Buff.timing.delta_t_compensation` |
| `pitch_compensation = 0.0;` | `Fu.cpp:184` | `Buff.control.pitch_compensation` |
| `yaw_compensation = 0.0;` | `Fu.cpp:185` | `Buff.control.yaw_compensation` |
| `kImuPoseHistoryCapacity = 200` | `Fu.cpp:91` | `Buff.sync.imu_pose_history_capacity` |
| `kMaximumImuImageSkewSeconds = 0.05` | `Fu.cpp:92` | `Buff.sync.max_imu_image_skew_seconds` |
| `time_parameter < 2.0` | `buff_node_runtime.hpp:173` | `Buff.timing.flight_time_valid_max` |
| `fallback_fps = 120.0` | `:318` | `Buff.timing.fallback_fps` |
| `interval_ms < 500.0` / `0.9,0.1` | `:325-326` | `Buff.timing.fps_interval_max_ms` / `fallback_fps_ema_*` |
| `angle_diff.size() > 300` | `:341` | `Buff.timing.angle_cache_len` |
| `radius_buff > 1.0f` | `:390` | `Buff.timing.divergence_min_radius_px` |
| `0.3f*R \|\| 3.5f*R` | `:392-393` | `Buff.timing.divergence_radius_ratio_min/max` |
| `kalman_bad_frames >= 20` | `:423` | `Buff.timing.kalman_bad_frames` |
| `default_omega = 1.047` | `:456` | `Buff.timing.default_omega` |
| `clamp(delta_t_, 0.01, 1.0)` | `:481` | `Buff.timing.delta_t_min/max` |
| `iteration < 2` | `:482` | `Buff.timing.flight_time_iterations` |
| `new_hit_time <= 0.01 \|\| >= 1.0` | `:494-495` | `Buff.timing.delta_t_min/max` |
| `delta_t_ = 0.8*x + 0.2*y` | `:528` | `Buff.timing.lead_ema_old/new` |
| `PITCH_MAX_RAD = 22.0°` | `:541` | `Buff.control.pitch_max_deg` |
| `PITCH_MIN_RAD = -15.0°` | `:542` | `Buff.control.pitch_min_deg` |
| `STUCK_THRESH_RAD = 19.0°` | `:551` | `Buff.control.stuck_threshold_deg` |
| 恢复下压 `-10.0°` | `:555` | `Buff.control.recovery_pitch_deg` |
| `stuck_frames > 30` | `:564` | `Buff.control.stuck_frames` |
| `recovery_remaining = 15` | `:566` | `Buff.control.recovery_frames` |
| `ROS_INFO_THROTTLE(0.2, ...)` | `:577` | `Buff.control.log_throttle_s` |
| 四元数模长窗口 `0.9~1.1` | `:23` | `Buff.sync.quaternion_norm_min/max` |

**不参数化**（记录为死代码，本次不动，避免混入行为改动）：`Fu.cpp:147 camp`、`Fu.cpp:150 fps=30`、`Fu.cpp:166 Qtn[4]`、`buff_node_runtime.hpp:118-122 smooth_R/smooth_Fan/alpha`、`:388 kalman_ever_healthy`、`:412 fallback_fail_cnt`。
**P3 验收**：改 yaml 后重启节点，打印值与 yaml 一致；pitch 限幅/兜底阈值改动可见于 `/auto_angle` 与 `/buff/track_ok`。

### P4 — PnP / 弹道参数化

`PnPSolver_gravity` 是 `buff_pnp_solver.hpp:51-743` 的一个类，直接加 `void configure(const buff::BallisticConfig&)` 即可（`Fu.cpp` 构造完 PnP 后调用）。

| 现在的硬编码 | 位置 | YAML 键 |
|---|---|---|
| `const int iteration_times = 30` | `buff_pnp_solver.hpp:360` | `Buff.ballistic.iteration_times` |
| `float velocity = 24` | `:361` | `Buff.ballistic.default_velocity` |
| `RmSerialData.BulletVec > 10` | `:362` | `Buff.ballistic.velocity_min_valid` |
| `const float gravity = 9.8f` | `:365` | `Buff.ballistic.gravity` |
| `M_PI*0.165`（4 处：382 / 407 / 431-432 / 651） | 同上 | `Buff.ballistic.angle_limit_deg` |
| `< 0.0001` 迭代收敛 | `:403` | `Buff.ballistic.converge_epsilon` |
| `> 0.01` 高度差门限 | `:407` | `Buff.ballistic.height_tolerance` |
| `kPhaseSignProbeRadians = 0.30` | `:538` | `Buff.ballistic.phase_probe_rad` |
| `>= 0.015` 探针有效性 | `:570` | `Buff.ballistic.probe_valid_rad` |
| `> 2.5` 物理相位上限 | `:593` | `Buff.ballistic.physical_phase_limit_rad` |
| `MAX_REPROJECTION_ERROR_PX = 12.0` | `:730` | `Buff.ballistic.max_reprojection_error_px` |
| 重投影校验被注释掉 | `:171-178` | `Buff.ballistic.reprojection_check_enabled`（默认 `false`，保持现状） |
| 主点人为偏移 `cx-23, cy+24` | `:322` | `Buff.ballistic.optical_center_offset_x/y` |

**不参数化**：`blade_points_` 的四点顺序 `[上,左,下,右]`（`buff_recognizer` 的隐式契约，改了就错）、`*0.001` mm→m 换算、`1e-6`/`1e-9` epsilon。
**P4 验收**：`buff_prediction_geometry_test` 绿；bag 回放时 `/auto_angle` 与改前逐帧一致（默认值未变）。

### 后续（下一轮，P5–P7）
P5 识别（conf/NMS/R 中心精化/跳变确认，~35 项）、P6 角度+预测（`Fu_angleprocess`、`SmallBuffPhaseTracker`、`buff_predicted` ~45 项、`MoveModeDetector`、世界 EKF 剩余门限）、P7 收尾（删旧键、补 `run_buff_without_serial` 传参、参数手册、Python 覆盖守卫测试）。
