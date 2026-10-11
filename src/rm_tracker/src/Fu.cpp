#include "opencv2/opencv.hpp"
 #include <ceres/ceres.h>
 #include <Eigen/Dense>
 #include <cmath>
 #include <stdexcept>
 #include <random>
 #include <algorithm>
 #include <iostream>
 #include <fftw3.h>
 #include <fstream>
 #include <vector>
 #include <deque>
 #include <limits>
 #include <string>
 #include <array>
 #include <ros/ros.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/Image.h>
 #include <geometry_msgs/Vector3.h>
 #include <image_transport/image_transport.h>
 #include <camera_info_manager/camera_info_manager.h>
 #include "Image.h"
 #include "buff_recognizer.hpp"   // Buff识别器: YOLO11关键点模型
 #include "blade_tracker.hpp"     // Small-buff phase tracking and prediction
#include "buff_world_tracker.hpp"
#include "buff_big_ransac_predictor.hpp"
#include "buff_big_variable_speed_predictor.hpp"
#include "buff_big_prediction_gate.hpp"
 #include "CoorConverter.hpp"    // 坐标变换: 相机→IMU→枪管
 #include "rm_msgs/RmSerial.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

 // 调试
////////////////////////////
#include <chrono>
#include <cstdint>
#include <thread>
////////////////////////////

using namespace cv;
using namespace std;


rm_msgs::RmSerial RmSerialData;

// Energy-mechanism-only runtime settings. They leave CoordinateTransformer,
// recognizer, serial messages, and the armor tracker unchanged.
bool g_buff_world_ekf_enabled = false;
bool g_buff_world_ekf_control_enabled = false;
bool g_buff_world_ekf_primary = false;
bool g_quaternion_is_imu_to_map = true;
bool g_buff_big_ransac_enabled = true;
bool g_buff_big_variable_speed_enabled = true;

// Persistent rotation direction shared across Kalman and fallback paths.
// +1 = polar angle increases over time (clockwise in OpenCV image space)
// -1 = polar angle decreases over time
//  0 = unknown / not yet determined
int g_buff_rotation_sign = 0;
double g_imu_stamp_offset_seconds = 0.0;

// Persistent direction that survives Angleprocess re-creation (declared extern in buff_angle_processor.hpp)
int g_persistent_rotation_dir = 0;
double g_buff_target_width_mm = 300.0;
double g_buff_target_height_mm = 300.0;
// The Buff PnP calibration is independent from the cam2Imu/gun2Imu
// extrinsics. Defaults match the active infantry camera calibration.
std::vector<double> g_buff_camera_matrix{
   1724.18353097744, -0.000579628569003521  , 680.697138243768, 0.0,  1726.83294106563,  504.398280989468, 0.0, 0.0, 1.0};
std::vector<double> g_buff_dist_coeffs{
    -0.0834722705527182, 0.496852388408421    , -0.000457625756594158  ,  0.00246230944307382, -0.0130632578222697};
BuffWorldTrackerConfig g_buff_world_config;
BuffWorldTracker g_buff_world_tracker;

// Every energy-mechanism frame must use the IMU pose measured at the same time.
// The direct and inverse quaternion conventions are both logged before one is chosen.
struct TimedImuPose {
    ros::Time stamp;
    Eigen::Quaterniond imu_to_map;
    double raw_norm = 0.0;
};

constexpr std::size_t kImuPoseHistoryCapacity = 200;
constexpr double kMaximumImuImageSkewSeconds = 0.05;
std::deque<TimedImuPose> g_imu_pose_history;
TimedImuPose g_frame_imu_pose;
ros::Time g_frame_image_stamp;
double g_frame_monotonic_time_seconds = 0.0;
std::uint64_t g_frame_sequence = 0;
std::uint64_t g_processed_frame_sequence = 0;
double g_last_big_buff_timestamp =
    std::numeric_limits<double>::quiet_NaN();
bool g_has_frame_imu_pose = false;
double g_frame_imu_image_skew_seconds =
    std::numeric_limits<double>::quiet_NaN();

bool findClosestImuPose(const ros::Time& image_stamp, TimedImuPose& matched_pose) {
    if (image_stamp.isZero() || g_imu_pose_history.empty()) {
        return false;
    }

    const TimedImuPose* closest_pose = nullptr;
    double closest_skew_seconds = std::numeric_limits<double>::infinity();
    for (const auto& pose : g_imu_pose_history) {
        const ros::Time corrected_stamp = pose.stamp +
            ros::Duration(g_imu_stamp_offset_seconds);
        const double skew_seconds =
            std::abs((corrected_stamp - image_stamp).toSec());
        if (skew_seconds < closest_skew_seconds) {
            closest_skew_seconds = skew_seconds;
            closest_pose = &pose;
        }
    }

    if (closest_pose == nullptr ||
        closest_skew_seconds > kMaximumImuImageSkewSeconds) {
        return false;
    }

    matched_pose = *closest_pose;
    return true;
}

// ═══ Buff 识别器全局对象 + 结果缓存 ═══
// g_recognizer: main()中创建, Fu_Task()中使用
// 模型路径+敌方颜色通过 ROS param: buff_model_path, enemy_color
// 标签映射: enemy_color → enemy_class_idx_
//   "blue" → 0 → tensor行4(class 0=red)  "red" → 1 → tensor行5(class 1=blue)
// 结果流向: g_R_center/g_Fan_center→Fu_angleprocess, g_side→PnPSolver_gravity
// 详细文档见 README_buff_upgrade.md
BuffRecognizer* g_recognizer = nullptr;
// 识别结果缓存
cv::Point2f g_R_center   = {0, 0};    // R中心 (旋转圆心, CV精化)
cv::Point2f g_Fan_center = {0, 0};    // 扇叶中心 (四边中点平均=靶心)
double      g_side       = 0;          // 扇叶边长 = |顶边中点-底边中点|
std::vector<cv::Point2f> g_corners;   // 4个PnP角点: [顶,左,底,右]边中点

//重要参数
char camp = 'r';//阵营
std::string MoveMode = "small";//小符
// std::string MoveMode = "big";//大符
double fps = 30;





#include "buff_motion_predictor.hpp"
#include "buff_angle_processor.hpp"
#include "buff_pnp_solver.hpp"

//前处理


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Vector3 Angle;
// rm_msgs::RmSerial RmSerialData;
double Qtn[4];
ros::Publisher PubAngle;
ros::Publisher debugpub;            //debug发布者
ros::Publisher debugpub1;           //debug1发布者
ros::Publisher debugpub2;           //debug2发布者  (buff世界卡尔曼 omega)
ros::Publisher debugpub3;           //debug3发布者  (buff世界卡尔曼 |K-M| px)
ros::Publisher debugpub4;           //debug4发布者  (buff世界卡尔曼 |P-M| px)
ros::Publisher world_ready_pub;     // /buff/world_ready 发布者
/*主程序*/

bool init = false;//初始化标识符
cv::Mat frame;//处理帧
cv::Mat frame_;//接收帧
bool functional = false;//Wait for the serial mode command before publishing Fu angles.
double delta_t_ = 0.3;//差分时间

// ═══ 时间补偿: 总提前量 = 子弹飞行时间 + 视觉链路延迟 + 电控延时 ═══
// 预测用的相位是在「图像曝光时刻」测到的(卡尔曼的时间基准 time_milliseconds 是
// 处理时刻, 但相位本身来自曝光那一帧), 而弹丸在
//   t_impact = t_exposure + 视觉链路延迟(曝光→出角度)
//                       + 电控延时(下发→云台到位/发弹)
//                       + 子弹飞行时间
// 命中。所以 delta_t_ 必须把这三段都算进去, 少一段打点就滞后一个固定角度
// (60°/s 的转速下, 少补 20ms 就是 1.2° 的常值偏差, 而且不随距离变化)。
//
// 旧代码把后两段合并成一个硬编码的 0.15, 现场没法标、改了要重编译。
// 现在拆成两个可配置项 + 一路视觉延迟实测。
//
// 视觉链路延迟: 曝光 → 相机传输 → ROS 队列 → YOLO 推理 → 跟踪/预测 → PnP → 下发。
//   buff_vision_delay_auto=true 时用图像时间戳实测(EMA), 否则用固定值。
// 电控延时: /auto_angle 下发 → 串口 → 电控解算 → 云台转到指令角 / 发弹出膛。
//   这个量问电控要, 视觉这边测不了。
double vision_delay_seconds = 0.10;     // 视觉链路延迟固定值(秒)
double ec_delay_seconds     = 0.05;     // 电控延时固定值(秒)
bool   vision_delay_auto    = false;    // true = 用图像时间戳实测视觉延迟
double vision_delay_ema_seconds = -1.0; // 实测视觉延迟EMA(<0 = 还没测出来)
double vision_delay_measured_seconds = -1.0; // 最近一次实测值(调试用)
constexpr double kVisionDelayMinSeconds = 0.005;  // 实测值合理区间下界(5ms)
constexpr double kVisionDelayMaxSeconds = 0.35;   // 实测值合理区间上界(350ms)

// 总时间补偿(秒): 视觉链路 + 电控。所有 delta_t_ 的计算都走这里。
double buffTotalLeadCompensation()
{
    if (vision_delay_auto && vision_delay_ema_seconds > 0.0) {
        return vision_delay_ema_seconds + ec_delay_seconds;
    }
    return vision_delay_seconds + ec_delay_seconds;
}

// 实测视觉链路延迟 = 当前时刻 - 图像时间戳(曝光时刻)。
// 必须在「下发角度之后」调用, 这样量到的就是本帧从曝光到出角度的完整链路耗时:
// 相机传输 + ROS 队列积压 + YOLO 推理 + PnP 全部包含在内。EMA 平滑后供下一帧
// 使用(本帧的 delta_t_ 在上游已经用掉了, 来不及改)。
// 回放 bag / 相机与主机时钟不同源时量到的是垃圾值, 落在合理区间外就丢弃,
// 回落到固定值, 不会把提前量带飞。
void buffUpdateVisionDelayEstimate()
{
    if (g_frame_image_stamp.isZero()) {
        return;  // 相机没给时间戳(header.stamp=0), 只能靠固定值
    }
    const double age = (ros::Time::now() - g_frame_image_stamp).toSec();
    vision_delay_measured_seconds = age;
    if (!std::isfinite(age) || age < kVisionDelayMinSeconds ||
        age > kVisionDelayMaxSeconds) {
        ROS_WARN_THROTTLE(
            5.0,
            "[BUFF_DELAY] 实测视觉延迟 %.1f ms 超出合理区间 [%.1f, %.1f] ms, "
            "已忽略(继续用固定值 %.1f ms)",
            age * 1000.0, kVisionDelayMinSeconds * 1000.0,
            kVisionDelayMaxSeconds * 1000.0, vision_delay_seconds * 1000.0);
        return;
    }
    vision_delay_ema_seconds = (vision_delay_ema_seconds < 0.0)
        ? age
        : 0.9 * vision_delay_ema_seconds + 0.1 * age;//EMA(约10帧时间常数)
}

double pitch_compensation = -0.06;//pitch补偿
double yaw_compensation = 0.02;//yaw补偿
// double pitch_compensation = 0.0;//pitch补偿
// double yaw_compensation = 0.0;//yaw补偿
Fu_angleprocess* Angleprocess = nullptr;;//创建角度处理器
buff_predicted* Predicted = nullptr;;//创建预测器
buff_zju::BigBuffRansacPredictor* g_big_buff_predictor = nullptr;
buff_zju::BigBuffVariableSpeedPredictor* g_big_variable_speed_predictor = nullptr;
buff_zju::BigBuffPredictionGate g_big_prediction_gate;
double g_big_variable_speed_prediction_lead = 0.0;
bool g_big_variable_speed_prediction_valid = false;
PnPSolver_gravity* pnpSolver_gravity = nullptr;;//创建重力补偿器
CoordinateTransformer* g_coorConverter = nullptr;   //相机→枪管坐标变换
SmallBuffPhaseTracker g_small_buff_tracker;

//创建中间变量
// Frame time shared by the legacy angle history and the large-buff predictor.
double time_milliseconds = 0;//计时(单位毫秒)
double angle_diff_temp = 0;//差分角度
vector<double> angle_diff;//差分角度容器
vector<double> angle_diff_time;//时间容器
double predicted_diff_angle = 0;//预测差分角
double final_angle = 0;//最终角度(原角度加上预测角)
cv::Point2f final_angle_2D = {0, 0};//最终角度对应的图像点
cv::Point2f euler_angles = {0, 0};//储存发送的空间角

//串口订阅回调函数
//作用：doserial是串口回调函数，拷贝电控云台、四元数姿态数据，接收指令切换视觉开关与红蓝阵营标识，控制能量机关视觉程序运行状态。
//rm_msgs::RmSerial：自定义 ROS 串口消息包，存储云台编码器、电控下发的各类数据；
//ConstPtr：只读智能指针，回调接收串口话题传来的数据；
//serail：消息变量，存放云台 Pitch、Yaw 角度等电控回传信息；

#include "buff_node_runtime.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Fu");
    ros::NodeHandle nh;//创建节点句柄，用于发布话题、订阅话题、获取参数等

    ros::NodeHandle private_nh("~");//创建私有节点句柄，用于获取私有参数等


    // ═══ 加载 Buff 关键点模型 + 创建识别器 ═══
    // 模型: YOLO11-pose, OpenVINO IR, [1,3,640,640]→[1,33,8400]
    // 标签: enemy_color → enemy_class_idx_ (0=red行4, 1=blue行5)
    // 构造函数: read_model→compile_model(CPU)→create_infer_request
    // 详细文档见 README_buff_upgrade.md
    std::string buff_model_path, enemy_color, fu_config_path;
    std::string buff_target_priority;
    double buff_target_lock_timeout_seconds = 0.25;
    private_nh.param<std::string>("buff_model_path", buff_model_path,
        "Model/fu_moxi/buff_repvgg.xml");
    private_nh.param<std::string>("enemy_color", enemy_color, "blue");
    private_nh.param<std::string>("fu_config_path", fu_config_path, "");
    private_nh.param<std::string>("buff_target_priority",
                                  buff_target_priority, "locked");
    private_nh.param<double>("buff_target_lock_timeout_seconds",
                             buff_target_lock_timeout_seconds, 0.25);
    private_nh.param<bool>("buff_big_ransac_enabled", g_buff_big_ransac_enabled, true);
    private_nh.param<bool>("buff_big_variable_speed_enabled",
                           g_buff_big_variable_speed_enabled, true);
    private_nh.param<std::string>("buff_move_mode", MoveMode, MoveMode);
    if (MoveMode != "small" && MoveMode != "big") {
        ROS_WARN("[Fu] Unsupported buff_move_mode='%s', fallback to small", MoveMode.c_str());
        MoveMode = "small";
    }

    private_nh.param<bool>("buff_world_ekf_enabled", g_buff_world_ekf_enabled, false);//是否启用世界帧跟踪器
    private_nh.param<bool>("buff_world_ekf_control_enabled",
                           g_buff_world_ekf_control_enabled, false);//是否启用世界帧跟踪器控制
    private_nh.param<bool>("buff_world_ekf_primary",
                           g_buff_world_ekf_primary, false);

    private_nh.param<bool>("buff_quaternion_is_imu_to_map",
                           g_quaternion_is_imu_to_map, true);
    private_nh.param<double>("buff_imu_stamp_offset_seconds",
                             g_imu_stamp_offset_seconds, 0.0);

    if (g_buff_big_variable_speed_enabled) {
        g_big_variable_speed_predictor =
            new buff_zju::BigBuffVariableSpeedPredictor();
    }

    // ═══ 时间补偿参数(提前量 = 飞行时间 + 视觉链路延迟 + 电控延时) ═══
    private_nh.param<double>("buff_vision_delay_seconds",
                             vision_delay_seconds, 0.10);
    private_nh.param<double>("buff_ec_delay_seconds",
                             ec_delay_seconds, 0.05);
    private_nh.param<bool>("buff_vision_delay_auto",
                           vision_delay_auto, false);
    // 负延时没有物理意义(会让提前量倒着算), 直接夹到 0
    vision_delay_seconds = std::max(0.0, vision_delay_seconds);
    ec_delay_seconds = std::max(0.0, ec_delay_seconds);
    private_nh.param<double>("buff_target_width_mm", g_buff_target_width_mm, 300.0);
    private_nh.param<double>("buff_target_height_mm", g_buff_target_height_mm, 300.0);
    const std::vector<double> default_buff_camera_matrix = g_buff_camera_matrix;//默认相机矩阵
    const std::vector<double> default_buff_dist_coeffs = g_buff_dist_coeffs;//默认畸变系数矩阵
    private_nh.param<std::vector<double>>(
        "buff_camera_matrix", g_buff_camera_matrix, default_buff_camera_matrix);//获取相机矩阵
    private_nh.param<std::vector<double>>(
        "buff_dist_coeffs", g_buff_dist_coeffs, default_buff_dist_coeffs);//获取畸变系数矩阵
    if (g_buff_camera_matrix.size() != 9 || g_buff_dist_coeffs.size() != 5) {
        ROS_FATAL("[Fu] Invalid Buff PnP calibration: camera matrix needs 9 values, distortion needs 5");
        return 1;
    }

    double buff_radius_mm = 0.0;
    double buff_hub_normal_offset_mm = 0.0;
    private_nh.param<double>("buff_radius_mm", buff_radius_mm, 0.0);
    private_nh.param<double>("buff_hub_normal_offset_mm",
                             buff_hub_normal_offset_mm, 0.0);
    private_nh.param<double>("buff_world_center_measurement_std_m",
                             g_buff_world_config.center_measurement_std_m, 0.015);
    private_nh.param<double>("buff_world_normal_measurement_std_rad",
                             g_buff_world_config.normal_measurement_std_rad, 0.030);
    private_nh.param<double>("buff_world_phase_measurement_std_rad",
                             g_buff_world_config.phase_measurement_std_rad, 0.040);
    private_nh.param<double>("buff_world_center_process_std_mps",
                             g_buff_world_config.center_process_std_mps, 0.010);
    private_nh.param<double>("buff_world_normal_process_std_radps",
                             g_buff_world_config.normal_process_std_radps, 0.010);
    private_nh.param<double>("buff_world_omega_process_std_radps2",
                             g_buff_world_config.omega_process_std_radps2, 0.250);
    g_buff_world_config.radius_m = std::max(0.0, buff_radius_mm * 0.001);
    g_buff_world_config.hub_normal_offset_m =
        buff_hub_normal_offset_mm * 0.001;
    g_buff_world_tracker.configure(g_buff_world_config);

    const bool big_ransac_active = buff_zju::is_big_buff_ransac_active(
        g_buff_big_ransac_enabled, MoveMode);
    if (big_ransac_active) {
        buff_zju::BigBuffRansacConfig big_ransac_config;
        private_nh.param<int>(
            "buff_big_ransac_max_iterations",
            big_ransac_config.ransac.max_iterations,
            big_ransac_config.ransac.max_iterations);
        private_nh.param<double>(
            "buff_big_ransac_threshold",
            big_ransac_config.ransac.threshold,
            big_ransac_config.ransac.threshold);
        private_nh.param<double>(
            "buff_big_ransac_min_omega",
            big_ransac_config.ransac.min_omega,
            big_ransac_config.ransac.min_omega);
        private_nh.param<double>(
            "buff_big_ransac_max_omega",
            big_ransac_config.ransac.max_omega,
            big_ransac_config.ransac.max_omega);
        private_nh.param<double>(
            "buff_big_ransac_min_amplitude",
            big_ransac_config.ransac.min_amplitude,
            big_ransac_config.ransac.min_amplitude);
        private_nh.param<double>(
            "buff_big_ransac_max_amplitude",
            big_ransac_config.ransac.max_amplitude,
            big_ransac_config.ransac.max_amplitude);
        private_nh.param<int>(
            "buff_big_ransac_min_inliers",
            big_ransac_config.min_inliers,
            big_ransac_config.min_inliers);
        private_nh.param<int>(
            "buff_big_ransac_fit_every_n_samples",
            big_ransac_config.fit_every_n_samples,
            big_ransac_config.fit_every_n_samples);
        private_nh.param<double>(
            "buff_big_ransac_max_abs_speed",
            big_ransac_config.max_abs_speed,
            big_ransac_config.max_abs_speed);
        private_nh.param<double>(
            "buff_big_ransac_max_gap_seconds",
            big_ransac_config.max_gap_seconds,
            big_ransac_config.max_gap_seconds);
        private_nh.param<int>(
            "buff_big_ransac_max_consecutive_rejections",
            big_ransac_config.max_consecutive_rejections,
            big_ransac_config.max_consecutive_rejections);
        // 参数清洗：非有限/越界值回落到默认值，保证 predictor 与 fitter 使用
        // 同一套规则（构造前完成清洗，避免两套 max_gap 规则不一致）
        const buff_zju::BigBuffRansacConfig sanitized_cfg =
            buff_zju::sanitize_big_buff_ransac_config(big_ransac_config);
        if (sanitized_cfg != big_ransac_config) {
            ROS_WARN("[Fu] buff_big_ransac_* params sanitized (non-finite or out-of-range values replaced by defaults)");
        }
        g_big_buff_predictor = new buff_zju::BigBuffRansacPredictor(sanitized_cfg);
        ROS_INFO(
            "[Fu] ZJU big-buff RANSAC enabled: omega=[%.3f, %.3f], min_inliers=%d, max_rejections=%d",
            sanitized_cfg.ransac.min_omega,
            sanitized_cfg.ransac.max_omega,
            sanitized_cfg.min_inliers,
            sanitized_cfg.max_consecutive_rejections);
    }

    ROS_INFO("[Fu] Buff mode: %s, variable-speed primary: %s, ZJU big-buff RANSAC: %s",
             MoveMode.c_str(),
             (MoveMode == "big" && g_buff_big_variable_speed_enabled)
                 ? "active" : "inactive",
             big_ransac_active ? "active" : "inactive");
    ROS_INFO("[Fu] Buff model: %s", buff_model_path.c_str());
    ROS_INFO("[Fu] Enemy color: %s", enemy_color.c_str());
    ROS_INFO("[Fu] World EKF: %s, control: %s, primary: %s, q IMU->map: %s, target: %.1f x %.1f mm",
             g_buff_world_ekf_enabled ? "enabled" : "disabled",
             g_buff_world_ekf_control_enabled ? "enabled" : "disabled",
             g_buff_world_ekf_primary ? "enabled" : "disabled",
             g_quaternion_is_imu_to_map ? "true" : "false",
             g_buff_target_width_mm, g_buff_target_height_mm);
    ROS_INFO("[Fu] Buff PnP K: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
             g_buff_camera_matrix[0], g_buff_camera_matrix[4],
             g_buff_camera_matrix[2], g_buff_camera_matrix[5]);
    ROS_INFO("[Fu] 时间补偿: 视觉链路延迟=%.1f ms(%s), 电控延时=%.1f ms, 合计=%.1f ms",
             vision_delay_seconds * 1000.0,
             vision_delay_auto ? "实测" : "固定",
             ec_delay_seconds * 1000.0,
             buffTotalLeadCompensation() * 1000.0);

    g_recognizer = new BuffRecognizer(buff_model_path, enemy_color);
    g_recognizer->setTargetPriority(
        MoveMode == "big" ? buff_target_priority : "confidence",
        buff_target_lock_timeout_seconds);
    g_recognizer->setEgoMotionIntrinsics(
        static_cast<float>(g_buff_camera_matrix[0]),
        static_cast<float>(g_buff_camera_matrix[4]));

    ros::Subscriber SubCamera = nh.subscribe("/hikrobot_camera/rgb", 1, &doimage);
    ros::Subscriber SubRmSerial = nh.subscribe("/RmSerialData", 1000, &doserial);
    PubAngle = nh.advertise<geometry_msgs::Vector3>("/auto_angle",1000);
    debugpub = nh.advertise<std_msgs::Float64>("/debugpub", 1000);
    debugpub1 = nh.advertise<std_msgs::Float64>("/debugpub1", 1000);
    debugpub2 = nh.advertise<std_msgs::Float64>("/debugpub2", 1000);
    debugpub3 = nh.advertise<std_msgs::Float64>("/debugpub3", 1000);
    debugpub4 = nh.advertise<std_msgs::Float64>("/debugpub4", 1000);
    world_ready_pub = nh.advertise<std_msgs::Bool>("/buff/world_ready", 1000);

    try {
        if (fu_config_path.empty()) {
            g_coorConverter = new CoordinateTransformer();
            ROS_WARN("[Fu] No Fu coordinate configuration supplied; using built-in defaults");
        } else {
            g_coorConverter = new CoordinateTransformer(fu_config_path);
            ROS_INFO("[Fu] CoordinateTransformer loaded from: %s", fu_config_path.c_str());
        }
    } catch (const std::exception& error) {
        ROS_FATAL("[Fu] Invalid coordinate configuration '%s': %s",
                  fu_config_path.c_str(), error.what());
        return 1;
    }

    while (ros::ok())
    {
        // // ROS_INFO("循环正在进行中");
        // // cout<<"fu_task"<<endl;   
                ros::spinOnce(); 
        // cout << "11111111111111111111111111111111111111111111111111" << endl;
        
        //校正云台原始角度，封装后分两路发布：
        //一路单独输出俯仰、偏航用于调试观测，一路下发完整三轴角度给电控控制云台追踪目标。
        if(!Fu_Task()){
            // 跟踪失败时的保底: 仅在打符模式下发布回传角度
            // 装甲模式下不发布, 由 Tracker 节点通过 /auto_angle 负责
            if(functional){
                Angle.x = -RmSerialData.Pitch; // 保底: 发送平台pitch反号
                Angle.y = RmSerialData.Yaw;
                Angle.z = 1;
                std_msgs::Float64 debug_msg_x;
                std_msgs::Float64 debug_msg_y;
                debug_msg_x.data = Angle.x;              // 期望角pitch
                debug_msg_y.data = -RmSerialData.Pitch;   // 平提出轴角度
                debugpub.publish(debug_msg_x);
                debugpub1.publish(debug_msg_y);
                PubAngle.publish(Angle);
            }
        }


        if (!frame.empty()) {
            imshow("Image with Point", frame);
            if (cv::waitKey(1) == 'q') {
                return 0; // 按'q'键退出
            }
        }else{
            // cout<<"fu_task222222"<<endl; 
        }
        // ros::spinOnce();
        
    }
    
    return 0;
}
