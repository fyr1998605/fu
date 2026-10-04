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
#include <functional>
 #include <rclcpp/rclcpp.hpp>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/msg/image.hpp>
 #include <sensor_msgs/image_encodings.hpp>
 #include <geometry_msgs/msg/vector3.hpp>
 #include <std_msgs/msg/float64.hpp>
 #include <std_msgs/msg/bool.hpp>
 #include "ros2_compat.hpp"
 #include "ros2_energy_helpers.hpp"
 #include "Image.h"
 #include "buff_recognizer.hpp"   // Buff识别器: YOLO11关键点模型
 #include "blade_tracker.hpp"     // Small-buff phase tracking and prediction
#include "buff_world_tracker.hpp"
#include "buff_big_ransac_predictor.hpp"
 #include "CoorConverter.hpp"    // 坐标变换: 相机→IMU→枪管
 #include "rm_msgs/msg/rm_serial.hpp"

 // 调试
////////////////////////////
#include <chrono>
#include <cstdint>
#include <thread>
////////////////////////////

using namespace cv;
using namespace std;


rm_msgs::msg::RmSerial RmSerialData;

// Energy-mechanism-only runtime settings. They leave CoordinateTransformer,
// recognizer, serial messages, and the armor tracker unchanged.
bool g_buff_world_ekf_enabled = false;
bool g_buff_world_ekf_control_enabled = false;
bool g_quaternion_is_imu_to_map = true;
bool g_buff_big_ransac_enabled = true;

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
   1692.72620365811, 0.363957161762693,643.964129877484, 0 ,1692.43288845269 ,502.492890249832, 0 ,0 , 1};
std::vector<double> g_buff_dist_coeffs{
    -0.111874561711212, -0.000296513404483851, 0.884391394501360, -0.00202166770416446,-1.84111324474713};
BuffWorldTrackerConfig g_buff_world_config;
BuffWorldTracker g_buff_world_tracker;

// Every energy-mechanism frame must use the IMU pose measured at the same time.
// The direct and inverse quaternion conventions are both logged before one is chosen.
struct TimedImuPose {
    rclcpp::Time stamp;
    Eigen::Quaterniond imu_to_map;
    double raw_norm = 0.0;
};

constexpr std::size_t kImuPoseHistoryCapacity = 200;
constexpr double kMaximumImuImageSkewSeconds = 0.05;
std::deque<TimedImuPose> g_imu_pose_history;
TimedImuPose g_frame_imu_pose;
rclcpp::Time g_frame_image_stamp(0, RCL_ROS_TIME);
std::uint64_t g_frame_sequence = 0;
std::uint64_t g_processed_frame_sequence = 0;
double g_last_big_buff_timestamp =
    std::numeric_limits<double>::quiet_NaN();
bool g_big_buff_timestamp_from_image = false;
bool g_has_frame_imu_pose = false;
double g_frame_imu_image_skew_seconds =
    std::numeric_limits<double>::quiet_NaN();

bool findClosestImuPose(const rclcpp::Time& image_stamp, TimedImuPose& matched_pose) {
    if (image_stamp.nanoseconds() == 0 || g_imu_pose_history.empty()) {
        return false;
    }

    const TimedImuPose* closest_pose = nullptr;
    double closest_skew_seconds = std::numeric_limits<double>::infinity();
    for (const auto& pose : g_imu_pose_history) {
        const rclcpp::Time corrected_stamp = pose.stamp +
            rclcpp::Duration::from_seconds(g_imu_stamp_offset_seconds);
        const double skew_seconds =
            std::abs((corrected_stamp - image_stamp).seconds());
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
geometry_msgs::msg::Vector3 Angle;
double Qtn[4];
Ros2Publisher<geometry_msgs::msg::Vector3> PubAngle;
Ros2Publisher<std_msgs::msg::Float64> debugpub;            //debug发布者
Ros2Publisher<std_msgs::msg::Float64> debugpub1;           //debug1发布者
Ros2Publisher<std_msgs::msg::Float64> debugpub2;           //debug2发布者  (buff世界卡尔曼 omega)
Ros2Publisher<std_msgs::msg::Float64> debugpub3;           //debug3发布者  (buff世界卡尔曼 |K-M| px)
Ros2Publisher<std_msgs::msg::Float64> debugpub4;           //debug4发布者  (buff世界卡尔曼 |P-M| px)
Ros2Publisher<std_msgs::msg::Bool> world_ready_pub;     // /buff/world_ready 发布者
/*主程序*/

bool init = false;//初始化标识符
cv::Mat frame;//处理帧
cv::Mat frame_;//接收帧
bool functional = false;//Wait for the serial mode command before publishing Fu angles.
double delta_t_ = 0.3;//差分时间
double delta_t_compensation = 0.15;//程序运行时间+发弹时间(时间补偿)
// double delta_t_compensation = 0.17;//程序运行时间+发弹时间(时间补偿)
double pitch_compensation = 0.0;//pitch补偿
double yaw_compensation = 0.0;//yaw补偿
// double pitch_compensation = 0.0;//pitch补偿
// double yaw_compensation = 0.0;//yaw补偿
Fu_angleprocess* Angleprocess = nullptr;;//创建角度处理器
buff_predicted* Predicted = nullptr;;//创建预测器
buff_zju::BigBuffRansacPredictor* g_big_buff_predictor = nullptr;
PnPSolver_gravity* pnpSolver_gravity = nullptr;;//创建重力补偿器
CoordinateTransformer* g_coorConverter = nullptr;   //相机→枪管坐标变换
SmallBuffPhaseTracker g_small_buff_tracker;

//创建中间变量
// Legacy processing-elapsed clock used by the original small-buff path. The
// large-buff predictor uses the image header clock in buff_node_runtime.hpp.
double time_milliseconds = 0;//计时(单位毫秒)
double angle_diff_temp = 0;//差分角度
vector<double> angle_diff;//差分角度容器
vector<double> angle_diff_time;//时间容器
double predicted_diff_angle = 0;//预测差分角
double final_angle = 0;//最终角度(原角度加上预测角)
cv::Point2f final_angle_2D = {0, 0};//最终角度对应的图像点
cv::Point2f euler_angles = {0, 0};//储存发送的空间角

//时间参数w
std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

//串口订阅回调函数
//作用：doserial是串口回调函数，拷贝电控云台、四元数姿态数据，接收指令切换视觉开关与红蓝阵营标识，控制能量机关视觉程序运行状态。
// rm_msgs::msg::RmSerial stores gimbal and controller state.
//ConstPtr：只读智能指针，回调接收串口话题传来的数据；
//serail：消息变量，存放云台 Pitch、Yaw 角度等电控回传信息；

#include "buff_node_runtime.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Fu");
    g_ros_logger = node->get_logger();
    g_ros_clock = node->get_clock();
    Ros2ParamNode private_nh(node);


    // ═══ 加载 Buff 关键点模型 + 创建识别器 ═══
    // 模型: YOLO11-pose, OpenVINO IR, [1,3,640,640]→[1,33,8400]
    // 标签: enemy_color → enemy_class_idx_ (0=red行4, 1=blue行5)
    // 构造函数: read_model→compile_model(CPU)→create_infer_request
    // 详细文档见 README_buff_upgrade.md
    std::string buff_model_path, enemy_color, fu_config_path;
    private_nh.param<std::string>("buff_model_path", buff_model_path,
        "Model/fu_moxi/buff_repvgg.xml");
    private_nh.param<std::string>("enemy_color", enemy_color, "blue");
    private_nh.param<std::string>("fu_config_path", fu_config_path, "");
    private_nh.param<bool>("buff_big_ransac_enabled", g_buff_big_ransac_enabled, true);
    private_nh.param<std::string>("buff_move_mode", MoveMode, MoveMode);
    if (MoveMode != "small" && MoveMode != "big") {
        ROS_WARN("[Fu] Unsupported buff_move_mode='%s', fallback to small", MoveMode.c_str());
        MoveMode = "small";
    }

    private_nh.param<bool>("buff_world_ekf_enabled", g_buff_world_ekf_enabled, false);//是否启用世界帧跟踪器
    private_nh.param<bool>("buff_world_ekf_control_enabled",
                           g_buff_world_ekf_control_enabled, false);//是否启用世界帧跟踪器控制

    private_nh.param<bool>("buff_quaternion_is_imu_to_map",
                           g_quaternion_is_imu_to_map, true);
    private_nh.param<double>("buff_imu_stamp_offset_seconds",
                             g_imu_stamp_offset_seconds, 0.0);
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

    ROS_INFO("[Fu] Buff mode: %s, ZJU big-buff RANSAC: %s",
             MoveMode.c_str(),
             big_ransac_active ? "active" : "inactive");
    ROS_INFO("[Fu] Buff model: %s", buff_model_path.c_str());
    ROS_INFO("[Fu] Enemy color: %s", enemy_color.c_str());
    ROS_INFO("[Fu] World EKF: %s, control: %s, q IMU->map: %s, target: %.1f x %.1f mm",
             g_buff_world_ekf_enabled ? "enabled" : "disabled",
             g_buff_world_ekf_control_enabled ? "enabled" : "disabled",
             g_quaternion_is_imu_to_map ? "true" : "false",
             g_buff_target_width_mm, g_buff_target_height_mm);
    ROS_INFO("[Fu] Buff PnP K: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
             g_buff_camera_matrix[0], g_buff_camera_matrix[4],
             g_buff_camera_matrix[2], g_buff_camera_matrix[5]);

    g_recognizer = new BuffRecognizer(buff_model_path, enemy_color);

    auto SubCamera = node->create_subscription<sensor_msgs::msg::Image>(
        "/hikrobot_camera/rgb", rclcpp::SensorDataQoS(),
        [](const sensor_msgs::msg::Image::ConstSharedPtr message) {
            doimage(message);
        });
    auto SubRmSerial = node->create_subscription<rm_msgs::msg::RmSerial>(
        "/RmSerialData", rclcpp::QoS(1000),
        [](const rm_msgs::msg::RmSerial::ConstSharedPtr message) {
            doserial(message);
        });
    PubAngle.handle = node->create_publisher<geometry_msgs::msg::Vector3>(
        "/auto_angle", rclcpp::QoS(1000));
    debugpub.handle = node->create_publisher<std_msgs::msg::Float64>(
        "/debugpub", rclcpp::QoS(1000));
    debugpub1.handle = node->create_publisher<std_msgs::msg::Float64>(
        "/debugpub1", rclcpp::QoS(1000));
    debugpub2.handle = node->create_publisher<std_msgs::msg::Float64>(
        "/debugpub2", rclcpp::QoS(1000));
    debugpub3.handle = node->create_publisher<std_msgs::msg::Float64>(
        "/debugpub3", rclcpp::QoS(1000));
    debugpub4.handle = node->create_publisher<std_msgs::msg::Float64>(
        "/debugpub4", rclcpp::QoS(1000));
    world_ready_pub.handle = node->create_publisher<std_msgs::msg::Bool>(
        "/buff/world_ready", rclcpp::QoS(1000));

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

    auto processing_timer = node->create_wall_timer(
        std::chrono::milliseconds(1),
        []() {
            if (!Fu_Task() && functional) {
                Angle.x = -RmSerialData.pitch;
                Angle.y = RmSerialData.yaw;
                Angle.z = 1;
                std_msgs::msg::Float64 debug_msg_x;
                std_msgs::msg::Float64 debug_msg_y;
                debug_msg_x.data = Angle.x;
                debug_msg_y.data = -RmSerialData.pitch;
                debugpub.publish(debug_msg_x);
                debugpub1.publish(debug_msg_y);
                PubAngle.publish(Angle);
            }

            if (!frame.empty()) {
                cv::imshow("Image with Point", frame);
                if (cv::waitKey(1) == 'q') {
                    rclcpp::shutdown();
                }
            }
        });

    (void)processing_timer;
    rclcpp::spin(node);
    rclcpp::shutdown();

    delete Angleprocess;
    delete Predicted;
    delete pnpSolver_gravity;
    delete g_coorConverter;
    delete g_big_buff_predictor;
    delete g_recognizer;
    return 0;
}
