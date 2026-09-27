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
#include "buff_move_mode_detector.hpp"
#include "buff_config.hpp"       // 能量机关参数: YAML 加载/校验/启动打印
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
bool g_buff_world_apply_gimbal_pose = true;
bool g_quaternion_is_imu_to_map = true;
bool g_buff_big_ransac_enabled = true;
bool g_buff_move_mode_auto = false;
bool g_run_buff_without_serial = false;
buff::MoveModeDetector g_buff_move_mode_detector;

// 能量机关全部参数 (YAML -> 结构体)。P1 阶段只加载与打印,
// 参数尚未被下游使用, 因此行为与改造前完全一致。
buff::BuffConfig g_buff_config;

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
   1329.24429634650, 0.518156995670085, 661.300343956123,
    0,                1325.89618530323,   452.645884191168,
    0,                0,                   1};
std::vector<double> g_buff_dist_coeffs{
        -0.0442540638030886,
    0.0902922934048078,
    -0.00375815319429763,
    0.00269367029458212,
    0.0308390738152262};
BuffWorldTrackerConfig g_buff_world_config;
BuffWorldTracker g_buff_world_tracker;

// Every energy-mechanism frame must use the IMU pose measured at the same time.
// The direct and inverse quaternion conventions are both logged before one is chosen.
struct TimedImuPose {
    ros::Time stamp;
    Eigen::Quaterniond imu_to_map;
    double raw_norm = 0.0;
};

// IMU 位姿历史容量与图-IMU 最大时差已参数化: 见 Buff.sync.imu_pose_history_capacity /
// Buff.sync.max_imu_image_skew_seconds (buff_config.hpp), 原 constexpr 常量已删除。
std::deque<TimedImuPose> g_imu_pose_history;
TimedImuPose g_frame_imu_pose;
ros::Time g_frame_image_stamp;
std::uint64_t g_frame_sequence = 0;
std::uint64_t g_processed_frame_sequence = 0;
double g_last_big_buff_timestamp =
    std::numeric_limits<double>::quiet_NaN();
bool g_big_buff_timestamp_from_image = false;
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
        closest_skew_seconds > g_buff_config.sync.max_imu_image_skew_seconds) {  // 原 constexpr 0.05
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
ros::Publisher debugpub4;           //debug4发布者  (|扇叶中心−R中心| px)
ros::Publisher track_ok_pub;        // /buff/track_ok 发布者 (本帧跟踪状态)
ros::Publisher world_ready_pub;     // /buff/world_ready 发布者
/*主程序*/

bool init = false;//初始化标识符
cv::Mat frame;//处理帧
cv::Mat frame_;//接收帧
bool functional = true;//写死打符模式: 电控暂未实现ShootFlag切换, 默认直接识别能量机关发pitch/yaw
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
    // ═══ 能量机关参数: 唯一真源是 rm_fu.yaml 的 Buff: 节 ═══
    // 与自瞄 (RmTracker::setParam / ModelObserver::setParam) 同一套做法:
    // 结构化分节 + 逐键默认值 + 取值域校验 + 启动打印实际生效值。
    // launch <param> 只覆盖少数需要运行时传入的键, 其余全部走 yaml。
    std::string buff_model_path, enemy_color, fu_config_path;
    private_nh.param<std::string>("fu_config_path", fu_config_path, "");
    private_nh.param<std::string>("buff_model_path", buff_model_path, "");
    private_nh.param<std::string>("enemy_color", enemy_color, "");

    if (fu_config_path.empty()) {
        ROS_WARN("[Fu] No Buff configuration supplied; Buff.* parameters use "
                 "built-in defaults (identical to the previous hardcoded values)");
    } else {
        std::vector<std::string> buff_config_warnings;
        std::string buff_config_error;
        if (!buff::loadBuffConfig(fu_config_path, g_buff_config,
                                  buff_config_warnings, buff_config_error)) {
            ROS_FATAL("[Fu] Buff configuration rejected: %s",
                      buff_config_error.c_str());
            return 1;
        }
        for (const std::string& warning : buff_config_warnings) {
            ROS_WARN("[Fu] %s", warning.c_str());
        }
        ROS_INFO("[Fu] Buff configuration loaded from: %s", fu_config_path.c_str());
        for (const std::string& line : buff::buffConfigToLines(g_buff_config)) {
            ROS_INFO("[Fu][cfg] %s", line.c_str());
        }
    }

    // launch <param> 覆盖 yaml (仅模型路径 / 敌方颜色 / 无串口调试开关)
    if (!buff_model_path.empty()) {
        g_buff_config.model.model_path = buff_model_path;
    }
    if (!enemy_color.empty()) {
        g_buff_config.model.enemy_color = enemy_color;
    }
    private_nh.param<bool>("run_buff_without_serial",
                           g_buff_config.model.run_without_serial,
                           g_buff_config.model.run_without_serial);

    // ═══ 从 Buff 配置派生全局状态 (保持下游 extern 引用不变, 行为与迁移前一致) ═══
    const buff::BuffConfig& cfg = g_buff_config;

    // 模型 / 模式
    buff_model_path = cfg.model.model_path;
    enemy_color = cfg.model.enemy_color;
    g_buff_move_mode_auto = (cfg.model.move_mode == "auto");
    MoveMode = g_buff_move_mode_auto ? "small" : cfg.model.move_mode;
    if (MoveMode != "small" && MoveMode != "big") {
        MoveMode = "small";
    }
    g_run_buff_without_serial = cfg.model.run_without_serial;
    functional = cfg.model.functional_default;

    // 同步 / 世界 EKF 开关
    g_imu_stamp_offset_seconds = cfg.sync.imu_stamp_offset_seconds;
    g_buff_world_ekf_enabled = cfg.world_tracker.ekf_enabled;
    g_buff_world_ekf_control_enabled = cfg.world_tracker.ekf_control_enabled;
    g_buff_world_apply_gimbal_pose = cfg.world_tracker.apply_gimbal_pose;
    g_quaternion_is_imu_to_map = cfg.world_tracker.quaternion_is_imu_to_map;

    // 靶面几何 / 相机标定
    g_buff_target_width_mm = cfg.geometry.target_width_mm;
    g_buff_target_height_mm = cfg.geometry.target_height_mm;
    g_buff_camera_matrix = cfg.geometry.camera_matrix;
    g_buff_dist_coeffs = cfg.geometry.dist_coeffs;
    if (g_buff_camera_matrix.size() != 9 || g_buff_dist_coeffs.size() != 5) {
        ROS_FATAL("[Fu] Invalid Buff PnP calibration: camera matrix needs 9 values, distortion needs 5");
        return 1;
    }

    // 时延 / 云台补偿 (全局变量, buff_node_runtime.hpp 使用)
    delta_t_ = cfg.timing.delta_t_init;
    delta_t_compensation = cfg.timing.delta_t_compensation;
    pitch_compensation = cfg.control.pitch_compensation;
    yaw_compensation = cfg.control.yaw_compensation;

    // 世界 EKF 配置
    g_buff_world_config.center_measurement_std_m = cfg.world_tracker.center_measurement_std_m;
    g_buff_world_config.normal_measurement_std_rad = cfg.world_tracker.normal_measurement_std_rad;
    g_buff_world_config.phase_measurement_std_rad = cfg.world_tracker.phase_measurement_std_rad;
    g_buff_world_config.center_process_std_mps = cfg.world_tracker.center_process_std_mps;
    g_buff_world_config.normal_process_std_radps = cfg.world_tracker.normal_process_std_radps;
    g_buff_world_config.omega_process_std_radps2 = cfg.world_tracker.omega_process_std_radps2;
    g_buff_world_config.radius_m = std::max(0.0, cfg.geometry.radius_mm * 0.001);
    g_buff_world_config.hub_normal_offset_m = cfg.geometry.hub_normal_offset_mm * 0.001;
    g_buff_world_tracker.configure(g_buff_world_config);
    g_buff_world_tracker.configure(cfg.world_tracker);  // 内部门限/方向锁/ready 滞回等

    // 小符像素卡尔曼 (Buff.angle.small_buff_*)
    g_small_buff_tracker.configure(cfg.angle);

    // 大小符自动判别 (Buff.legacy_predictor.mode_detector_*)
    g_buff_move_mode_detector.configure(cfg.legacy_predictor);

    // 大符 RANSAC
    g_buff_big_ransac_enabled = cfg.big_ransac.enabled;
    const bool big_ransac_active = buff_zju::is_big_buff_ransac_active(
        g_buff_big_ransac_enabled, MoveMode);
    if (big_ransac_active) {
        buff_zju::BigBuffRansacConfig big_ransac_config;
        big_ransac_config.ransac.max_iterations = cfg.big_ransac.max_iterations;
        big_ransac_config.ransac.threshold = cfg.big_ransac.threshold;
        big_ransac_config.ransac.min_omega = cfg.big_ransac.min_omega;
        big_ransac_config.ransac.max_omega = cfg.big_ransac.max_omega;
        big_ransac_config.ransac.min_amplitude = cfg.big_ransac.min_amplitude;
        big_ransac_config.ransac.max_amplitude = cfg.big_ransac.max_amplitude;
        big_ransac_config.ransac.random_seed = cfg.big_ransac.random_seed;
        big_ransac_config.min_inliers = cfg.big_ransac.min_inliers;
        big_ransac_config.max_abs_speed = cfg.big_ransac.max_abs_speed;
        big_ransac_config.max_gap_seconds = cfg.big_ransac.max_gap_seconds;
        big_ransac_config.max_consecutive_rejections =
            cfg.big_ransac.max_consecutive_rejections;
        // 参数清洗：非有限/越界值回落到默认值，保证 predictor 与 fitter 使用同一套规则
        const buff_zju::BigBuffRansacConfig sanitized_cfg =
            buff_zju::sanitize_big_buff_ransac_config(big_ransac_config);
        if (sanitized_cfg != big_ransac_config) {
            ROS_WARN("[Fu] Buff.big_ransac.* params sanitized (non-finite or out-of-range values replaced by defaults)");
        }
        g_big_buff_predictor = new buff_zju::BigBuffRansacPredictor(sanitized_cfg);
        ROS_INFO(
            "[Fu] ZJU big-buff RANSAC enabled: omega=[%.3f, %.3f], min_inliers=%d, max_rejections=%d",
            sanitized_cfg.ransac.min_omega,
            sanitized_cfg.ransac.max_omega,
            sanitized_cfg.min_inliers,
            sanitized_cfg.max_consecutive_rejections);
    }

    ROS_INFO("[Fu] Buff mode: %s%s, ZJU big-buff RANSAC: %s",
             g_buff_move_mode_auto ? "auto (start small, runtime switching), current=" : "",
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

         ROS_INFO("[Fu] run_buff_without_serial: %s",
              g_run_buff_without_serial ? "enabled" : "disabled");

     g_recognizer = new BuffRecognizer(buff_model_path, enemy_color,
                                       g_buff_config.model.model_device);
     g_recognizer->configure(g_buff_config.recognition, g_buff_config.model);   // 注入识别参数

    ros::Subscriber SubCamera = nh.subscribe("/hikrobot_camera/rgb", 1000, &doimage);
    ros::Subscriber SubRmSerial = nh.subscribe("/RmSerialData", 1000, &doserial);
    PubAngle = nh.advertise<geometry_msgs::Vector3>("/auto_angle",1000);
    debugpub = nh.advertise<std_msgs::Float64>("/debugpub", 1000);
    debugpub1 = nh.advertise<std_msgs::Float64>("/debugpub1", 1000);
    debugpub2 = nh.advertise<std_msgs::Float64>("/debugpub2", 1000);
    debugpub3 = nh.advertise<std_msgs::Float64>("/debugpub3", 1000);
    debugpub4 = nh.advertise<std_msgs::Float64>("/debugpub4", 1000);
    track_ok_pub = nh.advertise<std_msgs::Bool>("/buff/track_ok", 1000);
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
                // 保底帧不做跟踪, 期望角就是回传的平台pitch反号 (与上面的Angle.x一致)
                debug_msg_x.data = Angle.y;               // 期望角pitch
                debug_msg_y.data = RmSerialData.Yaw;    // 电控回传的云台pitch, 原始值
                debugpub.publish(debug_msg_x);
                debugpub1.publish(debug_msg_y);
                ROS_INFO_THROTTLE(
                    0.2,
                    "[Fu] yaw=%.2f deg pitch=%.2f deg mode=%s (fallback)",
                    Angle.y * 180.0 / PI,
                    Angle.x * 180.0 / PI,
                    MoveMode.c_str());
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