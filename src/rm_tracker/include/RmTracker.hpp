#pragma once
#include <vector>
#include <deque>
#include <array>
#include <limits>
#include <algorithm>

// ROS相关头文件
#include "ros/ros.h"
#include "rm_msgs/Armor.h"
#include "rm_msgs/ArmorArray.h"
#include "rm_msgs/RmSerial.h" 
#include <std_msgs/Float64.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

// OpenCV相关头文件
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// 标准库头文件
#include <iostream>
#include <cmath>
// 时间相关头文件
#include <chrono>
#include <thread>
#include <fstream>
#include <yaml-cpp/yaml.h>
// 自定义头文件
#include "Calculater.hpp"
#include "GimbalPos.hpp"
#include "TargetModel.hpp"
#include "visual.hpp"
#include "CoorConverter.hpp"
#include "math.hpp"
#include "blade_tracker.hpp"
#include "ImuData.hpp"
#include "MPC.hpp"
#include "trajectory_visualizer.hpp"
#include <ceres/ceres.h>

#ifndef D2R
#define D2R (M_PI / 180.0)
#endif

using namespace std;
using namespace cv;

// 工作模式枚举
enum class TrackTargetMode {
    ArmorMode,           // 装甲自瞄模式
    EnergyFanMode        // 能量机关模式
};

/*  
    自瞄文档
    标准模型状态向量
    X(0) -> 机器人中心的x坐标
    X(1) -> 机器人中心x方向速度
    X(2) -> 机器人中心的y坐标
    X(3) -> 机器人中心y方向速度
    X(4) -> 左侧装甲板的固定高度
    X(5) -> 右侧装甲板的固定高度
    X(6) -> 小装甲板旋转半径（左侧）
    X(7) -> 大装甲板旋转半径（右侧）
    X(8) -> 机器人整体偏航角yaw
    X(9) -> 偏航角速度palstance

    前哨站状态向量
    X(0): 机器人中心的x坐标
    X(1): 机器人中心的y坐标
    X(2): 第一块装甲板高度h1
    X(3): 第二块装甲板高度h2
    X(4): 第三块装甲板高度h3
    X(5): 偏航角yaw
    X(6): 偏航角速度palstance
*/

/**
* @brief 将弧度约束在[-pi, pi]范围内，大于n，减去2n；小于-n，加上2n
* @param angle 角度 
*/
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif


/**
 * @class ResistanceFuncLinear
 * @brief 含空气阻力的弹道计算 ceres版本
 */
class ResistanceFuncLinear {
private:
    const double g { 9.8 }; // g = 9.8
    const double w, h, v0;
    const double k;

public:
    ResistanceFuncLinear(const double& w, const double& h, const double& v0, const double k): w(w), h(h), v0(v0), k(k) {}

    template<typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = (this->k * this->v0 * ceres::sin(x[0]) + this->g) * this->k * this->w
                / (this->k * this->k * this->v0 * ceres::cos(x[0]))
            + this->g * ceres::log(1. - (this->k * this->w) / (this->v0 * ceres::cos(x[0]))) / this->k / this->k
            - this->h;
        return true;
    }
};

 
//追踪类
template<TimeSpan _trackTime>
class Tracker{
private:
    bool fan_last_valid_ = false;
    double fan_min_valid_radius_ = 10.0;      // 根据实际场景调整
    double fan_default_distance_ = 2.0;       // 默认距离
    double fan_max_valid_distance_ = 10.0;    // 最大有效距离
    double fan_target_height_ = 0.15;         // 目标高度
    double fan_last_angle_ = 0.0;
    ros::Time fan_last_publish_time_;

    double COMMAND_TIMESPAN;            //电控延迟
    double local_gravity_;              //重力加速度
    double eTime;                       //曝光时间
    double delay_time;                  //图像传输,程序运行处理等时间
    /* ======================== 系统参数 ======================== */
    //ROS相关
    ros::NodeHandle nh;                 //ROS节点句柄
    ros::Publisher debugpub;            //debug发布者  
    ros::Publisher debugpub1;           //debug1发布者
    ros::Publisher debugpub2;           //debug2发布者  
    std_msgs::Float64 debugdate;        //debug数据
    std_msgs::Float64 debugdate1;       //debug1数据
    std_msgs::Float64 debugdate2;       //debug2数据
    rm_msgs::RmSerial RmSerialData;     //接收串口数据
    tf2_ros::Buffer tfBuffer_;            // TF 缓冲区
    tf2_ros::TransformListener tfListener; // 声明一个tf2_ros::TransformListener对象，并传入tfBuffer
    rm_msgs::ArmorArrayConstPtr m_armors;  // 接收装甲板数据 

    //图像处理相关
    cv::Mat frame;          //接收的原始图像
    cv::Mat frame_;         //被处理的图像副本(保护原图像)
    cv::Mat bullet_frame;
    cv::Mat camera_matrix_; //相机内参(构造函数中)
    cv::Mat dist_coeffs_;   //畸变系数(构造函数中)
    Image img;              //图像工具

    //坐标变换工具
    CAL::Calculater* cal;   //计算工具

    CoordinateTransformer* coorConverter;

    //装甲板评分参数
    std::vector<int> col;                       // 数列中的每个数代表矩阵的每一列
    std::vector<int> row;                       // 数列中的每个数代表矩阵的每一行
    std::vector<int> tmp_v;                     // 存储C(n,k)的中间结果

    std::vector<std::vector<int>> result;       // 存储C(n,k)的结果
    std::vector<std::vector<int>> nAfour;       // 存储A(n,4)的结果
    std::vector<std::vector<int>> fourAfour;    // 存储A(4,4)的结果

    std::map<int, int> row_col;                 // 存储最终结果，row_col[i]=j表示矩阵的第i行第j列是要选取的数

    //数据关联参数
    double min = 0, tmp = 0;

    /* ======================== 跟踪控制参数 ======================== */
    //基本跟踪参数
    char TrackingID;            //跟踪中的装甲板ID
    bool Switch_Armor;          //装甲板切换标识符
    double trackTime;           //目标丢失判定时间(秒)

    //角度补偿参数
    double pitch_compensation;  //pitch补偿
    double yaw_compensation;    //yaw补偿

    // 打表硬补偿
    // ~4m硬补偿
    double m_pitch_compensation4meter;
    double m_yaw_compensation4meter;
    // 5.5m硬补偿
    double m_pitch_compensation5_5meter;
    double m_yaw_compensation5_5meter;
    // 6m硬补偿
    double m_pitch_compensation6meter;
    double m_yaw_compensation6meter;
    // 6.5m硬补偿 // 不考虑击打7m以外目标
    double m_pitch_compensation6_5meter;
    double m_yaw_compensation6_5meter;

    //火力控制参数
    bool all_fire = false;      //完全火力模式(不停止射击)

    /* ======================== 装甲板预测参数 ======================== */
    //空间参考点参数
    Eigen::Vector3d pegPos;         //空间标准点(用于装甲板跳变判断)
    double peg_point_pixel_now;     //当前装甲板在像素坐标系的位置
    double peg_point_pixel_last;    //位于像素标准坐标系的上一个装甲板位置

    //子弹参数
    double BulletVector = 21;           //子弹速度(初值给24m/s) 步兵: 21 英雄: 12
    double m_k;

    /* ======================== 时间戳管理 ======================== */
    //装甲板跟踪时间
    long long Now_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//当前时间戳(用于计算丢失时间)
    long long Track_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//丢失时间戳(用于计算丢失时间)

    //帧率计算
    long long begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//首时间
    long long end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//尾时间

    // C++工具时间戳
    long long tool_begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//首时间
    long long tool_end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//尾时间

    // ros工具时间戳
    ros::Time tool_begin_Time_ros = ros::Time::now();
    ros::Time tool_end_Time_ros = ros::Time::now();

    // 获取装甲板tf时间(判断是否是相同帧)
    ros::Time last_tf_time = ros::Time::now();
        
    /* ======================== 标志位 ======================== */
    bool functional = true;                 // 射击模式
    bool has_serial_data_ = false;
    bool observed_this_frame_ = false;
    std::chrono::steady_clock::time_point serial_received_at_, armors_received_at_;
    int m_center_tracked;                   // 锁中心状态标志位
    bool m_track_center;                    // 是否跟随中心
    bool m_fix_on;                          // 重力补偿开关

    std::string config_path_;               // 存储配置路径
    unique_ptr<TargetModel> targetModel;    // 目标模型
    ros::Publisher AngPub;                  // 角度话题发布者
    std::shared_ptr<MPC> m_MPC;             // 西工大mpc

    // =========================== 阈值 ====================================
    double m_score_tolerance;               //装甲板匹配得分最大值
    double m_switch_threshold;              // 更新装甲板切换的角度阈值，角度制
    double m_force_aim_palstance_threshold; // 强制允许发射的目标旋转速度最大值，弧度制
    double m_aim_angle_tolerance;           // 自动击发时目标装甲板相对偏角最大值，角度制
    double m_aim_pose_tolerance;            // 自动击发位姿偏差最大值，弧度制
    double m_aim_center_angle_tolerance;    // 跟随圆心自动击发目标偏角判断，角度制
    double m_switch_trackmode_threshold;    // 更换锁中心模式角速度阈值，弧度制
    double m_aim_center_palstance_threshold;// 跟随圆心转跟随装甲板的目标旋转速度最大值，弧度制

    // =========================== 可视化 ===================================
    std::vector<std::vector<Eigen::Vector3d>> visual_armor_position_pose_temp;  // 用于可视化存储观测装甲板的全局变量

    GimbalPose m_cur_pose;                  // 当前位姿
    GimbalPose m_target_pose;               // 目标位姿
    GimbalPose m_target_pose_fire;          // 用于火控判断(发弹如流水)
    GimbalPose m_target_pose_debug;         // 用于debug,比较mpc和传统模式

    // =========================== 串口补偿 =======================================
    double m_rollOffset = 0;
    double m_pitchOffset = 0;
    double m_yawOffset = 0;

    // 历史装甲板数组
    deque<Armor> history_armor_facing;
    deque<Armor> history_armor_target;
    deque<Armor> history_armor_target_debug;
     // ======================== 能量机关相关 ========================
    EnhancedMultiBladeTracker fan_tracker_;      // 扇叶跟踪器
    TrackTargetMode track_mode_;                  // 工作模式
    std::vector<cv::Point2f> detect_blade_pts_;   // 检测到的扇叶点
    cv::Point2f fan_circle_center_;               // 能量机关圆心
    bool fan_need_reset_ekf_;                     // 是否需要重置卡尔曼
    bool local_fan_force;  // 本地强制能量机关模式
    public:
     void setLocalFanForce(bool enable) {
        local_fan_force = enable;
 }

    // 修正：只有一个构造函数
    Tracker(ros::NodeHandle& nh, const std::string& config_path) 
        : config_path_(config_path), tfListener(tfBuffer_),
          fan_tracker_(100.0f), track_mode_(TrackTargetMode::ArmorMode), 
          fan_circle_center_(0.0f, 0.0f), fan_need_reset_ekf_(false), local_fan_force(false)
    {
        this->nh = nh;
        
        AngPub = nh.advertise<geometry_msgs::Vector3>("/auto_angle", 1000);
        debugpub = nh.advertise<std_msgs::Float64>("/debugpub", 1000);
        debugpub1 = nh.advertise<std_msgs::Float64>("/debugpub1", 1000);
        debugpub2 = nh.advertise<std_msgs::Float64>("/debugpub2", 1000);
        cal = new CAL::Calculater(this->nh, this->img);
        tfBuffer_.setUsingDedicatedThread(true);
        m_armors.reset();

        targetModel = std::make_unique<TargetModel>(config_path_);
        // 加载相机内参（用于能量机关距离估计）
        cv::FileStorage fs(config_path_ + "/camera.yaml", cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["camera_matrix"] >> camera_matrix_;
            fs["distortion_coefficients"] >> dist_coeffs_;
            ROS_INFO("Camera parameters loaded successfully from: %s/camera.yaml", config_path_.c_str());
        } else {
            ROS_WARN("Camera config file not found, using default parameters");
            // 默认相机内参（根据实际相机调整）
            camera_matrix_ = (cv::Mat_<double>(3,3) << 1692.72620365811, 0.363957161762693, 643.964129877484, 0, 1692.43288845269, 502.492890249832, 0, 0, 1);
        }
        coorConverter = new CoordinateTransformer(config_path_);
        m_MPC = std::make_shared<MPC>(config_path_);

        if (!config_path_.empty()) {
            try {
                setParam(config_path_);
                ROS_INFO("Successfully loaded parameters from: %s", config_path_.c_str());
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to load parameters: %s", e.what());
            }
        } else {
            ROS_WARN("No configuration file path provided. Using default parameters.");
        }
    }
    ~Tracker() {
        delete cal;
        delete coorConverter;
    }

    // ======================== 能量机关相关方法 ========================
    
    /**
     * @brief 设置工作模式
     * @param mode 目标工作模式
     */
    void setTrackMode(TrackTargetMode mode) {
        track_mode_ = mode;
        if (mode == TrackTargetMode::EnergyFanMode) {
            fan_tracker_.reset();
        }
    }
    
    /**
     * @brief 获取当前工作模式
     * @return 当前工作模式
     */
    TrackTargetMode getTrackMode() const {
        return track_mode_;
    }
    
    /**
     * @brief 处理能量机关跟踪
     * @param armors 识别到的装甲板数据（包含能量机关信息）
     * @return 是否成功跟踪到目标
     */
  bool processFanTracking(const rm_msgs::ArmorArrayConstPtr& armors) {
    if (!armors) {
        return false;
    }

    detect_blade_pts_.clear();
    fan_circle_center_ = cv::Point2f(0.f, 0.f);

    bool valid_circle = armors->has_fan_target;

    for (size_t i = 0; i < armors->blade_xs.size(); ++i) {
        cv::Point2f pt(armors->blade_xs[i], armors->blade_ys[i]);
        detect_blade_pts_.push_back(pt);
    }

    if (valid_circle) {
        fan_circle_center_.x = armors->circle_x;
        fan_circle_center_.y = armors->circle_y;
    }

    cv::Point2f target_blade_pt;
    bool need_reset;
    bool fan_ok = fan_tracker_.update(detect_blade_pts_, fan_circle_center_, target_blade_pt, need_reset);

    if (!fan_ok || !valid_circle) {
        fan_last_valid_ = false;
        return false;
    }

    // NaN 保护: 检查目标点和圆心是否有效
    if (target_blade_pt.x <= 0 || target_blade_pt.y <= 0 ||
        fan_circle_center_.x <= 0 || fan_circle_center_.y <= 0) {
        fan_last_valid_ = false;
        return false;
    }

    double pixel_radius = cv::norm(target_blade_pt - fan_circle_center_);
    if (pixel_radius < fan_min_valid_radius_ || pixel_radius < 1e-6) {
        fan_last_valid_ = false;
        return false;
    }

    double dx = target_blade_pt.x - fan_circle_center_.x;
    double dy = target_blade_pt.y - fan_circle_center_.y;
    double angle = atan2(dy, dx);
    if (std::isnan(angle)) {
        fan_last_valid_ = false;
        return false;
    }
    double estimated_distance = fan_default_distance_;
    if (!camera_matrix_.empty() && pixel_radius > 1e-6) {
        double focal_length = camera_matrix_.at<double>(0, 0);
        if (focal_length > 0) {
            double real_radius = 0.5;
            estimated_distance = (focal_length * real_radius) / pixel_radius;
            estimated_distance = std::max(1.5, std::min(fan_max_valid_distance_, estimated_distance));
        }
    }
    if (std::isnan(estimated_distance) || estimated_distance <= 0) {
        estimated_distance = fan_default_distance_;
    }

    Eigen::Vector3d fan_world_pos;
    fan_world_pos << estimated_distance * cos(angle),
                     estimated_distance * sin(angle),
                     fan_target_height_;

    // === 关键修复: 先更新 IMU 姿态再调用 map2Gun ===
    coorConverter->tfupdate_imu({RmSerialData.Roll, RmSerialData.Pitch, RmSerialData.Yaw});

    Eigen::Vector3d gun_pos = coorConverter->map2Gun(fan_world_pos);
    double horizontal_dist = sqrt(gun_pos.x() * gun_pos.x() + gun_pos.y() * gun_pos.y());
    double pitch = atan2(gun_pos.z(), horizontal_dist);
    double yaw   = atan2(gun_pos.y(), gun_pos.x());

    // NaN 保护: 防止 map2Gun 返回零向量导致 atan2(0,0) → NaN → 段错误
    if (std::isnan(pitch) || std::isnan(yaw)) {
        ROS_WARN("[processFanTracking] NaN detected in angle, skipping frame");
        fan_last_valid_ = false;
        return false;
    }

    if (m_fix_on && horizontal_dist > 1e-6) {
        double distance = gun_pos.norm();
        double bullet_speed = BulletVector;
        double cos_pitch = cos(pitch);
        double term = (9.8 * distance * cos_pitch * cos_pitch) / (bullet_speed * bullet_speed) - sin(pitch);
        if (term >= -1 && term <= 1) {
            double angle_fix = 0.5 * (asin(term) - pitch);
            pitch += angle_fix;
        }
    }

    geometry_msgs::Vector3 angle_msg;
    angle_msg.x = -pitch;
    angle_msg.y = yaw;
    angle_msg.z = 2;
    AngPub.publish(angle_msg);

    fan_last_angle_ = angle_msg;
    fan_last_valid_ = true;
    fan_last_publish_time_ = ros::Time::now();

    ROS_DEBUG("Fan tracking: distance=%.2fm, pitch=%.3f, yaw=%.3f", estimated_distance, pitch, yaw);
    return true;
}

// processFanTracking 结束 — 以下为原装甲模式 Track() 逻辑

   
    
    /**
     * @brief 计算瞄准角度
     * @param pos 目标世界坐标
     * @return 云台角度(pitch, yaw)
     */
    Eigen::Vector2d CalcAngle(Eigen::Vector3d pos) {
        Eigen::Vector2d result;
        result << acos(pos.x() / Eigen::Vector2d(pos.x(), pos.z()).norm()), 
                  atan2(pos.y(), pos.x());
        result.y() -= 1 * D2R;
        
        return result;
    }

    /****************
    * @brief 回调函数: 接受串口信息，并更新内部变量、TF、坐标系
    * @param _serial ROS 消息的智能指针，包含子弹速度、射击标志、补偿角度等
    */
    void SetSerial(const rm_msgs::RmSerialConstPtr _serial){
    if (_serial) {
        const double age = (ros::Time::now() - _serial->header.stamp).toSec();
        if (_serial->header.stamp.isZero() || age < 0.0 || age > 0.2 ||
            !std::isfinite(_serial->Roll) || !std::isfinite(_serial->Pitch) ||
            !std::isfinite(_serial->Yaw)) {
            has_serial_data_ = false;
            return;
        }
        if (has_serial_data_ && RmSerialData.header.stamp == _serial->header.stamp) return;
        RmSerialData = *_serial;
        has_serial_data_ = true;
        serial_received_at_ = std::chrono::steady_clock::now();
        
        // 从串口更新子弹速度（暂时注释，测试时可放开）
        // if(_serial->BulletVec > 10){
        //     BulletVector = _serial->BulletVec;
        // }

        // ========== 工作模式切换（本地强制模式优先级最高） ==========
        // 如果 local_fan_force = false，则由串口 mode 决定模式
        // 如果 local_fan_force = true，则强制能量机关模式（在 tracker.cpp 中设置）
        if (!local_fan_force) {
            if (_serial->ShootFlag == 'f') {
                track_mode_ = TrackTargetMode::EnergyFanMode;
            } else {
                track_mode_ = TrackTargetMode::ArmorMode;
            }
        }
        // 如果 local_fan_force = true，则 track_mode_ 保持为 EnergyFanMode（已在初始化或外部设置）

        // Only the MCU's explicit armor mode enables this control path.
        functional = _serial->ShootFlag == 'a';

        // 更新当前云台姿态
        m_cur_pose.roll = RmSerialData.Roll; 
        m_cur_pose.pitch = RmSerialData.Pitch;
        m_cur_pose.yaw = RmSerialData.Yaw;    

        // 调试信息（可按需启用）
        // ROS_INFO("cur: pitch:  %lf" , RmSerialData.Pitch);
        // ROS_INFO("BulletVector:  %lf" , BulletVector);
        // std::vector<Eigen::Vector3d> imuabsPos = cal->GetPos("map","imu",1);
        // cal->TFUpdata("map","imuabs",{0.0, 0.0, 0.0},{0.0, 0.0, imuabsPos[1].z()},0);
        // ROS_WARN("have serial 3333333333333333333333333333");
    }
}

    /*****************
    * @brief 回调函数: 接收相机节点发送的图像,用于debug
    * @param img_ 图像信息
    * @param frame  供算法线程直接使用的原始图
    * @param frame_ 额外再 clone 一份，用于可视化
    */
    void doimage(const sensor_msgs::ImageConstPtr img_)
    {
        
        cv_bridge::CvImagePtr cv_ptr; 
        try
        {
            cv_ptr =  cv_bridge::toCvCopy(img_, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        frame = cv_ptr->image.clone();
        frame_ = frame.clone();
        bullet_frame = cv_ptr->image.clone();
        // ROS_WARN("have image 111111111111111111");
    }

    // 回调函数: 接收识别发送的装甲板序列
    void doArmors(rm_msgs::ArmorArrayConstPtr armors)
    {
        if (!armors || armors->header.stamp.isZero()) { m_armors.reset(); return; }
        if (m_armors && m_armors->header.stamp == armors->header.stamp) return;
        m_armors = armors;
        armors_received_at_ = std::chrono::steady_clock::now();
        // ROS_WARN("have armor 222222222222222222");
    }

    /**
    * @brief  主跟踪循环：每帧调用一次，完成“决策 → 预测 → 补偿 → 发布”全链路
    */
    void Track()
    {
        observed_this_frame_ = false;
 
        // ========== 能量机关模式: Tracker 休眠, Fu 节点全权处理 ==========
        if (track_mode_ == TrackTargetMode::EnergyFanMode) {
            return;  // 不做任何装甲板逻辑, 不发布 /auto_angle
        }
        const auto wall_now = std::chrono::steady_clock::now();
        const double serial_age = (ros::Time::now() - RmSerialData.header.stamp).toSec();
        if (!has_serial_data_ || serial_age < 0.0 || serial_age > 0.2 ||
            std::chrono::duration<double>(wall_now - serial_received_at_).count() > 0.2) {
            Pub_Aangle(false);
            return;
        }
        // ========== 装甲模式 ==========
        if (!m_armors) {
            // Prevent the controller from keeping the last target/fire request.
            Pub_Aangle(false);
            return;
        }

        // === 数据新鲜度检查: 装甲板数据超过 200ms 判定为过期 ===
        //     避免将严重延迟/过期的数据喂给卡尔曼预测器导致错误收敛
        {
            ros::Duration data_age = ros::Time::now() - m_armors->header.stamp;
            if (data_age.toSec() < 0.0 || data_age.toSec() > 0.2 ||
                std::chrono::duration<double>(wall_now - armors_received_at_).count() > 0.2) {
                ROS_WARN_THROTTLE(0.5, "[Tracker] 装甲板数据过期 (%.0f ms), 跳过本帧以避免预测器错误收敛",
                                  data_age.toSec() * 1000);
                Pub_Aangle(false);
                return;
            }
        }

        try
        {
            //begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            if(!functional){
                Pub_Aangle(false);
                return;
            }
            
           
            
            TargetModel* targetModel_temp = nullptr;
            targetModel_temp = armorUpdate(m_armors);

            if(targetModel_temp != nullptr && observed_this_frame_){
                cv::Point2d finAngle = {0.0, 0.0};
                GimbalPose target_pose_temp = reconstruction_choose_compensation();

                // 调试输出(打印云台需要转动到的角度)
                // cout << "pitch: " << target_pose_temp.pitch << " " << "yaw: " << target_pose_temp.yaw << endl;
                // 打印需要移动的相对角
                // finAngle.x = target_pose_temp.pitch - m_cur_pose.pitch; // 计算需要移动的pitch角度
                // finAngle.y = target_pose_temp.yaw - m_cur_pose.yaw;     // 计算需要移动的yaw角度

                // // // 陀螺仪的绝对角
                finAngle.x = target_pose_temp.pitch; // 计算需要移动到的pitch角度
                finAngle.y = target_pose_temp.yaw;     // 计算需要移动到的yaw角度

                // debug: 云台跟随效果rqt_plot打印
                // yaw角
                // debugdate.data = m_cur_pose.yaw;            // 当前云台yaw角度 
                // debugdate1.data = target_pose_temp.yaw;     // 计算出云台需要转动的yaw角度
                // // pitch角
                // debugdate.data = m_cur_pose.pitch;          // 当前云台pitch角度 
                // debugdate1.data = target_pose_temp.pitch;   // 计算出云台需要转动的pitch角度

                // debug: 自动打弹打印
                // cout << "finAngle.x: " << finAngle.x << " " << "finAngle.y: " << finAngle.y << endl;
                // cout << "是否允许打弹: " << targetModel_temp->auto_fire << endl;
                // if (targetModel_temp->auto_fire) {
                //     debugdate.data = 1;
                // }
                // else {
                //     debugdate.data = 0;
                // }              
                Pub_Aangle(true, targetModel_temp->auto_fire, finAngle);
            }else{
                Pub_Aangle(false);
            }

            // 帧率控制 
            // double time_line = 30.0;
            // end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            // double frame_time = (end_Time - begin_Time)/1000.0;
            // if ((end_Time - begin_Time) - time_line < 0.0)// 帧率控制
            // {
            //     int sleep_time_ms = static_cast<int>(time_line - (end_Time - begin_Time));
            //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            //     //frame_time = 0.03; 
            //     end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            //     frame_time = (end_Time - begin_Time)/1000.0;
            //     // cout << "sleep_time_ms:" << sleep_time_ms << "ms" << endl;
            //     // cout << "frame_time:" << frame_time*1000 << "ms" << endl;
            // }
            // begin_Time = end_Time;
            // debugdate1.data = frame_time*1000; 
        
            debugpub.publish(debugdate);
            debugpub1.publish(debugdate1);
            // debugpub2.publish(debugdate2);
        } 
        catch (const std::exception& e) 
        {
            ROS_ERROR("[Exception] In Track function: %s", e.what());
            Pub_Aangle(false);
        }
    }

    // 重构选板
    GimbalPose reconstruction_choose_compensation() {
        double m_time_off = COMMAND_TIMESPAN + delay_time + eTime + 0.025;//
        // 选板模式
        /*
        这段代码有两种选板模式: 
            1.锁中心模式: 
                条件: 目标旋转角速度绝对值超过阈值

            2.瞄准最佳装甲板模式: 
                条件: 目标旋转角速度绝对值低于或接近阈值
        */

        // ====================================================================
        // 方案 1: 传统计算子弹飞行时间 (无空气阻力，匀速直线假定)
        // ====================================================================
        /**
         * 传统方法: 
         * 严格意义上来说，如果要准确预测击中时刻的装甲板位置的话，需要解一个非线性方程。老方法采用一种近似的解法
         * 根据当前最近装甲板距离计算击中时间，用于预测目标装甲板出现的位置
         * 事实上相当于一步牛顿迭代法，或者说一阶的线性化
         */

         /*
            迭代法求子弹飞行时间
            子弹的飞行时间取决于目标在被击中那一刻的位置(因为距离/高度差不同,飞行时间不同)
            目标在被击中那一刻的位置又依赖于子弹飞行时间的这段时间内目标的运动
            因此简单的一次性计算(用当前目标位置算飞行时间)存在误差,需要通过迭代逼近来求解真实的飞行时间         
         */
        // 目标装甲板
        Armor abs_facing_armor;//
        Armor abs_target_armor;//
        Armor abs_facing_armor_fire;
        Armor abs_target_armor_fire;
        double hit_time = 0;
        double center_hit_time = 0;
        // 计算击打时间 
        Point3d point_temp1 = Point3d(targetModel->m_status->getFacingArmor(0).position.x(),targetModel->m_status->getFacingArmor(0).position.y(),targetModel->m_status->getFacingArmor(0).position.z());
        center_hit_time = getDistance(point_temp1) / BulletVector + m_time_off;
        Point3d point_temp2 = Point3d(targetModel->m_status->getClosestArmor(0, 0).position.x(),targetModel->m_status->getClosestArmor(0, 0).position.y(),targetModel->m_status->getClosestArmor(0, 0).position.z());
        hit_time = getDistance(point_temp2) / BulletVector + m_time_off;

        double pre_hit_time = hit_time;
        double pre_center_hit_time = center_hit_time;

        for (int iter = 0; iter < 10; iter++) {
            // 选板 + 预测目标在子弹飞行时间后的位置
            abs_facing_armor = targetModel->m_status->getFacingArmor(center_hit_time);
            abs_target_armor = targetModel->m_status->getClosestArmor(hit_time, m_switch_threshold);

            // 根据新的瞄准点,重新计算新的子弹飞行时间
            point_temp1 = Point3d(abs_facing_armor.position.x(),abs_facing_armor.position.y(),abs_facing_armor.position.z());
            center_hit_time = getDistance(point_temp1) / BulletVector + m_time_off;
            point_temp2 = Point3d(abs_target_armor.position.x(),abs_target_armor.position.y(),abs_target_armor.position.z());
            hit_time = getDistance(point_temp2) / BulletVector + m_time_off;

            // 检查收敛: 如果新旧飞行时间小于0.001秒,认为收敛,跳出循环
            if (std::abs(center_hit_time - pre_center_hit_time) < 0.001 && std::abs(hit_time - pre_hit_time) < 0.001) {
                break;
            }

            // 否则更新上一次的飞行时间,继续下一轮迭代
            pre_hit_time = hit_time;
            pre_center_hit_time = center_hit_time;
        }

        abs_facing_armor = targetModel->m_status->getFacingArmor(center_hit_time);
        abs_target_armor = targetModel->m_status->getClosestArmor(hit_time, m_switch_threshold);

        history_armor_facing.push_back(abs_facing_armor);
        history_armor_target.push_back(abs_target_armor);

        if (history_armor_facing.size() < (int)(COMMAND_TIMESPAN / 0.025))//
        {
            cout << "历史装甲板的个数小于" << (int)(COMMAND_TIMESPAN / 0.025) << "时延不足" << COMMAND_TIMESPAN << "s" << endl;
            abs_facing_armor_fire = abs_facing_armor;
            abs_target_armor_fire = abs_target_armor;
        }
        else {
            abs_facing_armor_fire = history_armor_facing.front();
            abs_target_armor_fire = history_armor_target.front();

            // 弹出一块历史装甲板数据
            history_armor_facing.pop_front();
            history_armor_target.pop_front();
        }

        // // 选板(用于火控发弹如流水)
        // abs_facing_armor_fire = targetModel->m_status->getFacingArmor(center_hit_time - COMMAND_TIMESPAN);
        // abs_target_armor_fire = targetModel->m_status->getClosestArmor(hit_time - COMMAND_TIMESPAN, m_switch_threshold);
        

        // ====================================================================
        // 方案 2: 迭代法计算子弹飞行时间 - ceres版本
        // ====================================================================
        // Armor abs_facing_armor_air;
        // Armor abs_target_armor_air;
        // Armor abs_facing_armor_fire_air;
        // Armor abs_target_armor_fire_air;
        // double hit_time_air = 0;
        // double center_hit_time_air = 0;

        // Point3d point_center_init = Point3d(targetModel->m_status->getFacingArmor(0).position.x(),targetModel->m_status->getFacingArmor(0).position.y(),targetModel->m_status->getFacingArmor(0).position.z());
        // center_hit_time_air = getFlyTimeWithAirResistance(point_center_init, BulletVector, m_k) + m_time_off;
        // Point3d point_closest_init = Point3d(targetModel->m_status->getClosestArmor(0,0).position.x(),targetModel->m_status->getClosestArmor(0,0).position.y(),targetModel->m_status->getClosestArmor(0,0).position.z());
        // hit_time_air = getFlyTimeWithAirResistance(point_closest_init, BulletVector, m_k) + m_time_off;
        // const int ITERATION_NUM = 10; // 迭代10次
        // // 迭代 center_hit_time_air (锁中心模式下的击打时间)
        // for (int i = 0; i < ITERATION_NUM; i++) {
        //     Armor temp_predict_center = targetModel->m_status->getFacingArmor(center_hit_time_air);
        //     Point3d temp_p_center = Point3d(temp_predict_center.position.x(),temp_predict_center.position.y(),temp_predict_center.position.z());
        //     // 使用阻力时间进行迭代预测！
        //     center_hit_time_air = getFlyTimeWithAirResistance(temp_p_center, BulletVector, m_k) + m_time_off;
        // }
        // // 迭代 hit_time_air (跟板模式下的击打时间)
        // for (int i = 0; i < ITERATION_NUM; i++) {
        //     Armor temp_predict_closest = targetModel->m_status->getClosestArmor(hit_time_air,m_switch_threshold);
        //     Point3d temp_p_closest = Point3d(temp_predict_closest.position.x(),temp_predict_closest.position.y(),temp_predict_closest.position.z());
        //     // 使用阻力时间进行迭代预测！
        //     hit_time_air = getFlyTimeWithAirResistance(temp_p_closest, BulletVector, m_k) + m_time_off;
        // }

        // // 选板
        // abs_facing_armor_air = targetModel->m_status->getFacingArmor(center_hit_time_air);
        // abs_target_armor_air = targetModel->m_status->getClosestArmor(hit_time_air, m_switch_threshold);
        // // 选板(用于火控发弹如流水) -> 不算发弹如流水
        // abs_facing_armor_fire_air = targetModel->m_status->getFacingArmor(center_hit_time_air - COMMAND_TIMESPAN);
        // abs_target_armor_fire_air = targetModel->m_status->getClosestArmor(hit_time_air - COMMAND_TIMESPAN, m_switch_threshold);

        // ====================================================================
        // 方案 3: 迭代法计算子弹飞行时间 - 非ceres版本 烂
        // ====================================================================
        // Armor abs_facing_armor_fast;
        // Armor abs_target_armor_fast;
        // Armor abs_facing_armor_fire_fast;
        // Armor abs_target_armor_fire_fast;
        // double hit_time_fast = 0;
        // double center_hit_time_fast = 0;

        // // 拿初始装甲板做第一次时间猜测
        // double exact_fly_time_center = 0.0;
        // double exact_fly_time_closest = 0.0;
        // getAngle_air_fast(point_center_init, BulletVector, exact_fly_time_center); // 调用新解算器仅为了拿时间
        // getAngle_air_fast(point_closest_init, BulletVector, exact_fly_time_closest);

        // center_hit_time_fast = exact_fly_time_center + m_time_off;
        // hit_time_fast = exact_fly_time_closest + m_time_off;

        // // 物理闭环迭代 (由于算法极度逼近真实物理，通常迭代 3 次即收敛，设为 5 次保底)
        // const int ITERATION_NUM_FAST = 5; 
        
        // for (int i = 0; i < ITERATION_NUM_FAST; i++) {
        //     Armor temp_predict_center = targetModel->m_status->getFacingArmor(center_hit_time_fast);
        //     Point3d temp_p_center = Point3d(temp_predict_center.position.x(), temp_predict_center.position.y(), temp_predict_center.position.z());
        //     // 利用解算器抽出这块板在未来那个位置的真实物理飞行时间
        //     getAngle_air_fast(temp_p_center, BulletVector, exact_fly_time_center);
        //     center_hit_time_fast = exact_fly_time_center + m_time_off;
        // }
        
        // for (int i = 0; i < ITERATION_NUM_FAST; i++) {
        //     Armor temp_predict_closest = targetModel->m_status->getClosestArmor(hit_time_fast, m_switch_threshold);
        //     Point3d temp_p_closest = Point3d(temp_predict_closest.position.x(), temp_predict_closest.position.y(), temp_predict_closest.position.z());
        //     getAngle_air_fast(temp_p_closest, BulletVector, exact_fly_time_closest);
        //     hit_time_fast = exact_fly_time_closest + m_time_off;
        // }

        // // 选板
        // abs_facing_armor_fast = targetModel->m_status->getFacingArmor(center_hit_time_fast);
        // abs_target_armor_fast = targetModel->m_status->getClosestArmor(hit_time_fast, m_switch_threshold);
        // // 选板(用于火控发弹如流水)
        // abs_facing_armor_fire_fast = targetModel->m_status->getFacingArmor(center_hit_time_fast - COMMAND_TIMESPAN);
        // abs_target_armor_fire_fast = targetModel->m_status->getClosestArmor(hit_time_fast - COMMAND_TIMESPAN, m_switch_threshold);
        

        // 判断是否锁中心
        if (m_track_center && ((abs(targetModel->m_status->palstance) - m_switch_trackmode_threshold) > m_aim_center_palstance_threshold)){
            m_center_tracked = true;    // 允许锁中心
            ROS_INFO("锁中心");
        } 
        else {
            m_center_tracked = false;   // 不允许锁中心
            ROS_INFO("不锁中心");
        }

        /**
         * 判断自动击发，条件如下：
         * 1. 目标旋转速度小于一定阈值，或目标装甲板相对偏角不超过一定范围
         * 2. EKF先验稳定
         * 3. 没有长时间丢识别
         * 4. 当前云台跟随稳定，即当前位姿和目标位姿相近
         */
        // // 方案一
        cv::Point2d vector_c = targetModel->m_status->center;                                                                                                                   // 目标旋转中心本身的向量             
        cv::Point2d vector_a = cv::Point2d(targetModel->m_status->center.x - abs_target_armor_fire.position.x(), targetModel->m_status->center.y - abs_target_armor_fire.position.y());   // 从旋转中心指向当前要瞄准的具体装甲板的向量
        double abs_angle = R2D(acos((vector_c.x * vector_a.x + vector_c.y * vector_a.y) / (getDistance(vector_c) * getDistance(vector_a))));                                    // 计算上述两个向量之间的夹角

        // 方案二
        // cv::Point2d vector_c = targetModel->m_status->center;                                                                                                                           // 目标旋转中心本身的向量             
        // cv::Point2d vector_a = cv::Point2d(targetModel->m_status->center.x - abs_target_armor_air.position.x(), targetModel->m_status->center.y - abs_target_armor_air.position.y());   // 从旋转中心指向当前要瞄准的具体装甲板的向量
        // double abs_angle = R2D(acos((vector_c.x * vector_a.x + vector_c.y * vector_a.y) / (getDistance(vector_c) * getDistance(vector_a))));                                            // 计算上述两个向量之间的夹角

        // debug
        // cout << "角速度" << targetModel->m_status->palstance << " " << "角速度阈值" << m_force_aim_palstance_threshold << endl;     // 目标旋转角速度低于强制瞄准阈值
        // cout << "夹角" << abs_angle << " " << "夹角阈值" << m_aim_angle_tolerance << endl;                                         // 装甲板偏角度
        // cout << "云台和目标姿态差" << (m_target_pose - m_cur_pose).norm() << " " << "姿态差阈值" << m_aim_pose_tolerance << endl;   // 云台和目标姿态差(云台跟随效果)
        // cout << "!Switch_Armor:" << !Switch_Armor << endl;

        // 目标的旋转角度低于强制瞄准阈值或计算出装甲板偏角小于容差,这意味着要么目标转得慢,容易瞄准; 要么即使目标目标在快速旋转,但刚好有一个装甲板转到了非常正对摄像头(夹角很小),也是很好的设计时机
        // 观测器,目标持续可见,云台跟随良好(云台当前的实际姿态与需要旋转的目标之间的差异很小,说明云台基本对准目标)

        // 由于电控发送过来的角度是上面是负下面是正
        // 而我们计算出来的角度是上面是正下面是负数
        // 由于火控的判断需要和电控发送过来的角度进行比较
        // 为了保证逻辑的完整性,所以在火控这里创建局部变量,将pitch角变为负数
        // 然后不影响整体逻辑,整体的逻辑在发送角度的地方pitch角度给负号

        // 火控的判断不考虑电控的时延 -> 发弹如流水
        GimbalPose m_target_pose_temp = m_target_pose_fire;

        // // 严格火控
        // GimbalPose m_target_pose_temp = m_target_pose;
        m_target_pose_temp.pitch = -m_target_pose_temp.pitch;

        // debugdate.data =   m_cur_pose.pitch;           // pitch云台当前
        // debugdate1.data =  m_target_pose_temp.pitch;   // pitch火控计算
        // debugdate.data =   m_cur_pose.yaw;                // yaw云台计算当前
        // debugdate1.data =  m_target_pose_temp.yaw;        // yaw火控计算

        // debugdate.data =   m_cur_pose.yaw;                // yaw云台计算当前
        // debugdate1.data =  m_target_pose_fire.yaw;        // yaw火控计算
        // debugdate.data =   m_cur_pose.pitch;              // pitch云台计算当前
        // debugdate1.data =  -m_target_pose_fire.pitch;     // pitch火控计算

        // 自动开火
        // 比较当前真实的云台位置与上一帧下达的期望位置
        if ((abs(targetModel->m_status->palstance) < m_force_aim_palstance_threshold || abs_angle < m_aim_angle_tolerance) &&
            targetModel->ekf->stable() && !Switch_Armor &&
            (m_target_pose_temp - m_cur_pose).norm() < m_aim_pose_tolerance
            )
        {
            targetModel->auto_fire = true;
            if (m_center_tracked && !(abs_angle < m_aim_center_angle_tolerance))
            {
                targetModel->auto_fire = false;
            }
        }
        else
        {
            targetModel->auto_fire = false;
        }

        // 云台控制模式
        bool control_mode = 0; // 0: 普通模式 1: mpc模式
        if (control_mode == 0) {
            if (m_center_tracked) {
                // 方案一 无空气阻力
                m_target_pose = getAngle(Point3d(abs_facing_armor.position.x(),abs_facing_armor.position.y(),abs_facing_armor.position.z()), m_cur_pose, BulletVector);
                m_target_pose_fire = getAngle(Point3d(abs_facing_armor_fire.position.x(),abs_facing_armor_fire.position.y(),abs_facing_armor_fire.position.z()), m_cur_pose, BulletVector);

                // 方案二 线型空气阻力弹道解算 - ceres版本
                // m_target_pose = getAngle_air(Point3d(abs_facing_armor_air.position.x(),abs_facing_armor_air.position.y(),abs_facing_armor_air.position.z()), m_cur_pose, BulletVector);
                // m_target_pose_fire = getAngle_air(Point3d(abs_facing_armor_fire_air.position.x(),abs_facing_armor_fire_air.position.y(),abs_facing_armor_fire_air.position.z()), m_cur_pose, BulletVector);

                // 方案三 线型空气阻力弹道解算 - 非ceres版本
                // double dummy_time = 0;
                // m_target_pose = getAngle_air_fast(Point3d(abs_facing_armor_fast.position.x(),abs_facing_armor_fast.position.y(),abs_facing_armor_fast.position.z()), BulletVector, dummy_time);
                // m_target_pose_fire = getAngle_air_fast(Point3d(abs_facing_armor_fire_fast.position.x(),abs_facing_armor_fire_fast.position.y(),abs_facing_armor_fire_fast.position.z()), BulletVector, dummy_time);

                // debug 无空气阻力 线性空气阻力ceres版本 线性空气阻力 - 非ceres版本
                // GimbalPose m_target_pose_fire_temp = getAngle(Point3d(abs_facing_armor_fire.position.x(),abs_facing_armor_fire.position.y(),abs_facing_armor_fire.position.z()), m_cur_pose, BulletVector);
                // GimbalPose m_target_po se_fire_air_temp = getAngle_air(Point3d(abs_facing_armor_fire_air.position.x(),abs_facing_armor_fire_air.position.y(),abs_facing_armor_fire_air.position.z()), m_cur_pose, BulletVector);
                // GimbalPose m_target_pose_fire_fast_temp = getAngle_air_fast(Point3d(abs_facing_armor_fire_fast.position.x(),abs_facing_armor_fire_fast.position.y(),abs_facing_armor_fire_fast.position.z()), BulletVector, dummy_time);
                // cout << "无空气阻力pitch" << m_target_pose_fire_temp.pitch;
                // cout << "空气阻力pitch ceres版" << m_target_pose_fire_air_temp.pitch;
                // cout << "空气阻力pitch 非ceres版" << m_target_pose_fire_fast_temp.pitch;
                // debugdate.data = m_target_pose_fire_temp.pitch;
                // debugdate1.data = m_target_pose_fire_air_temp.pitch;
                // debugdate2.data = m_target_pose_fire_fast_temp.pitch;


                // 这里后面再改 不同弹道解算的不同时间的打印
                // projectMapPointsToImage(targetModel->m_status->getArmors(center_hit_time),visual_armor_position_pose_temp);   // 这里打印的是计算的子弹飞行击中的时间的那一刻的重构装甲板
                // projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);                    // 这里打印的重构装甲板是和观测装甲板同一时间的预测装甲板

                // // debug
                // Armors armors_temp_debug = targetModel->m_status->getArmors(center_hit_time);
                // for (int i = 0; i < armors_temp_debug.size(); i++) {
                //     cal->TFUpdata("map","armor" + std::to_string(i), armors_temp_debug[i].position, armors_temp_debug[i].rpy, i);
                // }
                // cal->TFUpdata("map","carcenter",{targetModel->m_status->center.x,targetModel->m_status->center.y,0},{0.0, 0.0, 0.0},0);//上传目标的TF

                // debug(理想弹道打印)
                // 记录当前时间
                // tools::ShootParam shoot_param;
                // shoot_param.v0 = BulletVector;
                // shoot_param.aim_angle = m_target_pose.pitch - pitch_compensation + m_cur_pose.pitch; // 弹道重现需要相对角
                // shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_facing_armor.position);
                // long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
                // draw_simulated_bullets(this->coorConverter,shoot_param,bullet_frame,draw_visual_now_time,static_cast<long long>(eTime * 1000),static_cast<long long>(COMMAND_TIMESPAN * 1000),static_cast<long long>(delay_time * 1000),m_k);  
            }  
            else {
                // 不考虑空气阻力的斜抛
                m_target_pose = getAngle(Point3d(abs_target_armor.position.x(),abs_target_armor.position.y(),abs_target_armor.position.z()), m_cur_pose, BulletVector);
                m_target_pose_fire = getAngle(Point3d(abs_target_armor_fire.position.x(),abs_target_armor_fire.position.y(),abs_target_armor_fire.position.z()), m_cur_pose, BulletVector);

                // 方案二 线型空气阻力弹道解算 - ceres版本
                // m_target_pose = getAngle_air(Point3d(abs_facing_armor_air.position.x(),abs_facing_armor_air.position.y(),abs_facing_armor_air.position.z()), m_cur_pose, BulletVector);
                // m_target_pose_fire = getAngle_air(Point3d(abs_facing_armor_fire_air.position.x(),abs_facing_armor_fire_air.position.y(),abs_facing_armor_fire_air.position.z()), m_cur_pose, BulletVector);

                // 方案三 线型空气阻力弹道解算 - 非ceres版本
                // double dummy_time = 0;
                // m_target_pose = getAngle_air_fast(Point3d(abs_target_armor_fast.position.x(),abs_target_armor_fast.position.y(),abs_target_armor_fast.position.z()), BulletVector, dummy_time);
                // m_target_pose_fire = getAngle_air_fast(Point3d(abs_target_armor_fire_fast.position.x(),abs_target_armor_fire_fast.position.y(),abs_target_armor_fire_fast.position.z()), BulletVector, dummy_time);

                // debug 无空气阻力 线性空气阻力ceres版本 线性空气阻力 - 非ceres版本
                // GimbalPose m_target_pose_fire_temp = getAngle(Point3d(abs_facing_armor_fire.position.x(),abs_facing_armor_fire.position.y(),abs_facing_armor_fire.position.z()), m_cur_pose, BulletVector);
                // GimbalPose m_target_pose_fire_air_temp = getAngle_air(Point3d(abs_facing_armor_fire_air.position.x(),abs_facing_armor_fire_air.position.y(),abs_facing_armor_fire_air.position.z()), m_cur_pose, BulletVector);
                // GimbalPose m_target_pose_fire_fast_temp = getAngle_air_fast(Point3d(abs_facing_armor_fire_fast.position.x(),abs_facing_armor_fire_fast.position.y(),abs_facing_armor_fire_fast.position.z()), BulletVector, dummy_time);
                // cout << "无空气阻力pitch" << m_target_pose_fire_temp.pitch;
                // cout << "空气阻力pitch ceres版" << m_target_pose_fire_air_temp.pitch;
                // cout << "空气阻力pitch 非ceres版" << m_target_pose_fire_fast_temp.pitch;
                // debugdate.data = m_target_pose_fire_temp.pitch;
                // debugdate1.data = m_target_pose_fire_air_temp.pitch;
                // debugdate2.data = m_target_pose_fire_fast_temp.pitch;


                // 这里后面再改 不同弹道解算的不同时间的打印
                // projectMapPointsToImage(targetModel->m_status->getArmors(hit_time),visual_armor_position_pose_temp);     // 这里打印的是计算的子弹飞行击中的时间的那一刻的重构装甲板
                // projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);               // 这里打印的重构装甲板是和观测装甲板同一时间的预测装甲板

                // // debug
                // Armors armors_temp_debug = targetModel->m_status->getArmors(hit_time);
                // for (int i = 0; i < armors_temp_debug.size(); i++) {
                //     cal->TFUpdata("map","armor" + std::to_string(i), armors_temp_debug[i].position, armors_temp_debug[i].rpy, i);
                // }
                // cal->TFUpdata("map","carcenter",{targetModel->m_status->center.x,targetModel->m_status->center.y,0},{0.0, 0.0, 0.0},0);//上传目标的TF

                // debug(理想弹道打印)
                // 记录当前时间
                // tools::ShootParam shoot_param;
                // shoot_param.v0 = BulletVector;
                // shoot_param.aim_angle = m_target_pose.pitch - pitch_compensation + m_cur_pose.pitch; // 弹道重现需要相对角
                // shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_target_armor.position);
                // long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
                // draw_simulated_bullets(this->coorConverter,shoot_param,bullet_frame,draw_visual_now_time,static_cast<long long>(eTime * 1000),static_cast<long long>(COMMAND_TIMESPAN * 1000),static_cast<long long>(delay_time*1000),m_k);  
            }
        }
       
       
        // // debug
        // /////////////////////////////////////////////
        // // 云台控制模式
        // if (m_center_tracked) {
        //     m_target_pose = getAngle(Point3d(abs_facing_armor.position.x(),abs_facing_armor.position.y(),abs_facing_armor.position.z()), m_cur_pose, BulletVector);
        //     // projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);
        // }  
        // else {
        //     m_target_pose = getAngle(Point3d(abs_target_armor.position.x(),abs_target_armor.position.y(),abs_target_armor.position.z()), m_cur_pose, BulletVector);
        //     // projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);
        // }
        
        // // 西工大轨迹规划器
        // m_target_pose_debug = m_MPC->getGimbalSpeed(targetModel->m_status,m_cur_pose,BulletVector);
        // // cout << "状态: " << targetModel->m_status->palstance << endl;
        // // cout << "弹速: " << BulletVector << endl;
        // // cout << "当前云台状态: " << "pitch: " << m_cur_pose.pitch << "yaw: " << m_cur_pose.yaw << endl;

        // // debugdate.data = m_target_pose.yaw;           // 未规划
        // // debugdate1.data = m_target_pose_debug.yaw;    // mpc规划
        // cout << "未规划yaw: " << m_target_pose.yaw << endl;
        // cout << "mpc规划yaw: " << m_target_pose_debug.yaw << endl;
        // // debugdate.data = m_target_pose.pitch;           // 未规划
        // // debugdate1.data = m_target_pose_debug.pitch;    // mpc规划
        // // cout << "未规划pitch: " << m_target_pose.pitch << endl;
        // // cout << "mpc规划pitch: " << m_target_pose_debug.pitch << endl;
        // /////////////////////////////////////////////

        return m_target_pose;
    }

    tools::AimCorrector aim_corrector = tools::AimCorrector();
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const tools::ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN, long long delay_time, const double k){
        tools::FlaskStream flask_aim;
        tools::FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();

        aim_corrector.update_aim(coorConverter,shoot_param);

        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN,delay_time);

        std::vector<tools::IdCircle> bullets = aim_corrector.get_circles(now_time, k);
        for (auto& bullet: bullets) {
            flask_aim << tools::FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << tools::FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }

        flask_aim >> img;
        imshow("draw_bullets",img);
        waitKey(1);
    }

    /**
     *@brief: 根据装甲板坐标、当前云台位姿、射速等信息计算目标位姿
     */
    GimbalPose getAngle(cv::Point3d position, GimbalPose cur_pose, double bullet_speed)
    {
        GimbalPose solve_angle;
        // cv::Point3d gun_base = position;
        Eigen::Vector3d gun_vec = this->coorConverter->map2Gun(this->coorConverter->Point2Vector(position));
        cv::Point3d gun_base =  this->coorConverter->Vector2Point(gun_vec);
        double pitch_in_gun = std::atan(gun_base.z / std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y));
        double yaw_in_gun   = std::atan(gun_base.y / gun_base.x);
        solve_angle.pitch = -cur_pose.pitch + pitch_in_gun;
        solve_angle.yaw   = cur_pose.yaw + yaw_in_gun;
        // solve_angle.pitch = pitch_in_gun;
        // solve_angle.yaw   = yaw_in_gun;

        // 重力补偿
        if (m_fix_on) {
            double dis = sqrt(pow(gun_base.x, 2) + pow(gun_base.z, 2) + pow(gun_base.y, 2));
            double angle_fix = 0;
            // debug /////////////////////////////////////////////////////////////
            // cout << "dis: " << dis << endl;
            // cout << "disxy: " << std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y) << endl;
            // cout << "position.z" << position.z << endl;
            // cout << "pitch: " << solve_angle.pitch << endl;
            //////////////////////////////////////////////////////////////////////

            if (abs(9.8 * dis * pow(cos(-solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(-solve_angle.pitch) <= 1)
            {
                angle_fix = 0.5 * (asin((9.8 * dis * pow(cos(-solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(-solve_angle.pitch)) - solve_angle.pitch);
                // cout << "重力补偿角: " << angle_fix << endl;
            }
            else
            {
                std::cout << "[AngleSolver] Wrong distance in angle_fix" << std::endl;
            }
            solve_angle.pitch += angle_fix;
        }
        // debug 判断枪口是否能够正确瞄准装甲板
        // //////////////////////////////////////////////////////////////////////
        // std::vector<Eigen::Vector3d> Postest = cal->GetPos("gun","DETECT4",1);
        // cal->TFUpdata("gun","DETECT6_gun", Postest[0], Postest[1], 1);
        
        // 安装误差导致的硬补偿
        solve_angle.pitch += pitch_compensation;
        solve_angle.yaw += yaw_compensation;


        // // // 打表硬补偿
        // if (targetModel->m_status < 4) {
        //     solve_angle.pitch += pitch_compensation4meter;
        //     solve_angle.yaw += yaw_compensation4meter;
        // } 
        // else if (targetModel->m_status < 5.5) {
        //     solve_angle.pitch += pitch_compensation5_5meter;
        //     solve_angle.yaw += yaw_compensation5_5meter;
        // }
        // else if (targetModel->m_status < 6) {
        //     solve_angle.pitch += pitch_compensation6meter;
        //     solve_angle.yaw += yaw_compensation6meter;
        // }
        // else if (targetModel->m_statu < 6.5) {
        //     solve_angle.pitch += pitch_compensation6_5meter;
        //     solve_angle.yaw += yaw_compensation6_5meter;
        // }


        return solve_angle;
    }


    /**
     * @brief 解析法计算带线性空气阻力的真实飞行时间
     */
    double getFlyTimeWithAirResistance(const cv::Point3d& p, double v0, double k) {
        // 计算目标到枪管的绝对三维直线距离
        double distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        
        // 物理推导解析解: t = -(1/k) * ln(1 - k * D / v0)
        double term = 1.0 - (k * distance) / v0;
        if (term > 0.001) { 
            return -std::log(term) / k;
        } else {
            // 如果由于极度超射导致 log 内部计算非法，退回无阻力保底时间
            return distance / v0; 
        }
    }


    // old
    // /**
    //  *@brief: 根据装甲板坐标、当前云台位姿、射速等信息计算目标位姿
    //  */
    GimbalPose getAngle_air(cv::Point3d position, GimbalPose cur_pose, double bullet_speed)
    {
        ROS_WARN("m_k: %f", m_k);
        GimbalPose solve_angle;
        Eigen::Vector3d gun_vec = this->coorConverter->map2Gun(this->coorConverter->Point2Vector(position));
        cv::Point3d gun_base =  this->coorConverter->Vector2Point(gun_vec);
        
        double yaw_in_gun   = std::atan(gun_base.y / gun_base.x);
        double pitch_in_gun = target_pos_to_shoot_angle(gun_vec,bullet_speed, m_k);

        solve_angle.pitch = -cur_pose.pitch + pitch_in_gun;
        solve_angle.yaw   = cur_pose.yaw + yaw_in_gun;
        
        // 安装误差导致的硬补偿
        solve_angle.pitch += pitch_compensation;
        solve_angle.yaw += yaw_compensation;
        return solve_angle;
    }
    double target_pos_to_shoot_angle(const Eigen::Vector3d& gun_vec, const double bs, const double k) const {
        const double target_xy = std::sqrt(
            gun_vec.x() * gun_vec.x()
            + gun_vec.y() * gun_vec.y()
        );
        const double target_z = gun_vec.z();

        const double shoot_angle = [&target_xy, &target_z, &bs, &k]() {

            double shoot_angle = 30. / 180. * M_PI;
            ceres::Problem problem;
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<ResistanceFuncLinear, 1, 1>(
                    new ResistanceFuncLinear(target_xy, target_z, bs,k)
                ),
                nullptr,
                &shoot_angle
            );

            ceres::Solver::Options options;
            options.max_num_iterations = 25;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            return shoot_angle;
        }();
        return shoot_angle;
    }


    // new
    // /**
    //  *@brief: 根据装甲板坐标、当前云台位姿、射速等信息计算目标绝对位姿 (新版阻力解算)
    //  */
    // GimbalPose getAngle_air(cv::Point3d position, GimbalPose cur_pose, double bullet_speed)
    // {
    //     GimbalPose solve_angle;
    //     Eigen::Vector3d gun_vec = this->coorConverter->map2Gun(this->coorConverter->Point2Vector(position));
        
    //     // 1. 获取三维直线几何参数
    //     double dist_3d = gun_vec.norm(); // 三维空间绝对直线距离 (不受坐标系倾斜影响)
    //     // 枪管指向目标的相对偏角
    //     double pitch_in_gun = std::atan2(gun_vec.z(), std::sqrt(gun_vec.x() * gun_vec.x() + gun_vec.y() * gun_vec.y()));
    //     double yaw_in_gun   = std::atan2(gun_vec.y(), gun_vec.x());

    //     // 2. 还原出绝对视线 Pitch 角 (世界水平面夹角)
    //     // 使用老代码逻辑: 绝对角 = 当前云台角补偿 + 相对角
    //     double theta_absolute = -cur_pose.pitch + pitch_in_gun; 

    //     // 3. 几何解构：通过绝对视线角和三维距离，还原出绝对的水平距离 w 和垂直高度 h
    //     double w_abs = dist_3d * std::cos(theta_absolute);
    //     double h_abs = dist_3d * std::sin(theta_absolute);

    //     // 4. 将绝对 w 和 h 喂给 Ceres，直接求出带阻力的绝对打弹 Pitch
    //     double absolute_pitch = target_pos_to_shoot_angle(w_abs, h_abs, bullet_speed, m_k);

    //     // 5. 赋值
    //     solve_angle.pitch = absolute_pitch; // 直接赋绝对角
    //     solve_angle.yaw   = cur_pose.yaw + yaw_in_gun;
        
    //     // 安装误差导致的硬补偿
    //     solve_angle.pitch += pitch_compensation;
    //     solve_angle.yaw += yaw_compensation;
        
    //     return solve_angle;
    // }

    // 基于枪管坐标系进行的修改
    // /**
    //  *@brief: 考虑空气阻力的绝对位姿解算（基于世界系绝对向量，无视云台姿态畸变）
    //  */
    // GimbalPose getAngle_air(cv::Point3d position, GimbalPose cur_pose, double bullet_speed)
    // {
    //     GimbalPose solve_angle;
        
    //     // 1. 目标在世界系 (map) 下的坐标
    //     Eigen::Vector3d target_map = this->coorConverter->Point2Vector(position);

    //     // 2. 获取当前枪口在世界系 (map) 下的绝对坐标
    //     // 将枪管系的原点 (0,0,0) 转换到 map 系，得到当前枪口在世界中的真实位置
    //     Eigen::Vector3d gun_origin_map = this->coorConverter->gun2Map(Eigen::Vector3d(0, 0, 0));

    //     // 3. 计算从枪口指向目标的真实向量 (在世界系下)
    //     // 此时，diff_map 的 Z 轴严格对齐重力反方向，X-Y 平面严格平行于地面！
    //     Eigen::Vector3d diff_map = target_map - gun_origin_map;

    //     // 4. 提取绝对水平距离 w 和绝对高度 h (不受云台怎么乱转的影响)
    //     double true_w = std::sqrt(diff_map.x() * diff_map.x() + diff_map.y() * diff_map.y());
    //     double true_h = diff_map.z();

    //     // 5. 弹道解算，获得绝对抬枪仰角 (Pitch)
    //     // (如果你上一轮把求解封装成了 solve_air_resistance_with_time，就用那个并提取.pitch)
    //     double absolute_pitch = target_pos_to_shoot_angle(true_w, true_h, bullet_speed, m_k);

    //     // 6. 计算绝对 Yaw 角 (基于世界系的水平 x, y)
    //     double absolute_yaw = std::atan2(diff_map.y(), diff_map.x());

    //     // 7. 安装误差导致的硬补偿
    //     solve_angle.pitch = absolute_pitch + pitch_compensation; 
    //     solve_angle.yaw   = absolute_yaw + yaw_compensation;
        
    //     return solve_angle;
    // }

    /*
    @brief 单方向空气阻力弹道模型
    @param s:m 水平绝对距离
    @param v:m/s 初速度
    @param angle:rad 绝对仰角
    @param t_fly: 传出参数，记录实际飞行时间
    @return z:m 垂直下坠高度
    */
    float monoDirectionalAirResistanceModel(float s, float v, float angle, float& t_fly)
    {
        float z;
        // 使用类内的空气阻力系数 m_k
        t_fly = (float)((exp(m_k * s) - 1) / (m_k * v * cos(angle)));
        
        if(t_fly < 0)
        {
            // 超出最大射程
            // ROS_WARN("[Trajectory] Exceeding the maximum range!");
            t_fly = 0;
            return 0;
        }
        
        // z为给定v与angle时的高度 (9.8为重力加速度)
        z = (float)(v * sin(angle) * t_fly - 9.8 * t_fly * t_fly / 2.0);
        return z;
    }

    /*
    @brief pitch轴解算
    @param s:m 水平绝对距离
    @param z:m 垂直绝对高度
    @param v:m/s 子弹初速
    @param final_t: 传出参数，记录最终收敛时的物理飞行时间
    @return angle_pitch:rad 最终发弹绝对仰角
    */
    float pitchTrajectoryCompensation(float s, float z, float v, float& final_t)
    {
        float z_temp = z, z_actual, dz;
        float angle_pitch = 0.0;
        
        // 迭代 20 次逼近最优解
        for (int i = 0; i < 20; i++)
        {
            angle_pitch = atan2(z_temp, s); // rad
            // 将 final_t 传进去，每次迭代都会更新这个时间
            z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch, final_t);
            
            if(z_actual == 0)
            {
                angle_pitch = 0;
                break;
            }
            
            dz = 0.3 * (z - z_actual);
            z_temp = z_temp + dz;
            
            // 精度达到 0.01 毫米级，退出迭代
            if (fabsf(dz) < 0.00001)
            {
                break;
            }
        }
        return angle_pitch;
    }


    /**
     * @brief 基于单向空气阻力迭代模型的快速弹道解算 (纯世界系向量减法版)
     * @param position 目标装甲板在世界坐标系下的三维坐标 (x,y,z)
     * @param bullet_speed 当前子弹射速
     * @param hit_time_output (传出参数) 子弹实际物理飞行时间
     * @return 目标云台位姿 (绝对角)
     */
    GimbalPose getAngle_air_fast(cv::Point3d position, double bullet_speed, double& hit_time_output) {
        GimbalPose solve_angle;
        Eigen::Vector3d target_map = this->coorConverter->Point2Vector(position);
        
        // 获取枪口在世界系map下的绝对坐标
        Eigen::Vector3d gun_origin_map = this->coorConverter->gun2Map(Eigen::Vector3d(0, 0, 0));
        // 枪口指向目标的真实向量
        Eigen::Vector3d diff_map = target_map - gun_origin_map;
        
        // 提取绝对水平距离 w 和 绝对高度 h
        double true_w = std::sqrt(diff_map.x() * diff_map.x() + diff_map.y() * diff_map.y());
        double true_h = diff_map.z();

        // 调用迭代模型求解 Pitch 角，并接住实际飞行时间 t_fly
        float t_fly = 0.0;
        double absolute_pitch = pitchTrajectoryCompensation(true_w, true_h, bullet_speed, t_fly);
        double absolute_yaw = std::atan2(diff_map.y(), diff_map.x());

        // 把时间传给外层的预测器！这是最关键的一步！
        hit_time_output = t_fly; 

        // 安装误差导致的硬补偿
        solve_angle.pitch = absolute_pitch + pitch_compensation;
        solve_angle.yaw   = absolute_yaw + yaw_compensation;

        return solve_angle;
    }


    /**
     * @brief Ceres 核心求解器（接收绝对水平距离和绝对高度）
     */
    double target_pos_to_shoot_angle(const double w, const double h, const double bs, const double k) const {
        double v2 = bs * bs;
        double g = 9.8;
        
        // 判别式: 计算无阻力下的标准抛物线角度作为 Ceres 的迭代初值
        double root = v2 * v2 - g * (g * w * w + 2 * h * v2);
        double shoot_angle = 0.0;
        if (root >= 0) {
            shoot_angle = std::atan((v2 - std::sqrt(root)) / (g * w));
        } else {
            // 超出无阻力射程，给一个抬枪初值兜底
            shoot_angle = std::atan2(h, w) + (5.0 * M_PI / 180.0);
        }

        ceres::Problem problem;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ResistanceFuncLinear, 1, 1>(
                new ResistanceFuncLinear(w, h, bs, k)
            ),
            nullptr,
            &shoot_angle
        );

        // 限制云台Pitch的求解范围，防止解出奇葩角度
        const double MAX_PITCH_DOWN = -45.0 * M_PI / 180.0;
        const double MAX_PITCH_UP = 55.0 * M_PI / 180.0;
        problem.SetParameterLowerBound(&shoot_angle, 0, MAX_PITCH_DOWN);
        problem.SetParameterUpperBound(&shoot_angle, 0, MAX_PITCH_UP);

        ceres::Solver::Options options;
        options.max_num_iterations = 25;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        return shoot_angle;
    }
    

    long long m_next_shoot_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
    bool shootable() {
        long long now_fire_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
        if (targetModel->auto_fire && now_fire_time > m_next_shoot_time) {
            m_next_shoot_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count() + 200;
            return true;
        }
        return false;
    }

    /**
    * @brief  发布最终云台控制角度与开火指令的“一站式”接口
    * @param  hasAngle          是否成功计算出目标角度（true=有，false=无）
    * @param  auto_fire         本帧是否允许自动开火（默认 false）
    * @param  euler_angles_temp 目标相对角度（弧度），默认 (0,0) 
    */
    /////////////////////
    // 周期性发送debug
    // int cntflag = 0; 
    // int flag = 0;
    ///////////////////
    void Pub_Aangle(bool hasAngle, bool auto_fire = false, const cv::Point2d & euler_angles_temp = cv::Point2d(0, 0)){
        ///////////////////////////////////////////
        // 周期性发送debug
        // cntflag++;
        // if (cntflag == 2000) {
        //     if (flag == 0) {
        //         flag = 1;
        //     }
        //     else {
        //         flag = 0;
        //     }
        //     cntflag = 0;
        // }
        /////////////////////////////////////////

        // cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

        // debug: rqt_plot打印 期望pitch vs 当前pitch
        geometry_msgs::Vector3 Angle;
        if(hasAngle && euler_angles_temp.x != 0.0 && euler_angles_temp.y != 0.0 &&
           std::isfinite(euler_angles_temp.x + m_pitchOffset) &&
           std::isfinite(euler_angles_temp.y - m_yawOffset)){
            Angle.x = (euler_angles_temp.x) + m_pitchOffset;
            Angle.y = euler_angles_temp.y - m_yawOffset;
            Angle.z = 3; // Valid target angle: enter aiming, per the MCU protocol.
            // 期望pitch轴角度
            debugdate.data = Angle.x;
            // 当前pitch轴角度
            debugdate1.data = m_cur_pose.pitch;
            // debugdate2.data = -((euler_angles_temp.x) + m_pitchOffset);
            // z is an aiming state, not an automatic-fire pulse.

            AngPub.publish(Angle);
        }else{

            // 如果没有角度信息,返回陀螺仪角度
            Angle.x = RmSerialData.Pitch - m_pitchOffset;
            Angle.y = RmSerialData.Yaw - m_yawOffset;
            Angle.z = observed_this_frame_ ? 1 : 0;
            // 期望pitch轴角度
            debugdate.data = Angle.x;
            // 当前pitch轴角度
            debugdate1.data = m_cur_pose.pitch;

            AngPub.publish(Angle);
        }
    }

    // 参数配置函数
    void setParam(const std::string &file_path) {
        try {
            // 1. 文件存在性检查
            if (file_path.empty()) {
                ROS_ERROR("Empty config file path provided");
                return;
            }
            
            //2. 检查文件是否打开
            std::ifstream file_check(file_path);
            if (!file_check.is_open()) {
                ROS_ERROR("Failed to open config file: %s. Reason: %s", 
                        file_path.c_str(), strerror(errno));
                return;
            }
            ROS_INFO("Loading parameters from: %s", file_path.c_str());
            
            // 3. 加载 YAML 文件并添加详细错误处理
            YAML::Node config;
            try {
                config = YAML::LoadFile(file_path);
            } catch (const YAML::BadFile& e) {
                ROS_ERROR("Bad YAML file: %s", e.what());
                return;
            } catch (const YAML::ParserException& e) {
                ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
                return;
            }
            
            //加载参数
            if (config["Tracker"]){
                YAML::Node track = config["Tracker"];
                COMMAND_TIMESPAN = track["COMMAND_TIMESPAN"].as<double>(0.11);
                eTime = track["eTime"].as<double>(0.002);
                delay_time = track["delay_time"].as<double>(0.08);
                local_gravity_ = track["local_gravity_"].as<double>(9.80665);
                trackTime = track["trackTime"].as<double>(0.7);
                pitch_compensation = track["pitch_compensation"].as<double>(0);
                yaw_compensation = track["yaw_compensation"].as<double>(0);
                m_score_tolerance = track["score_tolerance"].as<double>(1);
                m_switch_threshold = track["switch_threshold"].as<double>(15);                                    
                m_aim_angle_tolerance = track["aim_angle_tolerance"].as<double>(40);                              
                m_aim_pose_tolerance = track["aim_pose_tolerance"].as<double>(0.05);                              
                m_aim_center_angle_tolerance = track["aim_center_angle_tolerance"].as<double>(1);                 
                m_switch_trackmode_threshold = track["switch_trackmode_threshold"].as<double>(1.2);               
                m_aim_center_palstance_threshold = track["aim_center_palstance_threshold"].as<double>(6.5);       
                m_force_aim_palstance_threshold = track["force_aim_palstance_threshold"].as<double>(0.8);         
                m_track_center = track["track_center"].as<double>(1);
                m_fix_on = track["fix_on"].as<double>(1);
                m_k = track["k"].as<double>(1);
                BulletVector = track["BulletVector"].as<double>(21);
                // ~4m硬补偿
                m_pitch_compensation4meter = track["pitch_compensation4meter"].as<double>(0) ;
                m_yaw_compensation4meter = track["yaw_compensation4meter"].as<double>(0);
                // 5.5m硬补偿
                m_pitch_compensation5_5meter = track["pitch_compensation5_5meter"].as<double>(0);
                m_yaw_compensation5_5meter = track["yaw_compensation5_5meter"].as<double>(0);
                // 6m硬补偿
                m_pitch_compensation6meter = track["pitch_compensation6meter"].as<double>(0);
                m_yaw_compensation6meter = track["yaw_compensation6meter"].as<double>(0);
                // 6.5m硬补偿 // 不考虑击打7m以外目标
                m_pitch_compensation6_5meter = track["pitch_compensation6_5meter"].as<double>(0);
                m_yaw_compensation6_5meter = track["yaw_compensation6_5meter"].as<double>(0);
                // ROS_WARN("Tracker parameters loaded:");
                // ROS_WARN("COMMAND_TIMESPAN:%.3f",COMMAND_TIMESPAN);
                // ROS_WARN("eTime:%.3f",eTime);
                // ROS_WARN("delay_time:%3.f",delay_time);
                // ROS_WARN("local_gravity_:%.3f",local_gravity_);
                // ROS_WARN("trackTime:%.3f",trackTime);
                // ROS_WARN("pitch_compensation:%.3f",pitch_compensation);
                // ROS_WARN("yaw_compensation:%.3f",yaw_compensation);
                // ROS_WARN("track_center: %d", m_track_center);
                // ROS_WARN("m_k: %f", m_k);
                // ROS_WARN("BulletVector: %f", BulletVector);
                ROS_WARN("m_pitch_compensation4meter:%.3f", m_pitch_compensation4meter);
                ROS_WARN("m_yaw_compensation4meter:%.3f", m_yaw_compensation4meter);
                ROS_WARN("m_pitch_compensation5_5meter:%.3f", m_pitch_compensation5_5meter);
                ROS_WARN("m_yaw_compensation5_5meter:%.3f", m_yaw_compensation5_5meter);
                ROS_WARN("m_pitch_compensation6meter:%.3f", m_pitch_compensation6meter);
                ROS_WARN("m_yaw_compensation6meter:%.3f", m_yaw_compensation6meter);
                ROS_WARN("m_pitch_compensation6_5meter:%.3f", m_pitch_compensation6_5meter);
                ROS_WARN("m_yaw_compensation6_5meter:%.3f", m_yaw_compensation6_5meter);
            } 
            else {
                ROS_WARN("No 'Tracker' section in config file");
            }

            // 加载相机参数
            if (config["Camera"]){
                YAML::Node camera = config["Camera"];
                // 加载相机内参矩阵
                if (camera["camera_matrix"]) {
                    std::vector<double> cam_matrix_array = camera["camera_matrix"].as<std::vector<double>>();
                    if (cam_matrix_array.size() == 9) {
                        // 创建3*3相机内参矩阵
                        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                        cam_matrix_array[0], cam_matrix_array[1], cam_matrix_array[2],
                        cam_matrix_array[3], cam_matrix_array[4], cam_matrix_array[5],
                        cam_matrix_array[6], cam_matrix_array[7], cam_matrix_array[8]);
                    }
                }
                else {
                    ROS_WARN("No 'camera_matrix' section in config file");
                }

                // 加载畸变系数矩阵
                if (camera["dist_coeffs"]) {
                    std::vector<double> dist_coeffs_array = camera["dist_coeffs"].as<std::vector<double>>();
                    if (dist_coeffs_array.size() == 5) {
                        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 
                            dist_coeffs_array[0], dist_coeffs_array[1], dist_coeffs_array[2], 
                            dist_coeffs_array[3], dist_coeffs_array[4]);
                    }
                }
                else {
                    ROS_WARN("No 'dist_coeffs' section in config file");
                }

                // 加载补偿误差函数载串口补偿
                if (config["InstallOffset"])
                {
                    YAML::Node mod = config["InstallOffset"];
                    m_rollOffset = mod["rollOffset"].as<double>(0);
                    m_pitchOffset = mod["pitchOffset"].as<double>(0);
                    m_yawOffset = mod["yawOffset"].as<double>(0);
                    // ROS_INFO("InstallOffset parameters loaded:");
                    // ROS_INFO("m_rollOffset:%.3f",m_rollOffset);
                    // ROS_INFO("m_pitchOffset:%.3f",m_pitchOffset);
                    // ROS_INFO("m_yawOffset:%.3f",m_yawOffset);
                    // ROS_INFO("InstallOffset parameters loaded successfully");
                }
                else
                {
                    ROS_WARN("No 'CoordinateTransformer' section in config file");
                }
            }
        } 
        catch (const YAML::Exception& e) {
            ROS_ERROR("YAML parsing error: %s", e.what());
        } 
        catch (const std::exception& e) {
            ROS_ERROR("Error loading parameters: %s", e.what());
        } 
        catch (...) {
            ROS_ERROR("Unknown error occurred while loading config file");
        }
    }

    /**
    * @brief 把世界坐标系下的 3D 点投影到当前图像平面
    * @param objectPoints 世界坐标系中的一组 3D 点
    * @param frame 用于可视化的 OpenCV 图像
    * @param can_show 是否弹出窗口并打印调试信息
    * @return 投影到图像上的 2D 像素坐标
    */
   // 重投影这里传入了相机的内参和旋转矩阵,如果重投影的结果有问题(rviz也是这样)(很有可能是识别那里世界坐标系变换到相机坐标系有问题),这很大可能是因为相机内参和畸变系数不准导致的,这时候需要重新标定相机
    std::vector<cv::Point2d> projectPointsToImage(const std::vector<cv::Point3d>& objectPoints, cv::Mat& frame, bool can_show) {
        cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//旋转向量
        cv::Mat tvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//平移向量
        std::vector<cv::Point2d> imagePoints;
        cv::projectPoints(objectPoints, rvec, tvec, camera_matrix_, dist_coeffs_, imagePoints);
        if(can_show){
            for (size_t i = 0; i < imagePoints.size(); ++i) {
                cv::circle(frame, imagePoints[i], 7, cv::Scalar(0, 255, 0), -1); 
                //std::cout << "3D Point " << i << ": " << objectPoints[i] << std::endl;
                //std::cout << "Projected 2D Point " << i << ": " << imagePoints[i] << std::endl;
            }
            cv::imshow("Draw Points", frame);
            cv::waitKey(1);
        }
        return imagePoints;
    }

    /**
    * @brief 把“世界坐标系下的三维标准点”重映射到图像二维平面
    * @param pegPos 标准点在“map”坐标系下的三维坐标 (x,y,z)
    * @param cal TF/坐标变换工具类实例
    * @param projectPointsToImage 3D→2D 投影函数
    * @param frame_ 克隆图形用于可视化
    * @return 该点在图像上的像素坐标 (u,v)；若 TF 异常则返回 (0,0)
    */
    cv::Point2d project_pegPoints(Eigen::Vector3d& pegPos){
        Eigen::Vector3d resultPos;
        resultPos << pegPos.x(), pegPos.y(), pegPos.z();
        Eigen::Vector3d resultPose;
        resultPose << 0, 0, 0;
        cal->TFUpdata("map","PEG", resultPos, resultPose, 1);
        
        auto absPos_temp = cal->GetPos("cam","PEG",1);
        if((absPos_temp[0].x() == 0 && absPos_temp[0].y() == 0 && absPos_temp[0].z() == 0) || isnan(absPos_temp[1].z())){
            cout << "重投影坐标系未更新！" << endl;
            return cv::Point2d(0.0, 0.0);
        }
        
        cv::Point3d objectPoint( absPos_temp[0].x(), absPos_temp[0].y(), absPos_temp[0].z());
        std::vector<cv::Point3d> objectPoints = {objectPoint};
        std::vector<cv::Point2d> imagePoints = projectPointsToImage(objectPoints, frame_, false);
        
        return imagePoints[0];
    }

private:
    /**
     * @brief 装甲板处理总入口：决策 → 丢失检测 → 预测 → 返回平滑 3D 状态
     * @param armors 当前帧所有检测到的装甲板数组
     * @return TargetModel* 目标模型指针（正常状态）或nullptr（异常状态）
     */
    TargetModel* armorUpdate(rm_msgs::ArmorArrayConstPtr armors)
    {
        // 1.时间戳管理
        // 获取当前时间戳(毫秒)
        Now_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
        // 2.计算目标丢失时间(毫秒)
        double Lost_Time = (Now_Time_armor - Track_Time_armor)/1000.0;
        // 目标丢失状态处理(上一帧的目标状态)
        if(!Switch_Armor){
            // 跟踪中状态:打印跟踪当前ID
            // std::cout << "追踪中TrackingID: " << static_cast<int>(TrackingID) << std::endl;
        }else{
            // 目标丢失状态: 重置跟踪ID和旋转方向
            // std::cout << "目标丢失!重置TrackingID!" << std::endl;
            TrackingID = -1;
        }
        // 3.装甲板决策
        rm_msgs::Armor* detect = nullptr;
        if(armors->armors.size()){
            // 决策函数: 从所有装甲板中选择最佳跟踪目标(本帧决策)
            detect = decision(armors, TrackingID, Switch_Armor);
            if(detect != nullptr){ // 本帧状态更新
                if(Switch_Armor){
                    // 目标丢失后重新锁定: 更新跟踪ID
                    TrackingID = detect->number;
                    Switch_Armor = false;
                    // 更新追踪时间戳
                    Track_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//更新追踪时间戳
                }else if(detect->number == TrackingID){
                    // 正常跟踪: 更新时间戳
                    Track_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//更新追踪时间戳
                }else{
                    // 检测到非跟踪目标: 忽略
                    detect = nullptr;
                }
            }
        }
        // 4. 目标丢失判定
        observed_this_frame_ = detect != nullptr;
        if(Lost_Time > trackTime){
            // 普通目标: 超过跟踪时间预测判定丢失
            if(TrackingID != 6){
                Switch_Armor = true;
            // 前哨站:3倍跟踪时间阈值与判定丢失
            }else if(Lost_Time > trackTime*3){
                Switch_Armor = true;
            }
        }
        // 运行预测流程
        TargetModel* targetModel_temp = nullptr;
        targetModel_temp = runPrediction(Switch_Armor, detect);
        return targetModel_temp;
    }

    /**
    * @brief  装甲板决策函数 —— 从所有候选装甲板中选出“应被瞄准”的那一块
    * @param  armors          当前帧所有装甲板数组（ROS 消息）
    * @param  TrackingID_temp 正在跟踪的目标 id（字符形式）
    * @param  Switch_Armor_temp 是否允许切换目标（true = 允许重新选择）
    * @return rm_msgs::Armor*   选中的装甲板指针；nullptr 表示无有效目标
    * @note   决策流程：先按距离/ID 筛，再按旋转方向挑同号板
    */
    rm_msgs::Armor* decision(rm_msgs::ArmorArrayConstPtr armors,
                             const char& TrackingID_temp,
                             const bool& Switch_Armor_temp){
        rm_msgs::Armor* detect_temp = nullptr;
        double minDisToCenter = std::numeric_limits<double>::max();
        int cunt = 0;
        for(auto& armor:armors->armors){
            if(Eigen::Vector3d({armor.armorPose.position.x,armor.armorPose.position.y,armor.armorPose.position.z}).norm() >= 8.0 || armor.number == 9){
                if(armor.number != 6){
                    continue;
                }else if(Eigen::Vector3d({armor.armorPose.position.x,armor.armorPose.position.y,armor.armorPose.position.z}).norm() >= 10.0 || armor.number == 9){
                    continue;
                }
            }
            if(armor.number == TrackingID_temp || (Switch_Armor_temp && armor.armorToCenterDis < minDisToCenter)){
                if (!detect_temp || armor.armorToCenterDis < minDisToCenter) {
                    detect_temp = const_cast<rm_msgs::Armor*>(&armor);
                    minDisToCenter = armor.armorToCenterDis;
                }
            }
            if(armor.number == TrackingID_temp){
                cunt++;
            }
        }
        if(detect_temp == nullptr){
            //std::cout << "决策结果为空！" << std::endl;
            return nullptr;
        }
        return detect_temp;
    }

    /**
     * @brief 初始化目标模型类型 
     * @param detect 当前帧检测到的装甲板信息指针
     * @return bool true-模型已初始化完成，false-模型未初始化完成
     */
    bool initModel(rm_msgs::Armor* detect)
    {
        //detect->type删掉,观测不准,detect->type基地直接瞄，不过卡尔曼
        if (detect != nullptr && targetModel->m_type_init_cnt < 3) 
        {
            // 大装甲板1号->英雄
            // 小装甲板2~7号->普通步兵
            if ((detect->number >= 1 && detect->number <= 5) || detect->number == 7)
            {
                // 类型变化时重置计数器
                if (targetModel->m_model_type != KinematicModel::STANDARD)
                    targetModel->m_type_init_cnt = 0;
                // 设置为标准模型
                targetModel->m_model_type = KinematicModel::STANDARD;
                targetModel->m_type_init_cnt++;
            }
            else if  (detect->number == 6)
            {
                if (targetModel->m_model_type != KinematicModel::OUTPOST)
                    targetModel->m_type_init_cnt = 0;
                targetModel->m_model_type = KinematicModel::OUTPOST;
                targetModel->m_type_init_cnt++;
            }
            
            // 连续3帧确认后初始化模型
            if (targetModel->m_type_init_cnt >= 3)
            {
                setModelType(targetModel->m_model_type);
                return true; // 已经初始化
            }
            else
            {
                // 不允许击打
                targetModel->auto_fire = false;
                return false; // 没有初始化
            }
        }
        if (targetModel->m_type_init_cnt < 3) {
            return false;  // 没有检测到目标,没有初始化
        }
        else {
            return true;   // 已经初始化,大于3帧，处于跟踪状态,短暂丢失
        }
    }

    /**
     * @brief 重置预测模块和目标模型状态
     * @param lost_state 目标丢失状态（true-目标丢失，false-目标存在）
     * @param abnormal 预测状态异常标志（true-状态异常，false-状态正常）
     * @return bool 返回当前的目标丢失状态
     * @note 重置条件：目标丢失(lost_state)或预测状态异常(abnormal)
     */
    bool reset(bool lost_state, bool abnormal)
    {
         // 检查是否需要重置(目标丢失或状态异常)
        if(lost_state || abnormal){
            // 1. 创建新的目标模型实例(完全重置)
            targetModel = std::make_unique<TargetModel>(config_path_);

            history_armor_facing.clear();
            history_armor_target.clear();

            return lost_state; // 丢失状态
        }
        else
        {
             return lost_state; // 跟踪状态
        }
    }
    
    /**
     * @brief 执行预测流程的核心函数
     */
    TargetModel* runPrediction(bool Switch_Armor_temp, rm_msgs::Armor* detect){
       // 检查重置
        if (reset(Switch_Armor_temp, targetModel->abnormal)){
            return nullptr;
        }
        // 初始化
        bool isInitialized = initModel(detect);
        if (isInitialized){
            // 更新目标模型(执行核心预测流程)
            updateTargetModel(detect);
        }
        else{
            return nullptr;
        }
        // 返回预测结果(异常状态返回nullptr,正常状态返回目标模型指针)
        if (targetModel->abnormal) // 标志位异常返回空,正常的话检查是否初始化
        {
            return nullptr;
        }
        else
        {
            return targetModel.get();
        }
    }

    
    /**
     * @brief 更新目标模型状态
     * @param detect 当前帧检测到的装甲板信息指针
     */
    void updateTargetModel(rm_msgs::Armor* detect) {
        // 初始化装甲板属性(设置ID,类型)
        setArmorProps(detect);
        // 核心预测处理:使用扩展卡尔曼滤波器进行状态预测和更新
        updateEKFState(detect);
    }

    /**
    * @brief 设置装甲板基本属性
    * @param detect 当前检测到的装甲板消息指针（nullptr 则跳过）
    */
    void setArmorProps(rm_msgs::Armor* detect){
        if(detect != nullptr){
            // 设置装甲板ID
            targetModel->armor.id = detect->number;
            // 设置装甲板类型
            targetModel->armor.armor_type = detect->type;
            // 根据装甲板类型设置数量
            targetModel->count = (detect->number == 6) ? 3 : 4;
       }
    }

    /**
     * @brief 相机坐标系点重投影可视化
     * @param pre_armors 预测的装甲板位置信息
     * @param armors_position_pose 观测到的装甲板位置和姿态信息
     */
    void projectCameraPointsToImage(Armors pre_armors,std::vector<std::vector<Eigen::Vector3d>> armors_position_pose) {
        std::vector<cv::Point3d> objectPoints;

        // 将观测的装甲板放进预测的装甲板数组来投影观测的装甲板
        if (armors_position_pose.size()) {
            Armor temp;
            temp.position.x() = armors_position_pose[0][0].x();
            temp.position.y() = armors_position_pose[0][0].y();
            temp.position.z() = armors_position_pose[0][0].z();
            pre_armors.push_back(temp);
        }

        // 5个点的重投影(观测点和重构出来的四块装甲板)
        for (int i = 0; i < pre_armors.size(); i++) {
            coorConverter->tfupdate_imu({m_cur_pose.roll,m_cur_pose.pitch,m_cur_pose.yaw});
            Eigen::Vector3d temp = coorConverter->map2Cam(pre_armors[i].position);
            cv::Point3d point_temp_1 = Point3d(temp.x(),temp.y(),temp.z());
            objectPoints.push_back(point_temp_1);
        }            

        // 重投影装甲板中心点
        std::vector<cv::Point2d> imagePoints = projectPointsToImage(objectPoints, frame_, 0);
        for (size_t i = 0; i < imagePoints.size(); i++) {
            cv::circle(frame_,imagePoints[i],7,cv::Scalar(0,255,0),-1);
        }
        int count = targetModel->m_status->number;  
        // 为每个装甲板绘制四个角点和对角线
        for (int i = 0; i < count; i++) {
            Eigen::Vector3d rpy = pre_armors[i].rpy;           // 装甲板姿态
            Eigen::Vector3d center = pre_armors[i].position;   // 装甲板中心位置
            double yaw = pre_armors[i].rpy.z() - m_cur_pose.yaw;
            std::vector<Eigen::Vector3d> cornerpoint;
            if (targetModel->armor.id == 6) {
                cornerpoint = coorConverter->armor2Corner(center,targetModel->armor.id,-0.2618, yaw);
            }
            else {
                cornerpoint = coorConverter->armor2Corner(center,targetModel->armor.id,0.2618, yaw);
            }
            // 将角点从世界系 -> 相机系
            for (int i = 0; i < cornerpoint.size(); i++) {
                cornerpoint[i] = coorConverter->map2Cam(cornerpoint[i]);
            }
            // 调用前进行类型转换
            std::vector<cv::Point3d> cvPoints;
            cvPoints.reserve(cornerpoint.size());
            for (const auto& point : cornerpoint) {
                cvPoints.emplace_back(point.x(), point.y(), point.z());
            }
            // 投影每块装甲板的四个角点
            std::vector<cv::Point2d> cornerPoints2D = projectPointsToImage(cvPoints, frame_, 0);
            // 绘制四个角点
            for (const auto& point : cornerPoints2D) {
                cv::circle(frame_,point , 5, cv::Scalar(0, 0, 255), -1); // 红色角点
            }
            // 绘制装甲板边框(连接四个角点)
            for (int j = 0; j < 4; j++) {
                int next_j = (j + 1) % 4;
                cv::line(frame_, cornerPoints2D[j], cornerPoints2D[next_j], 
                cv::Scalar(255, 255, 0), 2); // 青色边框
            }
            // 绘制对角线
            cv::line(frame_, cornerPoints2D[0], cornerPoints2D[2], 
                cv::Scalar(0, 255, 255), 2); // 黄色对角线1（左上到右下）
            cv::line(frame_, cornerPoints2D[1], cornerPoints2D[3], 
                cv::Scalar(0, 255, 255), 2); // 黄色对角线2（右上到左下）
        }   
        // 最后显示图像
        cv::imshow("Draw Points", frame_);
        cv::waitKey(1);
    }
    
    /**
     * @brief 二维地图投影可视化
     * @param pre_armors 预测的装甲板位置信息
     * @param armors_position_pose 观测到的装甲板位置和姿态信息
     */
    void projectMapPointsToImage (Armors pre_armors,std::vector<std::vector<Eigen::Vector3d>> armors_position_pose) {
        // // debug
        // cout << "预测的第一块装甲板的x坐标: " << pre_armors[0].position.x() << endl;
        // cout << "预测的第一块装甲板的y坐标: " << pre_armors[0].position.y() << endl;
        const double PIXELS_PER_METER = 50; // 比例尺 -> 100像素对应实际观测1米
        // 创建白色背景图像
        Mat whiteImage(800, 800, CV_8UC3, Scalar(255, 255, 255));
        // 绘制基准线
        for (int i = 1; i <= 8; i++) {
            if (i == 6) {
                cv::circle(whiteImage, cv::Point(400, 400), i * PIXELS_PER_METER, cv::Scalar(0, 0, 255), 1);
                continue;
            }
            cv::circle(whiteImage, cv::Point(400, 400), i * PIXELS_PER_METER, cv::Scalar(0, 0, 0), 1);
        }
        // 绘制估计装甲板位置
        for (int i = 0; i < pre_armors.size(); i++) {
            // 绘制实心圆点，半径5像素，绿色
            cv::circle(whiteImage, cv::Point(-pre_armors[i].position.y()*PIXELS_PER_METER + 400, -pre_armors[i].position.x()*PIXELS_PER_METER + 400), 3, cv::Scalar(0, 255, 0), -1);
            // break; // 测试(只打印第一块预测装甲板)
        }
        // 绘制观测装甲板位置
        for (int i = 0; i < armors_position_pose.size(); i++) {
            // 绘制观测装甲板
            double x = armors_position_pose[i][0].x();
            double y = armors_position_pose[i][0].y();
            cv::circle(whiteImage, cv::Point(-y*PIXELS_PER_METER + 400, -x*PIXELS_PER_METER + 400), 3, cv::Scalar(0, 0, 0), -1);
            double r[2] = {0.25,0.25};
            // double yaw = armors_position_pose[0][1].z();
            double yaw = armors_position_pose[i][1].z();
            double center_x = armors_position_pose[i][0].x() - r[0] * cos(yaw);
            double center_y = armors_position_pose[i][0].y() - r[0] * sin(yaw);
            cv::circle(whiteImage, cv::Point(-center_y*PIXELS_PER_METER + 400, -center_x*PIXELS_PER_METER + 400), 3, cv::Scalar(255, 0, 0), -1);
            // whiteImage.at<Vec3b>(armor_x*20 + 400, armor_y*20 + 400) = Vec3b(0, 255, 0);
            // 绘制实心圆点，半径5像素，黑色
            // // 绘制重构出来的装甲板 蓝色
            // // if (i > 1) {
            // //     continue;
            // // }
            for (int j = 0; j < 4; j++) {
                double armor_x = center_x + r[j%2] * cos(yaw + 0.5 * j * PI);
                double armor_y = center_y + r[j%2] * sin(yaw + 0.5 * j * PI);
                cv::circle(whiteImage, cv::Point(-armor_y*PIXELS_PER_METER + 400, -armor_x*PIXELS_PER_METER + 400), 3, cv::Scalar(255, 0, 0), -1);
            }
        }
        int x = static_cast<int>(-pre_armors[(targetModel->m_status->index)].position.y()*PIXELS_PER_METER + 400);
        int y = static_cast<int>(-pre_armors[(targetModel->m_status->index)].position.x()*PIXELS_PER_METER + 400);
        // cout << "targetModel->m_status->index: " << targetModel->m_status->index << endl;
        // 选择距离中心点角度最小的点(这里和实际算法不一样,只是单纯编写一个算法框架)
        // 根据上一帧选择的重构装甲板index(编号)绘制击打线
        // 绘制击打线(击打预测)
        cv::line(whiteImage, Point(400,400), cv::Point(x,y), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        // 圈出打击目标
        cv::circle(whiteImage, cv::Point(x,y), 5, cv::Scalar(0, 0, 255), 1);
        // 绘制自身车辆位置(地图中心)
        cv::circle(whiteImage, cv::Point(400, 400), 5, cv::Scalar(0, 0, 0), -1);
        imshow("White Image with a Black Pixel", whiteImage);
        waitKey(1); // 等待按键
    }

    /**
     * @brief 使用扩展卡尔曼滤波器更新目标状态
     * @param detect 当前帧检测到的装甲板信息（rm_msgs::Armor指针）
     *              - 非nullptr：表示有装甲板检测，执行完整流程
     *              - nullptr：表示无装甲板检测，执行纯预测
     */
    void updateEKFState(rm_msgs::Armor* detect) 
    {
        /*
        命名规则:
        DETECT{id}(基础装甲板)
        DETECT{id}_0(另一块装甲板)
        */
        // 主要通过依赖TF的异常处理来跳过不存在的装甲板
        int armor_id = targetModel->armor.id;                                   // 获取当前装甲板的ID
        std::string base_armor_frame = "DETECT" + std::to_string(armor_id);     
        ros::Time unified_time;                                                  
        bool base_transform_found = false;                                       
        std::vector<std::vector<Eigen::Vector3d>> all_Armor_Position_Pose;      // 存储所有装甲板的位置和姿态
        std::vector<Eigen::Vector3d> base_position_pose;
        base_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第一个向量置零
        base_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第二个向量置零
        // 声明变量
        geometry_msgs::TransformStamped base_transform;
        geometry_msgs::TransformStamped other_transform;
        
        // 在拿装甲板前延时(使得获取装甲板的坐标尽可能精确)
        // // 帧率控制
        end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
        double frame_time = (end_Time - begin_Time)/1000.0;
        if ((end_Time - begin_Time) - 25.0 < 0.0)// 帧率控制
        {
            int sleep_time_ms = static_cast<int>(25.0 - (end_Time - begin_Time));
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            //frame_time = 0.03; 
            end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            frame_time = (end_Time - begin_Time)/1000.0;
            // cout << "sleep_time_ms:" << sleep_time_ms << "ms" << endl;
            // cout << "frame_time:" << frame_time*1000 << "ms" << endl;
        }
        begin_Time = end_Time;
        // debugdate1.data = frame_time*1000; 

        // 查询第一块装甲板
        try{
            
            // A missing transform is an invalid observation, never a default
            // pose. Do not block the control loop waiting for one second.
            base_transform = tfBuffer_.lookupTransform("map", base_armor_frame, ros::Time(0));
            const double transform_age = (ros::Time::now() - base_transform.header.stamp).toSec();
            if (base_transform.header.stamp.isZero() || transform_age < 0.0 || transform_age > 0.2) {
                targetModel->abnormal = true;
                return;
            }

            targetModel->update_count++; 
            
            // // debug: 计算tf上传时间差
            ////////////////////////////////////////////////////////////
            // tool_begin_Time_ros = base_transform.header.stamp;
            // ros::Duration test = tool_begin_Time_ros - tool_end_Time_ros;
            // debugdate.data = test.toSec() * 1000;
            // cout << "duration_time" << tool_begin_Time_ros - tool_end_Time_ros << "ms" << endl;
            // tool_end_Time_ros = tool_begin_Time_ros;
            // cout << "打印第一块装甲板///////////////////////" << endl;
            // cout << "打印位置x:" << base_transform.transform.translation.x << endl;
            // cout << "打印位置y:" << base_transform.transform.translation.y << endl;
            // cout << "打印位置z:" << base_transform.transform.translation.z << endl;
            //ROS_DEBUG("[ArmorQuery] 成功获取基础装甲板 %s 的变换，统一时间戳: %f", base_armor_frame.c_str(), unified_time.toSec());
            //////////////////////////////////////////////////////////////
            
            base_transform_found = true;
            unified_time = base_transform.header.stamp;

            // 初始化上一块装甲板的时间
            if (targetModel->update_count == 1) {
                last_tf_time = unified_time;
            }

            if (unified_time <= last_tf_time && targetModel->update_count != 1) {
                // ROS_WARN("时间戳相等装甲板未更新");
                base_transform_found = false;
            }
            last_tf_time = unified_time;

            // 提取装甲板的位置
            Eigen::Vector3d base_position(
                base_transform.transform.translation.x,
                base_transform.transform.translation.y,
                base_transform.transform.translation.z
            );

            // 提取装甲板的姿态
            tf2::Quaternion qtn(base_transform.transform.rotation.x,
                        base_transform.transform.rotation.y,
                        base_transform.transform.rotation.z,
                        base_transform.transform.rotation.w);
            tf2::Matrix3x3 matrix(qtn);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            Eigen::Vector3d base_pose;
            base_pose.x() = roll;
            base_pose.y() = pitch;
            base_pose.z() = yaw;
            base_pose.z() = targetModel->last_yaw + angles::shortest_angular_distance(targetModel->last_yaw, base_pose.z());//获得两帧之间的最小角度，更新当前yaw(避免π跳到-π)
            targetModel->last_yaw = base_pose.z();

            // rqt打印观测值
            // debugdate.data = base_position[0];           // rqt打印第一块装甲板观测x
            // debugdate.data = base_position[1];           // rqt打印第一块装甲板观测y
            // debugdate.data = base_position[2];           // rqt打印第一块装甲板观测z
            // debugdate.data = base_pose[2] / 3.14 * 180;  //rqt打印第一块装甲板观测yaw
            // 打印观测值
            // cout << "第一块装甲板观测值: " << endl;
            // cout << "机器人中心的x坐标: " << base_position[0] << endl;
            // cout << "机器人中心的y坐标: " << base_position[1] << endl;
            // cout << "装甲板的高度: " << base_position[2]<< endl;
            // cout << "机器人整体偏航角yaw: " << base_pose[2] << endl;
            // // 计算协方差 (用于给ekf初值R)
            // double R = calculate_R(base_position[0]);      // 计算x的协方差    -> pose
            // double R = calculate_R(base_position[1]);   // 计算y的协方差    -> pose
            // double R = calculate_R(base_position[2]);   // 计算z的协方差    -> pose
            // double R = calculate_R(base_pose[2]);       // 计算yaw的协方差  -> yaw
            // debugdate.data = R;
            // cout << "协方差: " << R << endl; 
            
            // 将位置和姿态存储到数据结构中
            base_position_pose[0] = base_position;  // 位置
            base_position_pose[1] = base_pose;      // 姿态

            /*
            如果串口没有发数据,那么坐标系就不会更新就会进这段代码提前返回,已经debug测试过,没有发送串口数据后会进入这段代码
            //坐标系异常检测
            */
            if((base_position_pose[0].x() == 0 && base_position_pose[0].y() == 0 && base_position_pose[0].z() == 0) 
                || isinf(base_position_pose[0].x()) || isinf(base_position_pose[0].y()) || isinf(base_position_pose[0].z())
                || isnan(base_position_pose[0].x()) || isnan(base_position_pose[0].y()) || isnan(base_position_pose[0].z())
                || isnan(base_position_pose[1].z()) || isinf(base_position_pose[1].z())){
                cout << "惯性系坐标系未更新！" << endl;
                targetModel->abnormal = true;
                return;
            }

            // 存储第一块装甲板的位置和姿态
            all_Armor_Position_Pose.push_back(base_position_pose);
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("[ArmorQuery] 无法获取基础装甲板 %s 的变换: %s", base_armor_frame.c_str(), ex.what());
            // 设置目标状态为异常并返回
            targetModel->abnormal = true;
            ROS_WARN("[ArmorQuery] 由于基础装甲板查询失败，跳过本帧处理");
            return;
        }

        // 查询第二块装甲板
        if (base_transform_found) {
            std::string other_armor_frame = "DETECT" + std::to_string(armor_id) + "_" + std::to_string(0);
            try {
                // 等待缓冲区更新
                other_transform = tfBuffer_.lookupTransform("map", other_armor_frame, unified_time, ros::Duration(0));

                // 提取装甲板位置
                Eigen::Vector3d other_position(
                    other_transform.transform.translation.x,
                    other_transform.transform.translation.y,
                    other_transform.transform.translation.z
                );

                // 提取装甲板姿态
                tf2::Quaternion qtn(other_transform.transform.rotation.x,
                            other_transform.transform.rotation.y,
                            other_transform.transform.rotation.z,
                            other_transform.transform.rotation.w);
                tf2::Matrix3x3 matrix(qtn);
                double roll, pitch, yaw;
                matrix.getRPY(roll, pitch, yaw);
                Eigen::Vector3d other_pose;
                other_pose.x() = roll;
                other_pose.y() = pitch;
                other_pose.z() = yaw;
                other_pose.z() = targetModel->last_yaw + angles::shortest_angular_distance(targetModel->last_yaw, other_pose.z());//获得两帧之间的最小角度，更新当前yaw(避免π跳到-π)
                targetModel->last_yaw = other_pose.z();

                //debugdate1.data = other_position[0];  // rqt打印第二块装甲板观测x
                //debugdate1.data = other_position[1];  // rqt打印第二块装甲板观测y
                //debugdate1.data = other_position[2];  // rqt打印第二块装甲板观测z
                //debugdate1.data = other_pose[2];      // rqt打印第二块装甲板观测yaw

                // cout << "第二块装甲板观测值: " << endl;
                // cout << "机器人中心的x坐标: " << other_position[0] << endl;
                // cout << "机器人中心的y坐标: " << other_position[1] << endl;
                // cout << "装甲板的高度: " << other_position[2]<< endl;
                // cout << "机器人整体偏航角yaw: " << other_pose[2] << endl;

                // 将位置和姿态存储到数据结构中
                std::vector<Eigen::Vector3d> other_position_pose;
                other_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第一个向量置零
                other_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第二个向量置零
                other_position_pose[0] = other_position;  // 位置
                other_position_pose[1] = other_pose;      // 姿态

                //坐标系异常检测
                if((other_position_pose[0].x() == 0 && other_position_pose[0].y() == 0 && other_position_pose[0].z() == 0) 
                    || isinf(other_position_pose[0].x()) || isinf(other_position_pose[0].y()) || isinf(other_position_pose[0].z())
                    || isnan(other_position_pose[0].x()) || isnan(other_position_pose[0].y()) || isnan(other_position_pose[0].z())
                    || isnan(other_position_pose[1].z()) || isinf(other_position_pose[1].z())){//坐标系异常检测
                    cout << "惯性系坐标系未更新！" << endl;
                    targetModel->abnormal = true;
                    return;
                }

                // 存储第二块装甲板的位置和姿态
                all_Armor_Position_Pose.push_back(other_position_pose);
            } 
            catch(tf2::TransformException &ex) {
                // ROS_INFO("未查询到第二块装甲板yyyyyyyyyyyyy");
            }   
        }

        visual_armor_position_pose_temp = all_Armor_Position_Pose; // 用于可视化


        // 有装甲板->上传惯性系+预测+更新+更新目标对象
        if (base_transform_found)
        {
            if (all_Armor_Position_Pose.empty()) {
                ROS_WARN("[ArmorQuery] 未找到任何有效的装甲板数据");
                targetModel->abnormal = true;
                return;
            } else {
                ROS_DEBUG("[ArmorQuery] 成功获取 %zu 块装甲板的数据", all_Armor_Position_Pose.size());
                targetModel->abnormal = false;
            }

            // 创建装甲板实例
            Armors armors(all_Armor_Position_Pose.size());
            for (int i = 0; i < all_Armor_Position_Pose.size(); i++) 
            {
                armors[i].position = Eigen::Vector3d(all_Armor_Position_Pose[i][0].x(),all_Armor_Position_Pose[i][0].y(),all_Armor_Position_Pose[i][0].z());
                armors[i].rpy = Eigen::Vector3d(all_Armor_Position_Pose[i][1].x(),all_Armor_Position_Pose[i][1].y(),all_Armor_Position_Pose[i][1].z());
            }

            // 先验状态预测
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> prior = targetModel->ekf->predict(armors);

            // cout << "输出状态矩阵:" << endl;
            // cout << "///////////////////////////////" << endl;
            // cout << prior.first << endl;
            // cout << "///////////////////////////////" << endl;

            // 创建运动模型实例
            std::shared_ptr<KinematicModel> prior_status;
            if (targetModel->m_model_type == KinematicModel::STANDARD)
            {
                prior_status = std::make_shared<StandardModel>(prior.first);
            }
            else if (targetModel->m_model_type == KinematicModel::BALANCE)
            {
                prior_status = std::make_shared<BalanceModel>(prior.first);
            }
            else if (targetModel->m_model_type == KinematicModel::OUTPOST)
            {
                prior_status = std::make_shared<OutpostModel>(prior.first);
            }

            // 数据关联匹配(计算检测到的装甲板与预测的装甲板之间的匹配度,在得到评分矩阵后为每个检测到的装甲板找到一个最合适的预测装甲板对应,形成一个最优匹配)
            Eigen::MatrixXd score = getScoreMat(armors,prior_status->getArmors(0));
            // cout << "输出分数:" << endl;
            // cout << score << endl;

            std::map<int, int> armor_match = getMatch(score, m_score_tolerance,targetModel->count);

            /*
            正常的数据关联匹配结果： 
            1.一一对应关系: 
                每个检测装甲板索引对应唯一的目标装甲板索引
                没有重复的匹配关系
            2. 索引范围合理

            3.得分低于阈值
                所有匹配对的得分都低于m_score_tolerance
                得分越低表示匹配度越高

            1.绿色点(预测)和黑色点(预测)是否在空间上接近
            2.匹配的装甲板在图像上是否对齐
            */
            // debug
            // for (auto it : armor_match)
            // {
            //     cout << "装甲板序列: " << it.first  << "目标装甲板索引: " << it.second << endl;
            //     debugdate.data = it.first;
            //     debugdate1.data = it.second;
            // } 

            int current_index = targetModel->m_status->index;
            targetModel->m_status = targetModel->ekf->update(armors, prior.first, prior.second,armor_match);
            targetModel->m_status->index = current_index;

            // debug (用于调试ekf)
            Eigen::VectorXd current_state = targetModel->m_status->getState();
            // prior_status->print("预测的数据");
            // debugdate1.data = current_state(0);                                    // rqt预测x车中心
            // debugdate1.data = targetModel->m_status->getArmors(0)[0].position.x(); // rqt打印预测第一块装甲板x坐标
            // debugdate1.data = current_state(2);                                    // rqt预测y车中心
            // debugdate.data = current_state[4];                                     // rqt预测z左侧
            // debugdate1.data = current_state[5];                                    // rqt预测z右侧
            // debugdate1.data = current_state(8) / 3.14 * 180;                       // rqt预测yaw
            // debugdate.data = current_state(6);                                     // rqt打印半径左侧
            // debugdate1.data = current_state(7);                                    // rqt打印半径右侧

            // 可视化
            projectCameraPointsToImage(targetModel->m_status->getArmors(0),all_Armor_Position_Pose);
        }
        // 无检测目标仅预测+更新目标对象
        else
        {   
            // 用于debug
            ROS_WARN("无装甲板仅预测");
            // 执行卡尔曼滤波器(传入空装甲板列表)
            Armors empty_armors;
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> prior = targetModel->ekf->predict(empty_armors);
            std::map<int, int> armor_match;
            armor_match.clear();
            // 后验状态更新
            targetModel->m_status = targetModel->ekf->update(empty_armors, prior.first, prior.second,armor_match);
            // 可视化
            Armors no_predict_armos = targetModel->m_status->getArmors(0);
            std::vector<std::vector<Eigen::Vector3d>> empty_Armor_Position_Pose;
            projectCameraPointsToImage(no_predict_armos,empty_Armor_Position_Pose);
            return;       
        }
    
    }

    /***
     * 工具函数
     * 用于实时计算R(协方差)
     * 
    */
    // 用于计算R值
    double max_num = INT_MIN;
    double min_num = INT_MAX;
    double calculate_R(double num) {
        if(Switch_Armor){ 
            min_num = INT_MAX;
            max_num = INT_MIN;
        }
        // 计算R值的函数
        if (num > max_num) {
            max_num = num;
        }
        if (num < min_num) {
            min_num = num;
        }
        double sqrt_R = (max_num - min_num) * 0.5;
        double R = sqrt_R * sqrt_R;
        cout << "协方差: " << R << endl; 
        return R;
    }


    /***
     * 工具函数
     * 用于计算单帧之间的最大角度差值
    */
    // 用于计算R值
    double last_yaw_test = 0;
    double max_between_yaw = 0;
    int cnt = 0;
    double calculate_max_yaw(double yaw) {

        if(cnt == 0){ 
            max_between_yaw = 0;
            last_yaw_test = yaw;
        }
        double yaw_temp = abs(yaw - last_yaw_test);
        if (yaw_temp > max_between_yaw) {
            max_between_yaw = yaw_temp;
        }
        last_yaw_test = yaw;
        cnt++;
        cout << "两帧之间最大角度差: " << max_between_yaw << endl; 
        return max_between_yaw; 
    }

    /**
     * @brief 基于熵权法计算装甲板匹配得分矩阵
     * @param detect_armors 检测到的装甲板集合
     * @param standard_armors 预测的标准装甲板集合
     * @return Eigen::MatrixXd 匹配得分矩阵(行:检测装甲板, 列:预测装甲板)
     */
    Eigen::MatrixXd getScoreMat(const Armors &detect_armors, const Armors &standard_armors)
    {
        int m = detect_armors.size();
        int n = standard_armors.size();
        // 计算两组装甲板之间的坐标差和角度差两个负向指标
        Eigen::Matrix<double, Eigen::Dynamic, 2> negative_score;
        negative_score.resize(m * n, 2);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                Eigen::Vector3d d1(detect_armors[i].position.x(),detect_armors[i].position.y(),detect_armors[i].position.z());
                Eigen::Vector3d d2(standard_armors[j].position.x(),standard_armors[j].position.y(),standard_armors[j].position.z());
                negative_score(i * n + j, 0) = (d1 - d2).norm();
                negative_score(i * n + j, 1) = abs(_std_radian(detect_armors[i].rpy.z() - standard_armors[j].rpy.z()));
            }
        }

        // 数据标准化
        Eigen::Matrix<double, Eigen::Dynamic, 2> regular_score;
        regular_score.resize(m * n, 2);
        for (int i = 0; i < regular_score.rows(); i++)
        {
            regular_score(i, 0) = (negative_score.col(0).maxCoeff() - negative_score(i, 0)) / (negative_score.col(0).maxCoeff() - negative_score.col(0).minCoeff());
            regular_score(i, 1) = (negative_score.col(1).maxCoeff() - negative_score(i, 1)) / (negative_score.col(1).maxCoeff() - negative_score.col(1).minCoeff());
        }

        // 计算样本值占指标的比重
        Eigen::Matrix<double, Eigen::Dynamic, 2> score_weight;
        score_weight.resize(m * n, 2);
        Eigen::VectorXd col_sum = regular_score.colwise().sum();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            score_weight(i, 0) = regular_score(i, 0) / col_sum(0);
            score_weight(i, 1) = regular_score(i, 1) / col_sum(1);
        }

        // 计算每项指标的熵值
        Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            if (score_weight(i, 0) != 0)
                entropy(0) -= score_weight(i, 0) * log(score_weight(i, 0));
            if (score_weight(i, 1) != 0)
                entropy(1) -= score_weight(i, 1) * log(score_weight(i, 1));
        }
        entropy /= log(score_weight.rows());

        // 计算权重
        Eigen::Vector2d weight = (Eigen::Vector2d::Ones() - entropy) / (2 - entropy.sum());

        // 计算匹配得分矩阵(综合评分)
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> score;
        score.resize(m, n);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i < detect_armors.size() && j < standard_armors.size())
                {
                    score(i, j) = negative_score.row(i * standard_armors.size() + j) * weight;
                }
            }
        }
        return score;
    }


    /**
    * @brief 执行装甲板匹配的核心函数
    * @param matrix 得分矩阵（rows:检测到的装甲板, cols:目标装甲板）
    * @param score_max 允许的最大匹配得分阈值
    * @param m 目标装甲板数量
    * @return std::map<int, int> 匹配结果（key:检测装甲板索引, value:目标装甲板索引）
    */
    std::map<int, int> getMatch(Eigen::MatrixXd matrix, double score_max, int m)
    {
        //初始化最终结果
        this->row_col.clear();
        for (int i = 0; i < matrix.rows(); i++) {
            this->row_col[i] = -1;
        }

        //初始化min
        for (int i = 0; i < matrix.rows() && i < m; i++)
        {
            this->min += matrix(i, i);//初始化最小总得分
            this->row_col[i] = i;
        }

        //初始化行数列
        this->row.clear();
        for (int i = 0; i < matrix.rows(); i++)
        {
            // 有多少行，行数列就有多少个数
            this->row.emplace_back(i);
        }

        /// 初始化列数列，n块装甲板，因此列为n
        this->col.clear();
        for (int i = 0; i < m; i++)
        {
            this->col.emplace_back(i);
        }

        this->tmp_v.clear();
        this->result.clear();
        this->nAfour.clear();
        this->fourAfour.clear();

        //计算C(n,m)
        if (matrix.rows() <= m)
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, matrix.rows());
        else
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, m);

        //使用上一步的结果计算A(m,m),最终得到A(n,m)
        for (auto it = this->result.begin(); it != this->result.end(); it++)
        {
            do{
                this->nAfour.emplace_back(*it);
            } while (next_permutation(it->begin(), it->end()));
        }

        //计算A(m,m)
        do{
            fourAfour.push_back(col);
        } while (next_permutation(col.begin(), col.end())); // stl自带全排列函数
        
    
        //使用A(n,m)和A(m,m)求出可能选择的所有行列的组合的结果
        for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
        {
            for (auto it2 = this->fourAfour.begin(); it2 != this->fourAfour.end(); it2++){
                this->tmp = 0;
                for (int i = 0; i < it->size(); i++)
                    this->tmp += matrix((*it)[i], (*it2)[i]);
                if (this->tmp < this->min){
                    this->min = this->tmp;
                    this->row_col.clear();
                    for (int i = 0; i < it->size(); i++){
                        this->row_col[it->at(i)] = it2->at(i);
                    }
                }
            }
        }

        //删除-1的键值对
        for (auto it = this->row_col.begin(); it != this->row_col.end();){
            if (it->second == -1 || (it->second != -1 && matrix(it->first, it->second) > score_max)){
                it = this->row_col.erase(it);
            }
            else
                ++it;
        }

        return this->row_col;
    }

    // 辅助函数：递归生成组合
    void getCombinationsNumbers(const std::vector<int>& input, std::vector<int>& tmp_v, std::vector<std::vector<int>>& result, int start, int k)
    {
        for (int i = start; i < input.size(); ++i) {
            tmp_v.emplace_back(input[i]);
            if (tmp_v.size() == k) {
                result.emplace_back(tmp_v);
                tmp_v.pop_back();
                continue;
            }
            else {
                getCombinationsNumbers(input, tmp_v, result, i + 1, k);
            }
            tmp_v.pop_back();
        }
    }

    /**
     * @brief 根据目标类型切换运动模型和观测器，并调整跟踪参数
     * @param type 目标运动模型类型（STANDARD/BALANCE/OUTPOST）
     */
    void setModelType(KinematicModel::Type type)
    {
        // 设置目标模型类型
        targetModel->m_model_type = type;

        /*2.运动模型与观测器初始化*/
        if (type == KinematicModel::STANDARD)
        {
            targetModel->m_status = std::make_shared<StandardModel>();   //创建标准模型
            targetModel->ekf = std::make_shared<StandardObserver>(config_path_);
        }
        else if (type == KinematicModel::BALANCE)
        {
            targetModel->m_status = std::make_shared<BalanceModel>();    //创建平衡步兵模型
            targetModel->ekf = std::make_shared<BalanceObserver>(config_path_);
        }
        else if (type == KinematicModel::OUTPOST)
        {
            targetModel->m_status = std::make_shared<OutpostModel>();    //创建哨兵模型
            targetModel->ekf = std::make_shared<OutpostObserver>(config_path_);
        }
    }
};
