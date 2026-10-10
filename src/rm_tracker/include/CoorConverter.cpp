#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include "math.hpp"
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include "CoorConverter.hpp"

using namespace std;
using namespace cv;

// 提示: 使用默认构造函数记得修改默认构造函数里面的参数
// 默认构造函数(不使用,提供接口i)
CoordinateTransformer::CoordinateTransformer()
{
    // 麦轮
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 1724.18353097744, -0.000579628569003521  , 680.697138243768, 0.0,  1726.83294106563,  504.398280989468, 0.0, 0.0, 1.0);
    dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.0834722705527182, 0.496852388408421    , -0.000457625756594158  ,  0.00246230944307382, -0.0130632578222697);

    // 陀螺仪 -> 世界
    m_imu2Map_trans_off_x = 0;
    m_imu2Map_trans_off_y = 0;
    m_imu2Map_trans_off_z = 0.055;

    // 相机 -> 陀螺仪
    m_cam2Imu_rpy_off_roll = -1.57079632679489661923;
    m_cam2Imu_rpy_off_pitch = 0;
    m_cam2Imu_rpy_off_yaw = -1.57079632679489661923;

    m_cam2Imu_trans_off_x = 0.135;
    m_cam2Imu_trans_off_y = 0;
    m_cam2Imu_trans_off_z = -0.012;

    // 枪管 -> 陀螺仪
    m_gun2Imu_rpy_off_roll = 0;
    m_gun2Imu_rpy_off_pitch = 0;
    m_gun2Imu_rpy_off_yaw = 0;

    m_gun2Imu_trans_off_x = 0.14524;
    m_gun2Imu_trans_off_y = 0;
    m_gun2Imu_trans_off_z = -0.055;

    /*
        这些旋转矩阵和平移矩阵是固定的,在初始化的时候就计算好,避免后续重复计算
        map世界系到imu陀螺仪系的变换是随着云台的运动不断变换的,所以map世界系到imu
        陀螺仪的旋转矩阵,是随着时间变化随着当前云台位置决定的
    */
    // 世界系 -> 陀螺仪
    imu2Map_trans_mat = {m_imu2Map_trans_off_x, m_imu2Map_trans_off_y, m_imu2Map_trans_off_z};

    // 相机 -> 陀螺仪
    cam2Imu_rot_mat = GetRotMatXYZ(m_cam2Imu_rpy_off_roll, m_cam2Imu_rpy_off_pitch, m_cam2Imu_rpy_off_yaw);
    cam2Imu_trans_mat = {m_cam2Imu_trans_off_x, m_cam2Imu_trans_off_y, m_cam2Imu_trans_off_z};

    // 枪管 -> 陀螺仪
    gun2Imu_rot_mat = GetRotMatXYZ(m_gun2Imu_rpy_off_roll, m_gun2Imu_rpy_off_pitch, m_gun2Imu_rpy_off_yaw);
    gun2Imu_trans_mat = {m_gun2Imu_trans_off_x, m_gun2Imu_trans_off_y, m_gun2Imu_trans_off_z};

    // 陀螺仪 -> 世界
    imu2Map_rot_mat = Eigen::Matrix3d::Identity();

    rt.imu2Map_trans_mat = imu2Map_trans_mat;    // 陀螺仪系 -> 世界系   平移矩阵
    rt.cam2Imu_rot_mat = cam2Imu_rot_mat;        // 相机系   -> 陀螺仪系 旋转矩阵
    rt.cam2Imu_trans_mat = cam2Imu_trans_mat;    // 相机系   -> 陀螺仪系 平移矩阵
    rt.gun2Imu_rot_mat = gun2Imu_rot_mat;        // 枪管系   -> 陀螺仪系 旋转矩阵
    rt.gun2Imu_trans_mat = gun2Imu_trans_mat;    // 枪管系   -> 陀螺仪系 平移矩阵
    rt.imu2Map_rot_mat = imu2Map_rot_mat;        // 陀螺仪系 -> 世界系   旋转矩阵
}

// 考虑用参数配置函数,在初始化确定时确定坐标变换
CoordinateTransformer::CoordinateTransformer(const std::string &file_path)
{
    // 参数配置
    if (!setparam(file_path)) {
        throw std::runtime_error("failed to load CoordinateTransformer configuration");
    }

    /*
        这些旋转矩阵和平移矩阵是固定的,在初始化的时候就计算好,避免后续重复计算
        map世界系到imu陀螺仪系的变换是随着云台的运动不断变换的,所以map世界系到imu
        陀螺仪的旋转矩阵,是随着时间变化随着当前云台位置决定的
    */
    // 陀螺仪 -> 世界
    imu2Map_trans_mat = {m_imu2Map_trans_off_x, m_imu2Map_trans_off_y, m_imu2Map_trans_off_z};

    // 相机 -> 陀螺仪
    cam2Imu_rot_mat = GetRotMatXYZ(m_cam2Imu_rpy_off_roll, m_cam2Imu_rpy_off_pitch, m_cam2Imu_rpy_off_yaw);
    cam2Imu_trans_mat = {m_cam2Imu_trans_off_x, m_cam2Imu_trans_off_y, m_cam2Imu_trans_off_z};

    // 枪管 -> 陀螺仪
    gun2Imu_rot_mat = GetRotMatXYZ(m_gun2Imu_rpy_off_roll, m_gun2Imu_rpy_off_pitch, m_gun2Imu_rpy_off_yaw);
    gun2Imu_trans_mat = {m_gun2Imu_trans_off_x, m_gun2Imu_trans_off_y, m_gun2Imu_trans_off_z};

    // 陀螺仪 -> 世界
    imu2Map_rot_mat = Eigen::Matrix3d::Identity();
}

// 参数配置
bool CoordinateTransformer::setparam(const std::string &file_path)
{
    try
    {
        // 1. 增强文件存在性检查
        if (file_path.empty())
        {
            ROS_ERROR("Empty config file path provided");
            return false;
        }

        // 2. 检查文件是否打开
        std::ifstream file_check(file_path);
        if (!file_check.is_open())
        {
            ROS_ERROR("Failed to open config file: %s. Reason: %s",
                      file_path.c_str(), strerror(errno));
            return false;
        }
        ROS_INFO("Loading parameters from: %s", file_path.c_str());

        // 2. 加载 YAML 文件并添加详细错误处理
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(file_path);
        }
        catch (const YAML::BadFile &e)
        {
            ROS_ERROR("Bad YAML file: %s", e.what());
            return false;
        }
        catch (const YAML::ParserException &e)
        {
            ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
            return false;
        }

        // 加载坐标变换参数
        if (config["CoordinateTransformer"] && config["CoordinateTransformer"]["off"])
        {
            YAML::Node mod = config["CoordinateTransformer"];
            YAML::Node offsets = mod["off"];
            auto require_double = [&offsets](const char* key) -> double {
                if (!offsets[key]) {
                    throw std::runtime_error(std::string("Missing CoordinateTransformer.off.") + key);
                }
                return offsets[key].as<double>();
            };

            m_imu2Map_trans_off_x = require_double("imu2Map_trans_off_x");
            m_imu2Map_trans_off_y = require_double("imu2Map_trans_off_y");
            m_imu2Map_trans_off_z = require_double("imu2Map_trans_off_z");

            m_cam2Imu_rpy_off_roll = require_double("cam2Imu_rpy_off_roll");
            m_cam2Imu_rpy_off_pitch = require_double("cam2Imu_rpy_off_pitch");
            m_cam2Imu_rpy_off_yaw = require_double("cam2Imu_rpy_off_yaw");

            m_cam2Imu_trans_off_x = require_double("cam2Imu_trans_off_x");
            m_cam2Imu_trans_off_y = require_double("cam2Imu_trans_off_y");
            m_cam2Imu_trans_off_z = require_double("cam2Imu_trans_off_z");

            m_gun2Imu_rpy_off_roll = require_double("gun2Imu_rpy_off_roll");
            m_gun2Imu_rpy_off_pitch = require_double("gun2Imu_rpy_off_pitch");
            m_gun2Imu_rpy_off_yaw = require_double("gun2Imu_rpy_off_yaw");

            m_gun2Imu_trans_off_x = require_double("gun2Imu_trans_off_x");
            m_gun2Imu_trans_off_y = require_double("gun2Imu_trans_off_y");
            m_gun2Imu_trans_off_z = require_double("gun2Imu_trans_off_z");
            // ROS_INFO("CoordinateTransformer parameters loaded:");
            // ROS_INFO("m_imu2Map_trans_off_x:%.3f",m_imu2Map_trans_off_x);
            // ROS_INFO("m_imu2Map_trans_off_y:%.3f",m_imu2Map_trans_off_y);
            // ROS_INFO("m_imu2Map_trans_off_z:%.3f",m_imu2Map_trans_off_z);

            // ROS_INFO("m_cam2Imu_rpy_off_roll:%.3f",m_cam2Imu_rpy_off_roll);
            // ROS_INFO("m_cam2Imu_rpy_off_pitch:%.3f",m_cam2Imu_rpy_off_pitch);
            // ROS_INFO("m_cam2Imu_rpy_off_yaw:%.3f",m_cam2Imu_rpy_off_yaw);

            // ROS_INFO("m_cam2Imu_trans_off_x:%.3f",m_cam2Imu_trans_off_x);
            // ROS_INFO("m_cam2Imu_trans_off_y:%.3f",m_cam2Imu_trans_off_y);
            // ROS_INFO("m_cam2Imu_trans_off_z:%.3f",m_cam2Imu_trans_off_z);

            // ROS_INFO("m_gun2Imu_rpy_off_roll:%.3f",m_gun2Imu_rpy_off_roll);
            // ROS_INFO("m_gun2Imu_rpy_off_pitch:%.3f",m_gun2Imu_rpy_off_pitch);
            // ROS_INFO("m_gun2Imu_rpy_off_yaw:%.3f",m_gun2Imu_rpy_off_yaw);

            // ROS_INFO("m_gun2Imu_trans_off_x:%.3f",m_gun2Imu_trans_off_x);
            // ROS_INFO("m_gun2Imu_trans_off_y:%.3f",m_gun2Imu_trans_off_y);
            // ROS_INFO("m_gun2Imu_trans_off_z:%.3f",m_gun2Imu_trans_off_z);
            // ROS_INFO("CoordinateTransformer parameters loaded successfully");
        }
        else
        {
            ROS_ERROR("Missing 'CoordinateTransformer.off' section in config file");
            return false;
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
        }
        return true;
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("YAML parsing error: %s", e.what());
        return false;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Error loading parameters: %s", e.what());
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown error occurred while loading config file");
        return false;
    }
}

// 绕x轴旋转
Eigen::Matrix3d CoordinateTransformer::GetRotMatX(float roll) const
{
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
        0, std::cos(roll), -std::sin(roll),
        0, std::sin(roll), std::cos(roll);
    return Rx;
}

// 绕y轴旋转
Eigen::Matrix3d CoordinateTransformer::GetRotMatY(float pitch) const
{
    Eigen::Matrix3d Ry;
    Ry << std::cos(pitch), 0, std::sin(pitch),
        0, 1, 0,
        -std::sin(pitch), 0, std::cos(pitch);
    return Ry;
}

// 绕z轴旋转
Eigen::Matrix3d CoordinateTransformer::GetRotMatZ(float yaw) const
{
    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0,
        std::sin(yaw), std::cos(yaw), 0,
        0, 0, 1;
    return Rz;
}

// 欧拉角转旋转矩阵
Eigen::Matrix3d CoordinateTransformer::GetRotMatXYZ(float roll, float pitch, float yaw) const
{
    return GetRotMatZ(yaw) * GetRotMatY(pitch) * GetRotMatX(roll);
}

// 坐标变换
// map世界系 -> imu陀螺仪系
Eigen::Vector3d CoordinateTransformer::map2Imu(const Eigen::Vector3d &point_Map) const
{
    return imu2Map_rot_mat.transpose() * (point_Map - imu2Map_trans_mat);
}

// imu陀螺仪系 -> cam相机系
Eigen::Vector3d CoordinateTransformer::imu2Cam(const Eigen::Vector3d &point_Imu) const
{
    return cam2Imu_rot_mat.transpose() * (point_Imu - cam2Imu_trans_mat);
}

// imu陀螺仪系 -> gun枪管系
Eigen::Vector3d CoordinateTransformer::imu2Gun(const Eigen::Vector3d &point_Imu) const
{
    return gun2Imu_rot_mat.transpose() * (point_Imu - gun2Imu_trans_mat);
}

// map世界系 -> gun枪管系(map世界系 -> imu陀螺仪系 -> gun枪管系)
Eigen::Vector3d CoordinateTransformer::map2Gun(const Eigen::Vector3d &point_Map) const
{
    return imu2Gun(map2Imu(point_Map));
}

// map世界系 -> cam相机系(map世界系 -> imu陀螺仪系 -> cam相机系)
Eigen::Vector3d CoordinateTransformer::map2Cam(const Eigen::Vector3d &point_Map) const
{
    return imu2Cam(map2Imu(point_Map));
}

// 坐标逆变换
// imu相机系 -> map世界系
Eigen::Vector3d CoordinateTransformer::imu2Map(const Eigen::Vector3d &point_Imu) const
{
    return imu2Map_rot_mat * point_Imu + imu2Map_trans_mat;
}

// cam相机系 -> imu陀螺仪系
Eigen::Vector3d CoordinateTransformer::cam2Imu(const Eigen::Vector3d &point_Cam) const
{
    return cam2Imu_rot_mat * point_Cam + cam2Imu_trans_mat;
}

// gun枪管系 -> imu陀螺仪系
Eigen::Vector3d CoordinateTransformer::gun2Imu(const Eigen::Vector3d &point_Gun) const
{
    return gun2Imu_rot_mat * point_Gun + gun2Imu_trans_mat;
}

// gun枪管系 -> map世界系(gun枪管系 -> imu陀螺仪系 -> map世界系)
Eigen::Vector3d CoordinateTransformer::gun2Map(const Eigen::Vector3d &point_Gun) const
{
    return imu2Map(gun2Imu(point_Gun));
}

//cam相机系 -> map世界系(cam相机系 -> imu陀螺仪系 -> map世界系)
Eigen::Vector3d CoordinateTransformer::cam2Map(const Eigen::Vector3d &point_Cam) const
{
    return imu2Map(cam2Imu(point_Cam));
}

// gun枪管系 -> cam相机系(gun枪管系 -> imu陀螺仪系 -> cam相机系)
Eigen::Vector3d CoordinateTransformer::gun2Cam(const Eigen::Vector3d &point_Gun) const
{
    return imu2Cam(gun2Imu(point_Gun));
}

//cam相机系 -> gun枪管系(cam相机系 -> imu陀螺仪系 -> gun枪管系)
Eigen::Vector3d CoordinateTransformer::cam2Gun(const Eigen::Vector3d &point_Cam) const
{
    return imu2Gun(cam2Imu(point_Cam));
}

// armor装甲板map系 ->  corner角点系
std::vector<Eigen::Vector3d> CoordinateTransformer::armor2Corner(
    const Eigen::Vector3d &point_Armor, // 装甲板中心位置
    char ArmorID,                       // 装甲板ID
    const double &pitch,                // 俯仰角
    const double &yaw                   // 相机系下的yaw角
) const
{
    // 确定装甲板类型
    char armortype = 's';
    if (ArmorID == 1)
    {
        armortype = 'l';
    }
    else
    {
        armortype = 's';
    }

    // 选择装甲板尺寸模板
    const std::vector<cv::Point3d> &pw = armortype == 'l' ? this->PW_BIG : this->PW_SMALL;

    // 计算半径方向向量(中心到装甲板中心的半径方向)
    Eigen::Vector2d radius_norm = math::rotate(get_camera_z_i2(), yaw);

    // 计算X轴方向
    Eigen::Vector3d x_norm;
    x_norm << math::rotate(radius_norm, M_PI / 2.), 0.;

    // 定义世界Z轴
    Eigen::Vector3d w_z_norm{0., 0., 1.};

    // 计算Y轴方向
    Eigen::Vector3d y_norm;
    y_norm << -radius_norm * std::sin(pitch), 0.;
    y_norm += w_z_norm * std::cos(pitch);

    // 计算4个角点坐标
    std::vector<Eigen::Vector3d> corners;
    for (int i = 0; i < 4; ++i)
    {
        corners.push_back(point_Armor + x_norm * pw[i].x + y_norm * pw[i].y);
    }
    return corners;
}

// 实时更新世界系 -> 陀螺仪的旋转矩阵
void CoordinateTransformer::tfupdate_imu(Eigen::Vector3d cur_pose) {
    imu2Map_rot_mat = GetRotMatXYZ(cur_pose.x(), cur_pose.y(), cur_pose.z());
}

// 获取相机Z轴在惯性坐标系中XY平面的投影方向(二维单位向量)
Eigen::Vector2d CoordinateTransformer::get_camera_z_i2() const {
    // 1.获取相机Z轴在惯性坐标系中的方向
    Eigen::Vector3d camera_z_i = this->get_camera_z_i();

    // 2.提取XY平面分量
    Eigen::Vector2d camera_z_i2{camera_z_i(0, 0), camera_z_i(1, 0)};

    // 3.归一化处理
    camera_z_i2.normalize();
    return camera_z_i2;
}

// 获取相机Z轴在惯性坐标系中的方向向量
Eigen::Vector3d CoordinateTransformer::get_camera_z_i() const {
    return this->cam2Map(Eigen::Vector3d(0., 0., 1.)).normalized();
}

/*角度变换*/
// map世界系 -> imu陀螺仪系
Eigen::Vector3d CoordinateTransformer::Angle_map2Imu(const Eigen::Vector3d &rpy_Map) const {
    Eigen::Matrix3d R_map_object = GetRotMatXYZ(rpy_Map(0), rpy_Map(1), rpy_Map(2));
    Eigen::Matrix3d R_imu_object = imu2Map_rot_mat.transpose() * R_map_object;
    return RotationMatrix2Euler(R_imu_object);
}

// imu陀螺仪系 -> cam相机系
Eigen::Vector3d CoordinateTransformer::Angle_imu2Cam(const Eigen::Vector3d &rpy_Imu) const {
    Eigen::Matrix3d R_imu_object = GetRotMatXYZ(rpy_Imu(0), rpy_Imu(1), rpy_Imu(2));
    Eigen::Matrix3d R_cam_object = cam2Imu_rot_mat.transpose() * R_imu_object;
    return RotationMatrix2Euler(R_cam_object);
}

// imu陀螺仪系 -> gun枪管系
Eigen::Vector3d CoordinateTransformer::Angle_imu2Gun(const Eigen::Vector3d &rpy_Imu) const {
    Eigen::Matrix3d R_imu_object = GetRotMatXYZ(rpy_Imu(0), rpy_Imu(1), rpy_Imu(2));
    Eigen::Matrix3d R_gun_object = gun2Imu_rot_mat.transpose() * R_imu_object;
    return RotationMatrix2Euler(R_gun_object);
}

// map世界系 -> cam相机系
Eigen::Vector3d CoordinateTransformer::Angle_map2Cam(const Eigen::Vector3d &rpy_Map) const {
    return Angle_imu2Cam(Angle_map2Imu(rpy_Map));
}

// map世界系 -> gun枪管系
Eigen::Vector3d CoordinateTransformer::Angle_map2Gun(const Eigen::Vector3d &rpy_Map) const {
    return Angle_imu2Gun(Angle_map2Imu(rpy_Map));
}

/*角度逆变换*/
// imu陀螺仪系 -> map世界系
Eigen::Vector3d CoordinateTransformer::Angle_imu2Map(const Eigen::Vector3d &rpy_Imu) const{
    Eigen::Matrix3d R_imu_object = GetRotMatXYZ(rpy_Imu(0), rpy_Imu(1), rpy_Imu(2));
    Eigen::Matrix3d R_map_object = imu2Map_rot_mat * R_imu_object;
    return RotationMatrix2Euler(R_map_object);
}

// cam相机系 -> imu陀螺仪系
Eigen::Vector3d CoordinateTransformer::Angle_cam2Imu(const Eigen::Vector3d &rpy_Cam) const{
    Eigen::Matrix3d R_cam_object = GetRotMatXYZ(rpy_Cam(0), rpy_Cam(1), rpy_Cam(2));
    Eigen::Matrix3d R_Imu_object = cam2Imu_rot_mat * R_cam_object;
    return RotationMatrix2Euler(R_Imu_object);
}

// gun枪管系 -> imu陀螺仪系
Eigen::Vector3d CoordinateTransformer::Angle_gun2Imu(const Eigen::Vector3d &rpy_Gun) const {
    Eigen::Matrix3d R_gun_object = GetRotMatXYZ(rpy_Gun(0), rpy_Gun(1), rpy_Gun(2));
    Eigen::Matrix3d R_Imu_object = gun2Imu_rot_mat * R_gun_object;
    return RotationMatrix2Euler(R_Imu_object);
}

// cam相机系 -> map世界系(cam相机系 -> imu陀螺仪系 -> map世界系)
Eigen::Vector3d CoordinateTransformer::Angle_cam2Map(const Eigen::Vector3d &rpy_Cam) const{
    return Angle_imu2Map(Angle_cam2Imu(rpy_Cam));
}

// gun枪管系 -> map世界系(gun枪管系 -> imu陀螺仪系 -> map世界系)
Eigen::Vector3d CoordinateTransformer::Angle_gun2Map(const Eigen::Vector3d &rpy_Gun) const{
    return Angle_imu2Map(Angle_gun2Imu(rpy_Gun));
}

// 世界坐标系 -> 相机坐标系 -> 图像坐标系
cv::Point2d CoordinateTransformer::map2pixel(const Eigen::Vector3d &pi) const {
    return this->cam2pixel(this->map2Cam(pi));
}

// 相机坐标系 -> 图像坐标系
cv::Point2d CoordinateTransformer::cam2pixel(const Eigen::Vector3d &pc) const {
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f); // 旋转向量
    cv::Mat tvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f); // 平移向量
    std::vector<cv::Point3d> Single_objectPoints;
    Single_objectPoints.push_back(Point3d(pc.x(), pc.y(), pc.z()));
    std::vector<cv::Point2d> imagePoints;
    // 使用OpenCV的projectPoints函数进行重投影
    cv::projectPoints(Single_objectPoints, rvec, tvec, camera_matrix_, dist_coeffs_, imagePoints);
    return imagePoints[0];
}

// 欧拉角转四元数
// 提示: 基本类型直接传值效率比较高
// 不需要改成const引用,double拷贝比引用开销小
Eigen::Vector4d CoordinateTransformer::Euler2QUaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion qtn;
    qtn.setRPY(roll,pitch,yaw);
    return Eigen::Vector4d(qtn.w(),qtn.x(),qtn.y(),qtn.z());
}

// 四元数转欧拉角
Eigen::Vector3d CoordinateTransformer::QUaternion2Euler(const geometry_msgs::Quaternion& qtn) {
    tf2::Quaternion quaternion(qtn.x,qtn.y,qtn.z,qtn.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll,pitch,yaw);
    return {roll,pitch,yaw};
}

// Eigen向量 -> Point点
cv::Point3d CoordinateTransformer::Vector2Point(Eigen::Vector3d src) {
    cv::Point3d result;
    result.x = src.x();
    result.y = src.y();
    result.z = src.z();
    return result;
}

// Point点 -> Eigen向量
Eigen::Vector3d CoordinateTransformer::Point2Vector(cv::Point3d src) {
    Eigen::Vector3d result;
    result << src.x,src.y,src.z;
    return result;
}

// Eigen向量 -> Opencv矩阵
cv::Mat CoordinateTransformer::VectorToMatrixPos(Eigen::Vector3d src){
    //3*1矩阵，存储CV_64F
    cv::Mat result(3,1,CV_64F);
    result.at<double>(0,0) = src.x();
    result.at<double>(1,0) = src.y();
    result.at<double>(2,0) = src.z();
    return result;
}

// Opencv矩阵 -> Eigen向量
Eigen::Vector3d CoordinateTransformer::MatrixPosToVector(cv::Mat tvec){
    Eigen::Vector3d result;
    // 检查矩阵是否为3*1的double类型
    if (tvec.rows != 3 || tvec.cols != 1 || tvec.type() != CV_64F) {
        return Eigen::Vector3d::Zero();
    }
    result << tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0);
    return std::move(result);
}

// 从旋转矩阵R提取欧拉角
Eigen::Vector3d CoordinateTransformer::Rvec2Euler(cv::Mat rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec,R);
    // 先绕z,再绕Y,再绕X
    double theta_x = atan2(R.at<double>(2,1),R.at<double>(2,2));
    double theta_y = atan2(-R.at<double>(2,0),sqrt(pow(R.at<double>(2,1),2) + pow(R.at<double>(2,2),2)));
    double theta_z = atan2(R.at<double>(1,0),R.at<double>(0,0));
    return {theta_x,theta_y,theta_z};
}

// 提供外部访问的公共接口
RT CoordinateTransformer::get_RT() const {
    RT rt;
    rt.imu2Map_rot_mat = imu2Map_rot_mat;
    rt.imu2Map_trans_mat = imu2Map_trans_mat;
    rt.cam2Imu_rot_mat = cam2Imu_rot_mat;
    rt.cam2Imu_trans_mat = cam2Imu_trans_mat;
    rt.gun2Imu_rot_mat = gun2Imu_rot_mat;
    rt.gun2Imu_trans_mat = gun2Imu_trans_mat;
    return rt;
}

// 旋转矩阵转欧拉角（ZYX顺序）
Eigen::Vector3d CoordinateTransformer::RotationMatrix2Euler(const Eigen::Matrix3d& R) const {
    // 检查矩阵是否为旋转矩阵（正交且行列式为1）
    const double epsilon = 1e-6;
    
    // ZYX顺序（先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll）
    // cos(pitch)的绝对值
    // 当sy约等于0时,意味着pitch约等于90度,此时出现万向锁
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0)); 
    
    bool singular = sy < epsilon; // 万向锁检查
    
    double roll, pitch, yaw;
    if (!singular) {
        roll = atan2(R(2,1), R(2,2));
        pitch = atan2(-R(2,0), sy);
        yaw = atan2(R(1,0), R(0,0));
    } else {
        // 万向锁情况（pitch = ±90°）
        roll = atan2(-R(1,2), R(1,1));
        pitch = atan2(-R(2,0), sy);
        yaw = 0;
    }
    
    return Eigen::Vector3d(roll, pitch, yaw);
}
