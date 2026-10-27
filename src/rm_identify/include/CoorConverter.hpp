#ifndef COORDINATE_TRANSFORMER_HPP
#define COORDINATE_TRANSFORMER_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>

using namespace cv;

struct RT{
    // 世界系->陀螺仪
    Eigen::Matrix3d imu2Map_rot_mat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d imu2Map_trans_mat = Eigen::Vector3d::Zero();
    // 陀螺仪 -> 相机
    Eigen::Matrix3d cam2Imu_rot_mat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d cam2Imu_trans_mat = Eigen::Vector3d::Zero();
    // 陀螺仪 -> 枪管
    Eigen::Matrix3d gun2Imu_rot_mat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d gun2Imu_trans_mat = Eigen::Vector3d::Zero();
};

// 坐标变换工具类
class CoordinateTransformer
{
private:
    cv::Mat camera_matrix_; // 相机内参
    cv::Mat dist_coeffs_;   // 畸变系数

    // 旋转矩阵和平移矩阵
    // 世界系 -> 陀螺仪
    public:
    Eigen::Matrix3d imu2Map_rot_mat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d imu2Map_trans_mat = Eigen::Vector3d::Zero();

    // 陀螺仪 -> 相机
    Eigen::Matrix3d cam2Imu_rot_mat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d cam2Imu_trans_mat = Eigen::Vector3d::Zero();

    // 陀螺仪 -> 枪管
    Eigen::Matrix3d gun2Imu_rot_mat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d gun2Imu_trans_mat = Eigen::Vector3d::Zero();


    // 坐标系偏移量
    // 世界系 -> 陀螺仪
    double m_imu2Map_trans_off_x = 0;
    double m_imu2Map_trans_off_y = 0;
    double m_imu2Map_trans_off_z = 0;

    // 陀螺仪 -> 相机
    double m_cam2Imu_rpy_off_roll = 0;
    double m_cam2Imu_rpy_off_pitch = 0;
    double m_cam2Imu_rpy_off_yaw = 0;

    double m_cam2Imu_trans_off_x = 0;
    double m_cam2Imu_trans_off_y = 0;
    double m_cam2Imu_trans_off_z = 0;

    // 陀螺仪 -> 枪管
    double m_gun2Imu_rpy_off_roll = 0;
    double m_gun2Imu_rpy_off_pitch = 0;
    double m_gun2Imu_rpy_off_yaw = 0;

    double m_gun2Imu_trans_off_x = 0;
    double m_gun2Imu_trans_off_y = 0;
    double m_gun2Imu_trans_off_z = 0;

    // 小装甲板尺寸
    const std::vector<cv::Point3d> PW_SMALL = { // 灯条坐标，单位：m
        {-0.066685, 0.02775, 0.},    // 左上0
        {-0.066685, -0.02775, 0.},   // 左下1
        {0.066685, -0.02775, 0.},    // 右下2
        {0.066685, 0.02775, 0.}};    // 右上3

    // 大装甲板尺寸
    const std::vector<cv::Point3d> PW_BIG = { // 灯条坐标，单位：m
        {-0.11644, 0.02775, 0.},
        {-0.11644, -0.02775, 0.},
        {0.11644, -0.02775, 0.},
        {0.11644, 0.02775, 0.}};

public: 
    RT rt;

public:
    CoordinateTransformer();

    CoordinateTransformer(const std::string &file_path);

    ~CoordinateTransformer() = default;

    // 参数配置
    void setparam(const std::string &file_path);

    // 绕x轴旋转
    Eigen::Matrix3d GetRotMatX(float roll) const;
    // 绕y轴旋转
    Eigen::Matrix3d GetRotMatY(float pitch) const;
    // 绕z轴旋转
    Eigen::Matrix3d GetRotMatZ(float yaw) const;
    // 欧拉角转旋转矩阵
    Eigen::Matrix3d GetRotMatXYZ(float roll, float pitch, float yaw) const;

    // 实时更新IMU旋转矩阵 (接收串口后更新)
    void tfupdate_imu(Eigen::Vector3d cur_pose);

    // 坐标变换
    // map世界系 ->  imu陀螺仪系
    Eigen::Vector3d map2Imu(const Eigen::Vector3d &point_Map) const;

    // imu陀螺仪系 ->  cam相机系
    Eigen::Vector3d imu2Cam(const Eigen::Vector3d &point_Imu) const;

    // imu陀螺仪系 ->  gun枪管系
    Eigen::Vector3d imu2Gun(const Eigen::Vector3d &point_Map) const;

    // map世界系 -> gun枪管系(map世界系 -> imu陀螺仪系 -> gun枪管系)
    Eigen::Vector3d map2Gun(const Eigen::Vector3d &point_Map) const;

    // map世界系 -> cam相机系(map世界系 -> imu陀螺仪系 -> cam相机系)
    Eigen::Vector3d map2Cam(const Eigen::Vector3d &point_Map) const;

    // 坐标逆变换
    // imu相机系 -> map世界系
    Eigen::Vector3d imu2Map(const Eigen::Vector3d &point_Imu) const;

    // cam相机系 -> imu陀螺仪系
    Eigen::Vector3d cam2Imu(const Eigen::Vector3d &point_Cam) const;

    // gun枪管系 -> imu陀螺仪系
    Eigen::Vector3d gun2Imu(const Eigen::Vector3d &point_Gun) const;

    // gun枪管系 -> map世界系(gun枪管系 -> imu陀螺仪系 -> map世界系)
    Eigen::Vector3d gun2Map(const Eigen::Vector3d &point_Gun) const;

    // cam相机系 -> map世界系(cam相机系 -> imu陀螺仪系 -> map世界系)
    Eigen::Vector3d cam2Map(const Eigen::Vector3d &point_Cam) const;

    // armor装甲板系 ->  corner角点系
    std::vector<Eigen::Vector3d> armor2Corner(
        const Eigen::Vector3d &pos, // 装甲板中心位置
        char ArmorID,               // 装甲板ID
        const double &pitch,        // 俯仰角
        const double &yaw) const;

    Eigen::Vector2d get_camera_z_i2() const;

    // 获取相机Z轴在惯性坐标系中的方向向量
    Eigen::Vector3d get_camera_z_i() const;

    /*角度变换*/
    // map世界系 -> imu陀螺仪系
    Eigen::Vector3d Angle_map2Imu(const Eigen::Vector3d &rpy_Map) const;

    // imu陀螺仪系 -> cam相机系
    Eigen::Vector3d Angle_imu2Cam(const Eigen::Vector3d &rpy_Imu) const;

    // imu陀螺仪系 -> gun枪管系
    Eigen::Vector3d Angle_imu2Gun(const Eigen::Vector3d &rpy_Imu) const;

    // map世界系 -> cam相机系
    Eigen::Vector3d Angle_map2Cam(const Eigen::Vector3d &rpy_Map) const;

    // map世界系 -> gun枪管系
    Eigen::Vector3d Angle_map2Gun(const Eigen::Vector3d &rpy_Map) const;

    /*角度逆变换*/
    // imu陀螺仪系 -> map世界系
    Eigen::Vector3d Angle_imu2Map(const Eigen::Vector3d &rpy_Imu) const;

    // cam相机系 -> imu陀螺仪系
    Eigen::Vector3d Angle_cam2Imu(const Eigen::Vector3d &rpy_Cam) const;

    // gun枪管系 -> imu陀螺仪系
    Eigen::Vector3d Angle_gun2Imu(const Eigen::Vector3d &rpy_Gun) const;

    // cam枪管系 -> map世界系
    Eigen::Vector3d Angle_cam2Map(const Eigen::Vector3d &rpy_Cam) const;

    // gun枪管系 -> map世界系
    Eigen::Vector3d Angle_gun2Map(const Eigen::Vector3d &rpy_Gun) const;

    // map世界系 -> pix图像系
    cv::Point2d map2pixel(const Eigen::Vector3d &pi) const;

    // cam相机系 -> pix图像系
    cv::Point2d cam2pixel(const Eigen::Vector3d &pc) const;

    // 欧拉角 -> 四元数
    Eigen::Vector4d Euler2QUaternion(double roll, double pitch, double yaw);

    // 四元数 -> 欧拉角
    Eigen::Vector3d QUaternion2Euler(const geometry_msgs::Quaternion& qtn);

    // Eigen向量 -> Point点
    cv::Point3d Vector2Point(Eigen::Vector3d src);

    // Point点 -> Eigen向量
    Eigen::Vector3d Point2Vector(Point3d src);

    // Eigen向量 -> Opencv矩阵
    cv::Mat VectorToMatrixPos(Eigen::Vector3d src);

    // Opencv矩阵 -> Eigen向量
    Eigen::Vector3d MatrixPosToVector(cv::Mat tvec);

    // 从旋转矩阵R提取欧拉角
    Eigen::Vector3d Rvec2Euler(cv::Mat rvec);

    // 提供外部访问旋转矩阵和平移矩阵的接口
    RT get_RT() const;

    Eigen::Vector3d RotationMatrix2Euler(const Eigen::Matrix3d& R) const;

};

#endif