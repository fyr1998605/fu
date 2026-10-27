#ifndef CALCULATE
#define CALCULATE
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <chrono>
#include "Image.h"
#include "Pid.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "Variable.h"
#include "visual.hpp"

namespace CAL{
    const std::string TempTF = "tempPos";
    //使用前别忘记setpose
    class Calculater{
    private:
    public:
        cv::Mat cameraM;
        Visual::VisualTF vtf;
        ros::NodeHandle nh;
        Calculater(ros::NodeHandle& nh,Image& img);
        Eigen::Vector3d MatrixPosToVector(cv::Mat tvec);
        cv::Mat VectorToMatrixPos(Eigen::Vector3d src);
        Eigen::Vector3d Point2Vector(geometry_msgs::Point src);
        void SetImuPose(Eigen::Vector3d imuPose);
        void SetImuPose(double roll,double pitch,double yaw);
        Eigen::Matrix3d CalMatrix(double roll,double pitch,double yaw);
        cv::Point3d VectorToPoint(Eigen::Vector3d src);
        Eigen::Vector3d CameraToGun(Eigen::Vector3d cpos);
        Eigen::Vector3d CameraToOdom(Eigen::Vector3d cpos);
        Eigen::Vector3d OdomToGun(Eigen::Vector3d opos);
        Eigen::Vector3d OdomToCam(Eigen::Vector3d opos);
        void TargetUpdate(Eigen::Vector3d pos,Eigen::Vector3d pose);
        Eigen::Vector3d GetCompensatedPoint(double compesation,Eigen::Vector3d tvec);
        double GetCompensation(Eigen::Vector3d pos,double vec);
        // 在第39行附近添加
        bool FanPnPSolve(const cv::Point2f& blade_pt, const cv::Point2f& circle_center, Eigen::Vector3d& world_pos);
        vector<Eigen::Vector3d> RebuildArmorInOdom(Eigen::Vector3d pose,Eigen::Vector3d center);
        Eigen::Vector4d Euler2QUaternion(double roll, double pitch, double yaw);
        Eigen::Vector3d CalCenter(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
        double CalPointsDis(std::vector<cv::Point2f> targetPoint, std::vector<cv::Point2f> point);
        cv::Rect Points2Rect(std::vector<cv::Point2f> pts);
        Eigen::Vector3d GetAbsolutePosition(const std::string& name);
        Eigen::Vector3d GetTargetPos(const std::string&frame);
        void TFUpdata(const std::string&frame,const std::string&name,Eigen::Vector3d pos,Eigen::Vector3d pose,int index);
        Eigen::Vector3d GetPos(const std::string&frame,const std::string&name);
        std::vector<Eigen::Vector3d> GetPos(const std::string& frame, const std::string& child,int);
        Eigen::Vector3d Rvec2Euler(cv::Mat rvec);
        Eigen::Vector3d MyQUaternion2Euler(geometry_msgs::Quaternion qtn);
        cv::Point2f ThreeToTwo(Eigen::Vector3d targetInCam);
        float t = 0.5f; // 飞行时间
        float monoDirectionalAirResistanceModel(float s, float v, float angle);
        float pitchTrajectoryCompensation(float s, float z, float v);
        // 新增
        std::vector<Eigen::Vector3d> GetPosAtTime(const std::string& frame, const std::string& name, const ros::Time& time);
    };
    cv::Point2f Calculater::ThreeToTwo(Eigen::Vector3d targetInCam)
    {
        cv::Vec3d tic(targetInCam.x(),targetInCam.y(),targetInCam.z());
        cv::Mat _pixel = this->cameraM * tic;
        cv::Point2f targetInPixel;
        targetInPixel.x = _pixel.at<double>(0,0) / _pixel.at<double>(2,0);
        targetInPixel.y = _pixel.at<double>(1,0) / _pixel.at<double>(2,0);
        return targetInPixel;
    }
    // 四元数转欧拉角
    Eigen::Vector3d Calculater::MyQUaternion2Euler(geometry_msgs::Quaternion qtn)
    {
        tf2::Quaternion quaternion(qtn.x, qtn.y, qtn.z, qtn.w);

        // 将四元数转换为欧拉角
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        return {roll, pitch, yaw};
    }
    Calculater::Calculater(ros::NodeHandle& nh,Image& img):vtf(nh){
        this->nh = nh;
        this->cameraM = img.cameraMatrix;
    }
    void Calculater::SetImuPose(Eigen::Vector3d imuPose){
        // Eigen::Matrix3d rotation_matrix_pitch;
        // Eigen::Matrix3d rotation_matrix_yaw;
        // Eigen::Matrix3d rotation_matrix_roll;
        // rotation_matrix_pitch << std::cos(imuPose.y()), 0 ,std::sin(imuPose.y()),
        //                                     0,1,0,
        //                                     -std::sin(imuPose.y()),0,std::cos(imuPose.y());
        // rotation_matrix_yaw << std::cos(imuPose.z()), -std::sin(imuPose.z()), 0,
        //                         std::sin(imuPose.z()), std::cos(imuPose.z()), 0,
        //                         0, 0, 1;

        // rotation_matrix_roll << 1, 0, 0,
        //                     0, std::cos(imuPose.x()), -std::sin(imuPose.x()),
        //                     0, std::sin(imuPose.x()), std::cos(imuPose.x());
        // this->R_odom = rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll;
        vtf.TfUpdate({imuPose.x(),imuPose.y(),imuPose.z()});
    }

    
    void Calculater::SetImuPose(double roll,double pitch,double yaw){
        Eigen::Vector3d temp;
        temp << roll,pitch,yaw;
        this->SetImuPose(temp);
    }

    Eigen::Matrix3d Calculater::CalMatrix(double roll,double pitch,double yaw){
        Eigen::Matrix3d rotation_matrix_pitch;
        Eigen::Matrix3d rotation_matrix_yaw;
        Eigen::Matrix3d rotation_matrix_roll;
        rotation_matrix_pitch << std::cos(pitch), 0 ,std::sin(pitch),
                                            0,1,0,
                                            -std::sin(pitch),0,std::cos(pitch);
        rotation_matrix_yaw << std::cos(yaw), -std::sin(yaw), 0,
                                std::sin(yaw), std::cos(yaw), 0,
                                0, 0, 1;

        rotation_matrix_roll << 1, 0, 0,
                            0, std::cos(roll), -std::sin(roll),
                            0, std::sin(roll), std::cos(roll);
        return rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll;
    }
    
    cv::Point3d Calculater::VectorToPoint(Eigen::Vector3d src){
        cv::Point3d result;
        result.x = src.x();
        result.y = src.y();
        result.z = src.z();
        return std::move(result);
    }
    cv::Mat Calculater::VectorToMatrixPos(Eigen::Vector3d src){
        //3*1矩阵，存储CV_64F
        cv::Mat result(3,1,6);
        result.at<double>(0,0) = src.x();
        result.at<double>(1,0) = src.y();
        result.at<double>(2,0) = src.z();
        return std::move(result);
    }
    Eigen::Vector3d Calculater::Point2Vector(geometry_msgs::Point src){
        Eigen::Vector3d result;
        result << src.x,src.y,src.z;
        return std::move(result);
    }
    Eigen::Vector3d Calculater::MatrixPosToVector(cv::Mat tvec){
        Eigen::Vector3d result;
        result << tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0);
        return std::move(result);
    }
    //c -> g
    //in:mm out:mm
    Eigen::Vector3d Calculater::CameraToGun(Eigen::Vector3d cpos){
        return {0,0,0};
        // return cpos - T_Camera_Gun;
        // vtf.TargetUpdate(cpos);
        vtf._Update("cam",cpos,TempTF);
        return vtf.GetChildInFrameData("gun",TempTF);
    }
    //c -> g -> o
    //in:mm out:mm
    Eigen::Vector3d Calculater::CameraToOdom(Eigen::Vector3d cpos){
        return {0,0,0};
        // Eigen::Vector3d result;
        // result = this->R_odom * (this->R_cap2odom * CameraToGun(cpos) + this->T_Odom_Gun);
        // // cout << "guanxi:" << endl << (this->R_cap2odom * CameraToGun(cpos) + this->T_Odom_Gun) << endl;
        // // cout << "guanxi2:" << endl << result << endl;
        // // cout << "R:" << endl << R_odom << endl;
        // return std::move(result);
        vtf._Update("cam",cpos,TempTF);
        return vtf.GetChildInFrameData("imu",TempTF);
    }
    //o -> g
    //in:mm out:mm
    Eigen::Vector3d Calculater::OdomToGun(Eigen::Vector3d opos){
        return {0,0,0};
        // return this->R_odom2cap * (this->R_odom.inverse() * opos - this->T_Odom_Gun);
        vtf._Update("imu",opos,TempTF);
        return vtf.GetChildInFrameData("gun",TempTF);
    }
    //o -> g -> c
    //in:mm out:mm
    Eigen::Vector3d Calculater::OdomToCam(Eigen::Vector3d opos){
        return {0,0,0};
        // return T_Camera_Gun + OdomToGun(opos);
        vtf._Update("imu",opos,TempTF);
        return vtf.GetChildInFrameData("cam",TempTF);
    }
    Eigen::Vector3d Calculater::GetCompensatedPoint(double compesation,Eigen::Vector3d tvec){
        // 先构造补偿点在陀螺仪系下的点
        Eigen::Vector3d opos = this->CameraToOdom(tvec),result;
        //odom
        result << opos[0],opos[1],opos[0] * tan(compesation);
        result = this->OdomToCam(result);
        return std::move(result);
    }
    vector<Eigen::Vector3d> Calculater::RebuildArmorInOdom(Eigen::Vector3d pose,Eigen::Vector3d center){
        throw NULL;
        // //四点陀螺仪系下坐标
        // Eigen::Vector3d P1, P2, P3, P4;
        // //四点相机系下坐标
        // Eigen::Vector3d P11, P22, P33, P44;
        // //陀螺仪旋转矩阵的逆
        // Eigen::Matrix3d R_odom_inverse;
        // //roll轴的旋转矩阵
        // Eigen::Matrix3d rotation_matrix_roll;
        // //pitch轴的旋转矩阵
        // Eigen::Matrix3d rotation_matrix_pitch;
        // //yaw轴的旋转矩阵
        // Eigen::Matrix3d rotation_matrix_yaw;

        // rotation_matrix_roll << 1, 0, 0,
        //                         0, std::cos(pose.x()), -std::sin(pose.x()),
        //                         0, std::sin(pose.x()), std::cos(pose.x());

        // rotation_matrix_pitch << std::cos(pose.y()), 0 ,std::sin(pose.y()),
        //                         0, 1, 0,
        //                         -std::sin(pose.y()), 0, std::cos(pose.y());

        // rotation_matrix_yaw << std::cos(pose.z()), -std::sin(pose.z()), 0,
        //                     std::sin(pose.z()), std::cos(pose.z()), 0,
        //                     0, 0, 1;


        // R_odom_inverse = R_odom.inverse();

        // P1 =  center + rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll * Eigen::Vector3d(0, -Half_Width / 1000, Half_Height / 1000);
        // P2 =  center + rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll * Eigen::Vector3d(0, Half_Width / 1000, Half_Height / 1000);
        // P3 =  center + rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll * Eigen::Vector3d(0, Half_Width / 1000, -Half_Height / 1000);
        // P4 =  center + rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll * Eigen::Vector3d(0, -Half_Width / 1000, -Half_Height / 1000);

        // P11 = this->R_odom2cap * (R_odom_inverse * P1 + T_Camera_Gun);
        // P22 = this->R_odom2cap * (R_odom_inverse * P2 + T_Camera_Gun);
        // P33 = this->R_odom2cap * (R_odom_inverse * P3 + T_Camera_Gun);
        // P44 = this->R_odom2cap * (R_odom_inverse * P4 + T_Camera_Gun);

        // vector<Eigen::Vector3d> pionts;
        // pionts.push_back(P11);
        // pionts.push_back(P22);
        // pionts.push_back(P33);
        // pionts.push_back(P44);

        // return pionts;
    }
    Eigen::Vector4d Calculater::Euler2QUaternion(double roll, double pitch, double yaw)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(roll, pitch, yaw);
        return Eigen::Vector4d(qtn.w(), qtn.x(), qtn.y(), qtn.z()); 
    } 
    double Calculater::GetCompensation(Eigen::Vector3d opos,double vec = 30.0){
        // opos /= 1000;
        // opos = OdomToGun(opos);
        // double _height = 0.0,height = -0.08,depth = fabs(opos.x());
        double _height = 0.0,height = opos.z(),depth = Eigen::Vector2d(opos.x(),opos.y()).norm();
        double xita = -PI / 4,minError = DOUBLE_MAX,error = 0.0,result = 0.0;
        // cout << "h:" << height << ' ' << "d:" << opos.x() << "p" << opos.y() << endl;
        while(xita < PI / 3)
        {
            _height = depth * std::tan(xita) - (LOCAL_G * FA * depth * depth) / (2 * (vec * FA * std::cos(xita)) * (vec * FA * std::cos(xita)));
            error = std::fabs(height - _height);
            if(error < 0.1) //误差已经很低了 不用更高精度了
            {
                // cout << "xita:" << xita << endl;
                return xita;
            }
            // else if(error < minError){
            //     result = xita;
            //     minError = error;
            // }
            xita += 0.005;//目前输出的值为相对于相机现在的姿态所要转的角度，即相机坐标系下的仰角
        }
        return result;
    }
    Eigen::Vector3d Calculater::CalCenter(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
    {
        tf2::Quaternion qtn1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
        tf2::Quaternion qtn2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);
        double roll1, pitch1, yaw1;
        double roll2, pitch2, yaw2;
        tf2::Matrix3x3(qtn1).getRPY(roll1, pitch1, yaw1);
        tf2::Matrix3x3(qtn2).getRPY(roll2, pitch2, yaw2);

        double line1_k = tan(yaw1);
        double line2_k = tan(yaw2);

        double b1 = p1.position.y -line1_k * p1.position.x;
        double b2 = p2.position.y -line2_k * p2.position.x;

        double x = (b2 - b1) / (line1_k - line2_k);
        double y = line1_k * x + b1;

        Eigen::Vector3d tp_c(x, y ,(p1.position.z + p2.position.z) / 2);
        Eigen::Vector3d tp_p(p1.position.x, p1.position.y , p1.position.z);
        Eigen::Vector3d r = tp_c - tp_p;
        double rr = sqrt((tp_c[0] - tp_p[0]) * (tp_c[0] - tp_p[0]) + (tp_c[1] - tp_p[1]) * (tp_c[1] - tp_p[1]));
        
        x = p1.position.x + rr * cos(yaw1 + M_PI);
        y = p1.position.y + rr * sin(yaw1 + M_PI);
        return Eigen::Vector3d(x, y, (p1.position.z + p2.position.z) / 2);
    }
    // 算四点距离
    double Calculater::CalPointsDis(std::vector<cv::Point2f> targetPoint, std::vector<cv::Point2f> point)
    {
        double dis;
        for(int i = 0; i < targetPoint.size(); i++){
            dis += sqrt(pow((targetPoint[i].x - point[i].x), 2) + pow((targetPoint[i].y - point[i].y), 2));
        }
        return dis / 4;
    }
    cv::Rect Calculater::Points2Rect(std::vector<cv::Point2f> pts){
        double  min_x = DOUBLE_MAX,min_y = DOUBLE_MAX,
                max_x = -1,max_y = -1;
        for(cv::Point2f& pt:pts){
            if(pt.x < min_x)
                min_x = pt.x;
            if(pt.x > max_x)
                max_x = pt.x;
            if(pt.y < min_y)
                min_y = pt.y;
            if(pt.y > max_y)
                max_y = pt.y;
        }
        double height = fabs(max_y - min_y);
        double width = fabs(max_x - min_x);
        height = (width - height)/2;
        width *= 0.1;
        // cout << "HRate:" << h_rate << endl;
        // cout << "h    :" << height << endl;
        min_x -= width;
        min_y -= height;
        max_x += width;
        max_y += height;
        if(min_x < 0)
            min_x = 0;
        if(min_y < 0)
            min_y = 0;
        // cout << min_x << ' '<< min_y << ' '<< max_x << ' '<< max_y << endl;
        return cv::Rect((int)min_x,(int)min_y,(int)(max_x-min_x),(int)(max_y-min_y));
    }
    void Calculater::TargetUpdate(Eigen::Vector3d pos,Eigen::Vector3d pose = {0,0,0}){
        vtf.TargetUpdate(pos,pose);
        return;
    }
    Eigen::Vector3d Calculater::GetAbsolutePosition(const std::string& name){
        return vtf.GetChildInFrameData("map",name);
    }
    Eigen::Vector3d Calculater::GetTargetPos(const std::string&frame){
        return vtf.GetChildInFrameData(frame,"target");
    }
    void Calculater::TFUpdata(const std::string&frame,const std::string&name,Eigen::Vector3d pos,Eigen::Vector3d pose = {0,0,0},int index = 0){
        vtf._Update(frame,pos,name,pose,index);
        return;
    }
    Eigen::Vector3d Calculater::GetPos(const std::string&frame,const std::string&name){
        return vtf.GetChildInFrameData(frame,name);
    }
    std::vector<Eigen::Vector3d> Calculater::GetPos(const std::string&frame,const std::string&name,int){
        return vtf.GetChildInFrameData(frame,name,1);
    }
    Eigen::Vector3d Calculater::Rvec2Euler(cv::Mat rvec){
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        // 从旋转矩阵 R 中提取欧拉角
        double theta_x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        double theta_y = atan2(-R.at<double>(2, 0), sqrt(pow(R.at<double>(2, 1), 2) + pow(R.at<double>(2, 2), 2)));
        double theta_z = atan2(R.at<double>(1, 0), R.at<double>(0, 0)); 
        return {theta_x,theta_y,theta_z};
    }

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float  Calculater::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    static float k = 0.1;//0.092
    nh.param("fa_test",k,k);
    //t为给定v与angle时的飞行时间
    t = (float)((exp(k* s) - 1) / (k * v * cos(angle)));
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float Calculater::pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

std::vector<Eigen::Vector3d> Calculater::GetPosAtTime(const std::string& frame, 
                                             const std::string& name, 
                                             const ros::Time& time)
{
        return vtf.GetChildInFrameDataAtTime(frame, name, time, 1);
}

/**
 * @brief 能量机关PnP解算
 * @param blade_pt 扇叶中心点（像素坐标）
 * @param circle_center 能量机关圆心（像素坐标）
 * @param world_pos 输出：能量机关在相机坐标系中的位置
 * @return 是否解算成功
 */
bool Calculater::FanPnPSolve(const cv::Point2f& blade_pt, const cv::Point2f& circle_center, Eigen::Vector3d& world_pos) {
    if (cameraM.empty()) {
        ROS_WARN("Camera matrix is empty, cannot perform Fan PnP solve");
        return false;
    }

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;

    // ====== 使用4个模型点 ======
    // 点1：底部圆心
    object_points.push_back(cv::Point3f(0, 0, 0));
    image_points.push_back(circle_center);

    // 点2：扇叶位置（沿x轴）
    object_points.push_back(cv::Point3f(0.5, 0, 0));
    image_points.push_back(blade_pt);

    // 点3：扇叶位置（沿y轴）
    double angle = atan2(blade_pt.y - circle_center.y, blade_pt.x - circle_center.x);
    cv::Point2f blade_pt_y(circle_center.x + 0.5*cos(angle + M_PI/2), 
                           circle_center.y + 0.5*sin(angle + M_PI/2));
    object_points.push_back(cv::Point3f(0, 0.5, 0));
    image_points.push_back(blade_pt_y);

    // 点4：顶部圆心（高度2.5米）
    object_points.push_back(cv::Point3f(0, 0, 2.5));
    image_points.push_back(circle_center);

    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    // 使用 SOLVEPNP_ITERATIVE 方法
    bool success = cv::solvePnP(object_points, image_points, cameraM, dist_coeffs, rvec, tvec);

    if (success) {
        world_pos << tvec.at<double>(0, 0), 
                     tvec.at<double>(1, 0), 
                     tvec.at<double>(2, 0);
        ROS_DEBUG("Fan PnP solved: x=%.2f, y=%.2f, z=%.2f", 
                  world_pos.x(), world_pos.y(), world_pos.z());
        return true;
    } else {
        ROS_WARN("Fan PnP solve failed, using fallback distance estimation");
        return false;
    }
}
}
#endif