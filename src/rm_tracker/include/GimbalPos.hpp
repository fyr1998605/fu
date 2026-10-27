#pragma once
#ifndef GIMBAL_POS_HPP
#define GIMBAL_POS_HPP
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;

#ifndef PI
#define PI 3.1415926
#endif

#ifdef D2R
#undef D2R
#endif

#ifdef R2D
#undef R2D  
#endif

// 提示: 
// 成员函数只能将*this作为左操作符,而友元函数可以灵活处理操作数的位置
// 云台位姿
struct GimbalPose
{
    float pitch;
    float yaw;
    float roll;

    GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0,double timestamp = 0.0)
    {
        this->pitch = pitch;
        this->yaw = yaw;
        this->roll = roll;
    }

    GimbalPose operator=(const GimbalPose &gm)
    {
        this->pitch = gm.pitch;
        this->yaw = gm.yaw;
        this->roll = gm.roll;
        return *this;
    }

    GimbalPose operator=(const float init_value)
    {
        this->pitch = init_value;
        this->yaw = init_value;
        this->roll = init_value;
        return *this;
    }

    friend GimbalPose operator-(const GimbalPose &gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch = gm1.pitch - gm2.pitch;
        temp.yaw = gm1.yaw - gm2.yaw;
        temp.roll = gm1.roll - gm2.roll;
        return temp;
    }

    friend GimbalPose operator+(const GimbalPose &gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch = gm1.pitch + gm2.pitch;
        temp.yaw = gm1.yaw + gm2.yaw;
        temp.roll = gm1.roll + gm2.roll;
        return temp;
    }

    friend GimbalPose operator*(const GimbalPose &gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch * k;
        temp.yaw = gm.yaw * k;
        temp.roll = gm.roll * k;
        return temp;
    }

    friend GimbalPose operator*(const float k, const GimbalPose &gm)
    {

        GimbalPose temp{};
        temp.pitch = gm.pitch * k;
        temp.yaw = gm.yaw * k;
        temp.roll = gm.roll * k;
        return temp;
    }

    friend GimbalPose operator/(const GimbalPose &gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch / k;
        temp.yaw = gm.yaw / k;
        temp.roll = gm.roll / k;
        return temp;
    }

    friend std::ostream &operator<<(std::ostream &out, const GimbalPose &gm)
    {
        out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]";
        return out;
    }

    float norm()
    {
        return sqrt(pow(this->pitch, 2) + pow(this->yaw, 2));
    }
};

// 将弧度转换为ie角度
inline double R2D(double rad)
{
    return rad * 180 / PI;
}

// 将角度转换为弧度
inline double D2R(double degree)
{
    return degree * PI / 180;
}

// 计算二维点到原点的欧几里得距离
inline double getDistance(cv::Point2d p)
{
    return sqrt(pow(p.x, 2) + pow(p.y, 2));
}

// 计算三维点到原点的距离,可选择是否考虑z坐标
inline double getDistance(cv::Point3d p, bool flag = true)
{
    return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2) * (int)flag);
}

#endif // GIMBAL_POS_HPP


