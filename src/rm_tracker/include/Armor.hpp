#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <ros/time.h>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

struct Armor 
{
    char id = -1;                                           // 装甲板ID
    Eigen::Vector3d position = Eigen::Vector3d::Zero();     // 3D位置 (x,y,z)未预测
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();          // 姿态 (roll,pitch,yaw) 未预测
    Eigen::Vector3d pre_position = Eigen::Vector3d::Zero(); //预测装甲板三维坐标
    Eigen::Vector3d pre_rpy = Eigen::Vector3d::Zero();      //预测车中心
    double height = 0.0;                                    // 高度
    double radius = 0.0;                                    // 旋转半径
    bool is_current = false;                                // 是否为当前追踪目标
    char armor_type = 's';                                  // 装甲板类型
    int current_index = 0;                                  // 当前装甲板索引
    // ros::Time timestamp;                                    // 时间戳成员变量
    
    // 构造函数
    Armor() : position(Eigen::Vector3d::Zero()), rpy(Eigen::Vector3d::Zero()) {}

    // 带时间戳的构造函数
    // Armor(ros::Time stamp) : timestamp(stamp), position(Eigen::Vector3d::Zero()), rpy(Eigen::Vector3d::Zero()) {}
};
typedef std::vector<Armor> Armors;