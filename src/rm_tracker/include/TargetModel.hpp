#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// ROS相关头文件
#include "ros/ros.h"
#include "rm_msgs/Armor.h"
#include "rm_msgs/ArmorArray.h"
#include "rm_msgs/RmSerial.h" 
#include <std_msgs/Float64.h>
#include <angles/angles.h>
// OpenCV相关头文件
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// 标准库头文件
#include <iostream>
#include <cmath>
// 时间相关头文件
#include <chrono>
#include <thread>
#include <time.h>
// 自定义头文件
#include "Calculater.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "KinematicModel.hpp"
#include "ModelObserver.hpp"
#include "Armor.hpp"
using namespace std;
using namespace cv;

/*
工厂模式: 它提供一种创建对象的最佳方式,而无需向客户端暴露创建逻辑
简单来说工厂模式就是:用一个专门的类来负责创建其他类的实例
解耦:    将对象的创建与使用分离
扩展性:  添加新产品时无需修改现有代码
封装性:  隐藏对象创建的复杂细节
*/
// 计划后续用工厂模式重构代码
// class ModelObserverFactory {
// public:
//     static std::shared_ptr<ModelObserver> createObserver(
//         KinematicModel::Type type, 
//         const std::string& config_path
//     ) {
//         switch(type) {
//             case KinematicModel::STANDARD:
//                 return std::make_shared<StandardObserver>(config_path);
//             case KinematicModel::BALANCE:
//                 return std::make_shared<BalanceObserver>(config_path);
//             case KinematicModel::OUTPOST:
//                 return std::make_shared<OutpostObserver>(config_path);
//             default:
//                 return std::make_shared<StandardObserver>(config_path);
//         }
//     }
// };

struct TargetModel
{
    Armor armor;                                //装甲板
    std::shared_ptr<ModelObserver> ekf;         //卡尔曼滤波器
    std::shared_ptr<KinematicModel> m_status = std::make_shared<StandardModel>();;   //运动状态(初始化为标准模型)
    
    // 装甲板属性
    int count = 4;
    double last_yaw = 0;
    
    //更新次数
    int update_count = 0;                                               //装甲板更新次数 (老代码像素判断装甲板跳变,现在也没有什么作用了)
    
    //状态标志
    bool auto_fire = false;                                             //是否允许自动开火
    bool abnormal = false;                                              //预测结果是否异常

    // 模型类型相关属性
    int m_type_init_cnt = 0;                                            // 模型类型识别计数器
    KinematicModel::Type m_model_type = KinematicModel::UNKNOWN;        //模型类型

    // 这里先默认是标准模型
    TargetModel(const std::string& config_path)
    {
        // 初始化为标准模型(初始化什么模型不影响)
        ekf = std::make_shared<StandardObserver>(config_path);
    }
};



