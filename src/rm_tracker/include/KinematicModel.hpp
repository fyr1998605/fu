#pragma once
#ifndef KINEMATIC_MODEL_HPP
#define KINEMATIC_MODEL_HPP


#include <opencv2/opencv.hpp>
#include <vector>
#include <Armor.hpp>
#include "GimbalPos.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace cv;


// 将弧度约束在[-pi, pi]范围内
// 将弧度约束在[-pi, pi]范围内
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif

#ifndef PI
#define PI 3.1415926
#endif

// 在GimbalPos.hpp开头添加：
#ifdef D2R
#undef D2R
#endif

#ifdef R2D
#undef R2D  
#endif

// // 角度转换宏
// #define D2R(angle) ((angle) * PI / 180.0)
// #define R2D(angle) ((angle) * 180.0 / PI)

class KinematicModel
{
public:
    KinematicModel() : index(0) // 初始化列表
    {
        height.clear(); // 清空height容器
        radius.clear(); // 清空radius容器 
    }
    ~KinematicModel() {}

    // 统一接口 
    virtual Armors getArmors(double predict_time = 0) = 0;
    virtual Armor getClosestArmor(double predict_time, double switch_threshold) = 0;
    virtual Armor getFacingArmor(double predict_time) = 0;
    virtual void print(std::string tag) = 0;

    // 获取状态
    virtual Eigen::VectorXd getState() const = 0;

    enum Type
    {
        UNKNOWN = -1,
        STANDARD = 0,
        BALANCE = 1,
        OUTPOST = 2
    };

public:
    cv::Point2d center;             // 旋转中心位置
    std::vector<double> height;     // 装甲高度。0对应索引0、2，1对应索引1、3
    std::vector<double> radius;     // 旋转半径。0对应索引0、2，1对应索引1、3
    double phase;                   // 角度（并非定值）
    double palstance;               // 角速度

    int number;                     // 装甲板数量
    int index;                      // 当前目标索引
};

class StandardModel : public KinematicModel
{
public:
    StandardModel() : StandardModel(Eigen::Matrix<double, 10, 1>::Zero()) {}
    StandardModel(const Eigen::Matrix<double, 10, 1> &X) : KinematicModel()
    {
        // 调整容器大小
        height.resize(2);
        radius.resize(2);

        // 设置装甲板数量
        number = 4;

        // 初始化未知和速度
        center = cv::Point2d(X(0,0),X(2,0));
        velocity = cv::Point2d(X(1,0),X(3,0));

        // 初始化高度
        height[0] = X(4,0);     // 左侧装甲板的高度
        height[1] = X(5,0);     // 右侧装甲板的高度

        // 初始化半径
        radius[0] = X(6,0);     // 小装甲板的旋转半径
        radius[1] = X(7,0);     // 大装甲板的旋转半径

        // 初始化相位角和角速度
        phase = X(8,0);
        palstance = X(9,0);
    }
    ~StandardModel() {}

    // 获取状态
    Eigen::VectorXd getState() const override {
        Eigen::VectorXd state(10);
        state << center.x, velocity.x, center.y, velocity.y, 
                 height[0], height[1], radius[0], radius[1], 
                 phase, palstance;
    /*  状态向量参数(再再再复习一下吧)
        X(0) -> 机器人中心的x坐标
        X(1) -> 机器人中心x方向速度
        X(2) -> 机器人中心的y坐标
        X(3) -> 机器人中心y方向速度
        X(4) -> 左侧装甲板的固定高度
        X(5) -> 右侧装甲板的固定高度
        X(6) -> 小装甲板旋转半径（左侧）
        X(7) -> 大装甲板旋转半径（右侧）
        X(8) -> 机器人整体偏航角（yaw）
        X(9) -> 偏航角速度
    */
        return state;
    }

    

    // 这段代码完全没有用上 -> 西工大写的有问题
    /**
     * @brief 为了保留index，赋值时不继承index
     */
    StandardModel operator=(const StandardModel &status)
    {
        int tmp_index= this->index;
        *this = status;
        this->index = tmp_index;
        cout << "this->index" << this->index << endl;
        return *this;
    }

    /**
     * @brief 获取该运动状态下所有装甲板
     * @param predict_time 预测时间
     */
    /**
     * @brief 获取该运动状态下所有装甲板
     * @param predict_time 预测时间
     */
    Armors getArmors(double predict_time) override {
        Armors armors(number);
        for (int i = 0; i < number; i++) {
            // 计算每个装甲板的预测角度
            double angle = _std_radian(phase + palstance * predict_time + i * PI / 2);
            // 计算装甲板中心位置
            cv::Point2d point = center + velocity * predict_time + 
                                cv::Point2d(radius[i % 2] * cos(angle), radius[i % 2] * sin(angle));
            // 设置装甲板位置和姿态
            armors[i].position = Eigen::Vector3d(point.x, point.y, height[i % 2]);
            armors[i].rpy = Eigen::Vector3d(0, 0, angle); // 只设置yaw角
            // cout << "armors[i].position: "<<armors[i].position << endl;
            // cout << "armors[i].rpy:" <<armors[i].rpy  << endl;
        }
        return armors;
    }

    /**
     * @brief 获取目标装甲板
     * @param predict_time 预测时间
     * @param switch_threshold 更新装甲板切换的最小距离差
     */
    Armor getClosestArmor(double predict_time, double switch_threshold) override {

        // 计算切换提前时间
        double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance)); // 切换时间 = 切换阈值 / 当前的角速度 
        // std::cout << "switch_advanced_time: " << switch_advanced_time << std::endl;
        // 获取预测装甲板位置
        Armors armors = getArmors(predict_time + switch_advanced_time);
        // 预测车辆中心
        cv::Point2d predict_center = center + velocity * predict_time;
        
        // 从多个装甲板中筛选出最正对相机视角的目标
        for (int i = 0; i < number; i++) {
            if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(predict_center.y, predict_center.x))) + D2R(switch_threshold) <
                abs(_std_radian(PI + armors[index].rpy.z() - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
                // cout << "选择装甲板的编号: " << index << endl; 
            }
        }
        return getArmors(predict_time)[index]; // 返回目标装甲板的坐标
    }

    /**
     * @brief 获取正对的装甲板
     */
    Armor getFacingArmor(double predict_time) override {
        // 在角度最优的基础上,主动规避正在远离的装甲板
        Armor armor;
        Armors armors = getArmors(predict_time);
        cv::Point2d predict_center = center + velocity * predict_time;
        
        // 找到最接近正对位置的装甲板
        for (int i = 0; i < number; i++) {
            if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(predict_center.y, predict_center.x))) <
                abs(_std_radian(PI + armors[index].rpy.z() - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
                // cout << "选择装甲板的编号: " << index << endl; 
            }
        }
        
        // 如果最近的装甲板处于远离状态，则击打下一装甲板 
        //通过角速度补偿预判装甲板远离状态
        //D2R(-50)补偿角
        if (abs(_std_radian(PI + armors[index].rpy.z() + (palstance / abs(palstance)) * D2R(-50) - atan2(predict_center.y, predict_center.x))) <
            abs(_std_radian(PI + armors[index].rpy.z() - atan2(predict_center.y, predict_center.x))))
        {
            index = (index + 1) % number;
        }
        double angle = _std_radian(atan2(predict_center.y, predict_center.x) - PI);
        armor.position = Eigen::Vector3d(predict_center.x + radius[index % 2] * cos(angle), predict_center.y + radius[index % 2] * sin(angle), height[index % 2]);
        armor.rpy.z() = angle;

        // cout << "armor.position.x(): " << armor.position.x() <<  "armor.position.y():" << armor.position.y() << "armor.position.z():" << armor.position.z() << endl;
        // cout << "armor.rpy.z(): " << armor.rpy.z() << endl;

        // 返回当前装甲板
        return armor;
    }

    /**
     * @brief 打印信息
     */
    void print(std::string tag)
    {
        std::string prefix = "[StandardModel] " + tag + " ";
        std::cout << prefix << "center: " << center << " += " << velocity << std::endl;
        std::cout << prefix << "height: " << height[0] << " " << height[1] << std::endl;
        std::cout << prefix << "radius: " << radius[0] << " " << radius[1] << std::endl;
        std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
        std::cout << prefix << "index: " << index << std::endl;
    }


public:
    cv::Point2d velocity;           // 旋转中心速度
};

class BalanceModel : public KinematicModel
{
public:
    BalanceModel() : BalanceModel(Eigen::Matrix<double, 7, 1>::Zero()) {}
    BalanceModel(const Eigen::Matrix<double, 7, 1> &X) : KinematicModel()
    {
        height.resize(1);
        radius.resize(1);
        number = 2;

        center = cv::Point2d(X(0, 0), X(1, 0));
        velocity = X(2, 0);
        height[0] = X(3, 0);
        radius[0] = X(4, 0);
        phase = X(5, 0);
        palstance = X(6, 0);
    }
    ~BalanceModel() {}


    // 获取状态
    Eigen::VectorXd getState() const override {
        Eigen::VectorXd state(7);
        state << center.x, center.y, velocity, height[0], radius[0], phase, palstance;
        return state;
    }


    /**
     * @brief 为了保留index，赋值时不继承index
     */
    BalanceModel operator=(const BalanceModel &status)
    {
        int tmp_index = this->index;
        *this = status;
        this->index = tmp_index;
        return *this;
    }

    /**
     * @brief 获取该运动状态下所有装甲板
     * @param predict_time 预测时间
     */
    Armors getArmors(double predict_time) override {
        Armors armors(number);
        for (int i = 0; i < number; i++) {
            double angle = _std_radian(phase + palstance * predict_time + i * PI);
            cv::Point2d point;
            
            // if (palstance == 0)
            {
                point.x = center.x + velocity * cos(phase) * predict_time;
                point.y = center.y + velocity * sin(phase) * predict_time;
            }
            // else
            // {
            //     point.x = center.x + velocity / palstance * (sin(palstance * predict_time + phase) - sin(phase));
            //     point.y = center.y - velocity / palstance * (cos(palstance * predict_time + phase) - cos(phase));
            // }
            
            // 添加旋转偏移
            point += cv::Point2d(radius[0] * cos(angle), radius[0] * sin(angle));
            armors[i].position = Eigen::Vector3d(point.x, point.y, height[0]);
            armors[i].rpy = Eigen::Vector3d(0, 0, angle); // 只设置yaw角
            // cout << "armors[i].position: "<<armors[i].position << endl;
            // cout << "armors[i].rpy:" <<armors[i].rpy  << endl;
        }
        return armors;
    }

   /**
     * @brief 获取目标装甲板
     * @param predict_time 预测时间
     * @param switch_threshold 更新装甲板切换的最小距离差
     */
    Armor getClosestArmor(double predict_time, double switch_threshold) override {
        double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance));
        Armors armors = getArmors(predict_time + switch_advanced_time);
        cv::Point2d predict_center(center.x + velocity * cos(phase) * predict_time, center.y + velocity * sin(phase) * predict_time);
        
        for (int i = 0; i < number; i++) {
            if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(predict_center.y, predict_center.x))) + D2R(switch_threshold) <
                abs(_std_radian(PI + armors[index].rpy.z() - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
            }
        }
        return getArmors(predict_time)[index];
    }

    /**
     * @brief 获取正对的装甲板
     */
    Armor getFacingArmor(double predict_time) override {
        Armor armor;
        Armors armors = getArmors(predict_time);
        cv::Point2d predict_center(center.x + velocity * cos(phase) * predict_time, center.y + velocity * sin(phase) * predict_time);
        
        // 找到最接近正对位置的装甲板
        for (int i = 0; i < number; i++) {
           if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(predict_center.y, predict_center.x))) <
                abs(_std_radian(PI + armors[index].rpy.z() - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
            }
        }
        double angle = _std_radian(atan2(predict_center.y, predict_center.x) - PI);
        armor.position = Eigen::Vector3d(predict_center.x + radius[0] * cos(angle), predict_center.y + radius[0] * sin(angle), height[0]);
        armor.rpy.z() = angle;
        // cout << "armor.position.x(): " << armor.position.x() <<  "armor.position.y():" << armor.position.y() << "armor.position.z():" << armor.position.z() << endl;
        // cout << "armor.rpy.z(): " << armor.rpy.z() << endl;
        return armor;
    }

    /**
     * @brief 打印信息
     */
    void print(std::string tag)
    {
        std::string prefix = "[BalanceModel] " + tag + " ";
        std::cout << prefix << "center: " << center << " += " << velocity << std::endl;
        std::cout << prefix << "height: " << height[0] << std::endl;
        std::cout << prefix << "radius: " << radius[0] << std::endl;
        std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
        std::cout << prefix << "index: " << index << std::endl;
    }

public:
    double velocity;                // 旋转中心速度
};

// 老前哨战模型代码
// class OutpostModel : public KinematicModel
// {
// public:
//     OutpostModel() : OutpostModel(Eigen::Matrix<double, 5, 1>::Zero()) {}
//     OutpostModel(const Eigen::Matrix<double, 5, 1> &X) : KinematicModel()
//     {
//         height.resize(1);
//         radius.resize(1);
//         number = 3;

//         center = cv::Point2d(X(0, 0), X(1, 0));
//         height[0] = X(2, 0);
//         radius[0] = 0.25685;
//         phase = X(3, 0);
//         palstance = X(4, 0);
//     }
//     ~OutpostModel() {}

//     /**
//      * @brief 为了保留index，赋值时不继承index
//      */
//     OutpostModel operator=(const OutpostModel &status)
//     {
//         int tmp_index = this->index;
//         *this = status;
//         this->index = tmp_index;
//         return *this;
//     }


//      // 获取状态
//     Eigen::VectorXd getState() const override {
//         Eigen::VectorXd state(5);
//         state << center.x, center.y, height[0], phase, palstance;
//         return state;
//     }

//     /**
//      * @brief 获取该运动状态下所有装甲板
//      * @param predict_time 预测时间
//      */
//     Armors getArmors(double predict_time) override {
//         Armors armors(number);
//         for (int i = 0; i < number; i++) {
//             double angle = _std_radian(phase + palstance * predict_time + 2 * i * PI / 3.0);
//             cv::Point2d point = center + cv::Point2d(radius[0] * cos(angle), radius[0] * sin(angle));
            
//             armors[i].position = Eigen::Vector3d(point.x, point.y, height[0]);
//             armors[i].rpy = Eigen::Vector3d(0, 0, angle); // 只设置yaw角
//             // cout << "armors[i].position: "<<armors[i].position << endl;
//             // cout << "armors[i].rpy:" <<armors[i].rpy  << endl;
//         }
//         return armors;
//     }

//     /**
//      * @brief 获取目标装甲板
//      * @param predict_time 预测时间
//      * @param switch_threshold 更新装甲板切换的最小距离差
//      */
//     Armor getClosestArmor(double predict_time, double switch_threshold) override {
//         double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance));
//         Armors armors = getArmors(predict_time + switch_advanced_time);
        
//         for (int i = 0; i < number; i++) {
//             if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(center.y, center.x))) + D2R(switch_threshold) <
//                 abs(_std_radian(PI + armors[index].rpy.z() - atan2(center.y, center.x))))
//             {
//                 index = i;
//             }
//         }
//         return getArmors(predict_time)[index];
//     }

//     /**
//      * @brief 获取正对的装甲板
//      */
//     Armor getFacingArmor(double predict_time) override {
//         Armor armor;
//         Armors armors = getArmors(predict_time);
//         // 找到最接近正对位置的装甲板
//         for (int i = 0; i < number; i++) {
//            if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(center.y, center.x))) <
//                 abs(_std_radian(PI + armors[index].rpy.z() - atan2(center.y, center.x))))
//             {
//                 index = i;
//             }
//         }
//         double angle = _std_radian(atan2(center.y, center.x) - PI);
//         armor.position = Eigen::Vector3d(center.x + radius[0] * cos(angle), center.y + radius[0] * sin(angle), height[0]);
//         // 调试
//         // cout << "armor.position.x(): " << armor.position.x() <<  "armor.position.y():" << armor.position.y() << "armor.position.z():" << armor.position.z() << endl;
//         // cout << "armor.rpy.z(): " << armor.rpy.z() << endl;
        
//         armor.rpy.z() = angle;
//         return armor;
//     }

//     /**
//      * @brief 打印信息
//      */
//     void print(std::string tag)
//     {
//         std::string prefix = "[OutpostModel] " + tag + " ";
//         std::cout << prefix << "center: " << center << std::endl;
//         std::cout << prefix << "height: " << height[0] << std::endl;
//         std::cout << prefix << "radius: " << radius[0] << std::endl;
//         std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
//         std::cout << prefix << "index: " << index << std::endl;
//     }
// };

// // 新前哨战三层装甲板代码
// class OutpostModel : public KinematicModel
// {
// public:
//     OutpostModel() : OutpostModel(Eigen::Matrix<double, 7, 1>::Zero()) {}
//     OutpostModel(const Eigen::Matrix<double, 7, 1> &X) : KinematicModel()
//     {
//         // 旧的: X状态矩阵
//         // 第0行: 中心x
//         // 第1行: 中心y
//         // 第2行: 高度h
//         // 第3行: 偏航角yaw
//         // 第4行: 角速度palstance

//         // 现在: X状态矩阵
//         // 第0行: 中心x
//         // 第1行: 中心y
//         // 第2行: 高度h1
//         // 第3行: 高度h2
//         // 第4行: 高度h3
//         // 第5行: 偏航角yaw
//         // 第6行: 角速度palstance

//         height.resize(3);
//         radius.resize(1);
//         number = 3;

//         center = cv::Point2d(X(0, 0), X(1, 0));
//         height[0] = X(2, 0);
//         height[1] = X(3, 0);
//         height[2] = X(4, 0);

//         radius[0] = 0.256;
//         phase = X(5, 0);
//         palstance = X(6, 0);
//     }
//     ~OutpostModel() {}

//     /**
//      * @brief 为了保留index，赋值时不继承index
//      */
//     // OutpostModel operator=(const OutpostModel &status)
//     // {
//     //     int tmp_index = this->index;
//     //     *this = status;
//     //     this->index = tmp_index;
//     //     return *this;
//     // }


//      // 获取状态
//     Eigen::VectorXd getState() const override {
//         // Eigen::VectorXd state(5);
//         // state << center.x, center.y, height[0], phase, palstance;
//         // return state;

//         Eigen::VectorXd state(7);
//         state << center.x, center.y, height[0], height[1], height[2], phase, palstance;
//         return state;
//     }

//     /**
//      * @brief 获取该运动状态下所有装甲板
//      * @param predict_time 预测时间
//      */
//     Armors getArmors(double predict_time) override {
//         Armors armors(number);
//         for (int i = 0; i < number; i++) {

//             // 1.计算未来时刻的装甲板角度
//             double angle = _std_radian(phase + palstance * predict_time + 2 * i * PI / 3.0);

//             // 2.计算装甲板在世界坐标系中的位置
//             cv::Point2d point = center + cv::Point2d(radius[0] * cos(angle), radius[0] * sin(angle));
            
//             // 3.设置装甲板的三维位置和朝向
//             armors[i].position = Eigen::Vector3d(point.x, point.y, height[i]);
//             armors[i].rpy = Eigen::Vector3d(0, 0, angle); // 只设置yaw角
//             // cout << "armors[i].position: "<<armors[i].position << endl;
//             // cout << "armors[i].rpy:" <<armors[i].rpy  << endl;
//         }
//         return armors;
//     }

//     /**
//      * @brief 获取目标装甲板
//      * @param predict_time 预测时间
//      * @param switch_threshold 更新装甲板切换的最小距离差
//      */
//     Armor getClosestArmor(double predict_time, double switch_threshold) override {
//         // 1.计算切换时间
//         double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance));

//         // 2.获取提前预测的装甲板状态
//         Armors armors = getArmors(predict_time + switch_advanced_time);
        
//         // 3.寻找最佳装甲板索引
//         for (int i = 0; i < number; i++) {
//             if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(center.y, center.x))) + D2R(switch_threshold) <
//                 abs(_std_radian(PI + armors[index].rpy.z() - atan2(center.y, center.x))))
//             {
//                 index = i;
//             }
//         }

//         // 4.返回预测时刻的最佳装甲板
//         return getArmors(predict_time)[index];
//     }

//     /**
//      * @brief 获取正对的装甲板
//      */
//     Armor getFacingArmor(double predict_time) override {
//         Armor armor;
//         Armors armors = getArmors(predict_time);
//         // 找到最接近正对位置的装甲板
//         // 基于预测时间选板
//         for (int i = 0; i < number; i++) {
//            if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(center.y, center.x))) <
//                 abs(_std_radian(PI + armors[index].rpy.z() - atan2(center.y, center.x))))
//             {
//                 index = i;
//             }
//         }

//         // 计算

//         // 前哨战不会动不用像前面两个模型一样考虑运动模型的中心在子弹飞行时间后的位置
//         // 计算从世界坐标系原点(0,0)到前哨战中心的方向角
//         // 转速太快直接锁中心击打,将前哨战中心作为击打的目标
//         double angle = _std_radian(atan2(center.y, center.x) - PI);
//         // 极坐标转笛卡尔坐标
//         armor.position = Eigen::Vector3d(center.x + radius[0] * cos(angle), center.y + radius[0] * sin(angle), height[index]);
//         // 设置角度
//         armor.rpy.z() = angle;
//         // 调试
//         // cout << "armor.position.x(): " << armor.position.x() <<  "armor.position.y():" << armor.position.y() << "armor.position.z():" << armor.position.z() << endl;
//         // cout << "armor.rpy.z(): " << armor.rpy.z() << endl;
//         return armor;
//     }

//     /**
//      * @brief 打印信息
//      */
//     void print(std::string tag)
//     {
//         std::string prefix = "[OutpostModel] " + tag + " ";
//         std::cout << prefix << "center: " << center << std::endl;
//         std::cout << prefix << "height: " << height[0] << " " << height[1] << " " <<  height[2] << std::endl;
//         std::cout << prefix << "radius: " << radius[0] << std::endl;
//         std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
//         std::cout << prefix << "index: " << index << std::endl;
//     }
// };



// 新前哨战三层装甲板代码 & 动态半径
class OutpostModel : public KinematicModel
{
public:
    OutpostModel() : OutpostModel(Eigen::Matrix<double, 10, 1>::Zero()) {}
    OutpostModel(const Eigen::Matrix<double, 10, 1> &X) : KinematicModel()
    {
        // X(0): 机器人中心的x坐标
        // X(1): 机器人中心的y坐标
        // X(2): 第一块装甲板高度h1
        // X(3): 第二块装甲板高度h2
        // X(4): 第三块装甲板高度h3
        // X(5): 第一块装甲板旋转半径r1
        // X(6): 第二块装甲板旋转半径r2
        // X(7): 第三块装甲板旋转半径r3
        // X(8): 偏航角yaw
        // X(9): 角速度palstance

        height.resize(3);
        radius.resize(3);
        number = 3;

        center = cv::Point2d(X(0, 0), X(1, 0));
        height[0] = X(2, 0);
        height[1] = X(3, 0);
        height[2] = X(4, 0);

        radius[0] = X(5, 0);
        radius[1] = X(6, 0);
        radius[2] = X(7, 0);
        phase = X(8, 0);
        palstance = X(9, 0);
    }
    ~OutpostModel() {}

    /**
     * @brief 为了保留index，赋值时不继承index
     */
    // OutpostModel operator=(const OutpostModel &status)
    // {
    //     int tmp_index = this->index;
    //     *this = status;
    //     this->index = tmp_index;
    //     return *this;
    // }


     // 获取状态
    Eigen::VectorXd getState() const override {
        // Eigen::VectorXd state(5);
        // state << center.x, center.y, height[0], phase, palstance;
        // return state;

        // Eigen::VectorXd state(7);
        // state << center.x, center.y, height[0], height[1], height[2], phase, palstance;
        // return state;

        Eigen::VectorXd state(10);
        state << center.x, center.y, height[0], height[1], height[2], radius[0], radius[1], radius[2], phase, palstance;
        return state;
    }

    /**
     * @brief 获取该运动状态下所有装甲板
     * @param predict_time 预测时间
     */
    Armors getArmors(double predict_time) override {
        Armors armors(number);
        for (int i = 0; i < number; i++) {

            // 1.计算未来时刻的装甲板角度
            double angle = _std_radian(phase + palstance * predict_time + 2 * i * PI / 3.0);

            // 2.计算装甲板在世界坐标系中的位置
            cv::Point2d point = center + cv::Point2d(radius[i % 3] * cos(angle), radius[i % 3] * sin(angle));
            
            // 3.设置装甲板的三维位置和朝向
            armors[i].position = Eigen::Vector3d(point.x, point.y, height[i]);
            armors[i].rpy = Eigen::Vector3d(0, 0, angle); // 只设置yaw角
            // cout << "armors[i].position: "<<armors[i].position << endl;
            // cout << "armors[i].rpy:" <<armors[i].rpy  << endl;
        }
        return armors;
    }

    /**
     * @brief 获取目标装甲板
     * @param predict_time 预测时间
     * @param switch_threshold 更新装甲板切换的最小距离差
     */
    Armor getClosestArmor(double predict_time, double switch_threshold) override {
        // 1.计算切换时间
        double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance));

        // 2.获取提前预测的装甲板状态
        Armors armors = getArmors(predict_time + switch_advanced_time);
        
        // 3.寻找最佳装甲板索引
        for (int i = 0; i < number; i++) {
            if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(center.y, center.x))) + D2R(switch_threshold) <
                abs(_std_radian(PI + armors[index].rpy.z() - atan2(center.y, center.x))))
            {
                index = i;
            }
        }

        // 4.返回预测时刻的最佳装甲板
        return getArmors(predict_time)[index];
    }

    /**
     * @brief 获取正对的装甲板
     */
    Armor getFacingArmor(double predict_time) override {
        Armor armor;
        Armors armors = getArmors(predict_time);
        // 找到最接近正对位置的装甲板
        // 基于预测时间选板
        for (int i = 0; i < number; i++) {
           if (abs(_std_radian(PI + armors[i].rpy.z() - atan2(center.y, center.x))) <
                abs(_std_radian(PI + armors[index].rpy.z() - atan2(center.y, center.x))))
            {
                index = i;
            }
        }

        // 计算
        // 前哨战不会动不用像前面两个模型一样考虑运动模型的中心在子弹飞行时间后的位置
        // 计算从世界坐标系原点(0,0)到前哨战中心的方向角
        // 转速太快直接锁中心击打,将前哨战中心作为击打的目标
        double angle = _std_radian(atan2(center.y, center.x) - PI);
        // 极坐标转笛卡尔坐标
        armor.position = Eigen::Vector3d(center.x + radius[index % 3] * cos(angle), center.y + radius[index % 3] * sin(angle), height[index]);
        // 设置角度
        armor.rpy.z() = angle;
        // 调试
        // cout << "armor.position.x(): " << armor.position.x() <<  "armor.position.y():" << armor.position.y() << "armor.position.z():" << armor.position.z() << endl;
        // cout << "armor.rpy.z(): " << armor.rpy.z() << endl;
        return armor;
    }

    /**
     * @brief 打印信息
     */
    void print(std::string tag)
    {
        std::string prefix = "[OutpostModel] " + tag + " ";
        std::cout << prefix << "center: " << center << std::endl;
        std::cout << prefix << "height: " << height[0] << " " << height[1] << " " <<  height[2] << std::endl;
        std::cout << prefix << "radius: " << radius[0] << " " << radius[1] << " " <<  radius[2] << std::endl;
        std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
        std::cout << prefix << "index: " << index << std::endl;
    }
};

#endif // KINEMATIC_MODEL_HPP

// 运动模型工厂
// class KinematicModelFactory {
// public:
//     static std::unique_ptr<KinematicModel> createModel(KinematicModel::Type type) {
//         switch (type) {
//             case KinematicModel::STANDARD:
//                 return std::make_unique<StandardModel>();
//             case KinematicModel::BALANCE:
//                 return std::make_unique<BalanceModel>();
//             case KinematicModel::OUTPOST:
//                 return std::make_unique<OutpostModel>();
//             default:
//                 return std::make_unique<StandardModel>();
//         }
//     }
    
//     static std::unique_ptr<KinematicModel> createModel(KinematicModel::Type type, const Eigen::VectorXd& state) {
//         switch (type) {
//             case KinematicModel::STANDARD:
//                 return std::make_unique<StandardModel>(state);
//             case KinematicModel::BALANCE:
//                 return std::make_unique<BalanceModel>(state);
//             case KinematicModel::OUTPOST:
//                 return std::make_unique<OutpostModel>(state);
//             default:
//                 return std::make_unique<StandardModel>(state);
//         }
//     }
// };



