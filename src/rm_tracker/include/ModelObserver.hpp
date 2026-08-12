#pragma once
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
using namespace std;
using namespace cv;

#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif

// explicit
// 作用: 用于修饰类的构造函数，防止编译器进行隐式子类型转换
// override
// override用于明确表示一个成员函数覆盖(重写)了基类中的虚函数

//状态观测类
class ModelObserver
{
public:
    ModelObserver() {}
    ~ModelObserver() {}


    //(先验估计)
    virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors) = 0;
    //(后验估计)
    virtual std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match) = 0;

    //重置观测器内部状态
    virtual void reset() = 0;


    //从配置文件加载参数
    virtual void setParam(const std::string &file_path) = 0;

    // 模型是否稳定
    virtual bool stable() = 0;

protected:

    /*观测方程相关函数*/
    //计算第i个装甲板的预测观测值
    virtual Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i) = 0;
    //计算观测方程的雅可比矩阵H(状态偏导数)
    virtual Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i) = 0;
    //观测噪声的偏导矩阵
    virtual Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i) = 0;

    virtual Eigen::VectorXd getState() const = 0;

protected:
    bool m_debug;
    bool m_init;

    // 参数
    double m_dt;                                // 单位时间
    double m_init_radius;                       // 初始半径
    double m_gain;                              // 测距噪声关于角度的增益倍数

    double m_process_noise[4];                  // 状态转移噪声系数
    double m_measure_noise[3];                  // 观测噪声系数

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X;          // 状态向量
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X_update;   // 状态修正
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_P;          // 状态协方差
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_Q;          // 状态转移噪声
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_R;          // 观测噪声
};

class StandardObserver : public ModelObserver
{
public:
    StandardObserver(const std::string &file_path)
    {
        setParam(file_path);

        reset();

        m_F << 1, m_dt, 0, 0,    0, 0, 0, 0, 0, 0,
                0, 1,    0, 0,    0, 0, 0, 0, 0, 0,
                0, 0,    1, m_dt, 0, 0, 0, 0, 0, 0,
                0, 0,    0, 1,    0, 0, 0, 0, 0, 0,
                0, 0,    0, 0,    1, 0, 0, 0, 0, 0,
                0, 0,    0, 0,    0, 1, 0, 0, 0, 0,
                0, 0,    0, 0,    0, 0, 1, 0, 0, 0,
                0, 0,    0, 0,    0, 0, 0, 1, 0, 0,
                0, 0,    0, 0,    0, 0, 0, 0, 1, m_dt,
                0, 0,    0, 0,    0, 0, 0, 0, 0, 1;

        double dd = m_process_noise[0];
        double da = m_process_noise[1];
        double dz = m_process_noise[2];
        double dr = m_process_noise[3];
        double t4 = pow(m_dt, 3) / 3;
        double t3 = pow(m_dt, 2) / 2;
        double t2 = pow(m_dt, 1);
        m_Q << t4 * dd, t3 * dd, 0,       0,       0,  0, 0,  0,  0,       0,
                t3 * dd, t2 * dd, 0,       0,       0,  0, 0,  0,  0,       0,
                0,       0,       t4 * dd, t3 * dd, 0,  0, 0,  0,  0,       0,
                0,       0,       t3 * dd, t2 * dd, 0,  0, 0,  0,  0,       0,
                0,       0,       0,       0,       dz, 0, 0,  0,  0,       0,
                0,       0,       0,       0,       0, dz, 0,  0,  0,       0,
                0,       0,       0,       0,       0,  0, dr, 0,  0,       0,
                0,       0,       0,       0,       0,  0, 0,  dr, 0,       0,
                0,       0,       0,       0,       0,  0, 0,  0,  t4 * da, t3 * da,
                0,       0,       0,       0,       0,  0, 0,  0,  t3 * da, t2 * da;

        Eigen::VectorXd measurement_noise4(4);
        measurement_noise4 << m_measure_noise[0],//xy平面位置噪声
                                m_measure_noise[0],//xy平面位置噪声
                                m_measure_noise[1],//高度z轴噪声
                                m_measure_noise[2];//偏航角(Yaw)噪声
        m_R = measurement_noise4.asDiagonal();//4维噪声矩阵(基础层)单装甲板观测模型
        Eigen::VectorXd measurement_noise8(8);
        measurement_noise8 << measurement_noise4,
                                measurement_noise4;
        m_RR = measurement_noise8.asDiagonal();//八维噪声矩阵(增强层)双装甲板观测模型(装甲板A+装甲板B)
    }
    ~StandardObserver() {}

    /**
     * @brief 先验预测
     * @param armors 绝对坐标系装甲板序列
     * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
     */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors)
    {
        if (!m_init)
        {
            for (auto armor : armors)
            {
                m_X << armor.position.x() - m_init_radius * cos(armor.rpy.z()),
                        0,
                        armor.position.y() - m_init_radius * sin(armor.rpy.z()),
                        0,
                        armor.position.z(),
                        armor.position.z(),
                        m_init_radius,
                        m_init_radius,
                        armor.rpy.z(),
                        0;
                m_init = true;
                break;
            }
        }

        // 预测
        Eigen::MatrixXd X_ = m_F * m_X;
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        // if (m_debug)
        // {
        //     std::cout << "[StandardObserver] X_: " << std::endl << X_ << std::endl;
        //     std::cout << "[StandardObserver] P_: " << std::endl << P_ << std::endl;
        // }

        return std::make_pair(X_, P_);
    }

    /**
     * @brief 匹配装甲板序列和标准装甲板
     * @param armors 绝对坐标系装甲板序列
     * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
     * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
     * @param match 装甲板关联
     * @return 运动状态
     */
    std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵

        if (match.size() == 1)//匹配到一个装甲板(单源观测)
        {
            //获取匹配装甲板索引
            int num = match.begin()->first;
            //构建实际观测向量Z
            Z.resize(4, 1);
            Z << armors[num].position.x(), armors[num].position.y(), armors[num].position.z(), armors[num].rpy.z();
            //计算预测观测向量
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            //计算雅可比矩阵
            H.resize(4, 10);
            H << getMeasurementPD(X_, match[num]);
            //计算噪声转换矩阵
            V.resize(4, 4);
            V << getMeasureNoisePD(X_, match[num]);
            //动态调整观测噪声
            R = m_R;
            R(2, 2) *= m_gain * abs(Z(3, 0));
            //计算卡尔曼增益
            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else if (match.size() == 2)//匹配到两个装甲板(单源观测)
        {
            int num1 = match.begin()->first;
            int num2 = (++match.begin())->first;
            Z.resize(8, 1);
            Z << armors[num1].position.x(), armors[num1].position.y(), armors[num1].position.z(), armors[num1].rpy.z(),
                    armors[num2].position.x(), armors[num2].position.y(), armors[num2].position.z(), armors[num2].rpy.z();
            h.resize(8, 1);
            h << getPredictiveMeasurement(X_, match[num1]),
                    getPredictiveMeasurement(X_, match[num2]);
            H.resize(8, 10);
            H << getMeasurementPD(X_, match[num1]),
                    getMeasurementPD(X_, match[num2]);
            V.resize(8, 8);
            V << getMeasureNoisePD(X_, match[num1]), Eigen::MatrixXd::Zero(4, 4),
                    Eigen::MatrixXd::Zero(4, 4), getMeasureNoisePD(X_, match[num2]);
            R = m_RR;
            R(2, 2) *= m_gain * abs(Z(3, 0));
            R(6, 6) *= m_gain * abs(Z(7, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else//匹配到0个或过多装甲板(异常处理)
        {
            // 若匹配数目不为1或2，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<StandardModel>(m_X);
        }

        
        // std::cout << "[StandardObserver] Z: " << std::endl << Z << std::endl; // 实际观测值
        // std::cout << "[StandardObserver] h: " << std::endl << h << std::endl; // 预测观测值
        // std::cout << "[StandardObserver] H: " << std::endl << H << std::endl; // 观测矩阵

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        //角度残差归一化
        for (int i = 0; i < match.size(); i++)
        {
            tmp(3 + i * 4, 0) = _std_radian(tmp(3 + i * 4, 0));
        }
        m_X_update = K * tmp;
        // if (m_debug)
        // {
        //     std::cout << "[StandardObserver] X_update: " << std::endl << m_X_update << std::endl;
        // }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P_;

        // if (m_debug)
        // {
        //     std::cout << "[StandardObserver] X:" << std::endl << m_X << std::endl;
        //     std::cout << "[StandardObserver] P:" << std::endl << m_P << std::endl;
        // }

        return std::make_shared<StandardModel>(m_X);
    }

    /**
     * @brief 重置
     */
    void reset()
    {
        m_init = false;
        m_P.setIdentity();  
        m_P *= 1e-5;
    }

    /**
     * @brief 模型是否稳定
    */
    bool stable() {
        return true;
    }

    /***
     * @brief 获取当前卡尔曼滤波器计算的罪行状态向量的公共接口
     * @return 返回当前状态向量 
     */
    Eigen::VectorXd getState() const {
        return m_X;
    }


    /**
     * @brief 读取参数配置文件
     * @param file_path 配置文件路径
     */
    void setParam(const std::string &file_path)
    {
        try {
                // 1. 增强文件存在性检查
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
                // ROS_INFO("Loading parameters from: %s", file_path.c_str());
                
                // 2. 加载 YAML 文件并添加详细错误处理
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
                if (config["ModelObserver"]){
                    YAML::Node mod = config["ModelObserver"];
                    m_dt = mod["dt"].as<double>(0);
                    m_init_radius = mod["Standard"]["init_radius"].as<double>(0.2);
                    m_gain = mod["Standard"]["gain"].as<double>(15);
                    m_process_noise[0] = mod["Standard"]["process_noise"]["displace_high_diff"].as<double>(3);
                    m_process_noise[1] = mod["Standard"]["process_noise"]["anglar_high_diff"].as<double>(15);
                    m_process_noise[2] = mod["Standard"]["process_noise"]["height"].as<double>(0.0001);
                    m_process_noise[3] = mod["Standard"]["process_noise"]["radius"].as<double>(0.000005);
                    
                    m_measure_noise[0] = mod["Standard"]["measure_noise"]["pose"].as<double>(0);    // 目前先写零不知道写什么
                    m_measure_noise[1] = mod["Standard"]["measure_noise"]["distance"].as<double>(0);
                    m_measure_noise[2] = mod["Standard"]["measure_noise"]["angle"].as<double>(0);
                    // ROS_INFO("StandardObserver parameters loaded successfully"); // 这里输出调试先放着过后再补
                    // ROS_INFO("m_init_radius:%lf ",m_init_radius);
                    // ROS_INFO("m_gain:%lf ",m_gain);

                    // ROS_INFO("displace_high_diff:%lf ",m_process_noise[0]);
                    // ROS_INFO("anglar_high_diff:%lf ",m_process_noise[1]);
                    // ROS_INFO("height:%lf ",m_process_noise[2]);
                    // ROS_INFO("radius:%lf ",m_process_noise[3]);

                    // ROS_INFO("pose:%lf ",m_measure_noise[0]);
                    // ROS_INFO("distance:%lf ",m_measure_noise[1]);
                    // ROS_INFO("angle:%lf ",m_measure_noise[2]);
                } 
                else {
                    ROS_WARN("No 'ModelObserver' section in config file");
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
            // 这里应该是不是还要再补充一个对象释放

            m_X.resize(10, 1);
            m_X_update.resize(10, 1);
            m_P.resize(10, 10);
            m_F.resize(10, 10);
            m_Q.resize(10, 10);
            m_R.resize(4, 4);
            m_RR.resize(8, 8);      
    }

private:

    /**
     * @brief 获取先验观测量
     * @param X Eigen::Matrix<double, 10, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
     */
    Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(6 + i % 2, 0) * cos(X(8, 0) + i * PI / 2),//x坐标预测值
                X(2, 0) + X(6 + i % 2, 0) * sin(X(8, 0) + i * PI / 2),//y坐标预测值
                X(4 + i % 2, 0),//z坐标预测值
                X(8, 0) + i * PI / 2;//偏航角预测值
        return h;
    }

    /**
     * @brief 获取观测方程偏导矩阵
     * @param X Eigen::Matrix<double, 10, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
     */
    Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 10> H;
        //第0行:装甲板x坐标的偏导
        //第1行:装甲板y坐标的偏导
        //第2行:装甲板高度的偏导
        //第3行:装甲板偏航角的偏导
        H << 1, 0, 0, 0, 0,           0,     ((i + 1) % 2) * cos(X(8, 0) + i * PI / 2), (i % 2) * cos(X(8, 0) + i * PI / 2),-X(6 + i % 2, 0) * sin(X(8, 0) + i * PI / 2), 0,
                0, 0, 1, 0, 0,           0,     ((i + 1) % 2) * sin(X(8, 0) + i * PI / 2), (i % 2) * sin(X(8, 0) + i * PI / 2), X(6 + i % 2, 0) * cos(X(8, 0) + i * PI / 2), 0,
                0, 0, 0, 0, (i + 1) % 2, i % 2, 0,                                         0,                                   0,                                           0,
                0, 0, 0, 0, 0,           0,     0,                                         0,                                   1,                                           0;
        return H;
    }

    /**
     * @brief 获取观测噪声偏导矩阵
     * @param X Eigen::Matrix<double, 10, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
     */
    Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
        double psi = atan2(h(1, 0), h(0, 0));
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
        Eigen::Matrix<double, 4, 4> V;
        V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
                d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
                0,                        d * cos(phi),            sin(phi),            0,
                0,                        0,                       0,                   1;
        return V;
    }


private:
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_RR;         // 增广观测噪声
};

class BalanceObserver : public ModelObserver
{
public:
    BalanceObserver(const std::string &file_path)
    {
        setParam(file_path);

        reset();

        Eigen::VectorXd measurement_noise4(4);
        measurement_noise4 << m_measure_noise[0],
                                m_measure_noise[0],
                                m_measure_noise[1],
                                m_measure_noise[2];
        m_R = measurement_noise4.asDiagonal();
    }
    ~BalanceObserver() {}

    /**
     * @brief 先验预测
     * @param armors 绝对坐标系装甲板序列
     * @return std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>>(X_, P_)
     */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors)
    {
        if (!m_init)
        {
            for (auto armor : armors)
            {
                m_X << armor.position.x() - m_init_radius * cos(armor.rpy.z()),
                        armor.position.y() - m_init_radius * sin(armor.rpy.z()),
                        0,
                        armor.position.z(),
                        m_init_radius,
                        armor.rpy.z(),
                        0;
                m_init = true;
                break;
            }
        }

        // 预测
        Eigen::MatrixXd X_ = getTransformation(m_X);
        m_F = getTransformDP(m_X);
        m_Q = getTransformNoise(m_X);
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        // if (m_debug)
        // {
        //     std::cout << "[BalanceObserver] X_: " << std::endl << X_ << std::endl;
        //     // std::cout << "[BalanceObserver] P_: " << std::endl << P_ << std::endl;
        // }

        return std::make_pair(X_, P_);
    }

    /**
     * @brief 匹配装甲板序列和标准装甲板
     * @param armors 绝对坐标系装甲板序列
     * @param X_ Eigen::Matrix<double, 7, 1> 先验状态
     * @param P_ Eigen::Matrix<double, 7, 7> 先验状态协方差
     * @param match 装甲板关联
     * @return 运动状态
     */
    std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵

        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].position.x(), armors[num].position.y(), armors[num].position.z(), armors[num].rpy.z();
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            H.resize(4, 7);
            H << getMeasurementPD(X_, match[num]);
            V.resize(4, 4);
            V << getMeasureNoisePD(X_, match[num]);
            R = m_R;
            R(2, 2) *= m_gain * abs(Z(3, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else
        {
            // 若匹配数目不为1，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<BalanceModel>(m_X);
        }

      
        // std::cout << "[BalanceObserver] Z: " << std::endl << Z << std::endl;
        // std::cout << "[BalanceObserver] h: " << std::endl << h << std::endl;
        // std::cout << "[BalanceObserver] H: " << std::endl << H << std::endl;
       

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        tmp(3, 0) = _std_radian(tmp(3, 0));//角度归一化(避免+-Π跳变)
        m_X_update = K * tmp;
        // if (m_debug)
        // {
        //     // std::cout << "[BalanceObserver] X_update: " << std::endl << m_X_update << std::endl;
        // }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 7, 7>::Identity() - K * H) * P_;

        // if (m_debug)
        // {
        //     std::cout << "[BalanceObserver] X:" << std::endl << m_X << std::endl;
        //     // std::cout << "[BalanceObserver] P:" << std::endl << m_P << std::endl;
        // }

        //返回后验估计的状态向量
        return std::make_shared<BalanceModel>(m_X);
    }

    /**
     * @brief 重置
     */
    void reset()
    {
        m_init = false;
        m_P.setIdentity();  
        m_P *= 1e-5;
    }

    /**
     * @brief 模型是否稳定
    */
    bool stable() {
        return true;
    }

    /**
     * @brief 提供获取当前卡尔曼滤波器计算的最新状态向量的公共接口
     * @return 返回当前状态向量
     */
    Eigen::VectorXd getState() const {
        return m_X;
    }


    /**
     * @brief 读取参数配置文件
     * @param file_path 配置文件路径
     */
    void setParam(const std::string &file_path)
    {
        try {
            // 1. 增强文件存在性检查
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
            // ROS_INFO("Loading parameters from: %s", file_path.c_str());
            
            // 2. 加载 YAML 文件并添加详细错误处理
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
            if (config["ModelObserver"]){
                YAML::Node mod = config["ModelObserver"];
                m_dt = mod["dt"].as<double>(0);
                m_init_radius = mod["Balance"]["init_radius"].as<double>(0.2);
                m_gain = mod["Balance"]["gain"].as<double>(15);
                m_process_noise[0] = mod["Balance"]["process_noise"]["displace_high_diff"].as<double>(3);
                m_process_noise[1] = mod["Balance"]["process_noise"]["anglar_high_diff"].as<double>(15);
                m_process_noise[2] = mod["Balance"]["process_noise"]["height"].as<double>(0.0001);
                m_process_noise[3] = mod["Balance"]["process_noise"]["radius"].as<double>(0.000005);
                m_measure_noise[0] = mod["Balance"]["measure_noise"]["pose"].as<double>(0);    // 目前先写零不知道写什么
                m_measure_noise[1] = mod["Balance"]["measure_noise"]["distance"].as<double>(0);
                m_measure_noise[2] = mod["Balance"]["measure_noise"]["angle"].as<double>(0);
                // ROS_INFO("BalanceObserver parameters loaded successfully"); 
                // ROS_INFO("m_init_radius:%lf ",m_init_radius);
                // ROS_INFO("m_gain:%lf ",m_gain);
                // ROS_INFO("m_process_noise[0]:%lf ",m_process_noise[0]);
                // ROS_INFO("m_process_noise[1]:%lf ",m_process_noise[1]);
                // ROS_INFO("m_process_noise[2]:%lf ",m_process_noise[2]);
                // ROS_INFO("m_process_noise[3]:%lf ",m_process_noise[3]);

                // ROS_INFO("m_measure_noise[0]:%lf ",m_measure_noise[0]);
                // ROS_INFO("m_measure_noise[1]:%lf ",m_measure_noise[1]);
                // ROS_INFO("m_measure_noise[2]:%lf ",m_measure_noise[2]);
            } 
            else {
                ROS_WARN("No 'ModelObserver' section in config file");
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

        m_X.resize(7, 1);
        m_X_update.resize(7, 1);
        m_P.resize(7, 7);
        m_F.resize(7, 7);
        m_Q.resize(7, 7);
        m_R.resize(4, 4);     
    }

private:

    /**
     * @brief 获取先验状态
     * @param X Eigen::Matrix<double, 7, 1> 上一时刻状态
     * @return Eigen::Matrix<double, 7, 1> 先验状态
     */
    Eigen::MatrixXd getTransformation(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 1> X_;
        // if (X(6, 0) == 0)
        {
            X_ << X(0, 0) + X(2, 0) * cos(X(5, 0)) * m_dt,
                  X(1, 0) + X(2, 0) * sin(X(5, 0)) * m_dt,
                  X(2, 0),
                  X(3, 0),
                  X(4, 0),
                  X(5, 0) + X(6, 0) * m_dt,
                  X(6, 0);
        }
        // else
        // {
        //     X_ << X(0, 0) + X(2, 0) / X(6, 0) * (sin(X(6, 0) * m_dt) + X(5, 0) - sin(X(5, 0))),
        //           X(1, 0) - X(2, 0) / X(6, 0) * (cos(X(6, 0) * m_dt) - X(5, 0) + cos(X(5, 0))),
        //           X(2, 0),
        //           X(3, 0),
        //           X(4, 0),
        //           X(5, 0) + X(6, 0) * m_dt,
        //           X(6, 0);
        // }
        return X_;
    }

    /**
     * @brief 获取状态转移偏导矩阵
     * @param X Eigen::Matrix<double, 7, 1> 先验状态
     * @return Eigen::Matrix<double, 7, 7> 状态转移偏导矩阵
     */
    Eigen::MatrixXd getTransformDP(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 7> F;
        // if (X(6, 0) == 0)
        {
            F << 1, 0, cos(X(5, 0)) * m_dt, 0, 0,-X(2, 0) * sin(X(5, 0)) * m_dt, 0,
                 0, 1, sin(X(5, 0)) * m_dt, 0, 0, X(2, 0) * cos(X(5, 0)) * m_dt, 0,
                 0, 0, 1,                   0, 0, 0,                             0,
                 0, 0, 0,                   1, 0, 0,                             0,
                 0, 0, 0,                   0, 1, 0,                             0,
                 0, 0, 0,                   0, 0, 1,                             m_dt,
                 0, 0, 0,                   0, 0, 0,                             1;
        }
        // else
        // {
        //     double cos_theta = cos(X(5, 0));
        //     double cos_theta_t = cos(X(6, 0) * m_dt + X(5, 0));
        //     double sin_theta = sin(X(5, 0));
        //     double sin_theta_t = sin(X(6, 0) * m_dt + X(5, 0));
        //     F << 1, 0, 1 / X(6, 0) * (sin_theta_t - sin_theta), 0, 0, X(2, 0) / X(6, 0) * (cos_theta_t - cos_theta), X(2, 0) / X(6, 0) * cos_theta_t * m_dt - X(2, 0) / pow(X(6, 0), 2) * (sin_theta_t - sin_theta),
        //          0, 1,-1 / X(6, 0) * (cos_theta_t - cos_theta), 0, 0, X(2, 0) / X(6, 0) * (sin_theta_t - sin_theta), X(2, 0) / X(6, 0) * sin_theta_t * m_dt + X(2, 0) / pow(X(6, 0), 2) * (cos_theta_t - cos_theta),
        //          0, 0, 1,                                       0, 0, 0,                                             0,
        //          0, 0, 0,                                       1, 0, 0,                                             0,
        //          0, 0, 0,                                       0, 1, 0,                                             0,
        //          0, 0, 0,                                       0, 0, 1,                                             m_dt,
        //          0, 0, 0,                                       0, 0, 0,                                             1;
        // }
        return F; 
    }

    /**
     * @brief 获取状态转移噪声矩阵
     * @param X Eigen::Matrix<double, 7, 1> 先验状态
     * @return Eigen::Matrix<double, 7, 7> 状态转移噪声矩阵
     */
    Eigen::MatrixXd getTransformNoise(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 7> Q;
        double theta = X(5, 0);
        double v = X(2, 0);
        double dd = m_process_noise[0];
        double da = m_process_noise[1];
        double dz = m_process_noise[2];
        double dr = m_process_noise[3];
        double t5 = pow(m_dt, 5) / 5;
        double t4 = pow(m_dt, 4) / 4;
        double t3 = pow(m_dt, 3) / 3;
        double t2 = pow(m_dt, 2) / 2;
        double cos2 = pow(cos(theta), 2);
        double sin2 = pow(sin(theta), 2);
        double sincos = sin(theta) * cos(theta);
        Q << t5 * da * pow(v, 2) * sin2 / 4 + t3 * dd * cos2,         sin(theta) * t3 * dd - pow(v, 2) * sincos * t5 * da / 4, t2 * dd * cos(theta), 0,        0,        -t4 * da * v * sin(theta) / 2,-t3 * da * v * sin(theta) / 2,
             sin(theta) * t3 * dd - pow(v, 2) * sincos * t5 * da / 4, t3 * dd * sin2 + t5 * da * pow(v, 2) * cos2 / 4,         t2 * dd * sin(theta), 0,        0,         t4 * da * v * cos(theta) / 2, t3 * da * v * cos(theta) / 2,
             t2 * dd * cos(theta),                                    t2 * dd * sin(theta),                                    m_dt * dd,            0,        0,         0,                            0,
             0,                                                       0,                                                       0,                    m_dt *dz, 0,         0,                            0,
             0,                                                       0,                                                       0,                    0,        m_dt * dr, 0,                            0,
            -t4 * da * v * sin(theta) / 2,                            t4 * da * v * cos(theta) / 2,                            0,                    0,        0,         t3 * da,                      t2 * da,
            -t3 * da * v * sin(theta) / 2,                            t3 * da * v * cos(theta) / 2,                            0,                    0,        0,         t2 * da,                      m_dt * da;
        return Q;
    }

    /**
     * @brief 获取先验观测量
     * @param X Eigen::Matrix<double, 7, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
     */
    Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(4, 0) * cos(X(5, 0) + i * PI),
             X(1, 0) + X(4, 0) * sin(X(5, 0) + i * PI),
             X(3, 0),
             X(5, 0) + i * PI;
        return h;
    }

    /**
     * @brief 获取观测方程偏导矩阵
     * @param X Eigen::Matrix<double, 7, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 7> 观测偏导矩阵
     */
    Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 7> H;
        H << 1, 0, 0, 0, cos(X(5, 0) + i * PI),-X(4, 0) * sin(X(5, 0) + i * PI), 0,
             0, 1, 0, 0, sin(X(5, 0) + i * PI), X(4, 0) * cos(X(5, 0) + i * PI), 0,
             0, 0, 0, 1, 0,                     0,                               0,
             0, 0, 0, 0, 0,                     1,                               0;
        return H;
    }

    /**
     * @brief 获取观测噪声偏导矩阵
     * @param X Eigen::Matrix<double, 7, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
     */
    Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
        double psi = atan2(h(1, 0), h(0, 0));
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
        Eigen::Matrix<double, 4, 4> V;
        V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
              d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
              0,                        d * cos(phi),            sin(phi),            0,
              0,                        0,                       0,                   1;
        return V;
    }

private:
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移偏导矩阵
};

// 老前哨站代码: 单层前哨战
// class OutpostObserver : public ModelObserver
// {
// public:
//     OutpostObserver(const std::string &file_path)
//     {
//         setParam(file_path);
//         reset();
//     }
//     ~OutpostObserver() {}

//     /**
//      * @brief 先验预测
//      * @param armors 绝对坐标系装甲板序列
//      * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
//      */
//     std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors) 
//     {
//         if (!m_init)
//         {
//             // 西工大在这里过滤了白色装甲板,我删除了
//             for (auto armor : armors)
//             {
//                 m_X << armor.position.x() - m_init_radius * cos(armor.rpy.z()),
//                         armor.position.y() - m_init_radius * sin(armor.rpy.z()),
//                         armor.position.z(),
//                         armor.rpy.z(),
//                         0;
//                 m_init = true;
//                 break;
//             }
//         }

//         // 如果未识别到装甲板且转速较高时，以实际转速作为状态量
//         if (armors.size() == 0 && abs(m_X(4, 0)) > 0.8)
//         {
//             double palstance = m_X(4, 0);

//             std::cout << "pal: " << palstance << std::endl;

//             // 根据变换后角速度判断转速
//             if(abs(palstance) < 1.65)
//                 palstance = (palstance / abs(palstance)) * 1.25;
//             else if (abs(palstance) > 2.1)
//                 palstance = (palstance / abs(palstance)) * 2.5;
//             else
//                 palstance = m_X(4, 0);

//             std::cout << "pal_: " << palstance << std::endl;

//             m_X(4, 0) = palstance;
//         }

//         // 预测
//         Eigen::MatrixXd X_ = m_F * m_X;
//         Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        
//         // std::cout << "[OutpostObserver] X_: " << std::endl << X_ << std::endl;
//         // std::cout << "[OutpostObserver] P_: " << std::endl << P_ << std::endl;
     
//         return std::make_pair(X_, P_);
//     }

//     /**
//      * @brief 匹配装甲板序列和标准装甲板
//      * @param armors 绝对坐标系装甲板序列
//      * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
//      * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
//      * @param match 装甲板关联
//      * @return 运动状态
//      */
//     std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
//     {
//          // 观测
//         Eigen::MatrixXd Z;      // 实际观测量
//         Eigen::MatrixXd h;      // 先验观测量
//         Eigen::MatrixXd H;      // 观测方程偏导矩阵
//         Eigen::MatrixXd V;      // 观测噪声偏导矩阵
//         Eigen::MatrixXd K;      // 置信度权重矩阵
//         Eigen::MatrixXd R;      // 时变观测噪声矩阵

//         if (match.size() == 1)
//         {
//             int num = match.begin()->first;
//             Z.resize(4, 1);
//             Z << armors[num].position.x(), armors[num].position.y(), armors[num].position.z(), armors[num].rpy.z();
//             h.resize(4, 1);
//             h << getPredictiveMeasurement(X_, match[num]);
//             H.resize(4, 5);
//             H << getMeasurementPD(X_, match[num]);
//             V.resize(4, 4);
//             V << getMeasureNoisePD(X_, match[num]);
//             R = m_R;
//             R(2, 2) *= m_gain * abs(Z(3, 0));

//             K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
//         }
//         else
//         {
//             // 若匹配数目不为1，则认为丢识别或者误判，直接返回先验状态
//             m_X = X_;
//             m_P = P_;

//             return std::make_shared<OutpostModel>(m_X);
//         }


//         // std::cout << "[OutpostObserver] Z: " << std::endl << Z << std::endl;
//         // std::cout << "[OutpostObserver] h: " << std::endl << h << std::endl;
//         // std::cout << "[OutpostObserver] H: " << std::endl << H << std::endl;


//         // 更新
//         Eigen::MatrixXd tmp = Z - h;
//         tmp(3, 0) = _std_radian(tmp(3, 0));
//         m_X_update = K * tmp;
//         // if (m_debug)
//         // {
//         //     std::cout << "[OutpostObserver] X_update: " << std::endl << m_X_update << std::endl;
//         // }

//         m_X = X_ + m_X_update;
//         m_P = (Eigen::Matrix<double, 5, 5>::Identity() - K * H) * P_;

//         // if (m_debug)
//         // {
//         //     std::cout << "[OutpostObserver] X:" << std::endl << m_X << std::endl;
//         //     std::cout << "[OutpostObserver] P:" << std::endl << m_P << std::endl;
//         // }

//         return std::make_shared<OutpostModel>(m_X);
//     }

//     /**
//      * @brief 重置
//      */
//     void reset()
//     {
//         m_init = false;
//         m_P.setIdentity();  
//         m_P *= 1e-5;
//     }

//     /**
//      * @brief 模型是否稳定
//     */
//     bool stable() {
//         return true;
//     }

//     /***
//      * @brief 获取当前卡尔曼滤波器计算的罪行状态向量的公共接口
//      * @return 返回当前状态向量 
//      */
//     Eigen::VectorXd getState() const {
//         return m_X;
//     }

//     /**
//      * @brief 读取参数配置文件
//      * @param file_path 配置文件路径
//      */
//     void setParam(const std::string &file_path)
//     {
//     try {
//             // 1. 增强文件存在性检查
//             if (file_path.empty()) {
//                 ROS_ERROR("Empty config file path provided");
//                 return;
//             }
            
//             //2. 检查文件是否打开
//             std::ifstream file_check(file_path);
//             if (!file_check.is_open()) {
//                 ROS_ERROR("Failed to open config file: %s. Reason: %s", 
//                         file_path.c_str(), strerror(errno));
//                 return;
//             }
//             ROS_INFO("Loading parameters from: %s", file_path.c_str());
            
//             // 2. 加载 YAML 文件并添加详细错误处理
//             YAML::Node config;
//             try {
//                 config = YAML::LoadFile(file_path);
//             } catch (const YAML::BadFile& e) {
//                 ROS_ERROR("Bad YAML file: %s", e.what());
//                 return;
//             } catch (const YAML::ParserException& e) {
//                 ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
//                 return;
//             }
//             //加载参数
//             if (config["ModelObserver"]){
//                 YAML::Node mod = config["ModelObserver"];
//                 m_dt = mod["dt"].as<double>(0);
//                 m_init_radius = mod["Outpost"]["init_radius"].as<double>(0.2);
//                 m_gain = mod["Outpost"]["gain"].as<double>(15);
//                 m_process_noise[0] = mod["Outpost"]["process_noise"]["displace_high_diff"].as<double>(3);
//                 m_process_noise[1] = mod["Outpost"]["process_noise"]["anglar_high_diff"].as<double>(15);
//                 m_process_noise[2] = mod["Outpost"]["process_noise"]["height"].as<double>(0.0001);

//                 m_measure_noise[0] = mod["Outpost"]["measure_noise"]["pose"].as<double>(0);    // 目前先写零不知道写什么
//                 m_measure_noise[1] = mod["Outpost"]["measure_noise"]["distance"].as<double>(0);
//                 m_measure_noise[2] = mod["Outpost"]["measure_noise"]["angle"].as<double>(0);
//                 // ROS_INFO("OutpostObserver parameters loaded successfully"); 
//                 // ROS_INFO("m_init_radius:%lf ",m_init_radius);
//                 // ROS_INFO("m_gain:%lf ",m_gain);
//                 // ROS_INFO("m_process_noise[0]:%lf ",m_process_noise[0]);
//                 // ROS_INFO("m_process_noise[1]:%lf ",m_process_noise[1]);
//                 // ROS_INFO("m_process_noise[2]:%lf ",m_process_noise[2]);

//                 // ROS_INFO("m_measure_noise[0]:%lf ",m_measure_noise[0]);
//                 // ROS_INFO("m_measure_noise[1]:%lf ",m_measure_noise[1]);
//                 // ROS_INFO("m_measure_noise[2]:%lf ",m_measure_noise[2]);
//             } 
//             else {
//                 ROS_WARN("No 'ModelObserver' section in config file");
//             }
//         } 
//         catch (const YAML::Exception& e) {
//             ROS_ERROR("YAML parsing error: %s", e.what());
//         } 
//         catch (const std::exception& e) {
//             ROS_ERROR("Error loading parameters: %s", e.what());
//         } 
//         catch (...) {
//             ROS_ERROR("Unknown error occurred while loading config file");
//         }

//         m_X.resize(5, 1);
//         m_X_update.resize(5, 1);
//         m_P.resize(5, 5);
//         m_F.resize(5, 5);
//         m_Q.resize(5, 5);
//         m_R.resize(4, 4);
//     }

// private:

//     /**
//      * @brief 获取先验观测量
//      * @param X Eigen::Matrix<double, 10, 1> 先验状态
//      * @param i 标准装甲板索引
//      * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
//      */
//     Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
//     {
//         Eigen::Matrix<double, 4, 1> h;
//         h << X(0, 0) + 0.25685 * cos(X(3, 0) + 2 * i * PI / 3),
//                 X(1, 0) + 0.25685 * sin(X(3, 0) + 2 * i * PI / 3),
//                 X(2, 0),
//                 X(3, 0) + 2 * i * PI / 3;
//         return h;
//     }

//     /**
//      * @brief 获取观测方程偏导矩阵
//      * @param X Eigen::Matrix<double, 10, 1> 先验状态
//      * @param i 标准装甲板索引
//      * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
//      */
//     Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i)
//     {
//         Eigen::Matrix<double, 4, 5> H;
//         H << 1, 0, 0, -0.25685 * sin(X(3, 0) + 2 * i * PI / 3), 0,
//                 0, 1, 0,  0.25685 * cos(X(3, 0) + 2 * i * PI / 3), 0,
//                 0, 0, 1,                                        0, 0,
//                 0, 0, 0,                                        1, 0;
//         return H;
//     }

//     /**
//      * @brief 获取观测噪声偏导矩阵
//      * @param X Eigen::Matrix<double, 10, 1> 先验状态
//      * @param i 标准装甲板索引
//      * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
//      */
//     Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
//     {
//         Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
//         double psi = atan2(h(1, 0), h(0, 0));
//         double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
//         double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
//         Eigen::Matrix<double, 4, 4> V;
//         V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
//                 d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
//                 0,                        d * cos(phi),            sin(phi),            0,
//                 0,                        0,                       0,                   1;
//         return V;
//     }
    
//     double m_process_noise[3];
// };



// class OutpostObserver : public ModelObserver
// {
// public:
//     OutpostObserver(const std::string &file_path)
//     {
//         setParam(file_path);

//         reset();

//         // m_F << 1, 0, 0, 0, 0,
//         //        0, 1, 0, 0, 0,
//         //        0, 0, 1, 0, 0,
//         //        0, 0, 0, 1, m_dt,
//         //        0, 0, 0, 0, 1;

//         // 状态转移矩阵(7*7)
//         m_F << 1, 0, 0, 0, 0, 0, 0,       // 中心x
//                0, 1, 0, 0, 0, 0, 0,       // 中心y
//                0, 0, 1, 0, 0, 0, 0,       // h1
//                0, 0, 0, 1, 0, 0, 0,       // h2
//                0, 0, 0, 0, 1, 0, 0,       // h3
//                0, 0, 0, 0, 0, 1, m_dt,    // 偏航角
//                0, 0, 0, 0, 0, 0, 1;       // 角速度

//         // 噪声
//         double dd = m_process_noise[0];     // 位移噪声
//         double da = m_process_noise[1];     // 角度噪声
//         double dz = m_process_noise[2];     // 高度噪声

//         // 时间项计算
//         double t4 = pow(m_dt, 3) / 3;   // dt³/3
//         double t3 = pow(m_dt, 2) / 2;   // dt²/2 
//         double t2 = pow(m_dt, 1);       // dt

//         // 过程噪声协方差矩阵m_Q(5*5)
//         // m_Q << dd,      0,       0, 0,       0,         // 位置噪声(x,y方向)(对角线: 表示位置预测的不确定性)
//         //        0,       dd,      0, 0,       0,         // 位置零非对角线: 假设位置预测的不确定性
//         //        0,       0,       dz,0,       0,         // 高度噪声(z方向)
//         //        0,       0,       0, t4 * da, t3 * da,   // 角度和角速度噪声
//         //        0,       0,       0, t3 * da, t2 * da;

//         // 过程噪声协方差矩阵m_Q(7*7)
//         m_Q << dd,      0,       0,  0,        0,       0,        0,             // 中心x噪声 // 位置噪声(x,y方向)(对角线: 表示位置预测的不确定性) // 前哨战中心不动
//                0,       dd,      0,  0,        0,       0,        0,             // 中心y噪声 // 位置零非对角线: 假设位置预测的不确定性
//                0,       0,       dz, 0,        0,       0,        0,             // h1噪声   // 第1块装甲板高度噪声(z方向)
//                0,       0,       0,  dz,       0,       0,        0,             // h2噪声   // 第2块装甲板高度噪声(z方向)
//                0,       0,       0,  0,        dz,      0,        0,             // h3噪声   // 第3块装甲板高度噪声(z方向)
//                0,       0,       0,  0,        0,       t4 * da,  t3 * da,       // 角度和角速度噪声(融合噪声)
//                0,       0,       0,  0,        0,       t3 * da,  t2 * da; 


//         Eigen::VectorXd measurement_noise4(4);
//         measurement_noise4 << m_measure_noise[0],   // xy平面位置噪声
//                               m_measure_noise[0],   // xy平面位置噪声
//                               m_measure_noise[1],   // 高度z轴噪声
//                               m_measure_noise[2];   // 偏航角(Yaw)噪声
//         m_R = measurement_noise4.asDiagonal();
//         Eigen::VectorXd measurement_noise8(8);
//     }
//     ~OutpostObserver() {}

//     /**
//      * @brief 先验预测
//      * @param armors 绝对坐标系装甲板序列
//      * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
//      */
//     std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors) 
//     {
//         // 老前哨战
//         // m_X状态转移矩阵
//         // 第0行: 前哨战中心x
//         // 第1行: 前哨战中心y
//         // 第2行: 前哨战中心z
//         // 第3行: 前哨站yaw
//         // 第4行: 前哨战旋转角度palstance

//         // 新前哨站
//         // 第0行: 中心x
//         // 第1行: 中心y
//         // 第2行: h1
//         // 第3行: h2
//         // 第4行: h3
//         // 第5行: 偏航角yaw
//         // 第6行: 角速度palstance

//         if (!m_init)
//         {
//             // 西工大在这里过滤了白色装甲板,我删除了
//             for (auto armor : armors)
//             {
//                 m_X << armor.position.x() - m_init_radius * cos(armor.rpy.z()),     // 中心x坐标 
//                         armor.position.y() - m_init_radius * sin(armor.rpy.z()),    // 中心y坐标
//                         armor.position.z(),                                         // h1
//                         armor.position.z(),                                         // h2
//                         armor.position.z(),                                         // h3
//                         armor.rpy.z(),      // yaw
//                         0;                  // palstance
//                 m_init = true;
//                 break;
//             }
//         }

//         // 如果未识别到装甲板且转速较高时，以实际转速作为状态量
//         // 当前帧没有检测到任何装甲板,滤波器估计的角速度绝对值大于0.8rad/s
//         // 这段丢失视觉检测时对角速度进行经验性修正
//         if (armors.size() == 0 && abs(m_X(6, 0)) > 0.8)
//         {
//             double palstance = m_X(6, 0);

//             std::cout << "pal: " << palstance << std::endl;

//             // 根据变换后角速度判断转速
//             if(abs(palstance) < 1.65)
//                 palstance = (palstance / abs(palstance)) * 1.25;
//             else if (abs(palstance) > 2.1)
//                 palstance = (palstance / abs(palstance)) * 2.5;
//             else
//                 palstance = m_X(6, 0);

//             std::cout << "pal_: " << palstance << std::endl;

//             m_X(6, 0) = palstance;
//         }

        
//         // 预测
//         Eigen::MatrixXd X_ = m_F * m_X;
//         Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;

        
        
//         // std::cout << "[OutpostObserver] X_: " << std::endl << X_ << std::endl;
//         // std::cout << "[OutpostObserver] P_: " << std::endl << P_ << std::endl;
     
//         return std::make_pair(X_, P_);
//     }

//     /**
//      * @brief 匹配装甲板序列和标准装甲板
//      * @param armors 绝对坐标系装甲板序列
//      * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
//      * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
//      * @param match 装甲板关联
//      * @return 运动状态
//      */
//     std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
//     {
       

//          // 观测
//         Eigen::MatrixXd Z;      // 实际观测量
//         Eigen::MatrixXd h;      // 先验观测量
//         Eigen::MatrixXd H;      // 观测方程偏导矩阵
//         Eigen::MatrixXd V;      // 观测噪声偏导矩阵
//         Eigen::MatrixXd K;      // 置信度权重矩阵
//         Eigen::MatrixXd R;      // 时变观测噪声矩阵

//         if (match.size() == 1)
//         {
//             // 根据匹配结果的装甲板索引,构建实际观测向量Z
//             // 获取匹配装甲板索引
//             int num = match.begin()->first;
//             // 实际观测向量Z = [x,y,z,yaw]
//             Z.resize(4, 1);
//             Z << armors[num].position.x(), armors[num].position.y(), armors[num].position.z(), armors[num].rpy.z();
            
//             // 先验观测量
//             h.resize(4, 1);
//             h << getPredictiveMeasurement(X_, match[num]);
            
//             // 雅可比矩阵计算
//             H.resize(4, 7);
//             H << getMeasurementPD(X_, match[num]);  // 观测函数对状态的偏导矩阵
//             V.resize(4, 4); 
//             V << getMeasureNoisePD(X_, match[num]); // 观测噪声对状态的偏导矩阵

//             // 自适应噪声调整
//             R = m_R;
//             R(2, 2) *= m_gain * abs(Z(3, 0));

//             // 卡尔曼增益计算
//             K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
//         }
//         else
//         {
//             // 若匹配数目不为1，则认为丢识别或者误判，直接返回先验状态
//             m_X = X_;
//             m_P = P_;

//             return std::make_shared<OutpostModel>(m_X);
//         }


//         // std::cout << "[OutpostObserver] Z: " << std::endl << Z << std::endl;
//         // std::cout << "[OutpostObserver] h: " << std::endl << h << std::endl;
//         // std::cout << "[OutpostObserver] H: " << std::endl << H << std::endl;



//         // 更新
//         Eigen::MatrixXd tmp = Z - h;
//         tmp(3, 0) = _std_radian(tmp(3, 0));
//         m_X_update = K * tmp;
//         // if (m_debug)
//         // {
//         //     std::cout << "[OutpostObserver] X_update: " << std::endl << m_X_update << std::endl;
//         // }

//         m_X = X_ + m_X_update;
//         m_P = (Eigen::Matrix<double, 7, 7>::Identity() - K * H) * P_;

//         // if (m_debug)
//         // {
//         //     std::cout << "[OutpostObserver] X:" << std::endl << m_X << std::endl;
//         //     std::cout << "[OutpostObserver] P:" << std::endl << m_P << std::endl;
//         // }

//         return std::make_shared<OutpostModel>(m_X);
//     }

//     /**
//      * @brief 重置
//      */
//     void reset()
//     {
//         m_init = false;
//         // m_P = Eigen::Matrix<double, 7, 7>::Identity() * 1e-5;
//         m_P.setIdentity();  
//         m_P *= 1e-5;
//     }

//     /**
//      * @brief 模型是否稳定
//     */
//     bool stable() {
//         return true;
//     }

//     /***
//      * @brief 获取当前卡尔曼滤波器计算的罪行状态向量的公共接口
//      * @return 返回当前状态向量 
//      */
//     Eigen::VectorXd getState() const {
//         return m_X;
//     }

//     /**
//      * @brief 读取参数配置文件
//      * @param file_path 配置文件路径
//      */
//     void setParam(const std::string &file_path)
//     {
//     try {
//             // 1. 增强文件存在性检查
//             if (file_path.empty()) {
//                 ROS_ERROR("Empty config file path provided");
//                 return;
//             }
            
//             //2. 检查文件是否打开
//             std::ifstream file_check(file_path);
//             if (!file_check.is_open()) {
//                 ROS_ERROR("Failed to open config file: %s. Reason: %s", 
//                         file_path.c_str(), strerror(errno));
//                 return;
//             }
//             // ROS_INFO("Loading parameters from: %s", file_path.c_str());
            
//             // 2. 加载 YAML 文件并添加详细错误处理
//             YAML::Node config;
//             try {
//                 config = YAML::LoadFile(file_path);
//             } catch (const YAML::BadFile& e) {
//                 ROS_ERROR("Bad YAML file: %s", e.what());
//                 return;
//             } catch (const YAML::ParserException& e) {
//                 ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
//                 return;
//             }
//             //加载参数
//             if (config["ModelObserver"]){
//                 YAML::Node mod = config["ModelObserver"];
//                 m_dt = mod["dt"].as<double>(0.025);
//                 m_gain = mod["Outpost"]["gain"].as<double>(100);
//                 m_init_radius = mod["Outpost"]["init_radius"].as<double>(0.265);
//                 m_process_noise[0] = mod["Outpost"]["process_noise"]["displace_high_diff"].as<double>(0.0001);
//                 m_process_noise[1] = mod["Outpost"]["process_noise"]["anglar_high_diff"].as<double>(0.4);
//                 m_process_noise[2] = mod["Outpost"]["process_noise"]["height"].as<double>(0.0001);

//                 m_measure_noise[0] = mod["Outpost"]["measure_noise"]["pose"].as<double>(0.002);
//                 m_measure_noise[1] = mod["Outpost"]["measure_noise"]["distance"].as<double>(0.04);
//                 m_measure_noise[2] = mod["Outpost"]["measure_noise"]["angle"].as<double>(0.1);

//                 // ROS_WARN("OutpostObserver parameters loaded successfully"); 
//                 // ROS_WARN("m_init_radius:%lf ",m_init_radius);
//                 // ROS_WARN("m_gain:%lf ",m_gain);
//                 // ROS_WARN("m_process_noise[0]:%lf ",m_process_noise[0]);
//                 // ROS_WARN("m_process_noise[1]:%lf ",m_process_noise[1]);
//                 // ROS_WARN("m_process_noise[2]:%lf ",m_process_noise[2]);
//                 // ROS_WARN("m_measure_noise[0]:%lf ",m_measure_noise[0]);
//                 // ROS_WARN("m_measure_noise[1]:%lf ",m_measure_noise[1]);
//                 // ROS_WARN("m_measure_noise[2]:%lf ",m_measure_noise[2]);
//             } 
//             else {
//                 ROS_WARN("No 'ModelObserver' section in config file");
//             }
//         } 
//         catch (const YAML::Exception& e) {
//             ROS_ERROR("YAML parsing error: %s", e.what());
//         } 
//         catch (const std::exception& e) {
//             ROS_ERROR("Error loading parameters: %s", e.what());
//         } 
//         catch (...) {
//             ROS_ERROR("Unknown error occurred while loading config file");
//         }

//         m_X.resize(7, 1);
//         m_X_update.resize(7, 1);
//         m_P.resize(7, 7);  
//         m_F.resize(7, 7);
//         m_Q.resize(7, 7);
//         m_R.resize(4, 4);
//     }

// private:

//     /**
//      * @brief 获取先验观测量
//      * @param X Eigen::Matrix<double, 10, 1> 先验状态
//      * @param i 标准装甲板索引
//      * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
//      */

//     // 观测预测函数,它根据当前状态估计来预测应该观测到的装甲板数据
//     Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
//     {
        
//         // m_X状态转移矩阵
//         // 第0行: 前哨战中心x
//         // 第1行: 前哨战中心y
//         // 第2行: 前哨战中心z
//         // 第3行: 前哨站yaw
//         // 第4行: 前哨战旋转角度palstance
//         // Eigen::Matrix<double, 4, 1> h;
//         // h << X(0, 0) + 0.25685 * cos(X(3, 0) + 2 * i * PI / 3),     // 装甲板x坐标
//         //         X(1, 0) + 0.25685 * sin(X(3, 0) + 2 * i * PI / 3),  // 装甲板y坐标
//         //         X(2, 0),                                            // 装甲板z坐标
//         //         X(3, 0) + 2 * i * PI / 3;                           // 装甲板yaw
//         // return h;

//         Eigen::Matrix<double, 4, 1> h;
//         h <<    X(0, 0) + m_init_radius * cos(X(5, 0) + 2 * i * PI / 3),  // 装甲板的x坐标
//                 X(1, 0) + m_init_radius * sin(X(5, 0) + 2 * i * PI / 3),  // 装甲板的y坐标
//                 X(2 + i, 0),                                        // 装甲板z坐标
//                 X(5, 0) + 2 * i * PI / 3;                           // 装甲板yaw
//         return h;
//     }

//     /**
//      * @brief 获取观测方程偏导矩阵
//      * @param X Eigen::Matrix<double, 10, 1> 先验状态
//      * @param i 标准装甲板索引
//      * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
//      */
//     // 观测矩阵雅可比计算函数,用于线性化非线性观测模型
//     Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i)
//     {
//         // Eigen::Matrix<double, 4, 5> H;
//         // H << 1, 0, 0, -0.25685 * sin(X(3, 0) + 2 * i * PI / 3), 0,      // 装甲板x坐标对状态的偏导
//         //         0, 1, 0,  0.25685 * cos(X(3, 0) + 2 * i * PI / 3), 0,   // 装甲板y坐标对状态的偏导
//         //         0, 0, 1,                                        0, 0,   // 装甲板z坐标对状态的偏导
//         //         0, 0, 0,                                        1, 0;   // 装甲板朝向对状态的偏导
//         // return H;

//         Eigen::Matrix<double, 4, 7> H;
//         H << 1, 0,  0,               0,               0,                 -m_init_radius * sin(X(5, 0) + 2 * i * PI / 3), 0,  // 装甲板x坐标对状态的偏导
//              0, 1,  0,               0,               0,                  m_init_radius * cos(X(5, 0) + 2 * i * PI / 3), 0,  // 装甲板y坐标对状态的偏导
//              0, 0,  (i==0) ? 1 : 0,  (i==1) ? 1 : 0,  (i == 2) ? 1 : 0,   0,                                       0,  // 装甲板z坐标对状态的偏导
//              0, 0,  0,               0,               0,                  1,                                       0;  // 装甲板yaw对状态的偏导
//         return H;
//     }

//     /**
//      * @brief 获取观测噪声偏导矩阵
//      * @param X Eigen::Matrix<double, 10, 1> 先验状态
//      * @param i 标准装甲板索引
//      * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
//      */
//     // 从球坐标系到笛卡尔坐标系的观测噪声转换雅可比矩阵
//     Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
//     {
//         // 1.获取预测的装甲板位置
//         Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);

//         // 2.计算球半径参数
//         double psi = atan2(h(1, 0), h(0, 0));                                   // 方位角
//         double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));   // 俯仰角
//         double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));   // 距离

//         // 3.构建噪声转换矩阵
//         Eigen::Matrix<double, 4, 4> V;
//         V <<   -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,     // x坐标的噪声转换(x坐标对球坐标参数的敏感度)
//                 d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,     // y坐标的噪声转换(y坐标对球坐标参数的敏感度)
//                 0,                        d * cos(phi),            sin(phi),            0,     // z坐标的噪声转换(高度对球坐标的敏感度)
//                 0,                        0,                       0,                   1;     // 朝向角的噪声转换
//         return V;
//     }
    
//     double m_process_noise[3];
// };



/*
    前哨战卡尔曼状态矩阵
    X(0): 机器人中心的x坐标
    X(1): 机器人中心的y坐标
    X(2): 第一块装甲板高度h1
    X(3): 第二块装甲板高度h2
    X(4): 第三块装甲板高度h3
    X(5): 第一块装甲板半径r1
    X(6): 第二块装甲板半径r2
    X(7): 第三块装甲板半径r3
    X(8): 偏航角yaw
    X(9): 偏航角速度palstance
*/

class OutpostObserver : public ModelObserver
{
public:
    OutpostObserver(const std::string &file_path)
    {
        setParam(file_path);

        reset();

        // m_F << 1, 0, 0, 0, 0,
        //        0, 1, 0, 0, 0,
        //        0, 0, 1, 0, 0,
        //        0, 0, 0, 1, m_dt,
        //        0, 0, 0, 0, 1;

        // // 状态转移矩阵(7*7)
        // m_F << 1, 0, 0, 0, 0, 0, 0,       // 中心x
        //        0, 1, 0, 0, 0, 0, 0,       // 中心y
        //        0, 0, 1, 0, 0, 0, 0,       // h1
        //        0, 0, 0, 1, 0, 0, 0,       // h2
        //        0, 0, 0, 0, 1, 0, 0,       // h3
        //        0, 0, 0, 0, 0, 1, m_dt,    // 偏航角
        //        0, 0, 0, 0, 0, 0, 1;       // 角速度

        // 状态转移矩阵(10 * 10)
        m_F << 1, 0, 0, 0, 0, 0, 0, 0,0,0,      // 中心x
               0, 1, 0, 0, 0, 0, 0, 0,0,0,      // 中心y
               0, 0, 1, 0, 0, 0, 0, 0,0,0,      // h1
               0, 0, 0, 1, 0, 0, 0, 0,0,0,      // h2
               0, 0, 0, 0, 1, 0, 0, 0,0,0,      // h3
               0, 0, 0, 0, 0, 1, 0, 0,0,0,      // r1
               0, 0, 0, 0, 0, 0, 1, 0,0,0,      // r2
               0, 0, 0, 0, 0, 0, 0, 1,0,0,      // r3
               0, 0, 0, 0, 0, 0, 0, 0,1,m_dt,   // 偏航角
               0, 0, 0, 0, 0, 0, 0, 0,0,1;      // 角速度

        // 噪声
        double dd = m_process_noise[0];     // 位移噪声
        double da = m_process_noise[1];     // 角度噪声
        double dz = m_process_noise[2];     // 高度噪声
        double dr = m_process_noise[3];     // 半径噪声


        // 时间项计算
        double t4 = pow(m_dt, 3) / 3;   // dt³/3
        double t3 = pow(m_dt, 2) / 2;   // dt²/2 
        double t2 = pow(m_dt, 1);       // dt

        // 过程噪声协方差矩阵m_Q(5*5)
        // m_Q << dd,      0,       0, 0,       0,         // 位置噪声(x,y方向)(对角线: 表示位置预测的不确定性)
        //        0,       dd,      0, 0,       0,         // 位置零非对角线: 假设位置预测的不确定性
        //        0,       0,       dz,0,       0,         // 高度噪声(z方向)
        //        0,       0,       0, t4 * da, t3 * da,   // 角度和角速度噪声
        //        0,       0,       0, t3 * da, t2 * da;

        // // 过程噪声协方差矩阵m_Q(7*7)
        // m_Q << dd,      0,       0,  0,        0,       0,        0,             // 中心x噪声 // 位置噪声(x,y方向)(对角线: 表示位置预测的不确定性) // 前哨战中心不动
        //        0,       dd,      0,  0,        0,       0,        0,             // 中心y噪声 // 位置零非对角线: 假设位置预测的不确定性
        //        0,       0,       dz, 0,        0,       0,        0,             // h1噪声   // 第1块装甲板高度噪声(z方向)
        //        0,       0,       0,  dz,       0,       0,        0,             // h2噪声   // 第2块装甲板高度噪声(z方向)
        //        0,       0,       0,  0,        dz,      0,        0,             // h3噪声   // 第3块装甲板高度噪声(z方向)
        //        0,       0,       0,  0,        0,       t4 * da,  t3 * da,       // 角度和角速度噪声(融合噪声)
        //        0,       0,       0,  0,        0,       t3 * da,  t2 * da; 


        // 过程噪声协方差矩阵m_Q(10*10)
        m_Q << dd,      0,       0,  0,        0,       0,        0,        0,       0,        0,            // 中心x噪声 // 位置噪声(x,y方向)(对角线: 表示位置预测的不确定性) // 前哨战中心不动
               0,       dd,      0,  0,        0,       0,        0,        0,       0,        0,            // 中心y噪声 // 位置零非对角线: 假设位置预测的不确定性
               0,       0,       dz, 0,        0,       0,        0,        0,       0,        0,            // h1噪声   // 第1块装甲板高度噪声(z方向)
               0,       0,       0,  dz,       0,       0,        0,        0,       0,        0,            // h2噪声   // 第2块装甲板高度噪声(z方向)
               0,       0,       0,  0,        dz,      0,        0,        0,       0,        0,            // h3噪声   // 第3块装甲板高度噪声(z方向)
               0,       0,       0,  0,        0,       dr,       0,        0,       0,        0,            // r1噪声   // 第一块装甲板半径噪声
               0,       0,       0,  0,        0,       0,        dr,       0,       0,        0,            // r2噪声   // 第二块装甲板半径噪声
               0,       0,       0,  0,        0,       0,        0,        dr,      0,        0,            // r3噪声   // 第三块装甲板半径噪声
               0,       0,       0,  0,        0,       0,        0,        0,       t4 * da,  t3 * da,      // 角度和角速度噪声(融合噪声)
               0,       0,       0,  0,        0,       0,        0,        0,       t3 * da,  t2 * da; 


        Eigen::VectorXd measurement_noise4(4);
        measurement_noise4 << m_measure_noise[0],   // xy平面位置噪声
                              m_measure_noise[0],   // xy平面位置噪声
                              m_measure_noise[1],   // 高度z轴噪声
                              m_measure_noise[2];   // 偏航角(Yaw)噪声
        m_R = measurement_noise4.asDiagonal();
        Eigen::VectorXd measurement_noise8(8);
    }
    ~OutpostObserver() {}

    /**
     * @brief 先验预测
     * @param armors 绝对坐标系装甲板序列
     * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
     */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors) 
    {
        // 新前哨站
        // 第0行: 中心x
        // 第1行: 中心y
        // 第2行: h1
        // 第3行: h2
        // 第4行: h3
        // 第5行: r1
        // 第6行: r2
        // 第7行: r3
        // 第8行: 偏航角yaw
        // 第9行: 角速度palstance

        if (!m_init)
        {
            // 西工大在这里过滤了白色装甲板,我删除了
            for (auto armor : armors)
            {
                m_X << armor.position.x() - m_init_radius * cos(armor.rpy.z()),     // 中心x坐标 
                        armor.position.y() - m_init_radius * sin(armor.rpy.z()),    // 中心y坐标
                        armor.position.z(),                                         // h1
                        armor.position.z(),                                         // h2
                        armor.position.z(),                                         // h3
                        m_init_radius,                                              // r1
                        m_init_radius,                                              // r2
                        m_init_radius,                                              // r3
                        armor.rpy.z(),                                              // yaw
                        0;                                                          // palstance
                m_init = true;
                break;
            }
        }

        
        // 预测
        Eigen::MatrixXd X_ = m_F * m_X;
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;

        
        
        // std::cout << "[OutpostObserver] X_: " << std::endl << X_ << std::endl;
        // std::cout << "[OutpostObserver] P_: " << std::endl << P_ << std::endl;
     
        return std::make_pair(X_, P_);
    }

    /**
     * @brief 匹配装甲板序列和标准装甲板
     * @param armors 绝对坐标系装甲板序列
     * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
     * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
     * @param match 装甲板关联
     * @return 运动状态
     */
    std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
       

         // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵

        if (match.size() == 1)
        {
            // 根据匹配结果的装甲板索引,构建实际观测向量Z
            // 获取匹配装甲板索引
            int num = match.begin()->first;
            // 实际观测向量Z = [x,y,z,yaw]
            Z.resize(4, 1);
            Z << armors[num].position.x(), armors[num].position.y(), armors[num].position.z(), armors[num].rpy.z();
            
            // 先验观测量
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            
            // 雅可比矩阵计算
            H.resize(4, 10);
            H << getMeasurementPD(X_, match[num]);  // 观测函数对状态的偏导矩阵
            V.resize(4, 4); 
            V << getMeasureNoisePD(X_, match[num]); // 观测噪声对状态的偏导矩阵

            // 自适应噪声调整
            R = m_R;
            R(2, 2) *= m_gain * abs(Z(3, 0));

            // 卡尔曼增益计算
            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else
        {
            // 若匹配数目不为1，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<OutpostModel>(m_X);
        }


        // std::cout << "[OutpostObserver] Z: " << std::endl << Z << std::endl;
        // std::cout << "[OutpostObserver] h: " << std::endl << h << std::endl;
        // std::cout << "[OutpostObserver] H: " << std::endl << H << std::endl;



        // 更新
        Eigen::MatrixXd tmp = Z - h;
        tmp(3, 0) = _std_radian(tmp(3, 0));
        m_X_update = K * tmp;
        // if (m_debug)
        // {
        //     std::cout << "[OutpostObserver] X_update: " << std::endl << m_X_update << std::endl;
        // }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P_;

        // if (m_debug)
        // {
        //     std::cout << "[OutpostObserver] X:" << std::endl << m_X << std::endl;
        //     std::cout << "[OutpostObserver] P:" << std::endl << m_P << std::endl;
        // }

        return std::make_shared<OutpostModel>(m_X);
    }

    /**
     * @brief 重置
     */
    void reset()
    {
        m_init = false;
        // m_P = Eigen::Matrix<double, 7, 7>::Identity() * 1e-5;
        m_P.setIdentity();  
        m_P *= 1e-5;
    }

    /**
     * @brief 模型是否稳定
    */
    bool stable() {
        return true;
    }

    /***
     * @brief 获取当前卡尔曼滤波器计算的罪行状态向量的公共接口
     * @return 返回当前状态向量 
     */
    Eigen::VectorXd getState() const {
        return m_X;
    }

    /**
     * @brief 读取参数配置文件
     * @param file_path 配置文件路径
     */
    void setParam(const std::string &file_path)
    {
    try {
            // 1. 增强文件存在性检查
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
            // ROS_INFO("Loading parameters from: %s", file_path.c_str());
            
            // 2. 加载 YAML 文件并添加详细错误处理
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
            if (config["ModelObserver"]){
                YAML::Node mod = config["ModelObserver"];
                m_dt = mod["dt"].as<double>(0.025);
                m_gain = mod["Outpost"]["gain"].as<double>(100);
                m_init_radius = mod["Outpost"]["init_radius"].as<double>(0.275);
                m_process_noise[0] = mod["Outpost"]["process_noise"]["displace_high_diff"].as<double>(0.0001);
                m_process_noise[1] = mod["Outpost"]["process_noise"]["anglar_high_diff"].as<double>(0.4);
                m_process_noise[2] = mod["Outpost"]["process_noise"]["height"].as<double>(0.0001);
                m_process_noise[3] = mod["Outpost"]["process_noise"]["radius"].as<double>(0.0005);

                m_measure_noise[0] = mod["Outpost"]["measure_noise"]["pose"].as<double>(0.002);
                m_measure_noise[1] = mod["Outpost"]["measure_noise"]["distance"].as<double>(0.04);
                m_measure_noise[2] = mod["Outpost"]["measure_noise"]["angle"].as<double>(0.1);

                // ROS_WARN("OutpostObserver parameters loaded successfully"); 
                // ROS_WARN("m_init_radius:%lf ",m_init_radius);
                // ROS_WARN("m_gain:%lf ",m_gain);
                // ROS_WARN("m_process_noise[0]:%lf ",m_process_noise[0]);
                // ROS_WARN("m_process_noise[1]:%lf ",m_process_noise[1]);
                // ROS_WARN("m_process_noise[2]:%lf ",m_process_noise[2]);
                // ROS_WARN("m_measure_noise[0]:%lf ",m_measure_noise[0]);
                // ROS_WARN("m_measure_noise[1]:%lf ",m_measure_noise[1]);
                // ROS_WARN("m_measure_noise[2]:%lf ",m_measure_noise[2]);
            } 
            else {
                ROS_WARN("No 'ModelObserver' section in config file");
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

        m_X.resize(10, 1);
        m_X_update.resize(10, 1);
        m_P.resize(10, 10);  
        m_F.resize(10, 10);
        m_Q.resize(10, 10);
        m_R.resize(4, 4);
    }

private:

    /**
     * @brief 获取先验观测量
     * @param X Eigen::Matrix<double, 10, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
     */

    // 观测预测函数,它根据当前状态估计来预测应该观测到的装甲板数据
    Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {
        // 第一版前哨卡尔曼
        // m_X状态转移矩阵
        // 第0行: 前哨战中心x
        // 第1行: 前哨战中心y
        // 第2行: 前哨战中心z
        // 第3行: 前哨站yaw
        // 第4行: 前哨战旋转角度palstance
        // Eigen::Matrix<double, 4, 1> h;
        // h << X(0, 0) + 0.25685 * cos(X(3, 0) + 2 * i * PI / 3),     // 装甲板x坐标
        //         X(1, 0) + 0.25685 * sin(X(3, 0) + 2 * i * PI / 3),  // 装甲板y坐标
        //         X(2, 0),                                            // 装甲板z坐标
        //         X(3, 0) + 2 * i * PI / 3;                           // 装甲板yaw
        // return h;

        // 第二版前哨卡尔曼
        // m_X状态转移矩阵
        // 第0行: 中心x
        // 第1行: 中心y
        // 第2行: h1
        // 第3行: h2
        // 第4行: h3
        // 第5行: 偏航角yaw
        // 第6行: 角速度palstance
        // Eigen::Matrix<double, 4, 1> h;
        // h <<    X(0, 0) + m_init_radius * cos(X(5, 0) + 2 * i * PI / 3),  // 装甲板的x坐标
        //         X(1, 0) + m_init_radius * sin(X(5, 0) + 2 * i * PI / 3),  // 装甲板的y坐标
        //         X(2 + i, 0),                                        // 装甲板z坐标
        //         X(5, 0) + 2 * i * PI / 3;                           // 装甲板yaw


        // 第三版前哨卡尔曼
        // m_X状态转移矩阵
        // 第0行: 中心x
        // 第1行: 中心y
        // 第2行: h1
        // 第3行: h2
        // 第4行: h3
        // 第5行: r1
        // 第6行: r2
        // 第7行: r3
        // 第8行: 偏航角yaw
        // 第9行: 角速度palstance
        Eigen::Matrix<double, 4, 1> h;
        h <<    X(0, 0) + X(5 + i, 0) * cos(X(8, 0) + 2 * i * PI / 3),  // 装甲板的x坐标
                X(1, 0) + X(5 + i, 0) * sin(X(8, 0) + 2 * i * PI / 3),  // 装甲板的y坐标
                X(2 + i, 0),                                        // 装甲板z坐标
                X(8, 0) + 2 * i * PI / 3;                           // 装甲板yaw
        return h;
    }

    /**
     * @brief 获取观测方程偏导矩阵
     * @param X Eigen::Matrix<double, 10, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
     */
    // 观测矩阵雅可比计算函数,用于线性化非线性观测模型
    Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {
        // Eigen::Matrix<double, 4, 5> H;
        // H << 1, 0, 0, -0.25685 * sin(X(3, 0) + 2 * i * PI / 3), 0,      // 装甲板x坐标对状态的偏导
        //         0, 1, 0,  0.25685 * cos(X(3, 0) + 2 * i * PI / 3), 0,   // 装甲板y坐标对状态的偏导
        //         0, 0, 1,                                        0, 0,   // 装甲板z坐标对状态的偏导
        //         0, 0, 0,                                        1, 0;   // 装甲板朝向对状态的偏导
        // return H;

        // Eigen::Matrix<double, 4, 7> H;
        // H << 1, 0,  0,               0,               0,                 -m_init_radius * sin(X(5, 0) + 2 * i * PI / 3), 0,  // 装甲板x坐标对状态的偏导
        //      0, 1,  0,               0,               0,                  m_init_radius * cos(X(5, 0) + 2 * i * PI / 3), 0,  // 装甲板y坐标对状态的偏导
        //      0, 0,  (i==0) ? 1 : 0,  (i==1) ? 1 : 0,  (i == 2) ? 1 : 0,   0,                                       0,  // 装甲板z坐标对状态的偏导
        //      0, 0,  0,               0,               0,                  1,                                       0;  // 装甲板yaw对状态的偏导

        Eigen::Matrix<double, 4, 10> H;
        H << 1, 0,  0,               0,               0,                 ((i==0) ? 1 : 0) * cos(X(8, 0) + 2 * i * PI / 3),  ((i==1) ? 1 : 0) * cos(X(8, 0) + 2 * i * PI / 3),  ((i == 2) ? 1 : 0) * cos(X(8, 0) + 2 * i * PI / 3), -X(5 + i, 0) * sin(X(8, 0) + 2 * i * PI / 3),  0,         // 装甲板x坐标对状态的偏导
             0, 1,  0,               0,               0,                 ((i==0) ? 1 : 0) * sin(X(8, 0) + 2 * i * PI / 3),  ((i==1) ? 1 : 0) * sin(X(8, 0) + 2 * i * PI / 3),  ((i == 2) ? 1 : 0) * sin(X(8, 0) + 2 * i * PI / 3),  X(5 + i, 0) * cos(X(8, 0) + 2 * i * PI / 3),  0,         // 装甲板y坐标对状态的偏导
             0, 0,  (i==0) ? 1 : 0,  (i==1) ? 1 : 0,  (i == 2) ? 1 : 0,  0,                                                 0,                                                 0,                                                   0,                                            0,         // 装甲板z坐标对状态的偏导
             0, 0,  0,               0,               0,                 0,                                                 0,                                                 0,                                                   1,                                            0;         // 装甲板yaw对状态的偏导
        return H;
    }

    /**
     * @brief 获取观测噪声偏导矩阵
     * @param X Eigen::Matrix<double, 10, 1> 先验状态
     * @param i 标准装甲板索引
     * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
     */
    // 从球坐标系到笛卡尔坐标系的观测噪声转换雅可比矩阵
    Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        // 1.获取预测的装甲板位置
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);

        // 2.计算球半径参数
        double psi = atan2(h(1, 0), h(0, 0));                                   // 方位角
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));   // 俯仰角
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));   // 距离

        // 3.构建噪声转换矩阵
        Eigen::Matrix<double, 4, 4> V;
        V <<   -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,     // x坐标的噪声转换(x坐标对球坐标参数的敏感度)
                d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,     // y坐标的噪声转换(y坐标对球坐标参数的敏感度)
                0,                        d * cos(phi),            sin(phi),            0,     // z坐标的噪声转换(高度对球坐标的敏感度)
                0,                        0,                       0,                   1;     // 朝向角的噪声转换
        return V;
    }
    
    double m_process_noise[4];
};




