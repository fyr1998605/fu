#ifndef VISUAL
#define VISUAL
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud.h>
#include <yaml-cpp/yaml.h>

using namespace std;

namespace Visual{

    class VisualTF{
    public:
        VisualTF(ros::NodeHandle &nh, const std::string& config_path);
        geometry_msgs::TransformStamped TfData(std::string frame, std::string child, Eigen::Vector3d rpy, Eigen::Vector3d point);
        geometry_msgs::TransformStamped TfData(std::string frame, std::string child, Eigen::Vector3d rpy, Eigen::Vector3d point,ros::Time time);
        void TfUpdate(Eigen::Vector3d rpy);
        void TfUpdate(Eigen::Vector3d rpy,ros::Time time);
        void TargetUpdate(Eigen::Vector3d target,Eigen::Vector3d pose);
        Eigen::Vector3d GetChildInFrameData(std::string frame, std::string child);
        void _Update(const std::string& axe,Eigen::Vector3d target,const std::string& name, Eigen::Vector3d pose,int index);
        std::vector<Eigen::Vector3d> GetChildInFrameData(const std::string& frame, const std::string& child,int);
        Eigen::Vector4d MyEuler2QUaternion(double roll, double pitch, double yaw);
        Eigen::Matrix3d MyEuler2Rotation(double roll, double pitch, double yaw);
        Eigen::Vector3d MyQUaternion2Euler(double x,double y,double z,double w);
        Eigen::Vector3d MyQUaternion2Euler(double qtn[4]);
        Eigen::Vector3d MyQUaternion2Euler(geometry_msgs::Quaternion qtn);
        std::vector<std::vector<Eigen::Vector3d>> RebuildArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy);
        std::vector<std::vector<Eigen::Vector3d>> RebuildOtherArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy);
        void setParam(const std::string &file_path);
    private:
        tf2_ros::TransformBroadcaster tfPubImu;
        tf2_ros::TransformBroadcaster tfPubCam;
        tf2_ros::TransformBroadcaster tfPubTarget;
        tf2_ros::TransformBroadcaster tfPubTemp[100];
        tf2_ros::TransformBroadcaster tfPubGun;
        geometry_msgs::TransformStamped imu2Map;
        geometry_msgs::TransformStamped cam2Imu;
        geometry_msgs::TransformStamped target2Cam;
        geometry_msgs::TransformStamped gun2Imu;
        geometry_msgs::TransformStamped temp[100];
        // 声明一个tf2_ros::Buffer对象
        tf2_ros::Buffer tfBuffer;
        // 声明一个tf2_ros::TransformListener对象，并传入tfBuffer
        tf2_ros::TransformListener tfListener;
        std::string config_path_;                   //存储配置路径
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
    };

    VisualTF::VisualTF(ros::NodeHandle &nh, const std::string& config_path): tfListener(tfBuffer) ,config_path_(config_path)
    {   
        // 配置参数
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

    geometry_msgs::TransformStamped VisualTF::TfData(std::string frame, std::string child, Eigen::Vector3d rpy, Eigen::Vector3d point)
    {
        geometry_msgs::TransformStamped tf_data;

        tf_data.header.frame_id = frame;
        tf_data.child_frame_id = child;
        tf_data.header.stamp = ros::Time::now();
        tf2::Quaternion qtn;
        qtn.setRPY(rpy[0], rpy[1], rpy[2]);
        tf_data.transform.rotation.w = qtn.getW();
        tf_data.transform.rotation.x = qtn.getX();
        tf_data.transform.rotation.y = qtn.getY();
        tf_data.transform.rotation.z = qtn.getZ();
        tf_data.transform.translation.x = point[0];
        tf_data.transform.translation.y = point[1];
        tf_data.transform.translation.z = point[2];
        return tf_data;
    }

    geometry_msgs::TransformStamped VisualTF::TfData(std::string frame, std::string child, Eigen::Vector3d rpy, Eigen::Vector3d point,ros::Time time)
    {
        geometry_msgs::TransformStamped tf_data;

        tf_data.header.frame_id = frame;
        tf_data.child_frame_id = child;
        tf_data.header.stamp = time;
        tf2::Quaternion qtn;
        qtn.setRPY(rpy[0], rpy[1], rpy[2]);
        tf_data.transform.rotation.w = qtn.getW();
        tf_data.transform.rotation.x = qtn.getX();
        tf_data.transform.rotation.y = qtn.getY();
        tf_data.transform.rotation.z = qtn.getZ();
        tf_data.transform.translation.x = point[0];
        tf_data.transform.translation.y = point[1];
        tf_data.transform.translation.z = point[2];
        return tf_data;
    }
    
    // 新(重载过后可以接收时间戳) // 识别那里上传串口时间
    void VisualTF::TfUpdate(Eigen::Vector3d rpy,ros::Time time)
    {
        // 发布陀螺仪与绝对的关系
        imu2Map = TfData("map", "imu", rpy, {m_imu2Map_trans_off_x,m_imu2Map_trans_off_y,m_imu2Map_trans_off_z},time);
        tfPubImu.sendTransform(this->imu2Map);
        // 发布相机与陀螺仪的关系
        cam2Imu = this->TfData("imu", "cam", {m_cam2Imu_rpy_off_roll, m_cam2Imu_rpy_off_pitch, m_cam2Imu_rpy_off_yaw}, {m_cam2Imu_trans_off_x,m_cam2Imu_trans_off_y,m_cam2Imu_trans_off_z},time); 
        tfPubCam.sendTransform(this->cam2Imu);  
        // 发布枪口与陀螺仪的关系
        gun2Imu = TfData("imu", "gun", {m_gun2Imu_rpy_off_roll,m_gun2Imu_rpy_off_pitch,m_gun2Imu_rpy_off_yaw}, {m_gun2Imu_trans_off_x, m_gun2Imu_trans_off_y, m_gun2Imu_trans_off_z},time);
        tfPubGun.sendTransform(this->gun2Imu);
    }

    // 旧(旧代码遗留没有动)
    void VisualTF::TfUpdate(Eigen::Vector3d rpy)
    { 
        // 发布陀螺仪与绝对的关系
        imu2Map = TfData("map", "imu", rpy, {0,0,0.055});
        tfPubImu.sendTransform(this->imu2Map);
        // 发布相机与陀螺仪的关系
        cam2Imu = this->TfData("imu", "cam", {-M_PI / 2, 0, -M_PI / 2}, {0.135,0,-0.012});
        tfPubCam.sendTransform(this->cam2Imu);
        // 发布枪口与陀螺仪的关系
        gun2Imu = TfData("imu", "gun", {0,0,0}, {0.14524, 0, -0.055});
        tfPubGun.sendTransform(this->gun2Imu);
    }

    void VisualTF::TargetUpdate(Eigen::Vector3d target,Eigen::Vector3d pose = {0,0,0})
    {
        // 发布目标与相机的关系
        target2Cam = TfData("cam", "target", pose, target);
        tfPubTarget.sendTransform(this->target2Cam);
    }

    void VisualTF::_Update(const std::string& axe,Eigen::Vector3d target,const std::string& name = "target", Eigen::Vector3d pose = {0,0,0},int index = 0)
    {
        // 发布目标与相机的关系
        temp[index] = TfData(axe, name, pose, target);
        tfPubTemp[index].sendTransform(this->temp[index]);
    }

    Eigen::Vector3d VisualTF::GetChildInFrameData(std::string frame, std::string child)
    {
        // 声明一个geometry_msgs::TransformStamped对象，用于存储变换关系
        geometry_msgs::TransformStamped transformStamped;
        
        try {
            // 查询从frame到child的变换关系
            transformStamped = tfBuffer.lookupTransform(frame, child, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            // 如果出现异常，返回一个默认构造的Vector3d
            return Eigen::Vector3d::Zero();
        }

        // 将变换应用于一个点或向量，这里我们将(0, 0, 0)应用于变换以获取在frame坐标系中child的位置
        Eigen::Vector3d child_in_frame;
        child_in_frame << transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z;

        return child_in_frame;
    }
    // 获取两坐标间的位姿
    // 输入：父坐标系名称， 子坐标系名称
    // 输出: pose[0] 表示位置信息 pose[1] 表示姿态信息
    std::vector<Eigen::Vector3d> VisualTF::GetChildInFrameData(const std::string& frame, const std::string& child,int)
    {
        // 声明一个geometry_msgs::TransformStamped对象，用于存储变换关系
        geometry_msgs::TransformStamped transformStamped;
        
        try {
            // 查询从frame到child的变换关系
            transformStamped = tfBuffer.lookupTransform(frame, child, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            // 如果出现异常，返回一个默认构造的Vector3d
            std::vector<Eigen::Vector3d> pose;
            pose.push_back(Eigen::Vector3d::Zero());
            pose.push_back(Eigen::Vector3d::Zero());
            return pose;
        }

        // 将变换应用于一个点或向量，这里我们将(0, 0, 0)应用于变换以获取在frame坐标系中child的位置
        std::vector<Eigen::Vector3d> pose;
        Eigen::Vector3d point;
        double roll, pitch, yaw;
        point << transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z;
        tf2::Quaternion qtn(transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y,
                            transformStamped.transform.rotation.z,
                            transformStamped.transform.rotation.w);
        tf2::Matrix3x3 matrix(qtn);
        matrix.getRPY(roll, pitch, yaw);
        
        pose.push_back(point);
        pose.push_back({roll, pitch, yaw});

        return pose;
    }

    /* 
    输入：目标imu下的三维点， 车中心imu下的三维点
    输出：	
            fourPoints[0]	Armor1的四个角点
            fourPoints[1]	Armor2的四个角点
            fourPoints[2]	Armor3的四个角点
            fourPoints[3]	Armor4的四个角点
            fourPoints[4]	Armors
            fourPoints[5]	Armors的Yaws
    */	
    // 注意！！！ map_yaw可以通过GetChildInFrameData("map", "target")获取，也可以这样获得 map_yaw = M_PI + pnp_yaw + imu;
    // 注意！！！ 这是car在imu下的计算方法：yaw = pnp_yaw;        	xCar = targetInImu.x + r * cos(yaw));yCar = targetInImu.y + r * sin(yaw));
    // 注意！！！ 这是car在map下的计算方法：yaw = map_yaw - M_PI; 	xCar = targetInMap.x + r * cos(yaw));yCar = targetInMap.y + r * sin(yaw));
    std::vector<std::vector<Eigen::Vector3d>> VisualTF::RebuildArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy)
    {
        double width = 0.07;
        double hight = 0.035; 
        // 四块装甲板中心
        std::vector<Eigen::Vector3d> Armors;
        std::vector<Eigen::Vector3d> Yaws;
        // 装甲板的四个角点
        std::vector<std::vector<Eigen::Vector3d>> fourPoints;
        // 当前装甲板在中心下的坐标
        Eigen::Vector3d targetInCarCenter = (carCenter - target);
        for(int i = 0; i < 4; i++)
        {
            // 获得其他装甲板在中心下的坐标
            Eigen::Vector3d temp = MyEuler2Rotation(rpy[0], rpy[1], rpy[2] + i * M_PI / 2) * targetInCarCenter;
            Yaws.push_back({rpy[0], rpy[1], rpy[2] + i * M_PI / 2});
            // 获得其他装甲板在绝对下的坐标
            temp = carCenter + temp;
            Armors.push_back(temp);
        }
        
        for(int i = 0; i < 4; i++)
        {
            Eigen::Vector3d P1, P2, P3, P4;
            P1 << 0, width / 2, hight / 2;
            P2 << 0, -width / 2, hight / 2;
            P3 << 0, -width / 2, -hight / 2;
            P4 << 0, width / 2, -hight / 2;

            Eigen::Matrix3d rotation = MyEuler2Rotation(rpy[0], rpy[1], rpy[2] + i * M_PI / 2);
            // 算出装甲板中心下四点坐标
            P1 = rotation * P1;
            P2 = rotation * P2;
            P3 = rotation * P3;
            P4 = rotation * P4;
            // 算出绝对坐标下四点坐标
            P1 = P1 + Armors[i];
            P2 = P2 + Armors[i];
            P3 = P3 + Armors[i];
            P4 = P4 + Armors[i];
            std::vector<Eigen::Vector3d> points;
            points.push_back(P1);
            points.push_back(P2);
            points.push_back(P3);
            points.push_back(P4);

            fourPoints.push_back(points);
        }
        fourPoints.push_back(Armors);
        fourPoints.push_back(Yaws);
        return fourPoints;
    }

    /*
        输入：
        target:         当前目标map系下三维点
        carCenter:      车的中心map系下三维点
        rpy:            当前目标map系下三轴角

        输出：
        [0][0]          armor1在map系下的三维点
        [0][1]          armor1在map系下的三轴角
        [0][2~5]        armor1四个角点在map系下的三维点
        [1][0]          armor2在map系下的三维点
        [1][1]          armor2在map系下的三轴角
        [1][2~5]        armor2四个角点在map系下的三维点
        [2][0]          armor3在map系下的三维点
        [2][1]          armor3在map系下的三轴角
        [2][2~5]        armor3四个角点在map系下的三维点
        [3][0]          armor4在map系下的三维点
        [3][1]          armor4在map系下的三轴角
        [3][2~5]        armor4四个角点在map系下的三维点
    */
    std::vector<std::vector<Eigen::Vector3d>> VisualTF::RebuildOtherArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy)
    {
        double r = (target - carCenter).norm();
        std::vector<std::vector<Eigen::Vector3d>> armors;
        for(int i = 0; i < 4; i++)
        {
            std::vector<Eigen::Vector3d> armor;
            //此步骤获得每块装甲位姿
            Eigen::Vector3d center;
            center = {carCenter.x() + r * cos(rpy[2]+i*PI/2+PI), carCenter.y() + r * sin(rpy[2]+i*PI/2+PI), carCenter.z()};
            armor.push_back(center);
            armor.push_back({rpy[0], rpy[1], rpy[2]+i*PI/2+PI});

            //此步骤获得每块装甲四点
            std::vector<Eigen::Vector3d> P;
            P.push_back({0.0, -Half_Width,Half_Height});
            P.push_back({0.0, Half_Width,Half_Height});
            P.push_back({0.0, Half_Width,-Half_Height});
            P.push_back({0.0, -Half_Width,-Half_Height});

            for(int j = 0; j < 4; j++)
            {
                Eigen::Vector3d pp = this->MyEuler2Rotation(rpy[0], rpy[1], rpy[2]+i*PI/2+PI) * P[j] + center;
                armor.push_back(pp);
            }
            armors.push_back(armor);
        }
        return armors;
    }


    // 欧拉角转四元数
    Eigen::Vector4d VisualTF::MyEuler2QUaternion(double roll, double pitch, double yaw)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(roll, pitch, yaw);

        return Eigen::Vector4d(qtn.w(), qtn.x(), qtn.y(), qtn.z()); 
    } 

    // 欧拉角转旋转矩阵
    Eigen::Matrix3d VisualTF::MyEuler2Rotation(double roll, double pitch, double yaw)
    {
        Eigen::Matrix3d rollRotation, pitchRotation, yawRotation;
        rollRotation << 1, 0, 0,
                        0, cos(roll), -sin(roll),
                        0, sin(roll),  cos(roll);

        pitchRotation << cos(pitch), 0, sin(pitch),
                        0, 1, 0,
                        -sin(pitch), 0, cos(pitch); 

        yawRotation << cos(yaw), -sin(yaw), 0,
                    sin(yaw),  cos(yaw), 0,
                    0, 0, 1;    

        return  yawRotation * pitchRotation * rollRotation;      

    }

    // 四元数转欧拉角
    Eigen::Vector3d VisualTF::MyQUaternion2Euler(double x,double y,double z,double w)
    {
        tf2::Quaternion quaternion(x,y,z,w);

        // 将四元数转换为欧拉角
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        return {roll, pitch, yaw};
    }
    // 四元数转欧拉角
    Eigen::Vector3d VisualTF::MyQUaternion2Euler(double qtn[4])
    {
        tf2::Quaternion quaternion(qtn[1], qtn[2], qtn[3], qtn[0]);

        // 将四元数转换为欧拉角
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        return {roll, pitch, yaw};
    }
    // 四元数转欧拉角
    Eigen::Vector3d VisualTF::MyQUaternion2Euler(geometry_msgs::Quaternion qtn)
    {
        tf2::Quaternion quaternion(qtn.x, qtn.y, qtn.z, qtn.w);

        // 将四元数转换为欧拉角
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        return {roll, pitch, yaw};
    }

    void VisualTF::setParam(const std::string &file_path)
    {
        try
        {
            // 1. 增强文件存在性检查
            if (file_path.empty())
            {
                ROS_ERROR("Empty config file path provided");
                return;
            }

            // 2. 检查文件是否打开
            std::ifstream file_check(file_path);
            if (!file_check.is_open())
            {
                ROS_ERROR("Failed to open config file: %s. Reason: %s",
                        file_path.c_str(), strerror(errno));
                return;
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
                return;
            }
            catch (const YAML::ParserException &e)
            {
                ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
                return;
            }

            // 加载坐标变换参数
            if (config["CoordinateTransformer"])
            {
                YAML::Node mod = config["CoordinateTransformer"];
                m_imu2Map_trans_off_x = mod["off"]["imu2Map_trans_off_x"].as<double>(0);
                m_imu2Map_trans_off_y = mod["off"]["imu2Map_trans_off_y"].as<double>(0);
                m_imu2Map_trans_off_z = mod["off"]["imu2Map_trans_off_z"].as<double>(0.055);

                m_cam2Imu_rpy_off_roll = mod["off"]["cam2Imu_rpy_off_roll"].as<double>(-1.57079632679489661923);
                m_cam2Imu_rpy_off_pitch = mod["off"]["cam2Imu_rpy_off_pitch"].as<double>(0);
                m_cam2Imu_rpy_off_yaw = mod["off"]["cam2Imu_rpy_off_yaw"].as<double>(-1.57079632679489661923);

                m_cam2Imu_trans_off_x = mod["off"]["cam2Imu_trans_off_x"].as<double>(0.135);
                m_cam2Imu_trans_off_y = mod["off"]["cam2Imu_trans_off_y"].as<double>(0);
                m_cam2Imu_trans_off_z = mod["off"]["cam2Imu_trans_off_z"].as<double>(-0.012);

                m_gun2Imu_rpy_off_roll = mod["off"]["gun2Imu_rpy_off_roll"].as<double>(0);
                m_gun2Imu_rpy_off_pitch = mod["off"]["gun2Imu_rpy_off_pitch"].as<double>(0);
                m_gun2Imu_rpy_off_yaw = mod["off"]["gun2Imu_rpy_off_yaw"].as<double>(0);

                m_gun2Imu_trans_off_x = mod["off"]["gun2Imu_trans_off_x"].as<double>(0.14524);
                m_gun2Imu_trans_off_y = mod["off"]["gun2Imu_trans_off_y"].as<double>(0);
                m_gun2Imu_trans_off_z = mod["off"]["gun2Imu_trans_off_z"].as<double>(-0.055);
                // ROS_WARN("tfupdate parameters loaded:");
                // ROS_WARN("m_imu2Map_trans_off_x:%.3f",m_imu2Map_trans_off_x);
                // ROS_WARN("m_imu2Map_trans_off_y:%.3f",m_imu2Map_trans_off_y);
                // ROS_WARN("m_imu2Map_trans_off_z:%.3f",m_imu2Map_trans_off_z);

                // ROS_WARN("m_cam2Imu_rpy_off_roll:%.3f",m_cam2Imu_rpy_off_roll);
                // ROS_WARN("m_cam2Imu_rpy_off_pitch:%.3f",m_cam2Imu_rpy_off_pitch);
                // ROS_WARN("m_cam2Imu_rpy_off_yaw:%.3f",m_cam2Imu_rpy_off_yaw);

                // ROS_WARN("m_cam2Imu_trans_off_x:%.3f",m_cam2Imu_trans_off_x);
                // ROS_WARN("m_cam2Imu_trans_off_y:%.3f",m_cam2Imu_trans_off_y);
                // ROS_WARN("m_cam2Imu_trans_off_z:%.3f",m_cam2Imu_trans_off_z);

                // ROS_WARN("m_gun2Imu_rpy_off_roll:%.3f",m_gun2Imu_rpy_off_roll);
                // ROS_WARN("m_gun2Imu_rpy_off_pitch:%.3f",m_gun2Imu_rpy_off_pitch);
                // ROS_WARN("m_gun2Imu_rpy_off_yaw:%.3f",m_gun2Imu_rpy_off_yaw);

                // ROS_WARN("m_gun2Imu_trans_off_x:%.3f",m_gun2Imu_trans_off_x);
                // ROS_WARN("m_gun2Imu_trans_off_y:%.3f",m_gun2Imu_trans_off_y);
                // ROS_WARN("m_gun2Imu_trans_off_z:%.3f",m_gun2Imu_trans_off_z);
                // ROS_WARN("tfupdate parameters loaded successfully");
            }
            else
            {
                ROS_WARN("No 'tfupdate' section in config file");
            }
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("YAML parsing error: %s", e.what());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error loading parameters: %s", e.what());
        }
        catch (...)
        {
            ROS_ERROR("Unknown error occurred while loading config file");
        }
    }
}
#endif