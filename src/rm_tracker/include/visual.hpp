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

// 这段代码由于历史遗留问题，未添加参数配置文件
// 目前这段代码仅仅使用到tf发送的部分,未涉及坐标系之间的偏移
// 所以不影响
using namespace std;
namespace Visual{
    class VisualTF{
    public:
        VisualTF(ros::NodeHandle &nh);
        geometry_msgs::TransformStamped TfData(std::string frame, std::string child, Eigen::Vector3d rpy, Eigen::Vector3d point);
        void TfUpdate(Eigen::Vector3d rpy);
        void TargetUpdate(Eigen::Vector3d target,Eigen::Vector3d pose);
        Eigen::Vector3d GetChildInFrameData(std::string frame, std::string child);
        void _Update(const std::string& axe,Eigen::Vector3d target,const std::string& name, Eigen::Vector3d pose,int index);
        std::vector<Eigen::Vector3d> GetChildInFrameData(const std::string& frame, const std::string& child,int);
        // 新增
        std::vector<Eigen::Vector3d> GetChildInFrameDataAtTime(const std::string& frame, const std::string& child, const ros::Time& time, int);
        
        Eigen::Vector4d MyEuler2QUaternion(double roll, double pitch, double yaw);
        Eigen::Matrix3d MyEuler2Rotation(double roll, double pitch, double yaw);
        Eigen::Vector3d MyQUaternion2Euler(double x,double y,double z,double w);
        Eigen::Vector3d MyQUaternion2Euler(double qtn[4]);
        Eigen::Vector3d MyQUaternion2Euler(geometry_msgs::Quaternion qtn);
        std::vector<std::vector<Eigen::Vector3d>> RebuildArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy,char type);
        std::vector<std::vector<Eigen::Vector3d>> RebuildOtherArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy,double dz,double dradius,int count,char type);
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
    };

    VisualTF::VisualTF(ros::NodeHandle &nh): tfListener(tfBuffer)
    {
        
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
            throw; // Missing TF must not look like a measured origin.
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
            transformStamped = tfBuffer.lookupTransform(frame, child, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            throw; // Propagate invalidity instead of inventing a zero pose.
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

    std::vector<Eigen::Vector3d> VisualTF::GetChildInFrameDataAtTime(const std::string& frame, 
                                                          const std::string& child, 
                                                          const ros::Time& time, 
                                                          int)
    {
        // 声明一个geometry_msgs::TransformStamped对象，用于存储变换关系
        geometry_msgs::TransformStamped transformStamped;
        
        try {
            // 查询从frame到child的变换关系
            transformStamped = tfBuffer.lookupTransform(frame, child, time);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            // 如果出现异常，返回一个默认构造的Vector3d
            return {};
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
    std::vector<std::vector<Eigen::Vector3d>> VisualTF::RebuildArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy,char type = 's')
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
    std::vector<std::vector<Eigen::Vector3d>> VisualTF::RebuildOtherArmor(Eigen::Vector3d target, Eigen::Vector3d carCenter,Eigen::Vector3d rpy,double dz,double dradius,int count = 4,char type = 's')
    {
        double _r = (target - carCenter).norm();
        std::vector<std::vector<Eigen::Vector3d>> armors;
        double step = 2*PI;
        step /= count;
        for(int i = 0; i < count; i++)
        {
            if(count != 4){
                double r = _r;
                std::vector<Eigen::Vector3d> armor;
                //此步骤获得每块装甲位姿
                Eigen::Vector3d center;
                center = {carCenter.x() - r * cos(rpy[2]+i*step), carCenter.y() - r * sin(rpy[2]+i*step), carCenter.z()};
                armor.push_back(center);
                armor.push_back({rpy[0], rpy[1], rpy[2]+i*step});

                //此步骤获得每块装甲四点
                std::vector<Eigen::Vector3d> P;
                P.push_back({0.0, -Half_Width,Half_Height});
                P.push_back({0.0, Half_Width,Half_Height});
                P.push_back({0.0, Half_Width,-Half_Height});
                P.push_back({0.0, -Half_Width,-Half_Height});

                for(int j = 0; j < count; j++)
                {
                    Eigen::Vector3d pp = this->MyEuler2Rotation(rpy[0], rpy[1], rpy[2]+i*step) * P[j] + center;
                    armor.push_back(pp);
                }
                armors.push_back(armor);
            }
            else{
                double r = _r,nZ = carCenter.z();
                if(i == 1 || i == 3){
                    r = _r + dradius;
                    nZ = carCenter.z() + dz;
                }
                else{
                    r = _r;
                    nZ = carCenter.z();
                }
                std::vector<Eigen::Vector3d> armor;
                //此步骤获得每块装甲位姿
                Eigen::Vector3d center;
                center = {carCenter.x() - r * cos(rpy[2]+i*step), carCenter.y() - r * sin(rpy[2]+i*step), nZ};
                armor.push_back(center);
                armor.push_back({rpy[0], rpy[1], rpy[2]+i*step});

                //此步骤获得每块装甲四点
                std::vector<Eigen::Vector3d> P;
                P.push_back({0.0, -Half_Width,Half_Height});
                P.push_back({0.0, Half_Width,Half_Height});
                P.push_back({0.0, Half_Width,-Half_Height});
                P.push_back({0.0, -Half_Width,-Half_Height});

                for(int j = 0; j < count; j++)
                {
                    Eigen::Vector3d pp = this->MyEuler2Rotation(rpy[0], rpy[1], rpy[2]+i*step) * P[j] + center;
                    armor.push_back(pp);
                }
                armors.push_back(armor);
            }
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


}
#endif
