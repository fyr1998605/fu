// 哨兵能量机关串口节点
// 上行: 解析电控49字节帧 -> 发布 /RmSerialData(视觉用姿态) /SentrySerial(哨兵状态) imu/data_raw
// 下行: 持续发送16字节帧(线速度/角速度/云台pitch/yaw角/各标志位)给电控
// 打符模式写死: 电控侧没有按f切换逻辑, ShootFlag 恒定 'f', 收到 /auto_angle 直接发 pitch/yaw

#include <ros/ros.h>
#include "Serial.h"
#include <std_msgs/Int32.h>
#include "rm_msgs/RmSerial.h"
#include "rm_msgs/SentrySerial.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TwistStamped.h"
#include <string>
#include <chrono>
#include <thread>
#include <fstream>
#include <yaml-cpp/yaml.h>

static std::string SPName = "/dev/ttyUSB0";///dev/ttyACM0
constexpr char* SerialPrefix = "/dev/";
const std::string Prefix[] = {"ttyUSB","ttyACM","ttyCH343USB"};
constexpr int Upper = 10;
constexpr int Lower = 0;

using namespace std;
float speed_x, speed_y, speed_w;
int up_flag = 0;
int super_speed = 1;
int lock_head = 0;

class RmSerial
{
public:

    int signal = 0;
    double roll, pitch, yaw, gyro[3], accel[3];
    char camp;
    char shootFlag = 'f';   //写死打符模式: 电控没有按f切换逻辑, 默认打符
    vector<int> _RmSerialData;
    geometry_msgs::Vector3 angle;

    std::string config_path_; //存储配置路径
    double m_rollOffset = 0;
    double m_pitchOffset = 0;
    double m_yawOffset = 0;
    double bullet_vec = 21.0; //哨兵弹速 m/s

    //串口函数
    SerialPort Sp;
    //自定义消息
    rm_msgs::RmSerial PubSerialData;
    rm_msgs::SentrySerial PubSentrySerial;
    //Imu消息
    sensor_msgs::Imu ImuData;
    //发布方
    ros::Publisher Serial_Imu;
    ros::Publisher Serial_Rm;//电控发过来的数据
    ros::Publisher Serial_Sentry;//哨兵自己的血量值和弹丸数

    //-------------------------------回调：发送自瞄角度模块----------------------------------------
    void ReceiveYolov8(const geometry_msgs::Vector3ConstPtr autoAngle)
    {
        angle = *autoAngle;
        signal = (int)angle.z;
    }
    //------------------------------------------------------------------------------------------

    void up_flag_callback(const std_msgs::Int32::ConstPtr& msg)
    {
        up_flag = msg->data;
    }

    //------------------------------------------------------------------------------------------

    void super_speed_callback(const std_msgs::Int32::ConstPtr& msg)
    {
        super_speed = msg->data;
    }

    //------------------------------------------------------------------------------------------

    void lock_head_callback(const std_msgs::Int32::ConstPtr& msg)
    {
        lock_head = msg->data;
    }

    //------------------------------------------------------------------------------------------

    void cmd_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        speed_x = msg->twist.linear.x;
        speed_y = msg->twist.linear.y;
        speed_w = msg->twist.angular.z;
    }
    //------------------------------------------------------------------------------------------

    void setParam(const std::string &file_path) {
        try{
            // 1. 文件存在性检查
            if (file_path.empty()) {
                ROS_ERROR("Empty config file path provided");
                return;
            }

            // 2.检查文件是否打开
            std::ifstream file_check(file_path);
            if (!file_check.is_open()) {
                ROS_ERROR("Failed to open config file: %s. Reason: %s",
                        file_path.c_str(), strerror(errno));
                return;
            }
            ROS_INFO("Loading parameters from: %s", file_path.c_str());

            // 3. 加载 YAML 文件并添加详细错误处理
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

            // 补偿误差函数加载
            if (config["InstallOffset"])
            {
                YAML::Node mod = config["InstallOffset"];
                m_rollOffset = mod["rollOffset"].as<double>(0);
                m_pitchOffset = mod["pitchOffset"].as<double>(0);
                m_yawOffset = mod["yawOffset"].as<double>(0);
                ROS_INFO("InstallOffset loaded: roll=%.3f pitch=%.3f yaw=%.3f",
                         m_rollOffset, m_pitchOffset, m_yawOffset);
            }
            else
            {
                ROS_WARN("No 'InstallOffset' section in config file");
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
    }

    RmSerial(ros::NodeHandle &nh, const std::string& config_path):Sp(SPName.c_str(), 32), config_path_(config_path)
    {
        Serial_Imu = nh.advertise<sensor_msgs::Imu>("imu/data_raw",1000);
        Serial_Rm  = nh.advertise<rm_msgs::RmSerial>("/RmSerialData",1000);
        Serial_Sentry=nh.advertise<rm_msgs::SentrySerial>("/SentrySerial",1000);

        // 配置参数
        if (!config_path_.empty()) {
            setParam(config_path_);
        } else {
            ROS_WARN("No configuration file path provided. Using default parameters.");
        }
    }


    //-------------------------------------读取电控数据模块-------------------------------------
    //从串口读到的数据进行赋值
    void RmSerialDataInit()
    {
        roll     = _RmSerialData[0] / 100000.0;
        pitch    = _RmSerialData[1] / 100000.0;
        yaw      = _RmSerialData[2] / 100000.0;
        gyro[0]  = _RmSerialData[3] / 100000.0;
        gyro[1]  = _RmSerialData[4] / 100000.0;
        gyro[2]  = _RmSerialData[5] / 100000.0;
        accel[0] = _RmSerialData[6] / 100000.0;
        accel[1] = _RmSerialData[7] / 100000.0;
        accel[2] = _RmSerialData[8] / 100000.0;

        //哨兵状态位
        PubSentrySerial.OurSentryHp=_RmSerialData[11];                 //哨兵血量
        PubSentrySerial.OurBaseHp=_RmSerialData[12];                   //基地血量
        if(_RmSerialData[13]==0)                                        //补给点是否被占用
           PubSentrySerial.Supply_Occupyed=false;
        else
            PubSentrySerial.Supply_Occupyed=true;

        PubSentrySerial.EnemyOutPostHp=_RmSerialData[14];              //敌方前哨站血量
        PubSentrySerial.HolderFlag=_RmSerialData[15];                  //云台手是否发布点位
        PubSentrySerial.holder_point_x=_RmSerialData[16]/100.0;        //云台手X坐标
        PubSentrySerial.holder_point_y=_RmSerialData[17]/100.0;        //云台手Y坐标
        PubSentrySerial.ResidualAmmunition=_RmSerialData[18];          //允许发弹量

        //--------------------------------------阵营判断-------------------------------------------
        // 0红方 1蓝方 (与哨兵电控约定一致)
        if(_RmSerialData[9] == 0)
            camp = 'r';
        else
            camp = 'b';

        //--------------------------------------模式判断-------------------------------------------
        // 电控侧没有按f切换能量机关的逻辑, 这里写死打符模式
        // (shootFlag 已在构造函数前由 launch 参数 shootflag 决定, 默认 'f')
    }
    //---------------------------------------------------------------------------------------

    //把imu消息和rm消息发布出去
    void PubRmSerialData()
    {
        tf2::Quaternion Q;
        Q.setRPY(roll, pitch, yaw);
        ImuData.header.frame_id = "imu_link";
        ImuData.header.stamp = ros::Time::now();

        ImuData.orientation.w = PubSerialData.Qw = Q.w();
        ImuData.orientation.x = PubSerialData.Qx = Q.x();
        ImuData.orientation.y = PubSerialData.Qy = Q.y();
        ImuData.orientation.z = PubSerialData.Qz = Q.z();

        PubSerialData.Roll = roll + m_rollOffset;
        PubSerialData.Pitch = pitch + m_pitchOffset;
        PubSerialData.Yaw = yaw + m_yawOffset;

        ImuData.angular_velocity.x = PubSerialData.AngularVelocity_X = gyro[0];
        ImuData.angular_velocity.y = PubSerialData.AngularVelocity_Y = gyro[1];
        ImuData.angular_velocity.z = PubSerialData.AngularVelocity_Z = gyro[2];

        ImuData.linear_acceleration.x = PubSerialData.LinearAcceleration_X = accel[0];
        ImuData.linear_acceleration.y = PubSerialData.LinearAcceleration_Y = accel[1];
        ImuData.linear_acceleration.z = PubSerialData.LinearAcceleration_Z = accel[2];

        PubSerialData.Camp = camp;
        PubSerialData.ShootFlag = shootFlag;    //写死 'f' 打符模式
        PubSerialData.BulletVec = bullet_vec;   //哨兵弹速

        PubSerialData.header.stamp = ros::Time::now();

        Serial_Rm.publish(PubSerialData);
        Serial_Imu.publish(ImuData);
        Serial_Sentry.publish(PubSentrySerial);
    }

    //从串口读到的数据
    void ReceiveSerial()
    {
        // 仅在收到完整校验帧时才发布, 防止旧数据被反复发布
        // (Fu节点用消息时间戳匹配图像, 旧姿态配新时间戳会导致预测点乱飘)
        if(Sp.Readcom_imu(39))
        {
            _RmSerialData = Sp.Rm_Serial_Data;  //串口赋值
            RmSerialDataInit();
            PubRmSerialData();
        }
    }

    //主线
    void Progress()
    {
        ReceiveSerial();            //读取电控数据模块
    }


};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 获取配置
    std::string config_path;
    private_nh.param<std::string>("armor_serial_config", config_path, "");
    std::string shootflag_param = "f";
    private_nh.param<std::string>("shootflag", shootflag_param, "f");
    double bullet_vec = 21.0;
    private_nh.param<double>("bullet_vec", bullet_vec, 21.0);

    // 查找串口
    std::cout << "Searching for available serial port..." << std::endl;
    bool hasAvailable = false;
    for(int ii = 0;ii < 3;ii++){
        for(int i = Lower;i < Upper;i++){
            SPName = SerialPrefix + *(Prefix + ii) + std::to_string(i);
            std::cout << "Serching " << SPName << " ...                    \r";
            if(ifstream(SPName.c_str()).good())
            {
                std::cout << "Serching " << SPName << " exist!                 \r";
                hasAvailable = true;
                break;
            }
            std::cout << "Serching " << SPName << " not exist!              \r";
        }
        if(hasAvailable)
            break;
    }
    std::cout << std::endl;
    if(!hasAvailable){
        std::cout << "No available serial port!" << std::endl;
        std::cout << "Process will be shutdown after 3 second!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        return -1;
    }
    std::cout << "Have one serial matched:" << SPName << std::endl;

    RmSerial rs(nh, config_path);
    // ShootFlag 写死: 电控没有按f切换能量机关的逻辑
    if(shootflag_param == "a")
        rs.shootFlag = 'a';
    else
    {
        if(shootflag_param != "f")
            ROS_WARN("[serial] 未知 shootflag='%s', 按打符模式 'f' 处理", shootflag_param.c_str());
        rs.shootFlag = 'f';
    }
    rs.bullet_vec = (bullet_vec > 0.0) ? bullet_vec : 21.0;
    ROS_INFO("[serial] ShootFlag='%c' (写死打符模式), BulletVec=%.2f", rs.shootFlag, rs.bullet_vec);

    //订阅方
    ros::Subscriber sub_cmd_vel = nh.subscribe("/cmd_vel" ,1000, &RmSerial::cmd_vel_callback,&rs);
    ros::Subscriber SubYolov8Data = nh.subscribe("/auto_angle",1000,&RmSerial::ReceiveYolov8,&rs);
    ros::Subscriber sub_up_flag = nh.subscribe("/up_flag",1000,&RmSerial::up_flag_callback,&rs);
    ros::Subscriber sub_speed_flag = nh.subscribe("/super_speed",1000,&RmSerial::super_speed_callback,&rs);
    ros::Subscriber sub_lock_head = nh.subscribe("/lock_head",1000,&RmSerial::lock_head_callback,&rs);

    while(ros::ok())
    {
        rs.Progress();

        ros::spinOnce();

        // 持续发送16字节帧给电控: 速度 + 云台pitch/yaw角(能量机关预测角) + 标志位
        rs.Sp.send_cmdvel(int(speed_x*100), int(speed_y*100), int(speed_w*100),
                          (rs.angle.x * 1000), (rs.angle.y* 1000),
                          1, up_flag, rs.signal, super_speed, lock_head);
    }
    return 0;
}
