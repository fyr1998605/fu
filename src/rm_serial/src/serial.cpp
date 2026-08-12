#include <ros/ros.h>
#include "Serial.h"
#include "rm_msgs/RmSerial.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <thread>
#include <string>
#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>
using namespace std;

static std::string SPName = "/dev/ttyUSB0";
constexpr char* SerialPrefix = "/dev/";
const std::string Prefix[] = {"ttyUSB","ttyACM","ttyCH343USB"};
constexpr int Upper = 10; 
constexpr int Lower = 0;

class RmSerial
{  
public: 
    double roll, pitch, yaw, gyro[3],accel[3],bv;
    char camp, shootflag;
    vector<int> _RmSerialData;
    geometry_msgs::Vector3 angle;
    std::string config_path_; //存储配置路径
    double m_rollOffset = 0;
    double m_pitchOffset = 0;
    double m_yawOffset = 0;

    //串口函数
    SerialPort Sp;
    //自定义消息
    rm_msgs::RmSerial PubSerialData;
    //Imu消息
    sensor_msgs::Imu ImuData;
    //发布方
    ros::Publisher Serial_Imu;
    ros::Publisher Serial_Rm;

    void ReceiveYolov8(const geometry_msgs::Vector3ConstPtr autoAngle)
    {
        angle = *autoAngle;
        Sp.Sendcom(angle.x * 1000, angle.y * 1000,angle.z);
        // ROS_DEBUG("serial send is ok hahahahaha");
        // cout << "Sub:" << angle << endl;
    }

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
                // ROS_INFO("InstallOffset parameters loaded:");
                // ROS_INFO("m_rollOffset:%.3f",m_rollOffset);
                // ROS_INFO("m_pitchOffset:%.3f",m_pitchOffset);
                // ROS_INFO("m_yawOffset:%.3f",m_yawOffset);
                // ROS_INFO("InstallOffset parameters loaded successfully");
            }
            else
            {
                ROS_WARN("No 'CoordinateTransformer' section in config file");
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

    RmSerial(ros::NodeHandle &nh, const std::string& config_path):Sp(SPName.c_str(), 36) , config_path_(config_path)
    {
        Serial_Imu = nh.advertise<sensor_msgs::Imu>("imu/data_raw",1000);
        Serial_Rm  = nh.advertise<rm_msgs::RmSerial>("/RmSerialData",1000);

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
        // cout << "roll" << roll << endl;
        // cout << "pitch" << pitch << endl;
        // cout << "yaw" << yaw << endl;
        // cout << "gyro[0]" << gyro[0] << endl;
        // cout << "gyro[1]" << gyro[1] << endl;
        // cout << "gyro[2]" << gyro[2] << endl;
        // cout << "accel[0]" << accel[0] << endl;
        // cout << "accel[1]" << accel[1] << endl;
        // cout << "accel[2]" << accel[2] << endl;

        if(_RmSerialData[9])
            camp = 'b';
        else
            camp = 'r';

        if(_RmSerialData[10] == 0)
            shootflag = 'a';
        else if(_RmSerialData[10] == 1)
            shootflag = 'f';
        else
            shootflag = 'a';//默认装甲板
            
        bv = _RmSerialData[11] / 100.0; 
    }
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
        PubSerialData.ShootFlag = shootflag;
        PubSerialData.BulletVec = bv;

        PubSerialData.header.stamp = ros::Time::now();

        Serial_Rm.publish(PubSerialData);
        Serial_Imu.publish(ImuData);
    }
    //从串口读到的数据
    void ReceiveSerial()
    {
        // Readcom_imu() reports true only after a complete validated IMU frame.
        if(Sp.Readcom_imu(36))
        {
            _RmSerialData = Sp.Rm_Serial_Data;
        }
        if(!_RmSerialData.empty()){
            RmSerialDataInit();
            PubRmSerialData();
        }
        _RmSerialData.clear();
    }

    //主线
    void Progress()
    {
        //Sp.send_com_test();
        ReceiveSerial();
        //cout << angle << endl;
    }
};

void runSimulation(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
    std::string shootflag_test = "a";
    std::string camp_test = "r";
    private_nh.param<std::string>("shootflag_test", shootflag_test, "a");
    private_nh.param<std::string>("camp_test", camp_test, "r");

    const char test_shootflag = !shootflag_test.empty() && shootflag_test[0] == 'f'
        ? 'f' : 'a';
    const char test_camp = !camp_test.empty() && camp_test[0] == 'b' ? 'b' : 'r';

    ros::Publisher serial_publisher = nh.advertise<rm_msgs::RmSerial>("/RmSerialData", 1000);
    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
    ros::Rate rate(100);

    ROS_WARN("[serial] Simulation mode: ShootFlag=%c", test_shootflag);
    while (ros::ok()) {
        const ros::Time stamp = ros::Time::now();

        rm_msgs::RmSerial serial_message;
        serial_message.header.stamp = stamp;
        serial_message.Qw = 1.0;
        serial_message.Qx = 0.0;
        serial_message.Qy = 0.0;
        serial_message.Qz = 0.0;
        serial_message.Camp = test_camp;
        serial_message.ShootFlag = test_shootflag;
        serial_message.BulletVec = 21.0;

        sensor_msgs::Imu imu_message;
        imu_message.header.stamp = stamp;
        imu_message.header.frame_id = "imu_link";
        imu_message.orientation.w = 1.0;
        imu_message.linear_acceleration.z = 9.8;

        serial_publisher.publish(serial_message);
        imu_publisher.publish(imu_message);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 创建私有节点句柄

    // 从参数服务器获取配置文件路径
    std::string config_path;
    private_nh.param<std::string>("armor_serial_config", config_path, "");
    bool sim_mode = false;
    private_nh.param<bool>("sim_mode", sim_mode, false);
    if (sim_mode) {
        runSimulation(nh, private_nh);
        return 0;
    }

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
        ROS_WARN("No available serial port; entering simulation mode");
        runSimulation(nh, private_nh);
        return 0;
    }
    std::cout << "Have one serial matched:" << SPName << std::endl;
    RmSerial rs(nh,config_path);

    //订阅方 
    ros::Subscriber SubYolov8Data = nh.subscribe("/auto_angle",1000,&RmSerial::ReceiveYolov8,&rs);
    thread th_process([&rs](){
    while(ros::ok()){
            ros::spinOnce();
        }
    });
    th_process.detach();
    while(ros::ok())
    {
        rs.Progress();
    }
    return 0;
}
