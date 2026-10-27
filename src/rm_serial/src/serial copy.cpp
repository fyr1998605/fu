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
    std::string config_path_;
    double m_rollOffset = 0;
    double m_pitchOffset = 0;
    double m_yawOffset = 0;

    SerialPort Sp;
    rm_msgs::RmSerial PubSerialData;
    sensor_msgs::Imu ImuData;
    ros::Publisher Serial_Imu;
    ros::Publisher Serial_Rm;

    void ReceiveYolov8(const geometry_msgs::Vector3ConstPtr autoAngle)
    {
        if (shootflag != 'f') {
            Sp.Sendcom(autoAngle->x * 1000, autoAngle->y * 1000, autoAngle->z);
        }
    }

    void ReceiveYolov8_Fan(const geometry_msgs::Vector3ConstPtr autoAngle)
    {
        if (shootflag == 'f') {
            Sp.Sendcom(autoAngle->x * 1000, autoAngle->y * 1000, autoAngle->z);
        }
    }

    void setParam(const std::string &file_path) {
        try{
            if (file_path.empty()) {
                ROS_ERROR("Empty config file path provided");
                return;
            }
            std::ifstream file_check(file_path);
            if (!file_check.is_open()) {
                ROS_ERROR("Failed to open config file: %s. Reason: %s",
                        file_path.c_str(), strerror(errno));
                return;
            }
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
            if (config["InstallOffset"]) {
                YAML::Node mod = config["InstallOffset"];
                m_rollOffset  = mod["rollOffset"].as<double>(0);
                m_pitchOffset = mod["pitchOffset"].as<double>(0);
                m_yawOffset   = mod["yawOffset"].as<double>(0);
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

    RmSerial(ros::NodeHandle &nh, const std::string& config_path)
        :Sp(SPName.c_str(), 36), config_path_(config_path)
    {
        shootflag = 'a';
        camp = 'r';
        Serial_Imu = nh.advertise<sensor_msgs::Imu>("imu/data_raw",1000);
        Serial_Rm  = nh.advertise<rm_msgs::RmSerial>("/RmSerialData",1000);
        if (!config_path_.empty()) {
            try {
                setParam(config_path_);
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to load parameters: %s", e.what());
            }
        }
    }

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

        if(_RmSerialData[9])
            camp = 'b';
        else
            camp = 'r';

        if(_RmSerialData[10] == 0)
            shootflag = 'a';
        else if(_RmSerialData[10] == 1)
            shootflag = 'f';
        else
            shootflag = 'a';

        bv = _RmSerialData[11] / 100.0;
    }

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

        PubSerialData.Roll  = roll  + m_rollOffset;
        PubSerialData.Pitch = pitch + m_pitchOffset;
        PubSerialData.Yaw   = yaw   + m_yawOffset;

        ImuData.angular_velocity.x = PubSerialData.AngularVelocity_X = gyro[0];
        ImuData.angular_velocity.y = PubSerialData.AngularVelocity_Y = gyro[1];
        ImuData.angular_velocity.z = PubSerialData.AngularVelocity_Z = gyro[2];

        ImuData.linear_acceleration.x = PubSerialData.LinearAcceleration_X = accel[0];
        ImuData.linear_acceleration.y = PubSerialData.LinearAcceleration_Y = accel[1];
        ImuData.linear_acceleration.z = PubSerialData.LinearAcceleration_Z = accel[2];

        PubSerialData.Camp      = camp;
        PubSerialData.ShootFlag = shootflag;
        PubSerialData.BulletVec = bv;
        PubSerialData.header.stamp = ros::Time::now();

        Serial_Rm.publish(PubSerialData);
        Serial_Imu.publish(ImuData);
    }

    void ReceiveSerial()
    {
        if(!Sp.Readcom_imu(36)) {
            _RmSerialData = Sp.Rm_Serial_Data;
        }
        if(!_RmSerialData.empty()){
            RmSerialDataInit();
            PubRmSerialData();
        }
        _RmSerialData.clear();
    }

    void Progress()
    {
        ReceiveSerial();
    }
};


// ============================================================
//  模拟模式: 无串口硬件时发布伪造的 /RmSerialData 和 /imu/data_raw
//  通过 ROS param "shootflag_test" 控制模式切换:
//    shootflag_test = 'a'  → 装甲板模式
//    shootflag_test = 'f'  → 能量机关模式
// ============================================================
void runSimulation(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const std::string &config_path)
{
    ROS_WARN("============================================");
    ROS_WARN("  串口模拟模式 (无硬件测试)");
    ROS_WARN("============================================");

    // 读取测试参数
    std::string shootflag_str = "a";
    private_nh.param<std::string>("shootflag_test", shootflag_str, "a");
    char test_shootflag = (shootflag_str[0] == 'f') ? 'f' : 'a';

    std::string camp_str = "r";
    private_nh.param<std::string>("camp_test", camp_str, "r");
    char test_camp = (camp_str[0] == 'b') ? 'b' : 'r';

    double test_bullet_vec = 21.0;
    private_nh.param<double>("bullet_vec_test", test_bullet_vec, 21.0);

    ROS_WARN("  ShootFlag = '%c'  (%s)", test_shootflag,
             test_shootflag == 'f' ? "能量机关模式" : "装甲板模式");
    ROS_WARN("  Camp      = '%c'", test_camp);
    ROS_WARN("  BulletVec = %.1f m/s", test_bullet_vec);

    // 存储接收到的角度
    geometry_msgs::Vector3 latest_angle_armor;
    geometry_msgs::Vector3 latest_angle_fan;
    char current_shootflag = test_shootflag;

    // 发布 fake IMU + RmSerial
    ros::Publisher pub_rm   = nh.advertise<rm_msgs::RmSerial>("/RmSerialData", 1000);
    ros::Publisher pub_imu  = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

    // 订阅两个角度话题（只存最新值，实际无串口发送所以不做 Sp.Sendcom）
    auto armor_cb = [&](const geometry_msgs::Vector3ConstPtr msg) {
        latest_angle_armor = *msg;
    };
    auto fan_cb   = [&](const geometry_msgs::Vector3ConstPtr msg) {
        latest_angle_fan = *msg;
    };
    ros::Subscriber sub_armor = nh.subscribe<geometry_msgs::Vector3>("/auto_angle", 1000, armor_cb);
    ros::Subscriber sub_fan   = nh.subscribe<geometry_msgs::Vector3>("/auto_angle_fan", 1000, fan_cb);

    ros::Rate rate(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();

        // 构造 fake RmSerial 数据
        rm_msgs::RmSerial fake_serial;
        fake_serial.header.stamp = ros::Time::now();
        // 模拟 IMU: pitch=0, yaw=0, roll=0 (水平放置)
        fake_serial.Roll  = 0.0;
        fake_serial.Pitch = 0.0;
        fake_serial.Yaw   = 0.0;
        fake_serial.AngularVelocity_X  = 0.0;
        fake_serial.AngularVelocity_Y  = 0.0;
        fake_serial.AngularVelocity_Z  = 0.0;
        fake_serial.LinearAcceleration_X = 0.0;
        fake_serial.LinearAcceleration_Y = 0.0;
        fake_serial.LinearAcceleration_Z = 0.0;
        fake_serial.Qw = 1.0; fake_serial.Qx = 0.0;
        fake_serial.Qy = 0.0; fake_serial.Qz = 0.0;
        fake_serial.Camp      = test_camp;
        fake_serial.ShootFlag = test_shootflag;
        fake_serial.BulletVec = test_bullet_vec;

        // 模拟 IMU
        sensor_msgs::Imu fake_imu;
        fake_imu.header.frame_id = "imu_link";
        fake_imu.header.stamp = ros::Time::now();
        fake_imu.orientation.w = 1.0;
        fake_imu.angular_velocity.x = 0.0;
        fake_imu.angular_velocity.y = 0.0;
        fake_imu.angular_velocity.z = 0.0;
        fake_imu.linear_acceleration.x = 0.0;
        fake_imu.linear_acceleration.y = 0.0;
        fake_imu.linear_acceleration.z = 9.8;

        pub_rm.publish(fake_serial);
        pub_imu.publish(fake_imu);
        current_shootflag = test_shootflag;

        rate.sleep();
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string config_path;
    private_nh.param<std::string>("armor_serial_config", config_path, "");

    // 检查是否强制模拟模式
    bool force_sim = false;
    private_nh.param<bool>("sim_mode", force_sim, false);

    if (force_sim)
    {
        runSimulation(nh, private_nh, config_path);
        return 0;
    }

    // 查找真实串口
    std::cout << "Searching for available serial port..." << std::endl;
    bool hasAvailable = false;
    for(int ii = 0; ii < 3; ii++){
        for(int i = Lower; i < Upper; i++){
            SPName = SerialPrefix + *(Prefix + ii) + std::to_string(i);
            std::cout << "Serching " << SPName << " ...                    \r";
            if(ifstream(SPName.c_str()).good()) {
                std::cout << "Serching " << SPName << " exist!                 \r";
                hasAvailable = true;
                break;
            }
            std::cout << "Serching " << SPName << " not exist!              \r";
        }
        if(hasAvailable) break;
    }
    std::cout << std::endl;

    if(!hasAvailable){
        ROS_FATAL("[serial] No available serial port; hardware mode cannot start. No simulated telemetry will be published.");
        return 1;
    }

    // ========== 真实串口模式 ==========
    std::cout << "Have one serial matched:" << SPName << std::endl;
    RmSerial rs(nh, config_path);

    ros::Subscriber SubYolov8Data     = nh.subscribe("/auto_angle",     1000, &RmSerial::ReceiveYolov8,     &rs);
    ros::Subscriber SubYolov8Data_Fan = nh.subscribe("/auto_angle_fan", 1000, &RmSerial::ReceiveYolov8_Fan, &rs);

    thread th_process([&rs](){
        while(ros::ok()){
            ros::spinOnce();
        }
    });
    th_process.detach();

    while(ros::ok()) {
        rs.Progress();
    }
    return 0;
}
