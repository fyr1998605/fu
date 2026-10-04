#include <cerrno>
#include <fstream>
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include "Serial.h"
#include "rm_msgs/msg/rm_serial.hpp"

namespace {
std::string g_serial_port_name = "/dev/ttyUSB0";
constexpr char kSerialPrefix[] = "/dev/";
const std::string kPortPrefixes[] = {"ttyUSB", "ttyACM", "ttyCH343USB"};
constexpr int kPortUpperBound = 10;

class RmSerialNode : public rclcpp::Node {
 public:
  explicit RmSerialNode() : Node("serial") {
    config_path_ = declare_parameter<std::string>("armor_serial_config", "");
    sim_mode_ = declare_parameter<bool>("sim_mode", false);
    shootflag_test_ = declare_parameter<std::string>("shootflag_test", "a");
    camp_test_ = declare_parameter<std::string>("camp_test", "r");

    serial_pub_ = create_publisher<rm_msgs::msg::RmSerial>("/RmSerialData", 1000);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1000);
    angle_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "/auto_angle", 1000,
        std::bind(&RmSerialNode::receive_angle, this, std::placeholders::_1));

    if (sim_mode_) {
      RCLCPP_WARN(get_logger(), "[serial] Simulation mode: ShootFlag=%c",
                  test_shoot_flag());
      timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                 std::bind(&RmSerialNode::publish_simulation, this));
      return;
    }

    if (!find_serial_port()) {
      RCLCPP_WARN(get_logger(), "No available serial port; entering simulation mode");
      sim_mode_ = true;
      timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                 std::bind(&RmSerialNode::publish_simulation, this));
      return;
    }

    serial_port_ = std::make_unique<SerialPort>(g_serial_port_name.c_str(), 36);
    load_config();
    timer_ = create_wall_timer(std::chrono::milliseconds(1),
                               std::bind(&RmSerialNode::receive_serial, this));
  }

 private:
  char test_shoot_flag() const {
    return !shootflag_test_.empty() && shootflag_test_[0] == 'f' ? 'f' : 'a';
  }

  char test_camp() const {
    return !camp_test_.empty() && camp_test_[0] == 'b' ? 'b' : 'r';
  }

  bool find_serial_port() {
    for (const auto &prefix : kPortPrefixes) {
      for (int index = 0; index < kPortUpperBound; ++index) {
        g_serial_port_name = std::string(kSerialPrefix) + prefix + std::to_string(index);
        if (std::ifstream(g_serial_port_name.c_str()).good()) {
          RCLCPP_INFO(get_logger(), "Using serial port %s", g_serial_port_name.c_str());
          return true;
        }
      }
    }
    return false;
  }

  void receive_angle(const geometry_msgs::msg::Vector3::ConstSharedPtr message) {
    angle_ = *message;
    if (serial_port_) {
      serial_port_->Sendcom(angle_.x * 1000, angle_.y * 1000, angle_.z);
    }
  }

  void load_config() {
    if (config_path_.empty()) {
      RCLCPP_WARN(get_logger(), "No configuration file path provided. Using default parameters.");
      return;
    }

    try {
      YAML::Node config = YAML::LoadFile(config_path_);
      if (config["InstallOffset"]) {
        const YAML::Node offsets = config["InstallOffset"];
        roll_offset_ = offsets["rollOffset"].as<double>(0.0);
        pitch_offset_ = offsets["pitchOffset"].as<double>(0.0);
        yaw_offset_ = offsets["yawOffset"].as<double>(0.0);
      } else {
        RCLCPP_WARN(get_logger(), "No 'InstallOffset' section in serial config");
      }
    } catch (const YAML::Exception &error) {
      RCLCPP_ERROR(get_logger(), "Failed to load serial config '%s': %s",
                   config_path_.c_str(), error.what());
    }
  }

  void publish_simulation() {
    const auto stamp = get_clock()->now();
    rm_msgs::msg::RmSerial serial_message;
    serial_message.header.stamp = stamp;
    serial_message.qw = 1.0;
    serial_message.qx = 0.0;
    serial_message.qy = 0.0;
    serial_message.qz = 0.0;
    serial_message.camp = static_cast<std::int8_t>(test_camp());
    serial_message.shoot_flag = static_cast<std::int8_t>(test_shoot_flag());
    serial_message.bullet_vec = 21.0;

    sensor_msgs::msg::Imu imu_message;
    imu_message.header.stamp = stamp;
    imu_message.header.frame_id = "imu_link";
    imu_message.orientation.w = 1.0;
    imu_message.linear_acceleration.z = 9.8;

    serial_pub_->publish(serial_message);
    imu_pub_->publish(imu_message);
  }

  void receive_serial() {
    if (!serial_port_ || !serial_port_->Readcom_imu(36)) {
      return;
    }
    const auto &data = serial_port_->Rm_Serial_Data;
    if (data.size() < 12) {
      RCLCPP_WARN(get_logger(), "Ignoring short serial frame with %zu values", data.size());
      return;
    }

    const double roll = data[0] / 100000.0;
    const double pitch = data[1] / 100000.0;
    const double yaw = data[2] / 100000.0;
    const double gyro_x = data[3] / 100000.0;
    const double gyro_y = data[4] / 100000.0;
    const double gyro_z = data[5] / 100000.0;
    const double accel_x = data[6] / 100000.0;
    const double accel_y = data[7] / 100000.0;
    const double accel_z = data[8] / 100000.0;
    const char camp = data[9] ? 'b' : 'r';
    const char shoot_flag = data[10] == 1 ? 'f' : 'a';

    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);

    const auto stamp = get_clock()->now();
    rm_msgs::msg::RmSerial serial_message;
    serial_message.header.stamp = stamp;
    serial_message.roll = roll + roll_offset_;
    serial_message.pitch = pitch + pitch_offset_;
    serial_message.yaw = yaw + yaw_offset_;
    serial_message.qw = orientation.w();
    serial_message.qx = orientation.x();
    serial_message.qy = orientation.y();
    serial_message.qz = orientation.z();
    serial_message.angular_velocity_x = gyro_x;
    serial_message.angular_velocity_y = gyro_y;
    serial_message.angular_velocity_z = gyro_z;
    serial_message.linear_acceleration_x = accel_x;
    serial_message.linear_acceleration_y = accel_y;
    serial_message.linear_acceleration_z = accel_z;
    serial_message.camp = static_cast<std::int8_t>(camp);
    serial_message.shoot_flag = static_cast<std::int8_t>(shoot_flag);
    serial_message.bullet_vec = data[11] / 100.0;

    sensor_msgs::msg::Imu imu_message;
    imu_message.header.stamp = stamp;
    imu_message.header.frame_id = "imu_link";
    imu_message.orientation.w = orientation.w();
    imu_message.orientation.x = orientation.x();
    imu_message.orientation.y = orientation.y();
    imu_message.orientation.z = orientation.z();
    imu_message.angular_velocity.x = gyro_x;
    imu_message.angular_velocity.y = gyro_y;
    imu_message.angular_velocity.z = gyro_z;
    imu_message.linear_acceleration.x = accel_x;
    imu_message.linear_acceleration.y = accel_y;
    imu_message.linear_acceleration.z = accel_z;

    serial_pub_->publish(serial_message);
    imu_pub_->publish(imu_message);
  }

  std::unique_ptr<SerialPort> serial_port_;
  std::string config_path_;
  std::string shootflag_test_;
  std::string camp_test_;
  bool sim_mode_ = false;
  double roll_offset_ = 0.0;
  double pitch_offset_ = 0.0;
  double yaw_offset_ = 0.0;
  geometry_msgs::msg::Vector3 angle_;
  rclcpp::Publisher<rm_msgs::msg::RmSerial>::SharedPtr serial_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RmSerialNode>());
  rclcpp::shutdown();
  return 0;
}
