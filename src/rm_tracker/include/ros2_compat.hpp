#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

inline rclcpp::Logger g_ros_logger = rclcpp::get_logger("rm_tracker");
inline rclcpp::Clock::SharedPtr g_ros_clock =
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

template <typename MessageT>
struct Ros2Publisher {
  typename rclcpp::Publisher<MessageT>::SharedPtr handle;

  void publish(const MessageT &message) const {
    if (handle) {
      handle->publish(message);
    }
  }
};

class Ros2ParamNode {
 public:
  explicit Ros2ParamNode(const rclcpp::Node::SharedPtr &node) : node_(node) {}

  template <typename T>
  void param(const std::string &name, T &value, const T &default_value) {
    value = node_->declare_parameter<T>(name, default_value);
  }

 private:
  rclcpp::Node::SharedPtr node_;
};

#define ROS_INFO(...) RCLCPP_INFO(g_ros_logger, __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(g_ros_logger, __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(g_ros_logger, __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(g_ros_logger, __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(g_ros_logger, __VA_ARGS__)
#define ROS_INFO_THROTTLE(period, ...) \
  RCLCPP_INFO_THROTTLE(g_ros_logger, *g_ros_clock, \
                       static_cast<std::int64_t>((period) * 1000.0), __VA_ARGS__)
#define ROS_WARN_THROTTLE(period, ...) \
  RCLCPP_WARN_THROTTLE(g_ros_logger, *g_ros_clock, \
                       static_cast<std::int64_t>((period) * 1000.0), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) \
  RCLCPP_ERROR_THROTTLE(g_ros_logger, *g_ros_clock, \
                        static_cast<std::int64_t>((period) * 1000.0), __VA_ARGS__)
