#include <chrono>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hikrobot_camera.hpp"

#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
#define FIT_min_x 420
#define FIT_min_y 70
#define FIT_max_x 2450
#define FIT_max_y 2000
#endif

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hikcamera");
  camera::Camera mvs_camera(*node);
  auto image_pub = image_transport::create_camera_publisher(
      node.get(), "/hikrobot_camera/rgb", rmw_qos_profile_sensor_data);

  auto publish_timer = node->create_wall_timer(
      std::chrono::microseconds(8333),
      [node, &mvs_camera, image_pub]() mutable {
        cv::Mat source;
        mvs_camera.ReadImg(source);
        if (source.empty()) {
          return;
        }

#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x - FIT_min_x,
                      FIT_max_y - FIT_min_y);
        cv::Mat image = source(area);
#else
        cv::Mat image = source;
#endif

        const auto stamp = node->get_clock()->now();
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = stamp;
        cv_image.header.frame_id = "hikrobot_camera";
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = image;

        sensor_msgs::msg::Image image_message = *cv_image.toImageMsg();
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header = image_message.header;
        image_pub.publish(image_message, camera_info);
      });

  (void)publish_timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
