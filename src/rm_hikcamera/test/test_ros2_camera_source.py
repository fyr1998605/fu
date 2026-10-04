from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "src" / "hikcamera.cpp"


class CameraRos2SourceTest(unittest.TestCase):
    def test_camera_uses_ros2_image_transport(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertIn("#include <rclcpp/rclcpp.hpp>", text)
        self.assertIn("image_transport::create_camera_publisher", text)
        self.assertIn("sensor_msgs::msg::Image", text)
        self.assertIn("sensor_msgs::image_encodings::BGR8", text)
        self.assertIn('"/hikrobot_camera/rgb"', text)
        self.assertNotIn("#include <ros/ros.h>", text)
        self.assertNotIn("ros::NodeHandle", text)
        self.assertNotIn("ros::Rate", text)

    def test_camera_keeps_frame_id_and_publish_clock(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertIn('"hikrobot_camera"', text)
        self.assertIn("get_clock()->now()", text)


if __name__ == "__main__":
    unittest.main()
