from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class FuRos2SourceTest(unittest.TestCase):
    def test_fu_uses_ros2_node_apis(self):
        source = (ROOT / "src" / "Fu.cpp").read_text(encoding="utf-8")
        runtime = (ROOT / "include" / "buff_node_runtime.hpp").read_text(
            encoding="utf-8"
        )
        combined = source + runtime

        self.assertIn("#include <rclcpp/rclcpp.hpp>", source)
        self.assertIn("rclcpp::spin", source)
        self.assertNotIn("#include <ros/ros.h>", combined)
        self.assertNotIn("ros::NodeHandle", combined)
        self.assertNotIn("ros::Publisher", combined)
        self.assertNotIn("ros::Subscriber", combined)

    def test_fu_keeps_energy_topics(self):
        source = (ROOT / "src" / "Fu.cpp").read_text(encoding="utf-8")
        self.assertIn('"/hikrobot_camera/rgb"', source)
        self.assertIn('"/RmSerialData"', source)
        self.assertIn('"/auto_angle"', source)

    def test_fu_matches_camera_sensor_qos(self):
        source = (ROOT / "src" / "Fu.cpp").read_text(encoding="utf-8")
        self.assertIn("rclcpp::SensorDataQoS()", source)


if __name__ == "__main__":
    unittest.main()
