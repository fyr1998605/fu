from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "src" / "serial.cpp"
HEADER = Path(__file__).resolve().parents[1] / "include" / "Serial.h"


class SerialRos2SourceTest(unittest.TestCase):
    def test_serial_uses_ros2_apis_and_topics(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertIn("#include <rclcpp/rclcpp.hpp>", text)
        self.assertIn("rclcpp::spin", text)
        self.assertIn('"/RmSerialData"', text)
        self.assertIn('"/auto_angle"', text)
        self.assertNotIn("#include <ros/ros.h>", text)
        self.assertNotIn("ros::NodeHandle", text)
        self.assertNotIn("ros::Publisher", text)
        self.assertNotIn("ros::Subscriber", text)

    def test_serial_keeps_simulation_parameters(self):
        text = SOURCE.read_text(encoding="utf-8")
        for parameter in ("shootflag_test", "camp_test", "sim_mode"):
            self.assertIn(parameter, text)

    def test_serial_uses_migrated_message_field_names(self):
        text = SOURCE.read_text(encoding="utf-8")
        self.assertIn("serial_message.shoot_flag", text)
        self.assertNotIn("serial_message.shootflag", text)

    def test_serial_driver_header_uses_ros2_shutdown(self):
        text = HEADER.read_text(encoding="utf-8")
        self.assertIn("rclcpp::shutdown()", text)
        self.assertNotIn("ros::shutdown()", text)


if __name__ == "__main__":
    unittest.main()
