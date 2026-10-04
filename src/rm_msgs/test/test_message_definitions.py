from pathlib import Path
import unittest


MSG_DIR = Path(__file__).resolve().parents[1] / "msg"


class MessageDefinitionTest(unittest.TestCase):
    def test_rm_serial_uses_ros2_byte_types(self):
        text = (MSG_DIR / "RmSerial.msg").read_text(encoding="utf-8")

        self.assertIn("int8 camp", text)
        self.assertIn("int8 shoot_flag", text)
        self.assertIn("int8 is_auto_fire", text)
        self.assertIn("int8 post_alive", text)
        self.assertIn("std_msgs/Header header", text)

    def test_energy_messages_keep_ros2_dependencies(self):
        armor = (MSG_DIR / "Armor.msg").read_text(encoding="utf-8")
        array = (MSG_DIR / "ArmorArray.msg").read_text(encoding="utf-8")
        debug = (MSG_DIR / "Debug.msg").read_text(encoding="utf-8")

        self.assertIn("geometry_msgs/Pose armor_pose", armor)
        self.assertIn("int8 number", armor)
        self.assertIn("int8 type", armor)
        self.assertIn("std_msgs/Header header", array)
        self.assertIn("rm_msgs/Armor[] armors", array)
        self.assertIn("int8 rebuild", debug)
