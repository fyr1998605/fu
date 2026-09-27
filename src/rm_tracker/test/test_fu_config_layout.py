#!/usr/bin/env python3
"""Regression checks for the Fu-only calibration/configuration boundary."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[3]
FU_CONFIG = REPO_ROOT / "src" / "rm_tracker" / "config" / "rm_fu.yaml"
FU_LAUNCH = REPO_ROOT / "src" / "rm_tracker" / "launch" / "rm_fu.launch"
FAN_LAUNCH = REPO_ROOT / "src" / "rm_node" / "launch" / "start_fan.launch"
FU_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "src" / "Fu.cpp"
COOR_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "include" / "CoorConverter.cpp"
SERIAL_SOURCE = REPO_ROOT / "src" / "rm_serial" / "src" / "serial.cpp"


class FuConfigLayoutTest(unittest.TestCase):
    def test_fu_uses_a_dedicated_configuration_file(self):
        self.assertTrue(FU_CONFIG.is_file())

        root = ET.parse(FU_LAUNCH).getroot()
        args = {element.attrib["name"]: element.attrib.get("default")
                for element in root.findall("arg")}
        self.assertEqual(args.get("fu_config"),
                         "$(find rm_tracker)/config/rm_fu.yaml")

        node = root.find("node")
        self.assertIsNotNone(node)
        self.assertTrue(any(element.attrib.get("file") == "$(arg fu_config)"
                            for element in node.findall("rosparam")))
        self.assertTrue(any(element.attrib.get("name") == "fu_config_path"
                            and element.attrib.get("value") == "$(arg fu_config)"
                            for element in node.findall("param")))

        source = FU_SOURCE.read_text(encoding="utf-8")
        self.assertIn('private_nh.param<std::string>("fu_config_path"', source)
        self.assertIn('new CoordinateTransformer(fu_config_path)', source)

    def test_fan_launch_selects_fan_mode_before_fu_processes_frames(self):
        root = ET.parse(FAN_LAUNCH).getroot()
        args = {element.attrib["name"]: element.attrib.get("default")
                for element in root.findall("arg")}
        self.assertEqual(args.get("shootflag_test"), "f")

        serial_include = next(
            element for element in root.findall("include")
            if element.attrib.get("file") == "$(find rm_serial)/launch/rm_serial.launch")
        self.assertTrue(any(element.attrib.get("name") == "shootflag_test"
                            and element.attrib.get("value") == "$(arg shootflag_test)"
                            for element in serial_include.findall("arg")))

    def test_fu_waits_for_serial_mode_and_rejects_invalid_extrinsics_config(self):
        fu_source = FU_SOURCE.read_text(encoding="utf-8")
        coor_source = COOR_SOURCE.read_text(encoding="utf-8")

        self.assertIn("bool functional = false;", fu_source)
        self.assertIn("bool CoordinateTransformer::setparam", coor_source)
        self.assertIn("throw std::runtime_error", coor_source)
        self.assertIn("auto require_double", coor_source)
        self.assertNotIn('mod["off"]["cam2Imu_rpy_off_yaw"].as<double>(', coor_source)

    def test_serial_node_uses_fan_launch_test_mode_without_hardware(self):
        source = SERIAL_SOURCE.read_text(encoding="utf-8")

        self.assertIn('private_nh.param<bool>("sim_mode"', source)
        self.assertIn('private_nh.param<std::string>("shootflag_test"', source)
        self.assertIn('runSimulation(nh, private_nh);', source)
        self.assertIn('serial_message.ShootFlag = test_shootflag;', source)

    def test_fan_pitch_command_is_a_correction_angle(self):
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn(
            "raw_angle_x = -RmSerialData.Pitch + (euler_angles.x + pitch_compensation);",
            runtime,
        )
        self.assertNotIn(
            "const double raw_angle_x = euler_angles.x + pitch_compensation;",
            runtime,
        )

    def test_fan_pitch_failure_does_not_publish_stale_angle(self):
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn("if (track_ok) {", runtime)
        self.assertIn(
            "debug_msg_x.data = track_ok ? Angle.x : -RmSerialData.Pitch;",
            runtime,
        )

    def test_world_tracker_latches_ready_state_across_short_bad_streaks(self):
        tracker = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_world_tracker.hpp").read_text(encoding="utf-8")

        self.assertIn("kReadyEnterFrames = 10", tracker)
        self.assertIn("kReadyExitFrames  = 15", tracker)
        self.assertIn("ready_enter_counter_", tracker)
        self.assertIn("ready_exit_counter_", tracker)

    def test_buff_tracking_does_not_start_as_success(self):
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn("bool track_ok = false;", runtime)
        self.assertNotIn("bool track_ok = true;", runtime)

    def test_buff_runtime_keeps_final_diagnostic_fields(self):
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")
        for field in (
            "raw_angle_x",
            "limited_angle_x",
            "pixel_track_ok",
            "kalman_healthy",
            "track_ok",
            "last_reprojection_error_",
        ):
            self.assertIn(field, runtime)

    def test_buff_pnp_exposes_last_reprojection_error_read_only(self):
        pnp = (REPO_ROOT / "src" / "rm_tracker" / "include" /
               "buff_pnp_solver.hpp").read_text(encoding="utf-8")

        self.assertIn("double lastReprojectionError() const", pnp)
        self.assertIn("return last_reprojection_error_;", pnp)

    def test_buff_pnp_exposes_prediction_failure_diagnostics(self):
        pnp = (REPO_ROOT / "src" / "rm_tracker" / "include" /
               "buff_pnp_solver.hpp").read_text(encoding="utf-8")
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn("lastPredictionFailureReason() const", pnp)
        self.assertIn("lastPredictedTargetGun() const", pnp)
        self.assertIn("last_gun_pitch_raw_deg_", pnp)
        self.assertIn("[BUFF_PRED_FAIL]", pnp)
        self.assertIn("prediction_fail_reason", runtime)
        self.assertIn("gun_pitch_raw_deg", runtime)
        self.assertIn("gun_yaw_deg", runtime)
        self.assertIn("compensated_pitch_deg", runtime)
        self.assertIn("beginPredictionFrame()", pnp)
        self.assertIn("pnpSolver_gravity->beginPredictionFrame()", runtime)
        self.assertIn('prediction_fail_reason != "ok"', runtime)
        self.assertNotIn('cout<<"4444444444444"', runtime)
        self.assertIn("[BUFF_WORLD_FAIL]", runtime)
        self.assertNotIn('cout<<"2222222222222"', runtime)
        self.assertNotIn('cout<<"3333333333333"', runtime)


if __name__ == "__main__":
    unittest.main()
