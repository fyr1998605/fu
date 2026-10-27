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

    def test_fu_tuning_surface_is_owned_by_yaml(self):
        config = FU_CONFIG.read_text(encoding="utf-8")
        source = FU_SOURCE.read_text(encoding="utf-8")
        launch = ET.parse(FU_LAUNCH).getroot()
        arg_names = {element.attrib["name"] for element in launch.findall("arg")}
        node_param_names = {
            element.attrib.get("name")
            for element in launch.find("node").findall("param")
        }

        for key in (
            "buff_model_path: Model/fu_moxi/buff_repvgg.xml",
            "enemy_color: blue",
            "buff_move_mode: small",
            "buff_detector_conf_threshold: 0.2",
            "buff_detector_nms_threshold: 0.2",
            "buff_r_center_binary_threshold: 120",
            "buff_r_center_dilate_kernel_size: 5",
            "buff_r_center_mask_ratio: 0.8",
            "buff_r_center_contour_distance_divisor: 3.0",
            "buff_r_center_refine_max_shift_px: 5.0",
            "buff_stabilizer_alpha: 0.9",
            "buff_pitch_compensation_rad: -0.06",
            "buff_yaw_compensation_rad: 0.02",
            "buff_pitch_max_rad: 0.383972",
            "buff_pitch_min_rad: -0.261799",
            "buff_stuck_threshold_rad: 0.331613",
            "buff_stuck_trigger_frames: 30",
            "buff_recovery_frames: 15",
            "buff_recovery_pitch_rad: -0.174533",
            "buff_small_converge_frames: 5",
            "buff_small_max_roll_residual_rad: 0.55",
            "buff_small_process_noise: 0.005",
            "buff_big_prediction_acquire_frames: 3",
            "buff_big_prediction_max_lead_rad: 1.5",
            "buff_ballistic_default_speed_mps: 24.0",
        ):
            self.assertIn(key, config)

        for param_name in (
            "buff_model_path",
            "enemy_color",
            "buff_move_mode",
            "buff_detector_conf_threshold",
            "buff_r_center_binary_threshold",
            "buff_pitch_compensation_rad",
            "buff_small_converge_frames",
            "buff_big_prediction_acquire_frames",
            "buff_ballistic_default_speed_mps",
        ):
            self.assertIn(
                f'private_nh.param', source,
            )
            self.assertIn(param_name, source)

        self.assertEqual(arg_names, {"fu_config"})
        self.assertNotIn("buff_model_path", node_param_names)
        self.assertNotIn("buff_move_mode", node_param_names)

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

    def test_serial_simulation_requires_an_explicit_mode(self):
        source = SERIAL_SOURCE.read_text(encoding="utf-8")

        self.assertIn('private_nh.param<bool>("sim_mode"', source)
        self.assertIn('private_nh.param<std::string>("shootflag_test"', source)
        self.assertIn('runSimulation(nh, private_nh);', source)
        self.assertIn('serial_message.ShootFlag = test_shootflag;', source)

    def test_fan_command_uses_image_time_pose(self):
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn(
            "const double expected_pitch_angle =",
            runtime,
        )
        self.assertIn(
            "const double expected_yaw_angle =",
            runtime,
        )
        self.assertNotIn(
            "raw_angle_x = -RmSerialData.Pitch",
            runtime,
        )

    def test_fan_failed_frame_holds_live_feedback_position(self):
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = (REPO_ROOT / "src/rm_tracker/include/buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn("if (!task_ok && functional)", source)
        self.assertIn("Angle.x = -RmSerialData.Pitch;", runtime)
        self.assertIn("Angle.y = RmSerialData.Yaw;", runtime)

    def test_world_tracker_latches_ready_state_across_short_bad_streaks(self):
        tracker = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_world_tracker.hpp").read_text(encoding="utf-8")

        self.assertRegex(tracker, r"kReadyEnterFrames\s*=\s*10\b")
        self.assertRegex(tracker, r"kReadyExitFrames\s*=\s*15\b")
        self.assertIn("ready_enter_counter_", tracker)
        self.assertIn("ready_exit_counter_", tracker)

    def test_target_priority_uses_configured_lock_for_both_modes(self):
        source = FU_SOURCE.read_text(encoding="utf-8")
        self.assertRegex(
            source,
            r"g_recognizer->setTargetPriority\(\s*buff_target_priority,\s*"
            r"buff_target_lock_timeout_seconds\)",
        )
        self.assertIn("buff_target_priority: locked",
                      FU_CONFIG.read_text(encoding="utf-8"))

    def test_buff_delay_components_are_independent(self):
        config = FU_CONFIG.read_text(encoding="utf-8")
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn("buff_vision_delay_seconds: 0.10", config)
        self.assertIn("buff_ec_delay_seconds: 0.05", config)
        self.assertIn(
            'private_nh.param<double>("buff_vision_delay_seconds"', source)
        self.assertIn(
            'private_nh.param<double>("buff_ec_delay_seconds"', source)
        self.assertIn("double buffVisionProcessingDelaySeconds()", source)
        self.assertIn("double buffCommandTimespanSeconds()", source)
        self.assertIn(
            "buffVisionProcessingDelaySeconds() + buffCommandTimespanSeconds()",
            source,
        )
        self.assertIn("std::isfinite(vision_delay_ema_seconds)", source)
        self.assertIn("std::max(0.0, ec_delay_seconds)", source)
        self.assertEqual(runtime.count("buffTotalLeadCompensation()"), 3)

    def test_expected_angles_use_synchronized_frame_pose_and_publish_for_rqt_plot(self):
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn("ros::Publisher expected_pitch_pub", source)
        self.assertIn("ros::Publisher expected_yaw_pub", source)
        self.assertIn('"/buff/expected_pitch"', source)
        self.assertIn('"/buff/expected_yaw"', source)
        self.assertIn(
            "const double expected_pitch_angle =",
            runtime,
        )
        self.assertIn(
            "-g_frame_serial_data.Pitch +",
            runtime,
        )
        self.assertIn(
            "const double expected_yaw_angle =",
            runtime,
        )
        self.assertIn(
            "g_frame_serial_data.Yaw +",
            runtime,
        )
        self.assertIn("current_yaw_msg.data = RmSerialData.Yaw", runtime)
        self.assertIn("expected_yaw_msg.data = track_ok", runtime)
        self.assertIn("debugpub.publish(current_yaw_msg)", runtime)
        self.assertIn("debugpub1.publish(expected_yaw_msg)", runtime)
        self.assertIn("expected_pitch_pub.publish(expected_pitch_msg)", runtime)
        self.assertIn("expected_yaw_pub.publish(expected_yaw_msg)", runtime)

    def test_angle_diagnostics_log_prediction_expectation_gimbal_and_delay(self):
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        self.assertIn("[BUFF_ANGLE]", runtime)
        self.assertIn("当前云台(p/y)", runtime)
        self.assertIn("期望无电控(p/y)", runtime)
        self.assertIn("最终预测(p/y)", runtime)
        self.assertIn("实际下发(p/y)", runtime)
        self.assertIn("时延(视觉实测/视觉生效/电控/飞行/总)", runtime)
        self.assertIn("euler_angles.x", runtime)
        self.assertIn("euler_angles.y", runtime)
        self.assertIn("RmSerialData.Pitch", runtime)
        self.assertIn("RmSerialData.Yaw", runtime)
        self.assertIn("g_frame_serial_data.Pitch", runtime)
        self.assertIn("g_frame_serial_data.Yaw", runtime)

    def test_rqt_angle_topics_separate_current_expected_and_final_prediction(self):
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                   "buff_node_runtime.hpp").read_text(encoding="utf-8")

        for topic in (
            '"/buff/current_pitch"',
            '"/buff/current_yaw"',
            '"/buff/expected_pitch"',
            '"/buff/expected_yaw"',
            '"/buff/predicted_pitch"',
            '"/buff/predicted_yaw"',
        ):
            self.assertIn(topic, source)
        self.assertIn("prediction_without_ec_angles", runtime)
        self.assertIn("expected_pitch_no_ec", runtime)
        self.assertIn("predicted_pitch_msg", runtime)
        self.assertIn("current_pitch_msg", runtime)
        self.assertIn("predicted_yaw_msg", runtime)
        self.assertIn("buffCommandTimespanSeconds()", runtime)


if __name__ == "__main__":
    unittest.main()
