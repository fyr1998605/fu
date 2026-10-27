#!/usr/bin/env python3
"""Regression checks for bounded latest-frame timing in the Fu node."""

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]
FU_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "src" / "Fu.cpp"
RUNTIME_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "include" / "buff_node_runtime.hpp"
CONFIG_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "config" / "rm_fu.yaml"
PREDICTOR_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "src" / "buff_big_ransac_predictor.cpp"
PREDICTOR_HEADER = REPO_ROOT / "src" / "rm_tracker" / "include" / "buff_big_ransac_predictor.hpp"
PNP_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "include" / "buff_pnp_solver.hpp"


class FuFrameTimingTest(unittest.TestCase):
    def test_existing_neural_detector_and_r_refinement_are_preserved(self):
        source = (REPO_ROOT / "src/rm_tracker/src/buff_recognizer.cpp").read_text(encoding="utf-8")
        detector = source[source.index("YoloBuffDetector::YoloBuffDetector("):
                          source.index("BuffRecognizer::BuffRecognizer(")]
        refinement = source[source.index("cv::Point2f BuffRecognizer::refineRCenter("):
                            source.index("int BuffRecognizer::selectTemporalDetection(")]
        # Compatibility contract from maintenance branch 2f4c164: model input,
        # classes, keypoint mapping, parser, NMS and CV refinement stay intact.
        self.assertIn("initModel(model_path, device)", detector)
        self.assertIn("conf_threshold", detector)
        for token in (
            "handleImage(bgr_roi, dilated_img)",
            "cv::findContours",
            "r_center_mask_ratio",
            "r_center_contour_distance_divisor",
            "r_center_refine_max_shift_px",
        ):
            self.assertIn(token, refinement)

    def test_big_acquisition_is_not_reset_when_waiting_for_three_good_frames(self):
        runtime = RUNTIME_SOURCE.read_text(encoding="utf-8")
        validation_start = runtime.index("if (pixel_track_ok) {")
        opening = runtime.index("{", validation_start)
        depth = 1
        closing = opening + 1
        while depth:
            depth += (runtime[closing] == "{") - (runtime[closing] == "}")
            closing += 1
        validation = runtime[opening:closing]
        decision = runtime[closing:runtime.index("track_ok = pixel_track_ok;", closing)]
        self.assertIn("g_big_buff_predictor.reject()", validation)
        self.assertNotIn("g_big_buff_predictor.reject()", decision)

    def test_small_and_big_predictors_consume_the_same_source_image_time(self):
        runtime = RUNTIME_SOURCE.read_text(encoding="utf-8")
        source = FU_SOURCE.read_text(encoding="utf-8")

        self.assertIn("g_frame_time_seconds", runtime)
        self.assertIn("g_frame_time_seconds * 1000.0", runtime)
        self.assertIn("g_frame_time_seconds,", runtime)
        self.assertIn("std::chrono::steady_clock::now()", runtime)
        self.assertIn('nh.subscribe("/hikrobot_camera/rgb", 1, &doimage)', source)
        self.assertIn("g_frame_time_seconds = g_frame_image_stamp.toSec();", runtime)
        self.assertNotIn("g_frame_monotonic_time_seconds", runtime)
        self.assertNotIn("steady_clock::time_point start", source)
        self.assertIn("g_frame_sequence == g_processed_frame_sequence", runtime)
        self.assertIn("bool fresh_frame", source)
        self.assertNotIn('if (display_frame) cv::imshow', source)
        self.assertNotIn("last_feed_time", runtime)

    def test_big_buff_refits_are_rate_limited(self):
        header = PREDICTOR_HEADER.read_text(encoding="utf-8")
        source = PREDICTOR_SOURCE.read_text(encoding="utf-8")

        self.assertIn("fit_every_n_samples = 5", header)
        self.assertIn("fit_samples_since_last_fit_", source)
        self.assertIn("config_.fit_every_n_samples", source)

    def test_big_buff_has_one_prediction_model_in_the_runtime(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = RUNTIME_SOURCE.read_text(encoding="utf-8")
        gate = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                "buff_big_prediction_gate.hpp").read_text(encoding="utf-8")

        self.assertRegex(config, r"buff_move_mode:\s*(small|big)")
        self.assertNotIn("buff_big_ransac_enabled:", config)
        self.assertNotIn('#include "buff_motion_predictor.hpp"', source)
        self.assertNotIn('#include "buff_big_variable_speed_predictor.hpp"', source)
        self.assertIn('param<std::string>("buff_move_mode"', source)
        self.assertIn('MoveMode == "big"', runtime)
        self.assertIn("g_big_buff_predictor.update", runtime)
        self.assertIn("g_big_buff_predictor.reject()", runtime)
        self.assertNotIn("Predicted->predicted", runtime)
        self.assertNotIn("default_omega", runtime)
        self.assertIn("kBigPredictionAcquireFrames = 3", gate)
        self.assertIn("g_buff_world_ekf_primary", runtime)
        self.assertNotIn("Prefer the Zhejiang University large-buff model", runtime)

    def test_target_priority_is_locked_by_default(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        recognizer = (REPO_ROOT / "src" / "rm_tracker" / "src" /
                      "buff_recognizer.cpp").read_text(encoding="utf-8")
        header = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                  "buff_target_association.hpp").read_text(encoding="utf-8")

        self.assertIn("buff_target_priority: locked", config)
        self.assertIn("buff_target_lock_timeout_seconds: 0.25", config)
        self.assertIn("setTargetPriority", recognizer)
        self.assertIn("association_.update", recognizer)
        self.assertIn("class BuffBladeAssociation", header)

    def test_world_ekf_observation_uses_shared_source_image_time(self):
        pnp = PNP_SOURCE.read_text(encoding="utf-8")
        build_start = pnp.index("bool buildWorldObservation")
        build_end = pnp.index("// Calculate the distance", build_start)
        build = pnp[build_start:build_end]
        self.assertIn("observation.stamp_seconds = g_frame_time_seconds;", build)
        self.assertNotIn("observation.stamp_seconds = g_frame_image_stamp.toSec();", build)
        self.assertIn("std::isfinite(g_frame_time_seconds)", build)

    def test_display_path_is_independent_from_fu_processing(self):
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = RUNTIME_SOURCE.read_text(encoding="utf-8")
        camera = (REPO_ROOT / "src" / "rm_hikcamera" / "src" /
                  "hikcamera.cpp").read_text(encoding="utf-8")

        # Fu keeps queue size 1 for the aiming path so it never works through
        # stale frames, while a second callback queue owns the live display.
        self.assertIn('nh.subscribe("/hikrobot_camera/rgb", 1, &doimage)', source)
        self.assertIn("ros::CallbackQueue g_display_callback_queue", source)
        self.assertIn("ros::AsyncSpinner display_spinner", source)
        self.assertIn("g_raw_display_queue.push", runtime)
        self.assertIn("g_processed_display_queue.push", source)
        self.assertIn("void buffDisplayLoop()", runtime)
        self.assertIn('cv::imshow("Image Live"', runtime)
        self.assertIn('cv::imshow("Image with Point"', runtime)
        # GUI calls must not remain in the inference/main-loop display block.
        self.assertNotIn("if (display_frame) cv::imshow", source)
        self.assertIn('advertiseCamera("/hikrobot_camera/rgb", 4)', camera)

    def test_processed_display_stream_only_carries_processed_frames(self):
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = RUNTIME_SOURCE.read_text(encoding="utf-8")
        # Bare camera frames must never enter the "Image with Point" stream:
        # alternating annotated/bare frames make the prediction point flash.
        self.assertIn("bool frame_processed = false;", source)
        self.assertIn(
            "if (frame_processed && (g_buff_show_image || g_buff_publish_debug_image) &&",
            source)
        self.assertIn("g_processed_display_queue.push(frame, g_frame_sequence);", source)
        self.assertNotIn("frame = std::move(frame_);", source)
        # The live window is rate-limited instead of driving highgui at camera rate.
        self.assertRegex(runtime, r"std::chrono::milliseconds\((33|50)\)")
        self.assertIn("bool raw_updated = false;", runtime)

    def test_detector_tensor_copy_uses_contiguous_rows(self):
        source = (REPO_ROOT / "src" / "rm_tracker" / "src" /
                  "buff_recognizer.cpp").read_text(encoding="utf-8")
        self.assertIn("const cv::Vec3f* row", source)
        self.assertIn("data[c * width * height + h * width + w]", source)
        self.assertNotIn("blob.at<cv::Vec<float, 3>>(h, w)[c]", source)


if __name__ == "__main__":
    unittest.main()
