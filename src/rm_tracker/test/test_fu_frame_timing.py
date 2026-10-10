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
    def test_small_and_big_predictors_consume_the_same_monotonic_frame_time(self):
        runtime = RUNTIME_SOURCE.read_text(encoding="utf-8")
        source = FU_SOURCE.read_text(encoding="utf-8")

        self.assertIn("g_frame_monotonic_time_seconds", runtime)
        self.assertIn("g_frame_monotonic_time_seconds * 1000.0", runtime)
        self.assertIn("g_frame_monotonic_time_seconds;", runtime)
        self.assertIn("std::chrono::steady_clock::now()", runtime)
        self.assertIn('nh.subscribe("/hikrobot_camera/rgb", 1, &doimage)', source)
        self.assertNotIn("g_frame_image_stamp.toSec()", source)
        self.assertNotIn("steady_clock::time_point start", source)
        self.assertIn("last_frame_time_seconds", runtime)
        self.assertIn("frame_interval_seconds", runtime)
        self.assertNotIn("last_feed_time", runtime)

    def test_big_buff_refits_are_rate_limited(self):
        header = PREDICTOR_HEADER.read_text(encoding="utf-8")
        source = PREDICTOR_SOURCE.read_text(encoding="utf-8")

        self.assertIn("fit_every_n_samples = 5", header)
        self.assertIn("fit_samples_since_last_fit_", source)
        self.assertIn("config_.fit_every_n_samples", source)

    def test_variable_speed_prediction_remains_enabled_for_big_buff(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        source = FU_SOURCE.read_text(encoding="utf-8")
        runtime = RUNTIME_SOURCE.read_text(encoding="utf-8")
        gate = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                "buff_big_prediction_gate.hpp").read_text(encoding="utf-8")

        self.assertIn("buff_move_mode: small", config)
        self.assertIn("buff_big_variable_speed_enabled: true", config)
        self.assertIn("g_buff_big_variable_speed_enabled = true", source)
        self.assertIn('param<std::string>("buff_move_mode"', source)
        self.assertIn('MoveMode == "big"', runtime)
        self.assertIn("g_big_variable_speed_prediction_valid", runtime)
        self.assertIn("variable_speed_update_ok", runtime)
        self.assertIn("g_big_prediction_gate.update", runtime)
        self.assertIn("g_big_prediction_gate.reject()", runtime)
        self.assertIn("kBigPredictionAcquireFrames = 3", gate)
        self.assertIn("g_buff_world_ekf_primary", runtime)
        self.assertNotIn("Prefer the Zhejiang University large-buff model", runtime)

    def test_target_priority_is_locked_by_default(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        recognizer = (REPO_ROOT / "src" / "rm_tracker" / "src" /
                      "buff_recognizer.cpp").read_text(encoding="utf-8")
        header = (REPO_ROOT / "src" / "rm_tracker" / "include" /
                  "buff_detection_stabilizer.hpp").read_text(encoding="utf-8")

        self.assertIn("buff_target_priority: locked", config)
        self.assertIn("buff_target_lock_timeout_seconds: 0.25", config)
        self.assertIn("setTargetPriority", recognizer)
        self.assertIn("target_lock_.select", recognizer)
        self.assertIn("class BuffTargetLock", header)

    def test_world_ekf_observation_uses_shared_monotonic_frame_time(self):
        pnp = PNP_SOURCE.read_text(encoding="utf-8")
        build_start = pnp.index("bool buildWorldObservation")
        build_end = pnp.index("// Calculate the distance", build_start)
        build = pnp[build_start:build_end]
        self.assertIn("observation.stamp_seconds = g_frame_monotonic_time_seconds;", build)
        self.assertNotIn("observation.stamp_seconds = g_frame_image_stamp.toSec();", build)
        self.assertIn("std::isfinite(g_frame_monotonic_time_seconds)", build)


if __name__ == "__main__":
    unittest.main()
