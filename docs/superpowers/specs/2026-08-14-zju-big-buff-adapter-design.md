# ZJU Big Buff RANSAC Adapter Design

Status: approved for implementation on 2026-08-14.

## Goal

Reuse the Zhejiang University large-buff motion model in the existing ROS1/OpenVINO `Fu` node without replacing the current detector, PnP solver, ROS messages, or world-frame EKF.

## Non-goals

- Do not replace the current OpenVINO `BuffRecognizer` or its YOLO keypoint model.
- Do not add TensorRT, CUDA, or C++20 dependencies.
- Do not replace `PnPSolver_gravity`, the ROS topics, or the serial protocol.
- Do not remove the legacy FFT/Ceres predictor; it remains the fallback until the new model is ready.

## Architecture

The existing recognizer provides `g_R_center`, `g_Fan_center`, and PnP corners. `Fu_angleprocess` exposes the current raw polar angle (`angle_polar_now`) and the current sample time is available in milliseconds. A new `buff_zju::BigBuffRansacPredictor` unwraps that raw angle, computes instantaneous angular velocity, and feeds the Zhejiang University RANSAC sine fitter:

```text
BuffRecognizer/OpenVINO
  -> Fu_angleprocess::angle_polar_now
  -> BigBuffRansacPredictor::update_polar(timestamp_seconds, raw_angle)
  -> RANSAC v(t) = A*sin(omega*t + phi) + C
  -> BigBuffRansacPredictor::predict_roll(horizon)
  -> Fu_angleprocess::post_predict_angle_process
  -> existing PnP + gravity compensation + ROS publish
```

The world-frame EKF path remains higher priority. If the RANSAC model is not ready, prediction is invalid, or the input stream has a timing/data error, the existing Kalman/legacy predictor path is used.

The polar adapter also preserves the legacy five-blade equivalence: unwrap
candidates are generated in multiples of `2*pi/5` and selected by the speed,
direction, and prior-velocity gates. This prevents a detector ID switch from
being treated as a physical 72-degree velocity spike.

## Interfaces

`buff_zju::BigBuffRansacPredictor` owns the model lifecycle and exposes:

- `reset()` to clear history and model state.
- `update(timestamp_seconds, continuous_roll)` to accept one observation.
- `update_polar(timestamp_seconds, raw_angle)` to unwrap detector angles across `2*pi` before fitting.
- `ready()` to report whether the configured inlier threshold is met.
- `predict_roll(horizon_seconds)` to integrate the fitted velocity model.
- `state()` for diagnostics (roll, velocity, amplitude, omega, phase, offset, inliers).

The default RANSAC limits retain the source model bounds: 200 iterations, 0.5 rad/s residual threshold, omega in [1.884, 2.0] rad/s, amplitude in [0.780, 1.045] rad/s, and 100 minimum inliers. The runtime accepts up to 3.2 rad/s instantaneous speed to cover the observed large-buff peak, and a five-second sample gap resets the fitter history. A single rejected sample is skipped without clearing the valid fitting window; repeated rejections re-anchor the polar baseline.

## Fu integration

`Fu.cpp` constructs/resets the predictor with ROS private parameters only when `buff_move_mode` is explicitly `big` and the feature flag is enabled. `buff_node_runtime.hpp` updates it immediately after the existing angle processor has accepted a frame. When the predictor is ready and the world EKF branch is not active, its predicted roll is converted through the existing `post_predict_angle_process` and passed to the unchanged PnP/gravity path. All legacy fallback code remains reachable; the default `buff_move_mode: small` cannot be hijacked by the large-buff model.

## Failure handling

- Reject non-finite timestamps, angles, non-positive time deltas, and speeds above the configured limit.
- Unwrap normal polar-angle crossings and reject physically impossible jumps.
- Reset the RANSAC history on a timestamp gap beyond five seconds.
- Reset the predictor at serial activation/deactivation edges so separate Fu
  sessions cannot share roll or timestamp history.
- Never publish a non-finite predicted point.
- Keep the previous behavior when recognition fails, RANSAC has too few inliers, or the predicted point fails current validity checks.

## Verification

- Unit-test fitting on deterministic sine data with injected outliers.
- Unit-test integration of the velocity model and constant-speed fallback.
- Build the `Fu` target and run the existing tracker tests to verify that the original OpenVINO/PnP/world-EKF paths still compile and run.
