# ZJU Big Buff Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the Zhejiang University large-buff RANSAC sine model and predictor to the existing ROS1/OpenVINO `Fu` node while preserving all existing interfaces and fallbacks.

**Architecture:** Port the small, dependency-light RANSAC fitter into `rm_tracker`, wrap it with a predictor that consumes the existing continuous image angle and timestamps, then select its predicted angle only when the model is ready and valid. The existing PnP, gravity compensation, world EKF, and legacy predictor remain unchanged and available as fallback paths.

**Tech Stack:** C++17, Eigen3, OpenCV, ROS1/catkin, existing rm_tracker tests.

## Global Constraints

- Keep the current OpenVINO detector and YOLO11 keypoint output unchanged.
- Do not add TensorRT, CUDA, C++20, or new ROS messages.
- Use `Fu_angleprocess::angle_polar_now` as the raw polar input; unwrap it inside the adapter and use seconds for timestamps.
- Preserve the existing legacy predictor and world-frame EKF fallback behavior.

---

### Task 1: RANSAC fitter core

**Files:**
- Create: `src/rm_tracker/include/buff_zju_ransac.hpp`
- Create: `src/rm_tracker/src/buff_zju_ransac.cpp`
- Create: `src/rm_tracker/test/buff_zju_ransac_test.cpp`
- Modify: `src/rm_tracker/CMakeLists.txt`

**Interfaces:**
- Produces `buff_zju::RansacSineFitter` with `add_data`, `fit`, `reset`, `relative_time`, and a public `Result` containing `A`, `omega`, `phi`, `C`, and `inliers`.

- [ ] **Step 1: Write the failing test**

  Add a deterministic fixture with `omega = 1.94`, `A = 0.90`, `phi = 0.35`, `C = 0.02`, at least 40 samples over 2 seconds, and three residual outliers. Assert after `fit()` that the result has at least 30 inliers, `omega` is within 0.08, and `A` is within 0.15 of the fixture.

- [ ] **Step 2: Run the test to verify it fails**

  Run `cmake --build build --target buff_zju_ransac_test -- -j2` from the workspace root. Expected: configuration/build fails because the new header/source/test do not exist yet.

- [ ] **Step 3: Write the minimal implementation**

  Port the approved Zhejiang University algorithm: keep a relative-time deque, sample three points per iteration, solve `[sin(omega*t), cos(omega*t), 1]` with Eigen SVD, reject amplitudes outside the configured range, count residual inliers, and retain the best model. Reset history when a new sample is more than five seconds after the last sample and cap history at 300 points.

- [ ] **Step 4: Add the target and run the test**

  Add the source to `rm_tracker`'s test-capable build and run the same command. Expected: the deterministic fitter test passes.

- [ ] **Step 5: Commit**

  Run `git add src/rm_tracker/include/buff_zju_ransac.hpp src/rm_tracker/src/buff_zju_ransac.cpp src/rm_tracker/test/buff_zju_ransac_test.cpp src/rm_tracker/CMakeLists.txt && git commit -m "feat: add large buff sine ransac fitter"`.

### Task 2: Predictor adapter

**Files:**
- Create: `src/rm_tracker/include/buff_big_ransac_predictor.hpp`
- Create: `src/rm_tracker/src/buff_big_ransac_predictor.cpp`
- Create: `src/rm_tracker/test/buff_big_ransac_predictor_test.cpp`
- Modify: `src/rm_tracker/CMakeLists.txt`

**Interfaces:**
- Produces `buff_zju::BigBuffRansacPredictor` with `reset`, `update(double, double)`, `ready`, `predict_roll(double)`, and `state`.

- [ ] **Step 1: Write the failing predictor tests**

  Test that a first sample initializes but does not make the model ready; test that a non-positive delta and a speed above `max_abs_speed` are rejected; test that a hand-derived model state with `A=0.9`, `omega=1.94`, `phi=0.35`, `C=0.02`, and `roll=0.4` predicts `roll + C*dt + A*(cos(phi)-cos(phi+omega*dt))/omega` for `dt=0.2` within `1e-6`.

- [ ] **Step 2: Run the tests to verify they fail**

  Run `cmake --build build --target buff_big_ransac_predictor_test -- -j2`. Expected: failure because the adapter files do not exist.

- [ ] **Step 3: Implement the adapter**

  Use the continuous `roll` input to calculate `roll_diff / dt`, reject invalid data, pass accepted velocities to `RansacSineFitter`, and expose a state snapshot. Before the model is ready, `predict_roll` returns the current roll plus the last accepted velocity times the horizon. Once ready, integrate the fitted sine velocity using the Zhejiang University formula and normalize the result to `[-pi, pi]` only at the public return boundary.

- [ ] **Step 4: Run the predictor tests**

  Run `cmake --build build --target buff_big_ransac_predictor_test -- -j2`. Expected: all predictor tests pass.

- [ ] **Step 5: Commit**

  Run `git add src/rm_tracker/include/buff_big_ransac_predictor.hpp src/rm_tracker/src/buff_big_ransac_predictor.cpp src/rm_tracker/test/buff_big_ransac_predictor_test.cpp src/rm_tracker/CMakeLists.txt && git commit -m "feat: add large buff predictor adapter"`.

### Task 3: Integrate with Fu without changing existing contracts

**Files:**
- Modify: `src/rm_tracker/src/Fu.cpp`
- Modify: `src/rm_tracker/include/buff_node_runtime.hpp`
- Modify: `src/rm_tracker/CMakeLists.txt`
- Modify: `src/rm_tracker/config/rm_fu.yaml`

**Interfaces:**
- Consumes `Angleprocess->angle_polar_now`; uses the image header timestamp in seconds when available and falls back to monotonic runtime milliseconds.
- Produces the same `final_angle_2D`, `euler_angles`, and `/auto_angle` messages as the existing path.

- [ ] **Step 1: Add a regression test fixture for the integration contract**

  Extend the predictor test with a finite-point check equivalent to `post_predict_angle_process`: for a finite roll and positive radius, the generated point must be finite; for an invalid prediction, the integration branch must be skipped.

- [ ] **Step 2: Run the regression test to verify the new integration assertion fails**

  Run `cmake --build build --target buff_big_ransac_predictor_test -- -j2`. Expected: the new assertion fails before the integration guard exists.

- [ ] **Step 3: Implement the minimal Fu integration**

  Construct one predictor with ROS private parameters using the Zhejiang defaults, reset it with the existing first-frame lifecycle and on serial activation/deactivation edges, call `update_polar(image_timestamp_seconds, Angleprocess->angle_polar_now)` after accepted angle processing, and in the image-phase prediction branch prefer `predict_roll(delta_t_)` only when `ready()` and the returned point is finite. Keep the existing Kalman and legacy predictor code as fallback and leave the world-EKF branch first. Consume each image sequence once, skip isolated speed outliers without clearing RANSAC history, correct five-blade ID switches with `2*pi/5` unwrap aliases, and re-anchor after persistent polar rejections.

- [ ] **Step 4: Build the Fu target and run all existing tracker tests**

  Run `catkin_make --pkg rm_tracker -j2` and then `ctest --test-dir build --output-on-failure` from the catkin workspace. Expected: `Fu` compiles and existing tests pass.

- [ ] **Step 5: Commit**

  Run `git add src/rm_tracker/src/Fu.cpp src/rm_tracker/include/buff_node_runtime.hpp src/rm_tracker/CMakeLists.txt src/rm_tracker/config/rm_fu.yaml && git commit -m "feat: integrate zju big buff predictor into Fu"`.

### Task 4: Final verification

**Files:**
- Modify: none unless verification exposes a defect.

- [ ] **Step 1: Inspect the complete diff**

  Run `git diff HEAD~3..HEAD --check` and `git status --short`; verify only the approved `rm_tracker` files and design documents changed, while pre-existing user changes remain untouched.

- [ ] **Step 2: Run fresh verification**

  Run `catkin_make --pkg rm_tracker -j2` and `ctest --test-dir build --output-on-failure`. Record exit codes and test counts before reporting completion.
