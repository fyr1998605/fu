# Fu YAML Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** Make `src/rm_tracker/config/rm_fu.yaml` the single editable source for Fu runtime tuning while preserving current defaults and direct `rm_fu.launch` execution.

**Architecture:** Keep ROS launch responsible only for selecting the Fu YAML file. Add typed configuration objects at the recognizer and prediction boundaries; load and validate all runtime values once in `Fu.cpp`, then pass validated objects to modules. Keep fixed tensor dimensions, keypoint layout, and structural safety constants in code.

**Tech Stack:** ROS1 parameters/rosparam, C++17, OpenCV, existing CTest/Python contract tests, YAML configuration.

## Global Constraints

- The canonical configuration file is `src/rm_tracker/config/rm_fu.yaml`.
- Existing parameter names and default behavior remain compatible.
- Invalid finite/range/enum values fall back to current defaults with a warning; structural configuration errors still fail startup.
- `rm_fu.launch` keeps `fu_config` defaulting to `$(find rm_tracker)/config/rm_fu.yaml`.
- No model, tensor, keypoint, unit, or ROS message schema changes.

### Task 1: Extend configuration contract tests before production changes

**Files:**
- Modify: `src/rm_tracker/test/test_fu_config_layout.py`

**Interfaces:** The tests describe required YAML keys and source/launch loading behavior for later tasks.

- [ ] **Step 1: Add failing assertions** for YAML-owned model path, enemy color, detector thresholds, recognizer refinement, small/big prediction tuning, angle compensation, output limits, and launch removal of model override parameters.
- [ ] **Step 2: Run the focused Python test**

  Run: `python -m unittest src/rm_tracker/test/test_fu_config_layout.py`

  Expected: FAIL because the new keys and source wiring do not exist yet.

### Task 2: Centralize the editable YAML surface

**Files:**
- Modify: `src/rm_tracker/config/rm_fu.yaml`
- Modify: `src/rm_tracker/launch/rm_fu.launch`

**Interfaces:** YAML keys are loaded into the Fu private namespace; launch exposes only `fu_config` by default.

- [ ] **Step 1: Add the grouped tuning block** at the top of `rm_fu.yaml`, using current code defaults for every newly exposed setting.
- [ ] **Step 2: Remove launch-level model/mode overrides** so `rosparam file="$(arg fu_config)"` supplies the editable values.
- [ ] **Step 3: Run YAML/XML parsing checks** and keep the existing coordinate-transform structure intact.

### Task 3: Make recognizer tuning configurable

**Files:**
- Modify: `src/rm_tracker/include/buff_recognizer.hpp`
- Modify: `src/rm_tracker/src/buff_recognizer.cpp`
- Modify: `src/rm_tracker/src/Fu.cpp`

**Interfaces:** Add `BuffRecognizerConfig` and construct `BuffRecognizer(model_path, enemy_color, config)`; preserve the old constructor defaults for independent `buff_test` callers.

- [ ] **Step 1: Add the minimal config type and constructor overload** for detector confidence/NMS, stabilizer alpha, and R-center refinement values.
- [ ] **Step 2: Replace hardcoded recognizer members with validated config values.**
- [ ] **Step 3: Read and validate the corresponding ROS parameters in `Fu.cpp`, then pass the config to the recognizer.**
- [ ] **Step 4: Run recognizer/config tests and compile the focused recognizer target when available.**

### Task 4: Make prediction and output tuning configurable

**Files:**
- Modify: `src/rm_tracker/include/blade_tracker.hpp`
- Modify: `src/rm_tracker/include/buff_target_association.hpp`
- Modify: `src/rm_tracker/include/buff_big_prediction_gate.hpp`
- Modify: `src/rm_tracker/include/buff_node_runtime.hpp`
- Modify: `src/rm_tracker/src/Fu.cpp`

**Interfaces:** Add explicit config objects/constructors for small phase tracking, blade association, big prediction gate, pitch/yaw compensation, pitch limits, and stuck recovery. Defaults exactly match current constants.

- [ ] **Step 1: Add failing source-contract assertions for config construction and parameter reads.**
- [ ] **Step 2: Add typed config defaults and replace direct constant reads in the active paths.**
- [ ] **Step 3: Load, sanitize, and log the values from `Fu.cpp`.**
- [ ] **Step 4: Run focused blade/prediction tests and the Python contract test.**

### Task 5: Finish validation and documentation

**Files:**
- Modify: `src/rm_tracker/README_buff_upgrade.md`
- Modify: `src/rm_tracker/test/test_fu_config_layout.py` if a discovered contract needs clarification

- [ ] **Step 1: Document the YAML tuning groups and direct launch command.**
- [ ] **Step 2: Run `git diff --check`.**
- [ ] **Step 3: Run `python -m unittest src/rm_tracker/test/test_fu_config_layout.py src/rm_tracker/test/test_fu_frame_timing.py`.**
- [ ] **Step 4: Run the available native Buff CTest suite through `scripts/test_buff.sh` or the focused CMake build.**
- [ ] **Step 5: Inspect the final diff and report any unavailable ROS/build checks explicitly.**
