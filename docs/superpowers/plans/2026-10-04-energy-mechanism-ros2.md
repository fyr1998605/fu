# 能量机关 ROS2 Humble Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将能量机关运行链路迁移为 ROS2 Humble/ament，并保持现有 buff 识别、预测、PnP、串口模式切换和角度输出行为一致。

**Architecture:** 保留现有 buff 算法类和数值处理流程，将 ROS1 节点边界替换为 ROS2 `rclcpp::Node`、ROS2 interfaces、ROS2 `image_transport` 和 Python launch。Fu 使用 subscription 更新图像/串口状态、wall timer 消费新帧；串口和相机保持独立节点，通过原有话题连接。

**Tech Stack:** ROS2 Humble, C++17, `ament_cmake`, `rclcpp`, `rosidl_default_generators`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`, `image_transport`, OpenCV, Eigen, Ceres, yaml-cpp, OpenVINO, MVS SDK.

## Global Constraints

- 保留 `/hikrobot_camera/rgb`、`/RmSerialData`、`/auto_angle`、`/debugpub`、`/debugpub1` 等现有话题名。
- `rm_msgs` 消息字段顺序和数值含义不变；ROS1 `char` 改为 ROS2 `int8`，代码中显式转换为模式字符。
- 不重写 buff 数学模型、识别器、预测器、RANSAC、PnP 和串口帧协议。
- 能量机关 launch 不依赖 `rm_identify` 和装甲板专用 tracker。
- 参数名和默认值保持不变；ROS2 参数文件使用节点命名空间。
- 在当前 Windows 工作区无法运行 ROS2 Humble、MVS 或 Linux OpenVINO；必须在 Ubuntu 22.04 + ROS2 Humble 环境做最终编译和运行验证。
- 每项实现遵循先写失败测试、确认失败、再写最小实现、确认通过的 TDD 顺序。

---

### Task 1: Convert `rm_msgs` to ROS2 interfaces

**Files:**
- Modify: `src/rm_msgs/CMakeLists.txt`
- Modify: `src/rm_msgs/package.xml`
- Modify: `src/rm_msgs/msg/RmSerial.msg`
- Modify: `src/rm_msgs/msg/Armor.msg`
- Modify: `src/rm_msgs/msg/ArmorArray.msg`
- Modify: `src/rm_msgs/msg/Debug.msg`
- Create: `src/rm_msgs/test/test_message_definitions.py`

**Interfaces:**
- Produces generated C++ headers under `rm_msgs/msg` for `Armor`, `ArmorArray`, `Debug`, and `RmSerial`.
- `RmSerial.camp`, `RmSerial.shoot_flag`, `RmSerial.i_s_auto_fire`, and `RmSerial.post_alive` are `int8` fields matching the old byte values.

- [ ] **Step 1: Write the failing interface test**

Create a Python test that parses the four `.msg` files and asserts the ROS2-safe field declarations, especially:

```python
assert "int8 Camp" in serial_text
assert "int8 ShootFlag" in serial_text
assert "std_msgs/Header header" in serial_text
```

- [ ] **Step 2: Run the test to verify it fails**

Run `python3 -m pytest src/rm_msgs/test/test_message_definitions.py -q`.
Expected: failure because the current package still declares ROS1 `char` fields and uses catkin generation.

- [ ] **Step 3: Implement ROS2 interface generation**

Replace catkin macros with `find_package(ament_cmake REQUIRED)`, `find_package(rosidl_default_generators REQUIRED)`, `rosidl_generate_interfaces(${PROJECT_NAME} ... DEPENDENCIES std_msgs geometry_msgs)`, and `ament_export_dependencies(rosidl_default_runtime)`. Update package metadata to `ament_cmake`, `rosidl_default_generators`, `rosidl_default_runtime`, `std_msgs`, and `geometry_msgs`. Change only unsupported `char` declarations to `int8`.

- [ ] **Step 4: Run the interface test to verify it passes**

Run the same pytest command and then, in ROS2 Humble, run `colcon build --packages-select rm_msgs` followed by `ros2 interface show rm_msgs/msg/RmSerial`. Confirm the generated fields and header dependency.

- [ ] **Step 5: Commit**

```bash
git add src/rm_msgs
git commit -m "build: convert rm_msgs to ROS2 interfaces"
```

### Task 2: Add ROS2 mode and message conversion helpers

**Files:**
- Create: `src/rm_tracker/include/ros2_energy_helpers.hpp`
- Create: `src/rm_tracker/test/ros2_energy_helpers_test.cpp`
- Modify: `src/rm_tracker/CMakeLists.txt`

**Interfaces:**
- `char rm_msgs_int8_to_char(int8_t value)` returns the byte as a character without sign-extension surprises.
- `bool is_energy_mode(int8_t shoot_flag)` returns true only for `'f'`.
- `bool is_armor_mode(int8_t shoot_flag)` returns true only for `'a'`.
- `char camp_to_internal(int8_t camp)` preserves the existing red/blue inversion behavior used by Fu.

- [ ] **Step 1: Write the failing unit tests**

Add gtest cases for `'f'`, `'a'`, unknown values, signed `int8_t`, and camp mapping:

```cpp
EXPECT_TRUE(is_energy_mode(static_cast<int8_t>('f')));
EXPECT_FALSE(is_energy_mode(static_cast<int8_t>('a')));
EXPECT_EQ(camp_to_internal(static_cast<int8_t>('b')), 'r');
EXPECT_EQ(camp_to_internal(static_cast<int8_t>('r')), 'b');
```

- [ ] **Step 2: Run the tests to verify they fail**

Run `colcon test --packages-select rm_tracker --ctest-args -R ros2_energy_helpers_test` after configuring the ROS2 package skeleton. Expected: compile failure because the helper header and ROS2 target do not exist.

- [ ] **Step 3: Implement the helpers and test target**

Implement the four inline functions without ROS dependencies. Add ROS2 `ament_cmake` test discovery and a gtest executable linked only against the helper header.

- [ ] **Step 4: Run the tests to verify they pass**

Run `colcon test --packages-select rm_tracker --ctest-args -R ros2_energy_helpers_test` and inspect `colcon test-result --verbose` for zero failures.

- [ ] **Step 5: Commit**

```bash
git add src/rm_tracker/include/ros2_energy_helpers.hpp src/rm_tracker/test/ros2_energy_helpers_test.cpp src/rm_tracker/CMakeLists.txt
git commit -m "test: cover ROS2 energy mode byte conversions"
```

### Task 3: Convert the Fu energy-mechanism node

**Files:**
- Modify: `src/rm_tracker/src/Fu.cpp`
- Modify: `src/rm_tracker/include/buff_node_runtime.hpp`
- Modify: `src/rm_tracker/CMakeLists.txt`
- Modify: `src/rm_tracker/package.xml`
- Create: `src/rm_tracker/test/fu_node_api_test.cpp`

**Interfaces:**
- Executable remains named `Fu` and is installed as a ROS2 executable.
- Subscriptions: `sensor_msgs::msg::Image` on `/hikrobot_camera/rgb`; `rm_msgs::msg::RmSerial` on `/RmSerialData`.
- Publishers: `geometry_msgs::msg::Vector3` on `/auto_angle`; existing `std_msgs::msg::Float64` debug publishers and buff status publishers.
- Fu retains a `run_processing_cycle()`/timer callback that invokes the existing `Fu_Task()` once per unseen `g_frame_sequence`.

- [ ] **Step 1: Write the failing node API test**

Create a source-level test that builds the ROS2 Fu target and checks its exported executable name and required topic string constants. Add a small pure function test for the “new image sequence only” gate so a repeated timer tick returns without processing.

- [ ] **Step 2: Run the test to verify it fails**

Run `colcon test --packages-select rm_tracker --ctest-args -R fu_node_api_test`. Expected: package configuration fails because `Fu.cpp` still includes ROS1 headers and catkin targets.

- [ ] **Step 3: Port the node boundary, preserving algorithm code**

Replace ROS1 includes and publisher/subscriber types with ROS2 equivalents. Introduce an `EnergyFuNode : public rclcpp::Node` (or an equivalent node-owned context) that:

```cpp
image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/hikrobot_camera/rgb", rclcpp::SensorDataQoS(),
    std::bind(&EnergyFuNode::doimage, this, std::placeholders::_1));
serial_sub_ = create_subscription<rm_msgs::msg::RmSerial>(
    "/RmSerialData", rclcpp::QoS(1000),
    std::bind(&EnergyFuNode::doserial, this, std::placeholders::_1));
timer_ = create_wall_timer(10ms, std::bind(&EnergyFuNode::run_processing_cycle, this));
```

Use `declare_parameter`/`get_parameter` for every existing Fu parameter. Replace `ros::Time`/`ros::Duration` with `rclcpp::Time`/`rclcpp::Duration`, `ROS_*` macros with `RCLCPP_*`, and `ros::spinOnce()` with `rclcpp::spin`. Preserve all frame gating, IMU history, model initialization, fallback angle publication, and shutdown cleanup.

- [ ] **Step 4: Run focused tests and compile**

Run `colcon build --packages-select rm_msgs rm_tracker --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`, then `colcon test --packages-select rm_tracker` and `colcon test-result --verbose`. Confirm no ROS1 headers, `ros::`, `catkin`, or `message_generation` references remain in the Fu compilation path.

- [ ] **Step 5: Commit**

```bash
git add src/rm_tracker/src/Fu.cpp src/rm_tracker/include/buff_node_runtime.hpp src/rm_tracker/CMakeLists.txt src/rm_tracker/package.xml src/rm_tracker/test/fu_node_api_test.cpp
git commit -m "feat: port energy mechanism Fu node to ROS2"
```

### Task 4: Convert the serial node and simulation mode

**Files:**
- Modify: `src/rm_serial/src/serial.cpp`
- Modify: `src/rm_serial/CMakeLists.txt`
- Modify: `src/rm_serial/package.xml`
- Modify: `src/rm_serial/config/rm_serial.yaml`
- Create: `src/rm_serial/test/serial_mode_test.cpp`

**Interfaces:**
- Publishes `rm_msgs::msg::RmSerial` on `/RmSerialData` and `sensor_msgs::msg::Imu` on `imu/data_raw`.
- Subscribes to `geometry_msgs::msg::Vector3` on `/auto_angle`.
- Parameters `shootflag_test`, `camp_test`, `sim_mode`, and `armor_serial_config` retain current names and values.

- [ ] **Step 1: Write failing simulation tests**

Test that a simulation configuration with `shootflag_test=f` and `camp_test=r` produces `int8_t('f')` and `int8_t('r')`, and that the auto-angle callback updates the outgoing pitch/yaw state.

- [ ] **Step 2: Run tests to verify failure**

Run the serial package test target. Expected: failure because the node is still a ROS1 `ros::NodeHandle` implementation and the ROS2 message type is not available.

- [ ] **Step 3: Port serial node**

Convert the class to `rclcpp::Node`, replace private parameter lookup with declared parameters, use ROS2 clocks for message headers, use a wall timer for the existing receive/publish loop, and map all byte fields through `ros2_energy_helpers.hpp`. Preserve real-port fallback-to-simulation behavior, protocol encoding, and `/auto_angle` callback semantics.

- [ ] **Step 4: Run compile and simulation tests**

Run `colcon build --packages-select rm_serial --symlink-install`, then `colcon test --packages-select rm_serial` and launch the node with `sim_mode:=true`. Use `ros2 topic echo /RmSerialData` to confirm mode bytes and timestamps.

- [ ] **Step 5: Commit**

```bash
git add src/rm_serial
git commit -m "feat: port serial energy control node to ROS2"
```

### Task 5: Convert the Hikrobot camera node

**Files:**
- Modify: `src/rm_hikcamera/src/hikcamera.cpp`
- Modify: `src/rm_hikcamera/include/hikrobot_camera.hpp`
- Modify: `src/rm_hikcamera/CMakeLists.txt`
- Modify: `src/rm_hikcamera/package.xml`
- Create: `src/rm_hikcamera/test/camera_publish_api_test.cpp`

**Interfaces:**
- Publishes `/hikrobot_camera/rgb` through ROS2 `image_transport` with `BGR8` encoding and `hikrobot_camera` frame id.
- Publishes matching `sensor_msgs::msg::CameraInfo` timestamps.

- [ ] **Step 1: Write the failing camera API test**

Test the conversion helper or node constants for BGR8 encoding, frame id, topic name, and empty-frame skip behavior.

- [ ] **Step 2: Run the test to verify failure**

Run the focused test; expected failure because the implementation uses ROS1 `image_transport::ImageTransport` and `ros::Time`.

- [ ] **Step 3: Port camera publishing**

Use `rclcpp::Node`, `image_transport::create_camera_publisher`, ROS2 cv_bridge message types, a wall timer or controlled loop at 120 Hz, and `node->get_clock()->now()`. Preserve MVS acquisition, optional crop compile flag, BGR8 encoding, frame id, and empty-frame handling.

- [ ] **Step 4: Build and run the API test**

Run `colcon build --packages-select rm_hikcamera --symlink-install` and `colcon test --packages-select rm_hikcamera`; when MVS is unavailable, verify configuration and compilation with the same include/library checks used by the target and record hardware execution as unavailable.

- [ ] **Step 5: Commit**

```bash
git add src/rm_hikcamera
git commit -m "feat: port Hikrobot camera publisher to ROS2"
```

### Task 6: Add ROS2 parameters, installation, and energy launch

**Files:**
- Modify: `src/rm_node/package.xml`
- Modify: `src/rm_node/CMakeLists.txt`
- Create: `src/rm_node/launch/start_fan.launch.py`
- Create: `src/rm_node/launch/start.launch.py`
- Create: `src/rm_tracker/config/fu_ros2.yaml`
- Create: `src/rm_node/test/test_launch_description.py`

**Interfaces:**
- `ros2 launch rm_node start_fan.launch.py shootflag_test:=f camp_test:=r sim_mode:=true` starts only camera, serial, and Fu.
- `ros2 launch rm_node start.launch.py` preserves the energy-only portion and accepts the existing model/config path arguments.

- [ ] **Step 1: Write the failing launch test**

Load the launch descriptions with `LaunchService` and assert that the generated actions include executables `hikcamera`, `serial`, and `Fu`, pass `shootflag_test`, `camp_test`, and `sim_mode`, and do not include `rm_identify` or `tracker` in `start_fan.launch.py`.

- [ ] **Step 2: Run the launch test to verify failure**

Run `python3 -m pytest src/rm_node/test/test_launch_description.py -q`. Expected: import or assertion failure because the Python launch files do not exist.

- [ ] **Step 3: Implement launch and installation**

Write Python launch files using `DeclareLaunchArgument`, `Node`, `IncludeLaunchDescription`, and `get_package_share_directory`. Install launch/config/model-adjacent files with `ament_cmake`. Convert the Fu YAML into ROS2 node-parameter format while preserving all keys and values.

- [ ] **Step 4: Run launch tests and parameter inspection**

Run the pytest launch test, `colcon build --packages-select rm_node`, then `ros2 launch rm_node start_fan.launch.py --show-args`. Confirm all declared arguments and node parameters are visible.

- [ ] **Step 5: Commit**

```bash
git add src/rm_node src/rm_tracker/config/fu_ros2.yaml
git commit -m "build: add ROS2 energy mechanism launch files"
```

### Task 7: Full ROS2 Humble build and simulated integration verification

**Files:**
- Modify: `docs/superpowers/specs/2026-10-04-energy-mechanism-ros2-design.md` only if verification limitations need recording.
- Create: `docs/ros2_humble_energy_mechanism.md`

**Interfaces:**
- Documents exact environment setup, build commands, topic checks, and known hardware limitations.

- [ ] **Step 1: Run the complete build**

In Ubuntu 22.04 with ROS2 Humble sourced, run:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Expected: all migrated packages compile with exit code 0.

- [ ] **Step 2: Run all tests**

Run `colcon test --event-handlers console_direct+` and `colcon test-result --verbose`. Record package/test counts and any tests skipped because MVS/OpenVINO hardware is absent.

- [ ] **Step 3: Run simulated energy launch**

Run `ros2 launch rm_node start_fan.launch.py shootflag_test:=f camp_test:=r sim_mode:=true`, then verify:

```bash
ros2 topic list
ros2 topic echo /RmSerialData --once
ros2 topic echo /auto_angle --once
ros2 node list
```

Switch the simulation flag to `a` and confirm Fu stops processing/publishing its fallback angle while the serial node continues publishing state.

- [ ] **Step 4: Document results**

Write the exact Humble version, build/test commands, exit codes, topic observations, and hardware limitations to `docs/ros2_humble_energy_mechanism.md`.

- [ ] **Step 5: Commit verification documentation**

```bash
git add docs/ros2_humble_energy_mechanism.md
git commit -m "docs: record ROS2 energy mechanism verification"
```

## Plan Self-Review

- Spec coverage: message generation is Task 1; mode byte compatibility is Task 2; Fu behavior is Task 3; serial and camera nodes are Tasks 4-5; launch/install and parameters are Task 6; build, test, and simulated runtime checks are Task 7.
- Placeholder scan: no unfinished marker or unspecified implementation step is used; each task names files, interfaces, commands, expected failures, and commits.
- Type consistency: all ROS2 callbacks use `::SharedPtr`, all `RmSerial` byte fields are `int8`, publishers/subscribers use the topic types listed in the design, and launch executable names match installed targets.
