# 能量机关链路 ROS2 Humble 迁移设计

## 目标

将工程中用于能量机关（大符/小符）运行的 ROS1/catkin 代码迁移为 ROS2 Humble/ament，保持现有视觉算法、预测结果、串口协议、话题名和参数语义不变。迁移后的能量机关模式应可通过 ROS2 launch 独立启动相机、串口和 Fu 视觉节点。

## 范围

迁移以下运行链路：

`rm_msgs` -> ROS2 自定义消息包；

`rm_hikcamera` -> ROS2 相机图像/相机信息发布节点；

`rm_serial` -> ROS2 串口节点和无硬件模拟模式；

`rm_tracker` 中的 `Fu`、buff recognizer、角度处理、预测、PnP 和调试发布路径；

`rm_node` 中的能量机关启动文件。

装甲板识别链路（`rm_identify` 及装甲板专用 tracker 工作流）不纳入本次迁移；能量机关 launch 不启动这些节点。

## 保持不变的运行接口

### 话题

- 图像输入：`/hikrobot_camera/rgb`
- 串口状态输入：`/RmSerialData`
- 云台角度输出：`/auto_angle`，类型 `geometry_msgs/msg/Vector3`
- 俯仰调试输出：`/debugpub`，类型 `std_msgs/msg/Float64`
- 偏航调试输出：`/debugpub1`，类型 `std_msgs/msg/Float64`
- 其他现有 buff 调试话题保持原名和消息类型

### 消息字段

`rm_msgs/msg/Armor`、`ArmorArray`、`Debug`、`RmSerial` 保留字段顺序和数值含义。ROS1 `.msg` 中的 `char` 在 ROS2 接口中改为 `int8`，代码层通过显式转换继续使用 `'a'`、`'f'`、`'r'`、`'b'` 等模式字符。

### 参数

Fu 的模型路径、`fu_config_path`、buff 模式、世界坐标 EKF、PnP 标定、时间偏移和 RANSAC 参数保持原参数名和默认值。ROS2 参数文件使用节点名命名空间，但不改变参数键的末级名称。串口节点继续支持 `shootflag_test`、`camp_test`、`sim_mode` 和串口配置路径参数。

## 节点设计

### Fu 节点

将当前自由函数回调和 `while (ros::ok())` 主循环包装到 ROS2 `rclcpp::Node` 生命周期中：

- 图像和串口使用 subscription 回调更新现有状态；
- 使用 wall timer 驱动原有 `Fu_Task()`，确保每个图像序列只处理一次；
- 使用 `rclcpp::Time`/`rclcpp::Duration` 替换 ROS1 时间类型；
- 使用节点 logger 替换 ROS1 日志宏，并保留节流日志频率；
- 使用 ROS2 publisher 发布角度、调试值和 buff 状态；
- 节点析构时释放现有预测器、PnP 和识别器资源。

视觉算法文件（识别、角度差分、预测、RANSAC、PnP、坐标变换）保持算法顺序和数值计算不变，只做消息指针、时间和日志 API 适配。

### 串口节点

使用独立 `rclcpp::Node` 保存串口线程/设备状态。真实串口路径和协议编码不变；无串口时继续按现有模拟模式发布 `RmSerial` 与 IMU。`/auto_angle` subscription 改为 ROS2 类型别名，队列深度与当前配置保持等价。

### 相机节点

使用 ROS2 `image_transport` 发布图像和 `sensor_msgs/msg/CameraInfo`。保留 MVS 采集、BGR8 编码、`hikrobot_camera` frame_id、120 Hz 循环和空帧跳过逻辑。图像时间戳继续使用发布时钟，以保持 Fu 的时间配对行为。

## 构建和启动

- 所有迁移包改为 `ament_cmake`，消息包使用 `rosidl_default_generators`。
- 依赖声明改为 ROS2 包名：`rclcpp`、`rclpy`（仅在需要时）、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`tf2`、`tf2_ros`、`cv_bridge`、`image_transport`、`camera_info_manager`、`rosidl_default_runtime` 等。
- 新增 Python launch：`rm_node/launch/start_fan.launch.py`，等价启动相机、串口和 Fu，并暴露当前测试参数。
- 安装可执行文件、头文件、配置、模型路径所需的 launch 和参数文件，使 `ros2 launch rm_node start_fan.launch.py` 可直接使用。
- ROS1 XML launch 文件不作为 ROS2 运行入口；为避免误用，在文档中标明迁移后的入口。

## 数据流和行为保持

1. 相机发布带时间戳的图像。
2. 串口发布姿态、模式和阵营；`ShootFlag == 'f'` 激活 Fu，`'a'` 使 Fu 休眠。
3. Fu 在图像回调中记录帧序号，并在 timer 中消费新帧。
4. 首帧完成识别器、角度处理器、预测器、PnP 和飞行时间初始化。
5. 后续帧沿用现有识别、角速度/角度预测、重力补偿和云台角度发布流程。
6. 处理失败时保留现有打符模式保底角度发布；装甲板模式不发布 Fu 保底结果。
7. 串口模式切换时重置大符预测器及时间窗口，避免跨会话数据混用。

## 错误处理

- cv_bridge 转换失败：记录错误并跳过当前图像；
- 消息时间戳无效或 IMU 四元数非法：沿用当前历史姿态过滤和无匹配分支；
- 模型/配置加载失败：节点记录 fatal/error，拒绝进入不完整的处理循环；
- 串口不可用：沿用现有自动模拟模式；
- ROS2 shutdown：停止 timer、释放设备和算法对象后退出。

## 测试和验证

- 先增加 ROS2 消息字段和模式字符转换的单元测试，验证 `int8` 消息与旧协议值一致；
- 保留并运行现有 buff 识别、预测、RANSAC、角度处理测试；
- 在 ROS2 Humble Linux 环境运行 `colcon build --symlink-install`，再执行 `colcon test`；
- 使用模拟串口启动能量机关 launch，检查 `/RmSerialData`、图像、`/auto_angle`、调试话题和 `ShootFlag` 切换；
- 在没有 MVS/OpenVINO 设备的环境中至少完成编译、消息生成、节点加载和参数解析检查，并明确记录真实相机/模型运行未验证的部分。

## 非目标和约束

- 不重写 buff 数学模型和目标预测策略；
- 不改变现有话题名、消息字段含义、串口帧协议和模型文件格式；
- 不迁移与能量机关 launch 无关的装甲板识别节点；
- 当前 Windows 工作区没有 ROS2 Humble 工具链，最终构建验证需要在 Ubuntu 22.04 + ROS2 Humble 环境执行。
