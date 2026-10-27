#pragma once

#include <string>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <utility>

#include "buff_big_predictor.hpp"
#include "buff_frame_timing.hpp"

// Buff callbacks and per-frame detection, prediction and PnP processing.

double buffWallSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void publishBuffCommand() {
    PubAngle.publish(Angle);
    g_buff_control_state.published(static_cast<int>(Angle.z), buffWallSeconds());
}

void publishBuffInactiveCommand(bool observed) {
    Angle.x = -RmSerialData.Pitch;
    Angle.y = RmSerialData.Yaw;
    Angle.z = BuffControlState::code(observed, false);
    publishBuffCommand();
    std_msgs::Float64 current_yaw, invalid_angle;
    current_yaw.data = RmSerialData.Yaw;
    invalid_angle.data = std::numeric_limits<double>::quiet_NaN();
    debugpub.publish(current_yaw);
    debugpub1.publish(invalid_angle);
    expected_pitch_pub.publish(invalid_angle);
    expected_yaw_pub.publish(invalid_angle);
    std_msgs::Bool world_ready;
    world_ready.data = false;
    world_ready_pub.publish(world_ready);
}

void prepareBuffSourceFrame(const BufferedBuffImage& source) {
    BuffStageTimer copy(g_buff_frame_timing, BuffTimingStage::Copy);
    // The callback borrows immutable message pixels. Only the selected image
    // is copied into algorithm-owned storage, independently of HUD drawing.
    // frame_ is consumed by init()/identify() before any drawing, so the HUD
    // can share the same buffer instead of paying a second 3.75MB clone.
    frame_ = source.pixels->image.clone();
    frame = frame_;
    g_frame_header = source.header;
    g_frame_image_stamp = g_frame_header.stamp;
    g_frame_time_seconds = g_frame_image_stamp.toSec();
    g_frame_sequence = source.sequence;
}

void pruneBuffSourceImages() {
    const auto oldest = g_buff_time_sync.oldestImageSequence();
    while (!g_pending_buff_images.empty() &&
           (oldest == 0 || g_pending_buff_images.front().sequence < oldest))
        g_pending_buff_images.pop_front();
}

bool observeBuffClock() {
    if (!g_buff_time_sync.observeClock(ros::Time::now().toSec())) return false;
    if (functional) publishBuffInactiveCommand(false);
    g_buff_control_state.reset();
    g_big_buff_predictor.reset();
    g_small_buff_tracker = SmallBuffPhaseTracker(g_small_buff_config);
    g_buff_world_tracker = BuffWorldTracker{};
    g_buff_world_tracker.configure(g_buff_world_config);
    if (g_recognizer) g_recognizer->resetTracking();
    g_buff_rotation_sign = 0;
    g_pending_buff_images.clear();
    g_prepared_frame_sequence = g_received_frame_sequence;
    g_processed_frame_sequence = g_frame_sequence;
    g_last_received_image_stamp = ros::Time(0);
    g_has_frame_imu_pose = g_buff_prediction_ready = g_buff_sync_waiting = false;
    g_buff_stuck_frames = g_buff_recovery_remaining = 0;
    g_buff_recovery_active = false;
    g_frame_image_stamp = ros::Time(0);
    g_frame_time_seconds = 0.0;
    frame_.release();
    frame.release();
    functional = false;
    RmSerialData = rm_msgs::RmSerial{};
    vision_delay_ema_seconds = vision_delay_measured_seconds = -1.0;
    delta_t_ = 0.3;
    ROS_WARN("[BUFF_SYNC] ROS clock moved backwards: cleared frames, IMU history and all prediction state");
    return true;
}

void acceptBuffFramePose(const BuffPoseMatch& matched) {
    g_frame_serial_data = RmSerialData; // Non-pose mode/ammo fields retain live feedback.
    g_frame_serial_data.header = g_frame_header;
    g_frame_serial_data.Roll = matched.pose.rpy.x();
    g_frame_serial_data.Pitch = matched.pose.rpy.y();
    g_frame_serial_data.Yaw = matched.pose.rpy.z();
    const auto& q = matched.pose.imu_to_map;
    g_frame_serial_data.Qw = q.w(); g_frame_serial_data.Qx = q.x();
    g_frame_serial_data.Qy = q.y(); g_frame_serial_data.Qz = q.z();
    g_frame_imu_pose = {g_frame_image_stamp, q, q.norm()};
    g_has_frame_imu_pose = true;
    g_frame_imu_image_skew_seconds = matched.pose.stamp_seconds - g_frame_time_seconds;
    std_msgs::Float64 residual;
    residual.data = g_frame_imu_image_skew_seconds * 1000.0;
    g_buff_pose_time_error_pub.publish(residual);
    geometry_msgs::Vector3Stamped bracket;
    bracket.header = g_frame_header;
    bracket.vector.x = (matched.lower_stamp_seconds - g_frame_time_seconds) * 1000.0;
    bracket.vector.y = (matched.upper_stamp_seconds - g_frame_time_seconds) * 1000.0;
    bracket.vector.z = (matched.upper_stamp_seconds - matched.lower_stamp_seconds) * 1000.0;
    g_buff_sync_bracket_pub.publish(bracket);
    ROS_INFO_THROTTLE(1.0, "[BUFF_SYNC] frame=%llu image=%.6f pose=%.6f bracket=[%+.2f,%+.2f]ms residual=%+.3fms",
        static_cast<unsigned long long>(g_frame_sequence), g_frame_time_seconds,
        matched.pose.stamp_seconds, bracket.vector.x, bracket.vector.y, residual.data);
}

void doserial(const rm_msgs::RmSerialConstPtr serail)
{
    observeBuffClock();
    BuffPoseSample pose;
    pose.stamp_seconds = serail->header.stamp.toSec();
    pose.rpy = {serail->Roll, serail->Pitch, serail->Yaw};
    pose.imu_to_map = Eigen::Quaterniond(serail->Qw, serail->Qx, serail->Qy, serail->Qz);
    if (!g_buff_time_sync.pushPose(pose)) {
        ROS_WARN_THROTTLE(1.0, "[BUFF_SYNC] Rejected invalid, stale or out-of-order serial pose");
        return;
    }
    RmSerialData = *serail;
    Qtn[0] = serail->Qw; Qtn[1] = serail->Qx;
    Qtn[2] = serail->Qy; Qtn[3] = serail->Qz;
    const bool previous_functional = functional;
    if(serail->ShootFlag == 'f'){
        ROS_INFO_THROTTLE(2.0, "[Fu] 串口指令: 能量机关激活, 配置模式=%s", MoveMode.c_str());
        functional = true;
    }
    else if(serail->ShootFlag == 'a'){
        ROS_INFO_THROTTLE(2.0, "[Fu] 串口指令: 装甲板模式, Fu节点休眠");
        functional = false;
    }else{
        ROS_WARN_THROTTLE(2.0, "[Fu] 未知 ShootFlag='%c', 默认休眠", serail->ShootFlag);
        functional = false;
    }
    if (functional != previous_functional) {
        g_buff_prediction_ready = false;
        g_buff_control_state.reset();
        // Release the previous aiming state once when leaving Fu mode too.
        publishBuffInactiveCommand(false);
        g_big_buff_predictor.reset();
        g_small_buff_tracker.reset();
        g_buff_rotation_sign = 0;
        if (g_recognizer) g_recognizer->resetTracking();
    }
    if(serail->Camp=='b')
    {
        camp = 'r';
    }
    else if(serail->Camp=='r')
    {
        camp = 'b';
    }
    else{
        camp = 'b';
    }

    // // cout << "7777777777777777777777777777777777777" << endl;
    // camp = 'b';
    // cout<<"----------"<<camp<<"------------"<<endl;
    }

    //图像订阅回调函数
    //doimage是ROS图像回调，接收相机图像转OpenCV格式，更新相机状态，调用能量机关主处理函数Fu_Task完成全套视觉运算。
    void doimage(const sensor_msgs::ImageConstPtr img)
    {
        observeBuffClock();
        const auto frame_received_at = std::chrono::steady_clock::now();
        static double previous_arrival_seconds = 0.0;
        const double arrival_seconds = std::chrono::duration<double>(
            frame_received_at.time_since_epoch()).count();
        if (previous_arrival_seconds > 0.0) {
            g_buff_frame_timing.arrival_gap_ms = std::max(
                g_buff_frame_timing.arrival_gap_ms,
                (arrival_seconds - previous_arrival_seconds) * 1000.0);
        }
        previous_arrival_seconds = arrival_seconds;
        if (!g_last_received_image_stamp.isZero() && !img->header.stamp.isZero()) {
            g_buff_frame_timing.capture_gap_ms = std::max(
                g_buff_frame_timing.capture_gap_ms,
                (img->header.stamp - g_last_received_image_stamp).toSec() * 1000.0);
        }
        if (!img->header.stamp.isZero()) {
            const double age_ms = (ros::Time::now() - img->header.stamp).toSec() * 1000.0;
            if (std::isfinite(age_ms) && age_ms >= 0.0) {
                g_buff_frame_timing.image_age_ms = age_ms;
            }
        }
        cv_bridge::CvImageConstPtr cv_ptr;
        {
        BuffStageTimer copy(g_buff_frame_timing, BuffTimingStage::Copy);
        try
        //try{}：异常监测代码块，包裹图像转换逻辑，捕捉转换时出现的错误。
        {
        //借用不可变的 BGR 图像；匹配后再复制到算法自己的图像内存。
            cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)  //异常处理
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());//打印异常信息，调试用

            return;
        }
        }

        const auto sequence = g_received_frame_sequence + 1;
        if (!g_buff_time_sync.offerImage(img->header.stamp.toSec(), arrival_seconds, sequence)) {
            ROS_WARN_THROTTLE(1.0, "[BUFF_SYNC] Rejected zero, stale, duplicate or out-of-order image stamp");
            return;
        }
        g_pending_buff_images.push_back({cv_ptr, img->header, sequence});
        while (g_pending_buff_images.size() > BuffTimeSynchronizer::kImageCapacity)
            g_pending_buff_images.pop_front();
        g_received_frame_sequence = sequence;
        g_last_received_image_stamp = img->header.stamp;
        g_has_frame_imu_pose = false;
        g_buff_sync_waiting = true;

        // cout << "111111111111111111111111111111111111111111111" << endl;
        // cout << "66666666666666666666666666666666666666666666" << endl;
        //cout<<"2"<<endl;
        // imshow("1",frame);
        // waitKey(1);
    }

// Display-only subscription. It runs on a private ROS callback queue, so
// camera conversion and copying continue while Fu_Task() is in inference.
// The live view is capped to 30 fps: at 120 fps a 1280x1024 clone plus an
// X11 blit saturates the GUI thread and the USB3/CPU budget, and the window
// starts dropping/blanking frames.
void doDisplayImage(const sensor_msgs::ImageConstPtr& img)
{
    static auto last_push = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    // 20fps is enough for a debug preview and halves the X11/CPU load compared
    // with 30fps; the processed window still updates at inference rate.
    if (last_push.time_since_epoch().count() != 0 &&
        now - last_push < std::chrono::milliseconds(50)) return;
    last_push = now;
    try {
        const auto cv_ptr = cv_bridge::toCvShare(
            img, sensor_msgs::image_encodings::BGR8);
        g_raw_display_queue.push(cv_ptr->image,
                                 g_display_sequence.fetch_add(1) + 1);
    } catch (const cv_bridge::Exception& error) {
        ROS_WARN_THROTTLE(1.0, "[Fu] display image conversion failed: %s", error.what());
    }
}

void buffDisplayLoop()
{
    // Debug windows must never preempt inference on this 4-core NUC. Lowering
    // only this thread's nice value keeps the aiming path responsive while
    // still allowing the UI to refresh whenever the CPU is idle.
    (void)setpriority(PRIO_PROCESS, static_cast<id_t>(::syscall(SYS_gettid)), 10);
    cv::Mat raw_image;
    cv::Mat processed_image;
    while (g_display_running.load() && ros::ok()) {
        cv::Mat candidate;
        std::uint64_t sequence = 0;
        bool raw_updated = false;
        bool processed_updated = false;
        while (g_raw_display_queue.tryPop(candidate, sequence)) {
            raw_image = std::move(candidate);
            raw_updated = true;
        }
        while (g_processed_display_queue.tryPop(candidate, sequence)) {
            processed_image = std::move(candidate);
            processed_updated = true;
        }

        // Refresh each window only when its own stream produced a frame.
        // Re-showing the processed image for every raw frame multiplies GUI
        // work by the camera rate. The processed window keeps its last frame
        // (with the prediction point) between inference results.
        bool drew = false;
        if (g_buff_show_image && raw_updated && !raw_image.empty()) {
            cv::imshow("Image Live", raw_image);
            drew = true;
        }
        if (g_buff_show_image && processed_updated && !processed_image.empty()) {
            cv::imshow("Image with Point", processed_image);
            drew = true;
        }
        if (g_buff_show_image && cv::waitKey(1) == 'q') {
            g_display_running.store(false);
            break;
        }
        if (!drew) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}



    // 相机正常：首帧初始化识别、预测、PnP 等算法模块，计算弹丸飞行时间；
    //逐帧识别装甲，解扇叶旋转角并时序缓存，拟合角速度算出击打提前量；
    //预测点经重力补偿 PnP 解算角度，叠加补偿后通过 ROS 下发云台；
    //分段计时查耗时，画面绘制瞄准点与帧率用于调试；
    //相机异常时释放内存、重置全局参数，避免程序崩溃。
    bool Fu_Task()
    {
        if (g_frame_sequence == 0 ||
            g_frame_sequence == g_processed_frame_sequence) {
            return false;
        }
        g_processed_frame_sequence = g_frame_sequence;
        g_buff_prediction_ready = false;
        if (!g_has_frame_imu_pose) return false;
    // functional is selected by the serial command, independently of image arrival.
    if(functional)
    {
        if(frame.empty()){
            return false;
        }
        g_recognizer->setExpectedAngularSpeed(MoveMode == "big"
            ? (g_buff_rotation_sign != 0 ? g_big_buff_predictor.angular_speed()
                : std::numeric_limits<double>::quiet_NaN())
            : g_small_buff_tracker.angularSpeed());
    //帧率计算

    std::chrono::steady_clock::time_point fps_start = std::chrono::steady_clock::now();//记录当前时间点


    bool initialized_this_frame = false;
    if (!init)
    {
        BuffStageTimer initialization(g_buff_frame_timing, BuffTimingStage::Initialize);
        // ═══ 首帧初始化: ①识别 ②建模块 ③算飞行时间 ═══
        //
        // ① BuffRecognizer::init(): YOLO推理→remap 9→6→R中心精化→输出坐标
        //    内部调用链: getBestBox→fillTensor→infer→parse→remapKeypoints→refineRCenter
        cv::Point2f init_R, init_Fan;
        double init_side;
        // cout << "7777777777777777777" << endl;
        if (!g_recognizer->init(frame_, init_R, init_Fan, init_side,
                                g_frame_time_seconds))

        {
            ROS_WARN("[Fu] 首帧识别失败!");
            return false;
        }
        // cout << "78888888888877" << endl;
        g_R_center   = init_R;       // 精化后R中心
        g_Fan_center = init_Fan;     // 扇叶中心
        g_side       = init_side;    // 扇叶边长
        g_corners    = g_recognizer->lastCorners();  // ★ 从识别器获取模型检测的真实PnP角点
        ROS_INFO("[Fu] 首帧识别成功: R(%d,%d) Fan(%d,%d) side=%.1f",
                 (int)init_R.x, (int)init_R.y, (int)init_Fan.x, (int)init_Fan.y, init_side);

        // ②③ 创建下游处理模块
        g_big_buff_predictor.reset();
        pnpSolver_gravity = new PnPSolver_gravity(
            g_buff_target_width_mm, g_buff_target_height_mm,
            g_buff_camera_matrix, g_buff_dist_coeffs);//创建PnP解算器，初始化相机参数，返回是否成功

        g_small_buff_tracker.reset();//重置小弹丸跟踪器，释放内存
// cout << "79999999999977777" << endl;

        // ④ 飞行时间初始化: 用模型检测的真实PnP角点→PnP→算飞行时间→初始化delta_t_
        pnpSolver_gravity->PnPSolver_and_gravity(
            g_corners[0], g_corners[1], g_corners[2], g_corners[3],  // ★ 真实角点
            g_frame_serial_data.Pitch);//计算PnP解算时间，返回是否成功
        if (pnpSolver_gravity->time_parameter < 2.0){  // ★ 放宽阈值: 0.5→2.0秒
            delta_t_ = pnpSolver_gravity->time_parameter + buffTotalLeadCompensation();//飞行时间 + 视觉链路延迟 + 电控延时
            // cout << "70000000000000007777" << endl;

        }else{
            ROS_WARN("[Fu] 时间初始化失败!");
            delete pnpSolver_gravity; pnpSolver_gravity = nullptr;
            return false;
        }

        final_angle_2D = {0, 0};//最终角度对应的图像点
        euler_angles = {0, 0};//储存发送的空间角

        init = true;//完成初始化
        initialized_this_frame = true;
        // cout << "71111111111111111177" << endl;
    }

    // 帧计数，用于限频打印
    //记录时间
    time_milliseconds = g_frame_time_seconds * 1000.0;
    //time_milliseconds = 1/fps__*1000+time_milliseconds;
    //cout << "时间：" << time_milliseconds << endl;

    // ═══ 逐帧识别: BuffRecognizer::identify() ═══
    // 内部7步: ①YOLO推理→②remap 9→6→③R中心精化→④扇叶中心→⑤边长→⑥PnP角点→⑦时序校验
    // 模型: YOLO11-pose, OpenVINO CPU, 输入[1,3,640,640], 输出[1,33,8400]
    // 标签: class_row=4+enemy_class_idx_ (0→行4=red, 1→行5=blue)
    // 输出: g_R_center/g_Fan_center→所选运动模型, g_side→PnPSolver_gravity
    {
    BuffStageTimer identify(g_buff_frame_timing, BuffTimingStage::Identify);
    if (!initialized_this_frame &&
        !g_recognizer->identify(frame_, g_R_center, g_Fan_center, g_side, g_corners,
                                g_frame_time_seconds))

    {
        ROS_DEBUG("[Fu] 识别失败");
        // A pending blade selection has a valid wheel association. Keep the
        // learned model's readiness while withholding output; otherwise every
        // switch incurs another three-frame warm-up after selection confirmation.
        if (!g_recognizer->hasAssociatedObservation()) g_big_buff_predictor.reject();
        return false;   // 检测失败 → 无目标, 不画图
    }
// cout << "22222222222222222222777777" << endl;
    }
    // ★ 识别成功后无论下游是否失败, 都先画检测框再跑跟踪链
    // 下游失败时只画检测, 不发布角度 (track_ok = false)
    bool track_ok = false;

    // Debug only: use this frame's measured keypoints, not the predicted point.
    bool world_observation_ready = false;  // 世界帧观测是否准备就绪
    bool has_world_reproj = false;                  // 是否有有效的世界卡尔曼重投影
    double world_reproj_error = 0.0;                // 重投影误差(px)
    cv::Point2d world_kalman_reproj_pixel;           // 世界卡尔曼重投影像素
    cv::Point2d world_measured_reproj_pixel;         // 世界观测重投影像素

    {
    BuffStageTimer observation(g_buff_frame_timing, BuffTimingStage::Observe);
    pnpSolver_gravity->logObservedPnP(
        g_corners[0], g_corners[1], g_corners[2], g_corners[3]);//记录当前帧的PnP角点
//  cout << "9999999999999999" << endl;

    if (g_buff_world_ekf_enabled) {//如果世界帧跟踪器已启用
    // cout << "3333333333333333333337" << endl;

        BuffWorldObservation world_observation;//世界帧观测
        if (pnpSolver_gravity->buildWorldObservation(
                g_corners[0], g_corners[1], g_corners[2], g_corners[3],
                g_R_center, world_observation)) {//构建世界帧观测，返回是否成功
            world_observation_ready =
                g_buff_world_tracker.update(world_observation);
            ROS_INFO_THROTTLE(
                0.5,
                "[BUFF_WORLD] ready=%s radius=%.3fm omega=%.3frad/s",
                world_observation_ready ? "true" : "false",
                g_buff_world_tracker.radiusMeters(),
                g_buff_world_tracker.omegaRadPerSecond());
                // cout<<"8888888888"<<endl;

        // ── 世界卡尔曼重投影（参照装甲板EKF模式: world→Cam→cv::projectPoints）──
        if (world_observation_ready && g_coorConverter != nullptr && g_has_frame_imu_pose) {
            Eigen::Vector3d kalman_target_map;
            if (g_buff_world_tracker.predictTarget(0.0, kalman_target_map)) {
                // Step 1: 世界→IMU（使用帧同步IMU位姿，与buildWorldObservation变换链一致）
                const Eigen::Matrix3d q_rot = g_frame_imu_pose.imu_to_map.toRotationMatrix();
                const Eigen::Matrix3d map2imu_rot = g_quaternion_is_imu_to_map
                    ? q_rot.transpose() : q_rot;
                const Eigen::Vector3d& map_trans = g_coorConverter->imu2Map_trans_mat;

                Eigen::Vector3d kalman_imu = map2imu_rot * (kalman_target_map - map_trans);
                Eigen::Vector3d measured_imu = map2imu_rot * (world_observation.target_center_map - map_trans);

                // Step 2: IMU→相机（使用CoordinateTransformer外参）
                const Eigen::Matrix3d& cam_rot = g_coorConverter->cam2Imu_rot_mat;
                const Eigen::Vector3d& cam_trans = g_coorConverter->cam2Imu_trans_mat;
                Eigen::Vector3d kalman_cam = cam_rot.transpose() * (kalman_imu - cam_trans);
                Eigen::Vector3d measured_cam = cam_rot.transpose() * (measured_imu - cam_trans);

                // Step 3: 相机→像素（cv::projectPoints, rvec=tvec=0, 与装甲板projectPointsToImage一致）
                cv::Mat camera_mat = (cv::Mat_<double>(3, 3) <<
                    g_buff_camera_matrix[0], g_buff_camera_matrix[1], g_buff_camera_matrix[2],
                    g_buff_camera_matrix[3], g_buff_camera_matrix[4], g_buff_camera_matrix[5],
                    g_buff_camera_matrix[6], g_buff_camera_matrix[7], g_buff_camera_matrix[8]);
                cv::Mat dist_mat = (cv::Mat_<double>(1, 5) <<
                    g_buff_dist_coeffs[0], g_buff_dist_coeffs[1], g_buff_dist_coeffs[2],
                    g_buff_dist_coeffs[3], g_buff_dist_coeffs[4]);

                std::vector<cv::Point3d> cam_points = {
                    cv::Point3d(kalman_cam.x(), kalman_cam.y(), kalman_cam.z()),
                    cv::Point3d(measured_cam.x(), measured_cam.y(), measured_cam.z())
                };
                std::vector<cv::Point2d> image_points;
                cv::projectPoints(cam_points,
                    cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                    camera_mat, dist_mat, image_points);

                world_kalman_reproj_pixel = image_points[0];
                world_measured_reproj_pixel = image_points[1];
                world_reproj_error = cv::norm(image_points[0] - image_points[1]);
                has_world_reproj = true;
            }
        }

        } else {
            ROS_WARN_THROTTLE(1.0, "[BUFF_WORLD] observation build failed");
        }
    }

    } // observation timing

    // Select one motion model at launch. Rejection/initialization produces no
    // prediction; it never switches to FFT, another model or an assumed speed.
    bool pixel_track_ok = false;
    bool prediction_without_ec_ok = false;
    cv::Point2f prediction_without_ec_2D = {0, 0};
    double measured_radius = 0.0;
    {
        BuffStageTimer prediction(g_buff_frame_timing, BuffTimingStage::Predict);
        const cv::Point2f offset = g_Fan_center - g_R_center;
        measured_radius = cv::norm(offset);
        const double measured_angle = std::atan2(offset.y, offset.x);
        if (MoveMode == "big") {
            double predicted_angle = 0.0;
            if (!std::isfinite(measured_radius) || measured_radius <= 1.0) {
                g_big_buff_predictor.reject();
            } else if (g_big_buff_predictor.update(g_frame_time_seconds,
                    measured_angle, delta_t_, predicted_angle)) {
                final_angle_2D = g_R_center + cv::Point2f(
                    static_cast<float>(measured_radius * std::cos(predicted_angle)),
                    static_cast<float>(measured_radius * std::sin(predicted_angle)));
                pixel_track_ok = true;
            }
            const double speed = g_big_buff_predictor.angular_speed();
            g_buff_rotation_sign = speed > 0.05 ? 1 : speed < -0.05 ? -1 : 0;
        } else {
            pixel_track_ok = g_small_buff_tracker.update(
                g_R_center, g_Fan_center, g_frame_time_seconds,
                delta_t_, final_angle_2D, g_recognizer->targetChanged());
            g_buff_rotation_sign = g_small_buff_tracker.lockedDirection();
        }
        if (pixel_track_ok) {
            const double predicted_radius = cv::norm(final_angle_2D - g_R_center);
            pixel_track_ok = std::isfinite(final_angle_2D.x) &&
                std::isfinite(final_angle_2D.y) &&
                final_angle_2D.x > 0.0f && final_angle_2D.x <= frame.cols &&
                final_angle_2D.y > 0.0f && final_angle_2D.y <= frame.rows &&
                predicted_radius >= 0.3 * measured_radius &&
                predicted_radius <= 3.5 * measured_radius;
            // update() also returns false while acquiring three good frames.
            // Only reject a produced point that fails geometry validation.
            if (MoveMode == "big" && !pixel_track_ok) g_big_buff_predictor.reject();
        }
        if (pixel_track_ok) {
            const double no_ec_horizon = std::max(
                0.0, delta_t_ - buffCommandTimespanSeconds());
            if (MoveMode == "big") {
                double predicted_angle_without_ec = 0.0;
                prediction_without_ec_ok = g_big_buff_predictor.predictAt(
                    no_ec_horizon, measured_angle, predicted_angle_without_ec);
                if (prediction_without_ec_ok) {
                    prediction_without_ec_2D = g_R_center + cv::Point2f(
                        static_cast<float>(measured_radius *
                            std::cos(predicted_angle_without_ec)),
                        static_cast<float>(measured_radius *
                            std::sin(predicted_angle_without_ec)));
                }
            } else {
                prediction_without_ec_ok = g_small_buff_tracker.predictAt(
                    no_ec_horizon, prediction_without_ec_2D);
            }
        }
        track_ok = pixel_track_ok;
    }

    cv::Point2f prediction_without_ec_angles = {0, 0};
    bool prediction_without_ec_angles_ok = false;
    {
    BuffStageTimer solve(g_buff_frame_timing, BuffTimingStage::Solve);
    // --- World-frame EKF + two flight-time iterations (disabled by default) ---
     if (g_buff_world_ekf_enabled && g_buff_world_ekf_control_enabled &&
         g_buff_world_ekf_primary) {// Explicitly selected world output never falls back to pixels.
        track_ok = world_observation_ready;
        euler_angles.x = 0;
        euler_angles.y = 0;
        double hit_time = std::clamp(delta_t_, 0.01, 1.0);//限制时间步长在0.01-1.0之间
        for (int iteration = 0; iteration < 2 && track_ok; ++iteration) {//迭代两次
            Eigen::Vector3d future_target_map;
            if (!g_buff_world_tracker.predictTarget(hit_time, future_target_map)) {
                track_ok = false;
                break;
            }
            euler_angles = pnpSolver_gravity->solveMapPointAndGravity(
                future_target_map);
            const double new_hit_time =
                pnpSolver_gravity->time_parameter + buffTotalLeadCompensation();
            if ((euler_angles.x == 0.0 && euler_angles.y == 0.0) ||
                !std::isfinite(new_hit_time) || new_hit_time <= 0.01 ||
                new_hit_time >= 1.0) {
                track_ok = false;
                break;


            }
            hit_time = new_hit_time;
        }
    } else if (track_ok) {
        // Image phase prediction -> PnP plane rotation -> gun-frame trajectory.
        if (prediction_without_ec_ok) {
            prediction_without_ec_angles =
                pnpSolver_gravity->PnPSolver_predicted_center_and_gravity(
                    g_corners[0], g_corners[1], g_corners[2], g_corners[3],
                    g_R_center, g_Fan_center, prediction_without_ec_2D,
                    g_frame_serial_data.Pitch);
            prediction_without_ec_angles_ok =
                prediction_without_ec_angles.x != 0.0f ||
                prediction_without_ec_angles.y != 0.0f;
        }
        euler_angles.x = 0;
        euler_angles.y = 0;
        euler_angles = pnpSolver_gravity->PnPSolver_predicted_center_and_gravity(
            g_corners[0], g_corners[1], g_corners[2], g_corners[3],
            g_R_center, g_Fan_center, final_angle_2D,
            g_frame_serial_data.Pitch);//
        ROS_INFO_THROTTLE(
            0.5,
            "[BUFF_PNP] reprojection_error=%.3f px",
            pnpSolver_gravity->lastReprojectionError());
        if (euler_angles.x == 0.0 && euler_angles.y == 0.0) {
            track_ok = false;
            ROS_DEBUG_THROTTLE(1.0, "[Fu] Predicted PnP rejected");

        }//如果PnP求解失败
        // cout<<"55555555555555555"<<endl;



    }

    // World-EKF primary currently exposes its solved target for both angle
    // channels until a separate world no-EC horizon is selected.
    if (g_buff_world_ekf_enabled && g_buff_world_ekf_control_enabled &&
        g_buff_world_ekf_primary && track_ok) {
        prediction_without_ec_angles = euler_angles;
        prediction_without_ec_angles_ok = true;
    }

    } // solve timing

    track_ok = track_ok && std::isfinite(euler_angles.x) &&
        std::isfinite(euler_angles.y) && std::isfinite(g_frame_serial_data.Pitch) &&
        std::isfinite(g_frame_serial_data.Yaw);

    {
    BuffStageTimer diagnostics(g_buff_frame_timing, BuffTimingStage::Diagnostics);
    ROS_INFO_THROTTLE(
        0.5,
        "[BUFF_STATE] mode=%s prediction_ok=%s track_ok=%s "
        "final_angle_2d=(%.1f,%.1f) radius=%.1f",
        MoveMode.c_str(), pixel_track_ok ? "true" : "false",
        track_ok ? "true" : "false", final_angle_2D.x, final_angle_2D.y,
        measured_radius);
    }

    // /clock can reset while synchronous inference/solve is running. Discard
    // that epoch's result before publishing any control or debug output.
    if (observeBuffClock()) return false;
    // Refresh the next-frame lead time from whichever valid trajectory path ran.
    if (track_ok) {
        const double measured_lead_time =
            pnpSolver_gravity->time_parameter + buffTotalLeadCompensation();//飞行时间 + 视觉链路延迟 + 电控延时
        if (std::isfinite(measured_lead_time) &&
            measured_lead_time > 0.01 && measured_lead_time < 1.0) {
            delta_t_ = 0.8 * delta_t_ + 0.2 * measured_lead_time;//更新时间步长
            // cout<<"66666666666666666"<<endl;


        }
    }

    {
    BuffStageTimer output(g_buff_frame_timing, BuffTimingStage::Output);
    // --- 发布角度 ---
    double expected_pitch_no_ec = std::numeric_limits<double>::quiet_NaN();
    double expected_yaw_no_ec = std::numeric_limits<double>::quiet_NaN();
    if (track_ok) {
        // 原始计算角度（限幅前）
        const double expected_pitch_angle =
            -g_frame_serial_data.Pitch +
            euler_angles.x + pitch_compensation;
        const double expected_yaw_angle =
            g_frame_serial_data.Yaw +
            euler_angles.y + yaw_compensation;
        expected_pitch_no_ec = prediction_without_ec_angles_ok
            ? -g_frame_serial_data.Pitch +
                prediction_without_ec_angles.x + pitch_compensation
            : std::numeric_limits<double>::quiet_NaN();
        expected_yaw_no_ec = prediction_without_ec_angles_ok
            ? g_frame_serial_data.Yaw +
                prediction_without_ec_angles.y + yaw_compensation
            : std::numeric_limits<double>::quiet_NaN();
        double raw_angle_x = expected_pitch_angle;
        double limited_angle_x = raw_angle_x;
        Angle.x = limited_angle_x;   // 暂存，随后可能被限幅覆盖
        Angle.y = expected_yaw_angle;

        // ========== Pitch limiter + aggressive stuck recovery ==========
        if (limited_angle_x > buff_pitch_max_rad) limited_angle_x = buff_pitch_max_rad;
        if (limited_angle_x < buff_pitch_min_rad) limited_angle_x = buff_pitch_min_rad;
        Angle.x = limited_angle_x;   // 应用限幅后的值

        // Stuck recovery: when gimbal is near mechanical limit for >250ms,
        // command a strong downward pulse to unwind the MCU PID integrator.
        int& stuck_frames = g_buff_stuck_frames;
        bool& recovery_active = g_buff_recovery_active;
        int& recovery_remaining = g_buff_recovery_remaining;
        if (recovery_active) {
            // Force gimbal down aggressively for several frames.
            Angle.x = buff_recovery_pitch_rad;
            recovery_remaining--;
            if (recovery_remaining <= 0) {
                recovery_active = false;
                stuck_frames = 0;
                ROS_INFO("[Fu] Gimbal recovery complete, resuming normal tracking");
            }
        } else if (RmSerialData.Pitch > buff_stuck_threshold_rad) {
            stuck_frames++;
            if (stuck_frames > buff_stuck_trigger_frames) {
                recovery_active = true;
                recovery_remaining = buff_recovery_frames;
                ROS_WARN("[Fu] Gimbal STUCK at %.1f deg, forcing recovery down!",
                         RmSerialData.Pitch * 180.0 / CV_PI);
            }
        } else {
            stuck_frames = 0;
        }
        // =====================================================================

        Angle.z = BuffControlState::code(g_recognizer->freshObservation(), true);

        const bool pitch_limited =
            std::isfinite(raw_angle_x) &&
            std::abs(raw_angle_x - limited_angle_x) > 1e-9;

        if (pitch_limited) {
            ROS_WARN_THROTTLE(
                0.2,
                "[BUFF_ANGLE] raw_angle_x=%.6f rad (%.2f deg), "
                "limited_angle_x=%.6f rad (%.2f deg), "
                "prediction_ok=%s track_ok=%s "
                "reproj=%.3f px",
                raw_angle_x, raw_angle_x * 180.0 / CV_PI,
                limited_angle_x, limited_angle_x * 180.0 / CV_PI,
                pixel_track_ok ? "true" : "false",
                track_ok ? "true" : "false",
                pnpSolver_gravity->lastReprojectionError());
        }
        publishBuffCommand();

        // ═══ 实测视觉链路延迟: 曝光 → 本帧角度下发 ═══
        // 在 publish 之后取 (现在 - 图像时间戳), 量到的就是本帧从曝光到出角度的
        // 完整链路耗时(相机传输 + ROS 队列积压 + YOLO 推理 + PnP)。只在本帧真的
        // 下发了角度时采样 —— 失败帧/空转帧的图像年龄没有意义, 会污染 EMA。
        // auto 关掉时也照样采样, 只是不参与计算: 日志里的「实测」就是现场标定值。
        buffUpdateVisionDelayEstimate();
        ROS_INFO_THROTTLE(
            1.0,
            "[BUFF_DELAY] 视觉链路 实测=%.1f ms 生效=%.1f ms, 电控=%.1f ms | "
            "总提前量=%.1f ms (其中弹丸飞行 %.1f ms)",
            vision_delay_measured_seconds * 1000.0,
            buffVisionProcessingDelaySeconds() * 1000.0,
            buffCommandTimespanSeconds() * 1000.0,
            delta_t_ * 1000.0,
            pnpSolver_gravity->time_parameter * 1000.0);
        ROS_INFO_THROTTLE(
            0.2,
            "[BUFF_ANGLE] 当前云台(p/y)=%.2f/%.2f deg, "
            "期望无电控(p/y)=%.2f/%.2f deg, "
            "最终预测(p/y)=%.2f/%.2f deg, "
            "实际下发(p/y)=%.2f/%.2f deg, "
            "时延(视觉实测/视觉生效/电控/飞行/总)=%.1f/%.1f/%.1f/%.1f/%.1f ms",
            RmSerialData.Pitch * 180.0 / CV_PI,
            RmSerialData.Yaw * 180.0 / CV_PI,
            expected_pitch_no_ec * 180.0 / CV_PI,
            expected_yaw_no_ec * 180.0 / CV_PI,
            expected_pitch_angle * 180.0 / CV_PI,
            expected_yaw_angle * 180.0 / CV_PI,
            Angle.x * 180.0 / CV_PI,
            Angle.y * 180.0 / CV_PI,
            vision_delay_measured_seconds * 1000.0,
            buffVisionProcessingDelaySeconds() * 1000.0,
            buffCommandTimespanSeconds() * 1000.0,
            pnpSolver_gravity->time_parameter * 1000.0,
            delta_t_ * 1000.0);

    } else {
        publishBuffInactiveCommand(true);

    }
    // ★ debug发布: 始终发布, 不依赖track_ok
    // /buff/current_*   → 当前云台反馈
    // /buff/expected_*  → 只含视觉链路延迟的期望角，用于火控和调参
    // /buff/predicted_* → 加电控延迟后的最终预测/发送角
    // /debugpub         → 当前yaw（兼容旧 rqt_plot）
    // /debugpub1        → 期望yaw（兼容旧 rqt_plot）
    // /debugpub2     → 世界EKF omega (rad/s), 无效时NaN
    // /debugpub3     → |K-M| 像素距离, 无效时NaN
    // /buff/world_ready → EKF ready 状态 (Bool)
    {
        std_msgs::Float64 current_pitch_msg, current_yaw_msg;
        std_msgs::Float64 expected_pitch_msg, expected_yaw_msg;
        std_msgs::Float64 predicted_pitch_msg, predicted_yaw_msg;
        std_msgs::Float64 debug_msg_omega, debug_msg_reproj;
        current_pitch_msg.data = RmSerialData.Pitch;
        current_yaw_msg.data = RmSerialData.Yaw;
        expected_pitch_msg.data = track_ok
            ? expected_pitch_no_ec
            : std::numeric_limits<double>::quiet_NaN();
        expected_yaw_msg.data = track_ok
            ? expected_yaw_no_ec
            : std::numeric_limits<double>::quiet_NaN();
        predicted_pitch_msg.data = track_ok
            ? Angle.x
            : std::numeric_limits<double>::quiet_NaN();
        predicted_yaw_msg.data = track_ok
            ? Angle.y
            : std::numeric_limits<double>::quiet_NaN();
        debug_msg_omega.data = (g_buff_world_ekf_enabled && world_observation_ready)
            ? g_buff_world_tracker.omegaRadPerSecond()
            : std::numeric_limits<double>::quiet_NaN();
        debug_msg_reproj.data = has_world_reproj
            ? world_reproj_error
            : std::numeric_limits<double>::quiet_NaN();
        current_pitch_pub.publish(current_pitch_msg);
        current_yaw_pub.publish(current_yaw_msg);
        expected_pitch_pub.publish(expected_pitch_msg);
        expected_yaw_pub.publish(expected_yaw_msg);
        predicted_pitch_pub.publish(predicted_pitch_msg);
        predicted_yaw_pub.publish(predicted_yaw_msg);
        debugpub.publish(current_yaw_msg);
        debugpub1.publish(expected_yaw_msg);
        debugpub2.publish(debug_msg_omega);
        debugpub3.publish(debug_msg_reproj);

        std_msgs::Bool world_ready_msg;
        world_ready_msg.data = world_observation_ready;
        world_ready_pub.publish(world_ready_msg);
    }

    } // output timing

    // 重力补偿完成

    g_buff_prediction_ready = track_ok;
    if (!g_buff_show_image) return true;
    BuffStageTimer drawing(g_buff_frame_timing, BuffTimingStage::Draw);

    //显示结果(数据可视化)
    //打印预测点
    // ★ 绘制识别器检测结果: R中心(红圆) 扇叶中心(绿圆) PnP角点(黄圆) 检测框(白框)

    // ── 绘制世界卡尔曼重投影点（参照装甲板EKF可视化）──
    if (has_world_reproj) {
        // 紫色实心圆: 卡尔曼滤波估计值
        circle(frame, world_kalman_reproj_pixel, 6, Scalar(255, 0, 255), -1);
        // 青色实心圆: PnP观测值
        circle(frame, world_measured_reproj_pixel, 6, Scalar(255, 255, 0), -1);
        // 黄色连线: 估计值与观测值的偏差
        line(frame, world_kalman_reproj_pixel, world_measured_reproj_pixel,
             Scalar(0, 255, 255), 2);
        // 标注文字
        cv::putText(frame, "K", world_kalman_reproj_pixel + cv::Point2d(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 2);
        cv::putText(frame, "M", world_measured_reproj_pixel + cv::Point2d(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 2);
    }

    if (track_ok) {
        circle(frame, final_angle_2D, 3, Scalar(0, 0, 255), -1);
    } else {
        cv::putText(frame, "TRACK FAIL (angle/predict/PnP)", cv::Point(15, 180),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }
    //打印发射角度,时间间隔，发送角度
    //cout << "delta_t_:" << delta_t_ << endl;
    //cout << "相机pitch=" << euler_angles.x*180/PI << "  " << "相机yaw=" << euler_angles.y*180/PI << endl;
    //cout << "发送补偿角度·:" << -RmSerialData.Pitch+-(euler_angles.x+pitch_compensation) *180 /PI << endl;
    //打印帧率
    //帧率显示参数
    double fps_;
    char buffer[10]; // 用于存放帧率的字符串
    //这段代码统计每一帧图像处理消耗的时间，算出帧率 FPS；
    //一方面用来排查代码卡顿、优化运行速度；
    //另一方面给能量机关装甲旋转预测算法提供帧间隔 Δt，是预测命中必不可少的时间参数。
    double fps_time_milliseconds = 0;
    std::chrono::steady_clock::time_point fps_end = std::chrono::steady_clock::now();//首时间
    std::chrono::steady_clock::duration fps_time = fps_end - fps_start;//计算时间差
    fps_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(fps_time).count();//转换时间单位
    //cout << "运行总时间：" << fps_time_milliseconds << endl;
    fps_ = fps_time_milliseconds > 0.0
        ? 1000.0 / fps_time_milliseconds
        : 0.0;
    sprintf(buffer, "%.2f", fps_); // 帧率保留两位小数
    std::string fpsString = "FPS:" + std::string(buffer); // 在"FPS:"后加入帧率数值字符串
    cv::putText(frame, fpsString, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));// 将帧率信息写在输出帧上l
        // ========== 新增：预测点像素坐标 + 下发云台角度 ==========
    char info_buf[150];//用于存放预测点像素坐标 + 下发云台角度的字符串

    if (track_ok) {
        sprintf(info_buf, "Pred X:%d Y:%d", (int)final_angle_2D.x, (int)final_angle_2D.y);
        cv::putText(frame, info_buf, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
        sprintf(info_buf, "Send Pitch:%.1f Yaw:%.1f", Angle.x*180/PI, Angle.y*180/PI);
        cv::putText(frame, info_buf, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    }

    return true;
        
    }else if(!functional){
        // 装甲板模式: 不做任何处理, 保持已初始化的状态, 等切回大符模式时可直接恢复
        return false;
    }

return false;
}
