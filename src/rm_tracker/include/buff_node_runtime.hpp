#pragma once

#include <string>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <limits>

#include "buff_big_ransac_predictor.hpp"

extern int g_buff_rotation_sign;

extern int g_persistent_rotation_dir;
extern bool g_buff_big_ransac_enabled;
extern buff_zju::BigBuffRansacPredictor* g_big_buff_predictor;
extern std::string MoveMode;
extern std::uint64_t g_frame_sequence;
extern std::uint64_t g_processed_frame_sequence;
extern double g_last_big_buff_timestamp;
extern bool g_big_buff_timestamp_from_image;
// Buff serial/image callbacks and Fu_Task extracted from Fu.cpp without logic changes.

void doserial(const rm_msgs::msg::RmSerial::ConstSharedPtr serail)
{
    // serail->qw/qx/qy/qz: controller-provided orientation quaternion.
//作用：用来完整记录战车 IMU 采集的车身三维姿态（横滚、俯仰、偏航），相比欧拉角不会出现万向锁，稳定描述车体倾斜状态。
    RmSerialData = *serail;
    Qtn[0] = serail->qw;
    Qtn[1] = serail->qx;
    Qtn[2] = serail->qy;
    Qtn[3] = serail->qz;

    const Eigen::Quaterniond imu_to_map(
        serail->qw, serail->qx, serail->qy, serail->qz);
    const double quaternion_norm = imu_to_map.norm();
    const rclcpp::Time serial_stamp(serail->header.stamp);
    if (serial_stamp.nanoseconds() != 0 &&
        std::isfinite(quaternion_norm) &&
        quaternion_norm > 0.9 && quaternion_norm < 1.1) {
        g_imu_pose_history.push_back(
            {serial_stamp, imu_to_map.normalized(), quaternion_norm});
        while (g_imu_pose_history.size() > kImuPoseHistoryCapacity) {
            g_imu_pose_history.pop_front();
        }
    }
    const bool previous_functional = functional;
    if (is_energy_mode(serail->shoot_flag)) {
        ROS_INFO_THROTTLE(2.0, "[Fu] 串口指令: 大符模式, Fu节点激活");
        functional = true;
    }
    else if (is_armor_mode(serail->shoot_flag)) {
        ROS_INFO_THROTTLE(2.0, "[Fu] 串口指令: 装甲板模式, Fu节点休眠");
        functional = false;
    }else{
        ROS_WARN_THROTTLE(2.0, "[Fu] 未知 ShootFlag='%c', 默认休眠",
                          rm_msgs_int8_to_char(serail->shoot_flag));
        functional = false;
    }
    if (functional != previous_functional && g_big_buff_predictor != nullptr) {
        // A stop/start command starts a new observation session. Do not mix
        // the previous roll/velocity window with the next activation after a
        // potentially long pause, even when the camera clock is continuous.
        g_big_buff_predictor->reset();
        g_last_big_buff_timestamp =
            std::numeric_limits<double>::quiet_NaN();
        g_big_buff_timestamp_from_image = false;
    }
    camp = camp_to_internal(serail->camp);

    // // cout << "7777777777777777777777777777777777777" << endl;
    // camp = 'b';
    // cout<<"----------"<<camp<<"------------"<<endl;
    }

    //图像订阅回调函数
    //doimage是ROS图像回调，接收相机图像转OpenCV格式，更新相机状态，调用能量机关主处理函数Fu_Task完成全套视觉运算。
    void doimage(const sensor_msgs::msg::Image::ConstSharedPtr img)
    {
        cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
        try
        //try{}：异常监测代码块，包裹图像转换逻辑，捕捉转换时出现的错误。
        {
        //拷贝转换函数，把 ROS 图像转为 OpenCV 通用 BGR 三通道 8 位 Mat，生成独立图像内存。
            cv_ptr =  cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
        }
        catch(cv_bridge::Exception& e)  //异常处理
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());//打印异常信息，调试用

            return;
        }
        frame_ = cv_ptr->image.clone();//将OpenCV图像复制给frame_，避免直接操作ROS图像

        g_frame_image_stamp = rclcpp::Time(img->header.stamp);//记录当前图像的时间戳
        ++g_frame_sequence;
        g_has_frame_imu_pose = findClosestImuPose(
            g_frame_image_stamp, g_frame_imu_pose);//查找最近的IMU姿态，记录在g_frame_imu_pose中，返回是否找到

        g_frame_imu_image_skew_seconds = g_has_frame_imu_pose
            ? (g_frame_imu_pose.stamp +
               rclcpp::Duration::from_seconds(g_imu_stamp_offset_seconds) -
               g_frame_image_stamp).seconds()
            : std::numeric_limits<double>::quiet_NaN();//计算当前图像与最近IMU时间偏移，单位秒

        // cout << "111111111111111111111111111111111111111111111" << endl;
        // cout << "66666666666666666666666666666666666666666666" << endl;
        //cout<<"2"<<endl;
        // imshow("1",frame);
        // waitKey(1);
    }



    // 相机正常：首帧初始化识别、预测、PnP 等算法模块，计算弹丸飞行时间；
    //逐帧识别装甲，解扇叶旋转角并时序缓存，拟合角速度算出击打提前量；
    //预测点经重力补偿 PnP 解算角度，叠加补偿后通过 ROS 下发云台；
    //分段计时查耗时，画面绘制瞄准点与帧率用于调试；
    //相机异常时释放内存、重置全局参数，避免程序崩溃。
    bool Fu_Task()
    {
    	// Sync persistent direction to global before any processing.
    	// Ensures g_buff_rotation_sign is valid even when Kalman diverges
    	// and the angle processor has not yet accumulated 21 frames.
    	if (g_persistent_rotation_dir != 0) {
    		g_buff_rotation_sign = g_persistent_rotation_dir;
    	}
        static cv::Point2f smooth_R = g_R_center;
    static cv::Point2f smooth_Fan = g_Fan_center;
    float alpha = 0.3; // 越小越平滑
    smooth_R = smooth_R * (1 - alpha) + g_R_center * alpha;
    smooth_Fan = smooth_Fan * (1 - alpha) + g_Fan_center * alpha;
    frame = frame_.clone();//将frame_复制给frame，避免直接操作frame_
    // cout<<"testtest"<<endl;
    //functional 全局标记：true = 相机正常输出图像；false = 断流 / 打不开摄像头；
    if(functional)
    {
        if(frame.empty()){
            return false;
        }
        // The main loop can run faster than the image callback. Consume each
        // image sequence exactly once so processing jitter is never used as a
        // motion timestamp and the same detection is not fitted repeatedly.
        if (g_frame_sequence == 0 ||
            g_frame_sequence == g_processed_frame_sequence) {
            return false;
        }
        g_processed_frame_sequence = g_frame_sequence;
    //帧率计算

    std::chrono::steady_clock::time_point fps_start = std::chrono::steady_clock::now();//记录当前时间点


    if (!init)
    {
        // ═══ 首帧初始化: ①识别 ②建模块 ③算飞行时间 ═══
        //
        // ① BuffRecognizer::init(): YOLO推理→remap 9→6→R中心精化→输出坐标
        //    内部调用链: getBestBox→fillTensor→infer→parse→remapKeypoints→refineRCenter
        cv::Point2f init_R, init_Fan;
        double init_side;
        // cout << "7777777777777777777" << endl;
        if (!g_recognizer->init(frame, init_R, init_Fan, init_side))//初始化识别器，返回是否成功

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
        Angleprocess = new Fu_angleprocess();        // 角度处理器: 极坐标+累加+差分
        Predicted = new buff_predicted();            // 预测器: FFT频率+正弦拟合+滑动平均
        if (g_big_buff_predictor != nullptr) {
            g_big_buff_predictor->reset();
            g_last_big_buff_timestamp =
                std::numeric_limits<double>::quiet_NaN();
            g_big_buff_timestamp_from_image = false;
        }
        pnpSolver_gravity = new PnPSolver_gravity(
            g_buff_target_width_mm, g_buff_target_height_mm,
            g_buff_camera_matrix, g_buff_dist_coeffs);//创建PnP解算器，初始化相机参数，返回是否成功

        g_small_buff_tracker.reset();//重置小弹丸跟踪器，释放内存
// cout << "79999999999977777" << endl;

        // ④ 飞行时间初始化: 用模型检测的真实PnP角点→PnP→算飞行时间→初始化delta_t_
        pnpSolver_gravity->PnPSolver_and_gravity(
            g_corners[0], g_corners[1], g_corners[2], g_corners[3],  // ★ 真实角点
            RmSerialData.pitch);//计算PnP解算时间，返回是否成功
        if (pnpSolver_gravity->time_parameter < 2.0){  // ★ 放宽阈值: 0.5→2.0秒
            delta_t_ = pnpSolver_gravity->time_parameter + delta_t_compensation;//计算弹丸飞行时间，单位秒，补偿时间
            // cout << "70000000000000007777" << endl;

        }else{
            ROS_WARN("[Fu] 时间初始化失败!");
            delete Angleprocess; Angleprocess = nullptr;
            delete Predicted; Predicted = nullptr;
            delete pnpSolver_gravity; pnpSolver_gravity = nullptr;
            return false;
        }

        angle_diff.clear();//差分角度容器
        angle_diff_time.clear();//时间容器
        predicted_diff_angle = 0;//预测差分角
        final_angle = 0;//最终角度(原角度加上预测角)
        final_angle_2D = {0, 0};//最终角度对应的图像点
        euler_angles = {0, 0};//储存发送的空间角

        start = std::chrono::steady_clock::now();//时间基础

        init = true;//完成初始化
        // cout << "71111111111111111177" << endl;
    }

    // 帧计数，用于限频打印
    //记录时间
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();//首时间
    std::chrono::steady_clock::duration time = end - start;//计算时间差
    // 将时间差强制转为毫秒，获取数值
    time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();//转换时间单位
    //time_milliseconds = 1/fps__*1000+time_milliseconds;
    //cout << "时间：" << time_milliseconds << endl;

    std::chrono::steady_clock::time_point Identifies_start = std::chrono::steady_clock::now();

    // ═══ 逐帧识别: BuffRecognizer::identify() ═══
    // 内部7步: ①YOLO推理→②remap 9→6→③R中心精化→④扇叶中心→⑤边长→⑥PnP角点→⑦时序校验
    // 模型: YOLO11-pose, OpenVINO CPU, 输入[1,3,640,640], 输出[1,33,8400]
    // 标签: class_row=4+enemy_class_idx_ (0→行4=red, 1→行5=blue)
    // 输出: g_R_center/g_Fan_center→Fu_angleprocess, g_side→PnPSolver_gravity
    if (!g_recognizer->identify(frame, g_R_center, g_Fan_center, g_side, g_corners))//识别目标，返回是否成功

    {
        ROS_DEBUG("[Fu] 识别失败");
        return false;   // 检测失败 → 无目标, 不画图
    }
// cout << "22222222222222222222777777" << endl;
    // ★ 识别成功后无论下游是否失败, 都先画检测框再跑跟踪链
    // 下游失败时只画检测, 不发布角度 (track_ok = false)
    bool track_ok = false;

    // Debug only: use this frame's measured keypoints, not the predicted point.
    pnpSolver_gravity->logObservedPnP(
        g_corners[0], g_corners[1], g_corners[2], g_corners[3]);//记录当前帧的PnP角点
//  cout << "9999999999999999" << endl;

    bool world_observation_ready = false;  // 世界帧观测是否准备就绪
    bool has_world_reproj = false;                  // 是否有有效的世界卡尔曼重投影
    double world_reproj_error = 0.0;                // 重投影误差(px)
    cv::Point2d world_kalman_reproj_pixel;           // 世界卡尔曼重投影像素
    cv::Point2d world_measured_reproj_pixel;         // 世界观测重投影像素

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
            cout<<"999899999988"<<endl;
        }
    }

    // --- Small-buff tracking and prediction ---
    // This remains the default control path until the world tracker is enabled
    // and has received enough valid world-frame observations.
   // ═══════════════════════════════════════════════════════════════
      // 混合预测: 卡尔曼(主) + 角度差分线性外推(兜底)
      // ═══════════════════════════════════════════════════════════════

      // ---- Step 1: 每帧喂 Angleprocess，保持角度缓存始终热 ----
      // 这一步无论卡尔曼是否正常都会执行，确保切兜底时不需要预热
      {
          static double fallback_fps = 120.0;  // 运行中 EMA 估计
          static auto last_feed_time = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();
          double interval_ms =
  std::chrono::duration_cast<std::chrono::milliseconds>(
              now - last_feed_time).count();
          last_feed_time = now;
          if (interval_ms > 0.0 && interval_ms < 500.0) {
              fallback_fps = 0.9 * fallback_fps + 0.1 * (1000.0 / interval_ms);
          }

          Angleprocess->set_image_size(frame.cols, frame.rows);
          double angle_diff_temp = Angleprocess->pre_predict_angle_process(
              g_R_center, g_Fan_center, time_milliseconds, delta_t_,
  fallback_fps);
          // RANSAC 采样与旧角度差分解耦：只要本帧极角新鲜（图幅内）就喂给
          // RANSAC，不再依赖旧差分算法返回有效值（旧算法首帧/丢帧/扇叶跳变会返回 -1）
          if (buff_zju::is_big_buff_ransac_active(
                  g_buff_big_ransac_enabled, MoveMode) &&
              g_big_buff_predictor != nullptr &&
              Angleprocess->last_frame_polar_valid) {
              if (Angleprocess->anticlockwise_flag) {
                  g_buff_rotation_sign = Angleprocess->anticlockwise ? 1 : -1;
              }
              g_big_buff_predictor->set_rotation_sign(g_buff_rotation_sign);
              const double predictor_timestamp =
                  g_frame_image_stamp.nanoseconds() != 0
                      ? g_frame_image_stamp.seconds()
                      : std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start).count();
              if (std::isfinite(predictor_timestamp)) {
                  const bool timestamp_from_image =
                      g_frame_image_stamp.nanoseconds() != 0;
                  if (timestamp_from_image != g_big_buff_timestamp_from_image) {
                      g_big_buff_predictor->reset();
                      g_last_big_buff_timestamp =
                          std::numeric_limits<double>::quiet_NaN();
                      g_big_buff_timestamp_from_image = timestamp_from_image;
                  }
                  const bool timestamp_repeated =
                      std::isfinite(g_last_big_buff_timestamp) &&
                      predictor_timestamp == g_last_big_buff_timestamp;
                  if (std::isfinite(g_last_big_buff_timestamp) &&
                      predictor_timestamp < g_last_big_buff_timestamp) {
                      // A restarted bag or a non-monotonic camera clock
                      // cannot be mixed into the existing fit window.
                      g_big_buff_predictor->reset();
                  }
                  if (!timestamp_repeated) {
                      g_big_buff_predictor->update_polar(
                          predictor_timestamp,
                          static_cast<double>(Angleprocess->angle_polar_now));
                      g_last_big_buff_timestamp = predictor_timestamp;
                  }
              }
          }
          if (angle_diff_temp != -1) {
             angle_diff_temp = std::remainder(angle_diff_temp, 2.0 * CV_PI);
				// Update global rotation direction from angle processor
				if (Angleprocess->anticlockwise_flag) {
					g_buff_rotation_sign = Angleprocess->anticlockwise ? 1 : -1;
				}
              angle_diff.push_back(angle_diff_temp);
              angle_diff_time.push_back(time_milliseconds);
              // 限制缓存长度，防止无限增长
              while (angle_diff.size() > 300) {
                  angle_diff.erase(angle_diff.begin());
                  angle_diff_time.erase(angle_diff_time.begin());
              }
          }
      }

      // ---- Step 2: 卡尔曼主路径 (原逻辑) ----
      const bool pixel_track_ok = g_small_buff_tracker.update(
          g_R_center,
          g_Fan_center,
          time_milliseconds * 0.001,
          delta_t_,
          final_angle_2D);

      // ---- Step 3: 发散检测 ----
      // 卡尔曼输出的预测点必须满足:
      //   (a) 坐标有限
      //   (b) 到圆心的距离在 [0.4×半径, 2.5×半径] 之间
      // 如果连续3帧不满足，判定卡尔曼发散
      bool kalman_healthy = pixel_track_ok;
		// Update global rotation direction from Kalman tracker
		if (g_small_buff_tracker.lockedDirection() != 0) {
			g_buff_rotation_sign = g_small_buff_tracker.lockedDirection();
		}
      static int kalman_bad_frames = 0;

      if (kalman_healthy && Angleprocess->radius_buff > 1.0f) {
          float pred_dist = cv::norm(final_angle_2D - g_R_center);
          if (pred_dist < 0.3f * Angleprocess->radius_buff ||
              pred_dist > 3.5f * Angleprocess->radius_buff ||
              !std::isfinite(final_angle_2D.x) ||
  !std::isfinite(final_angle_2D.y)) {
              kalman_healthy = false;
          }
      }

      kalman_healthy ? kalman_bad_frames = 0 : kalman_bad_frames++;

      // ====== 调试日志：状态概览（已移至此，确保变量已定义）======
      ROS_INFO_THROTTLE(
          0.5,
          "[BUFF_STATE] pixel_track_ok=%s kalman_healthy=%s track_ok=%s "
          "final_angle_2d=(%.1f,%.1f) radius=%.1f bad_frames=%d",
          pixel_track_ok ? "true" : "false",
          kalman_healthy ? "true" : "false",
          track_ok ? "true" : "false",
          final_angle_2D.x, final_angle_2D.y,
          Angleprocess != nullptr ? Angleprocess->radius_buff : -1.0f,
          kalman_bad_frames);

      // ---- Step 4: 决策 ----
      static bool using_fallback = false;
      static int  fallback_fail_cnt = 0;

      if (kalman_healthy) {
          // ── 卡尔曼正常 ──
          track_ok = true;
          if (using_fallback) {
              ROS_INFO("[Fu] Kalman recovered, back to primary path");
              using_fallback = false;
              fallback_fail_cnt = 0;
          }
      } else if (kalman_bad_frames >= 4 && Angleprocess != nullptr && Predicted
  != nullptr) {
          // ── 卡尔曼已连续异常 ≥3 帧，启用兜底 ──
          if (!using_fallback) {
             ROS_WARN("[Fu] Kalman diverged (bad=%d frames), fallback to angle-diff", kalman_bad_frames);
              using_fallback = true;
          }

          // ★ 核心兜底逻辑 ★
          // predicted_diff_angle = 过去 delta_t 秒内转过的角度 (EMA平滑)
          // final_angle = 当前极角 + 预测提前量
          // final_angle_2D = 极坐标 → 像素坐标
          static std::string fallback_mode = MoveMode;
        //   angle_diff = std::remainder(angle_diff, 2.0 * CV_PI); // 角度归一化
          predicted_diff_angle = Predicted->predicted(
              angle_diff, angle_diff_time, fallback_mode, delta_t_,
              /* fps 用 EMA 估计值 */ (angle_diff_time.size() > 1)
                  ? 1000.0 * angle_diff_time.size() /
                        (angle_diff_time.back() - angle_diff_time.front())
                  : 120.0);

          if (predicted_diff_angle != 0.0) {
              final_angle = predicted_diff_angle +
  Angleprocess->angle_polar_now;
  

       final_angle = std::remainder(final_angle, 2.0 * CV_PI);// 角度归一化
              final_angle_2D = Angleprocess->post_predict_angle_process(
                  final_angle, Angleprocess->radius_buff);
              track_ok = true;
              fallback_fail_cnt = 0;
          } else {
              // Last resort: use persistent direction for constant-speed prediction.
              if (g_buff_rotation_sign != 0 && Angleprocess != nullptr) {
              	const double default_omega = 1.047;  // 60 deg/s default
              	const double sign = g_buff_rotation_sign > 0 ? 1.0 : -1.0;
              	predicted_diff_angle = sign * default_omega * delta_t_;
              	final_angle = predicted_diff_angle + Angleprocess->angle_polar_now;
              	final_angle = std::remainder(final_angle, 2.0 * CV_PI);
              	final_angle_2D = Angleprocess->post_predict_angle_process(
              	    final_angle, Angleprocess->radius_buff);
              	track_ok = true;
              	fallback_fail_cnt = 0;
              } else {
              	track_ok = false;
              	fallback_fail_cnt++;
              }
          }
      } else {
          // ── 卡尔曼异常但还没攒够 3 帧，或兜底对象为空 ──
          track_ok = false;
      }


    // Prefer the Zhejiang University large-buff model after convergence.
    // The existing Kalman and legacy predictor paths remain fallbacks.
    if (buff_zju::is_big_buff_ransac_active(
            g_buff_big_ransac_enabled, MoveMode) &&
        g_big_buff_predictor != nullptr &&
        g_big_buff_predictor->ready() && Angleprocess->radius_buff > 1.0f) {
        const double predicted_roll =
            g_big_buff_predictor->predict_roll(delta_t_);
        const cv::Point2f zju_predicted_point =
            Angleprocess->post_predict_angle_process(
                predicted_roll, Angleprocess->radius_buff);
        const bool finite_point =
            std::isfinite(zju_predicted_point.x) &&
            std::isfinite(zju_predicted_point.y);
        const bool inside_frame =
            finite_point && zju_predicted_point.x > 0.0f &&
            zju_predicted_point.x <= static_cast<float>(frame.cols) &&
            zju_predicted_point.y > 0.0f &&
            zju_predicted_point.y <= static_cast<float>(frame.rows);
        if (inside_frame) {
            final_angle_2D = zju_predicted_point;
            track_ok = true;
        }
    }

    // --- World-frame EKF + two flight-time iterations (disabled by default) ---
     if (g_buff_world_ekf_enabled && g_buff_world_ekf_control_enabled &&world_observation_ready) {//g_buff_world_ekf_enabled && g_buff_world_ekf_control_enabled &&//如果世界帧跟踪器已启用world_observation_ready
        track_ok = true;
        euler_angles.x = 0;
        euler_angles.y = 0;
        double hit_time = std::clamp(delta_t_, 0.01, 1.0);//限制时间步长在0.01-1.0之间
        for (int iteration = 0; iteration < 2; ++iteration) {//迭代两次
            Eigen::Vector3d future_target_map;
            if (!g_buff_world_tracker.predictTarget(hit_time, future_target_map)) {
                track_ok = false;
                break;
                cout<<"2222222222222"<<endl;
            }
            euler_angles = pnpSolver_gravity->solveMapPointAndGravity(
                future_target_map);
            const double new_hit_time =
                pnpSolver_gravity->time_parameter + delta_t_compensation;
            if ((euler_angles.x == 0.0 && euler_angles.y == 0.0) ||
                !std::isfinite(new_hit_time) || new_hit_time <= 0.01 ||
                new_hit_time >= 1.0) {
                track_ok = false;
                break;
                cout<<"3333333333333"<<endl;


            }
            hit_time = new_hit_time;
        }
    } else if (track_ok) {
        // Image phase prediction -> PnP plane rotation -> gun-frame trajectory.
        euler_angles.x = 0;
        euler_angles.y = 0;
        euler_angles = pnpSolver_gravity->PnPSolver_predicted_center_and_gravity(
            g_corners[0], g_corners[1], g_corners[2], g_corners[3],
            g_R_center, g_Fan_center, final_angle_2D,
            RmSerialData.pitch);//
        ROS_INFO_THROTTLE(
            0.5,
            "[BUFF_PNP] reprojection_error=%.3f px",
            pnpSolver_gravity->lastReprojectionError());
        if (euler_angles.x == 0.0 && euler_angles.y == 0.0) {
            track_ok = false;
            cout<<"4444444444444"<<endl;

        }//如果PnP求解失败
        // cout<<"55555555555555555"<<endl;



    }

    // Refresh the next-frame lead time from whichever valid trajectory path ran.
    if (track_ok) {
        const double measured_lead_time =
            pnpSolver_gravity->time_parameter + delta_t_compensation;//计算当前帧的时间间隔
        if (std::isfinite(measured_lead_time) &&
            measured_lead_time > 0.01 && measured_lead_time < 1.0) {
            delta_t_ = 0.8 * delta_t_ + 0.2 * measured_lead_time;//更新时间步长
            // cout<<"66666666666666666"<<endl;


        }
    }

    // --- 发布角度 ---
    if (track_ok) {
        // 原始计算角度（限幅前）
        double raw_angle_x = -RmSerialData.pitch + (euler_angles.x + pitch_compensation);
        double limited_angle_x = raw_angle_x;
        Angle.x = limited_angle_x;   // 暂存，随后可能被限幅覆盖
        Angle.y = RmSerialData.yaw + (euler_angles.y + yaw_compensation);

        // ========== Pitch limiter + aggressive stuck recovery ==========
        const double PITCH_MAX_RAD = 22.0 * CV_PI / 180.0;
        const double PITCH_MIN_RAD = -15.0 * CV_PI / 180.0;
        if (limited_angle_x > PITCH_MAX_RAD) limited_angle_x = PITCH_MAX_RAD;
        if (limited_angle_x < PITCH_MIN_RAD) limited_angle_x = PITCH_MIN_RAD;
        Angle.x = limited_angle_x;   // 应用限幅后的值

        // Stuck recovery: when gimbal is near mechanical limit for >250ms,
        // command a strong downward pulse to unwind the MCU PID integrator.
        static int   stuck_frames = 0;
        static bool  recovery_active = false;
        static int   recovery_remaining = 0;
        const double STUCK_THRESH_RAD = 19.0 * CV_PI / 180.0;

        if (recovery_active) {
            // Force gimbal down aggressively for several frames.
            Angle.x = -10.0 * CV_PI / 180.0;
            recovery_remaining--;
            if (recovery_remaining <= 0) {
                recovery_active = false;
                stuck_frames = 0;
                ROS_INFO("[Fu] Gimbal recovery complete, resuming normal tracking");
            }
        } else if (RmSerialData.pitch > STUCK_THRESH_RAD) {
            stuck_frames++;
            if (stuck_frames > 30) {
                recovery_active = true;
                recovery_remaining = 15;  // ~125ms at 120fps
                ROS_WARN("[Fu] Gimbal STUCK at %.1f deg, forcing recovery down!",
                         RmSerialData.pitch * 180.0 / CV_PI);
            }
        } else {
            stuck_frames = 0;
        }
        // =====================================================================

        Angle.z = 1;

        const bool pitch_limited =
            std::isfinite(raw_angle_x) &&
            std::abs(raw_angle_x - limited_angle_x) > 1e-9;

        if (pitch_limited || !track_ok || !pixel_track_ok ||
            !kalman_healthy) {
            ROS_WARN_THROTTLE(
                0.2,
                "[BUFF_ANGLE] raw_angle_x=%.6f rad (%.2f deg), "
                "limited_angle_x=%.6f rad (%.2f deg), "
                "pixel_track_ok=%s kalman_healthy=%s track_ok=%s "
                "reproj=%.3f px",
                raw_angle_x, raw_angle_x * 180.0 / CV_PI,
                limited_angle_x, limited_angle_x * 180.0 / CV_PI,
                pixel_track_ok ? "true" : "false",
                kalman_healthy ? "true" : "false",
                track_ok ? "true" : "false",
                pnpSolver_gravity->lastReprojectionError());
        }
        PubAngle.publish(Angle);
    }
    // ★ debug发布: 始终发布, 不依赖track_ok
    // /debugpub      → 期望角pitch (跟踪成功=计算值, 失败=-平台pitch)
    // /debugpub1     → 平提出轴角度
    // /debugpub2     → 世界EKF omega (rad/s), 无效时NaN
    // /debugpub3     → |K-M| 像素距离, 无效时NaN
    // /buff/world_ready → EKF ready 状态 (Bool)
    {
        std_msgs::msg::Float64 debug_msg_x, debug_msg_y;
        std_msgs::msg::Float64 debug_msg_omega, debug_msg_reproj;
        debug_msg_x.data = track_ok ? Angle.x : -RmSerialData.pitch;
        debug_msg_y.data = -RmSerialData.pitch;    // 平提出轴角度
        debug_msg_omega.data = (g_buff_world_ekf_enabled && world_observation_ready)
            ? g_buff_world_tracker.omegaRadPerSecond()
            : std::numeric_limits<double>::quiet_NaN();
        debug_msg_reproj.data = has_world_reproj
            ? world_reproj_error
            : std::numeric_limits<double>::quiet_NaN();
        debugpub.publish(debug_msg_x);
        debugpub1.publish(debug_msg_y);
        debugpub2.publish(debug_msg_omega);
        debugpub3.publish(debug_msg_reproj);

        std_msgs::msg::Bool world_ready_msg;
        world_ready_msg.data = world_observation_ready;
        world_ready_pub.publish(world_ready_msg);
    }

    // 重力补偿完成

    //显示结果(数据可视化)
    //打印预测点
    // ★ 绘制识别器检测结果: R中心(红圆) 扇叶中心(绿圆) PnP角点(黄圆) 检测框(白框)
    g_recognizer->visual(frame);

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
    // Angle publication remains the final controller-facing output.
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
