#pragma once

#include "buff_prediction_geometry.hpp"

// Buff PnP, coordinate conversion, trajectory, and world-observation code extracted from Fu.cpp without logic changes.

// 前置声明: 坐标变换器(全局, main中初始化)
extern CoordinateTransformer* g_coorConverter;
// All image-derived predictors, including the optional world EKF, share the
// callback monotonic timestamp. The ROS header stamp is only used to match the
// IMU pose and is not the predictor time base.
extern double g_frame_monotonic_time_seconds;

// Persistent rotation direction: +1=CW, -1=CCW, 0=unknown
extern int g_buff_rotation_sign;

//重力补偿器
class PnPSolver_gravity
{
public:
double lastReprojectionError() const {
    return last_reprojection_error_;
}
// Debug only: compare both quaternion directions without affecting the
// existing camera -> gun -> trajectory -> /auto_angle control path.
void logImuImageSync(const Eigen::Vector3d& target_cam_m) const//打印IMU和相机同步信息
{
    if (!g_has_frame_imu_pose) {
        ROS_WARN_THROTTLE(
            1.0,
            "[BUFF_SYNC] no IMU match for image stamp %.6f; history=%zu",
            g_frame_image_stamp.toSec(), g_imu_pose_history.size());
        return;
    }
    if (g_coorConverter == nullptr) {
        ROS_WARN_THROTTLE(1.0, "[BUFF_SYNC] CoordinateTransformer is not ready");
        return;
    }

    const Eigen::Vector3d target_imu = g_coorConverter->cam2Imu(target_cam_m);//将相机坐标转换为IMU坐标

    const Eigen::Matrix3d rotation =
        g_frame_imu_pose.imu_to_map.toRotationMatrix();//将IMU坐标转换为地图坐标
    const Eigen::Vector3d target_map_direct =
        rotation * target_imu + g_coorConverter->imu2Map_trans_mat;//将IMU坐标转换为地图坐标
    const Eigen::Vector3d target_map_inverse =
        rotation.transpose() * target_imu + g_coorConverter->imu2Map_trans_mat;
    const Eigen::Quaterniond& q = g_frame_imu_pose.imu_to_map;
    // cout<<"1111111111"<<endl;
    ROS_INFO_THROTTLE(
        0.5,
        "[BUFF_SYNC] img=%.6f imu=%.6f skew=%+.2fms q_norm=%.6f "
        "q=(%.6f,%.6f,%.6f,%.6f) cam=(%.3f,%.3f,%.3f) "
        "direct=(%.3f,%.3f,%.3f) inverse=(%.3f,%.3f,%.3f)",
        g_frame_image_stamp.toSec(), g_frame_imu_pose.stamp.toSec(),
        g_frame_imu_image_skew_seconds * 1000.0,
        g_frame_imu_pose.raw_norm,
        q.w(), q.x(), q.y(), q.z(),
        target_cam_m.x(), target_cam_m.y(), target_cam_m.z(),
        target_map_direct.x(), target_map_direct.y(), target_map_direct.z(),
        target_map_inverse.x(), target_map_inverse.y(), target_map_inverse.z());
}

// 构造函数初始化相机内参、畸变和Buff靶面的局部三维模型。
// PnP局部系: 原点=靶心, x=靶面右方, y=靶面下方, z=靶面法向。
// 该局部系只用于PnP，PnP输出仍是OpenCV相机系，随后严格由
// CoordinateTransformer::cam2Gun()使用步兵cam2Imu/gun2Imu外参转换。
PnPSolver_gravity(double target_width_mm, double target_height_mm,
                  const std::vector<double>& camera_matrix_values,
                  const std::vector<double>& dist_coeff_values)
{
  if (camera_matrix_values.size() != 9 || dist_coeff_values.size() != 5) {
    throw std::invalid_argument(
        "Buff PnP requires 9 camera-matrix values and 5 distortion values");
  }

  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      camera_matrix_values[0], camera_matrix_values[1], camera_matrix_values[2],
      camera_matrix_values[3], camera_matrix_values[4], camera_matrix_values[5],
      camera_matrix_values[6], camera_matrix_values[7], camera_matrix_values[8]);
  dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
      dist_coeff_values[0], dist_coeff_values[1], dist_coeff_values[2],
      dist_coeff_values[3], dist_coeff_values[4]);

  // Unit: mm. Defaults are supplied by the existing 300 mm model until
  // measured CAD values are provided through the private ROS parameters.
  const float half_width = static_cast<float>(
      std::max(1.0, target_width_mm) * 0.5);
  const float half_height = static_cast<float>(
      std::max(1.0, target_height_mm) * 0.5);

  // The recognizer returns image points in this exact order:
  // [top edge center, left edge center, bottom edge center, right edge center].
  // The object points must use the same physical meaning and order.
  blade_points_.emplace_back(cv::Point3f(0.0f, -half_height, 0.0f));
  blade_points_.emplace_back(cv::Point3f(-half_width, 0.0f, 0.0f));
  blade_points_.emplace_back(cv::Point3f(0.0f, half_height, 0.0f));
  blade_points_.emplace_back(cv::Point3f(half_width, 0.0f, 0.0f));
  cout<<"88888888888888888"<<endl;

}

// Input points are the four Buff edge centers in recognizer order.
// Output tvec is the target center in OpenCV camera coordinates (mm).
bool solvePnP(const cv::Point2f & top, const cv::Point2f & left,
              const cv::Point2f & bottom, const cv::Point2f & right,
              cv::Mat & rvec, cv::Mat & tvec)
{
  const std::vector<cv::Point2f> image_blade_points{top, left, bottom, right};

  // IPPE gives a stable initial pose for a planar target. ITERATIVE then refines
  // that pose using the same four edge-center correspondences.
  if (!cv::solvePnP(
        blade_points_, image_blade_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
        cv::SOLVEPNP_IPPE)) {
            cout<<"11111111111111"<<endl;
    return false;
  }
  if (!cv::solvePnP(
        blade_points_, image_blade_points, camera_matrix_, dist_coeffs_, rvec, tvec, true,
        cv::SOLVEPNP_ITERATIVE)) {
                        cout<<"22222222222222222"<<endl;

    return false;
  }

  std::vector<cv::Point2f> reprojected_points;
  cv::projectPoints(
      blade_points_, rvec, tvec, camera_matrix_, dist_coeffs_, reprojected_points);
  if (reprojected_points.size() != image_blade_points.size()) return false;     
//   cout<<"33333333333333"<<endl;


  double total_error = 0.0;
  for (size_t i = 0; i < image_blade_points.size(); ++i) {
    total_error += cv::norm(reprojected_points[i] - image_blade_points[i]);
  }
  last_reprojection_error_ = total_error / image_blade_points.size();
//   if (last_reprojection_error_ > MAX_REPROJECTION_ERROR_PX) {
//     ROS_WARN_THROTTLE(
//         1.0, "[Fu] Reject Buff PnP: reprojection error %.2f px > %.2f px",
//         last_reprojection_error_, MAX_REPROJECTION_ERROR_PX);
//                                 cout<<"55555555555555555"<<endl;

//     return false;
//   }

  return tvec.at<double>(2, 0) > 0.0;
}

// Use the recognizer's unshifted points for coordinate-convention debugging.
// The control PnP below continues to use the future predicted point unchanged.
void logObservedPnP(const cv::Point2f& top, const cv::Point2f& left,
                    const cv::Point2f& bottom, const cv::Point2f& right)
{
    cv::Mat rvec;
    cv::Mat tvec;
    if (!solvePnP(top, left, bottom, right, rvec, tvec)) {
        ROS_WARN_THROTTLE(1.0, "[BUFF_SYNC] observed-point PnP failed");
        return;
    }

    logImuImageSync(Eigen::Vector3d(
        tvec.at<double>(0, 0) * 0.001,
        tvec.at<double>(1, 0) * 0.001,
        tvec.at<double>(2, 0) * 0.001));
}

// Build a world-frame observation from raw recognizer points. The R-center
// pixel is intersected with the PnP target plane; hub_normal_offset_m accounts
// for a measured offset if the physical hub is not exactly on that plane.
bool buildWorldObservation(const cv::Point2f& top, const cv::Point2f& left,
                           const cv::Point2f& bottom, const cv::Point2f& right,
                           const cv::Point2f& r_center_pixel,
                           BuffWorldObservation& observation)
{
    if (!g_has_frame_imu_pose || g_frame_image_stamp.isZero() ||
        !std::isfinite(g_frame_monotonic_time_seconds) ||
        g_coorConverter == nullptr) {
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    if (!solvePnP(top, left, bottom, right, rvec, tvec)) return false;

    cv::Mat rotation_cv;
    cv::Rodrigues(rvec, rotation_cv);
    Eigen::Matrix3d rotation_target_to_cam;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            rotation_target_to_cam(row, column) = rotation_cv.at<double>(row, column);
        }
    }

    const Eigen::Vector3d target_center_cam(
        tvec.at<double>(0, 0) * 0.001,
        tvec.at<double>(1, 0) * 0.001,
        tvec.at<double>(2, 0) * 0.001);
    const Eigen::Vector3d normal_cam =
        rotation_target_to_cam * Eigen::Vector3d::UnitZ();

    cv::Mat src_rc(1, 1, CV_64FC2);
    src_rc.at<cv::Vec2d>(0, 0) = cv::Vec2d(r_center_pixel.x, r_center_pixel.y);
    cv::Mat undistorted_r_center;
    cv::undistortPoints(src_rc, undistorted_r_center, camera_matrix_, dist_coeffs_);
    if (undistorted_r_center.total() != 1) return false;
    // Sanity: verify output type matches expectation
    if (undistorted_r_center.type() != CV_64FC2) {
        ROS_ERROR_THROTTLE(1.0, "[BUFF_WORLD] undistortPoints output type %d != CV_64FC2", undistorted_r_center.type());
        return false;
    }
    cv::Vec2d urc = undistorted_r_center.at<cv::Vec2d>(0, 0);
    if (!std::isfinite(urc[0]) || !std::isfinite(urc[1])) return false;

    Eigen::Vector3d r_center_ray_cam(
        urc[0], urc[1], 1.0);
    const double ray_plane_denominator = normal_cam.dot(r_center_ray_cam);
    if (std::abs(ray_plane_denominator) < 1e-6) return false;
    const double ray_scale = normal_cam.dot(target_center_cam) /
        ray_plane_denominator;
    if (!std::isfinite(ray_scale) || ray_scale <= 0.0) return false;
    const Eigen::Vector3d r_center_plane_cam = ray_scale * r_center_ray_cam;
    const Eigen::Vector3d r_center_cam = r_center_plane_cam +
        g_buff_world_config.hub_normal_offset_m * normal_cam;

    const Eigen::Matrix3d quaternion_rotation =
        g_frame_imu_pose.imu_to_map.toRotationMatrix();
    const Eigen::Matrix3d imu_to_map_rotation = g_quaternion_is_imu_to_map
        ? quaternion_rotation : quaternion_rotation.transpose();
    const Eigen::Vector3d map_translation = g_coorConverter->imu2Map_trans_mat;

    observation.stamp_seconds = g_frame_monotonic_time_seconds;
    observation.target_center_map = imu_to_map_rotation *
        g_coorConverter->cam2Imu(target_center_cam) + map_translation;
    observation.r_center_map = imu_to_map_rotation *
        g_coorConverter->cam2Imu(r_center_cam) + map_translation;
    observation.target_normal_map = (imu_to_map_rotation *
        g_coorConverter->cam2Imu_rot_mat * normal_cam).normalized();
    return observation.target_center_map.allFinite() &&
        observation.r_center_map.allFinite() &&
        observation.target_normal_map.allFinite();
}

// Calculate the distance between blade center and image center
  //用于计算图像中某一点到图像中心的距离
float calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

//计算空间角
cv::Point2f calculate_space_angle(const cv::Point2f & center)
{
    static cv::Point2f cameraCenter = cv::Point2f(camera_matrix_.at<double>(0, 2)-23,camera_matrix_.at<double>(1, 2)+24); 
    //static cv::Point2f cameraCenter = cv::Point2f(camera_matrix_.at<double>(0, 2),camera_matrix_.at<double>(1, 2)); 
    // return cv::Point2f(center.y, center.x);
	double pitch, yaw;
	double fx, fy;
	fx = camera_matrix_.at<double>(0, 0);
	fy = camera_matrix_.at<double>(1, 1);
	//     
	yaw = atan((cameraCenter.x - center.x) / fx);
	pitch = -atan((cameraCenter.y - center.y) / fy);
    return cv::Point2f(pitch, yaw);

//     double pitch, yaw;
//     // 检查分母是否为零
//     if (tvec.at < double >(2, 0) == 0) {
//         std::cerr << "Error: z component is zero, cannot calculate yaw." << std::endl;
//         return cv::Point2f(0, 0);
//     }
//     if (sqrt(tvec.at < double >(2, 0)* tvec.at < double >(2, 0) + tvec.at < double >(0, 0) * tvec.at < double >(0, 0)) == 0) {
//         std::cerr << "Error: x and z components are both zero, cannot calculate pitch." << std::endl;
//         return cv::Point2f(0, 0);
//     }
//     //计算空间角(相机坐标系是右手系)
//     //角度
//     pitch = -atan2(tvec.at < double >(1, 0),  sqrt(tvec.at < double >(2, 0)* tvec.at < double >(2, 0) + tvec.at < double >(0, 0) * tvec.at < double >(0, 0)));
//     yaw = atan2(tvec.at < double >(0, 0), tvec.at < double >(2, 0));
//     //cout << "pitch" << pitch*180/PI << "度  " << "yaw" << yaw*180/PI << "度  " << endl;
//     cout << "pitch" << pitch << "弧度  " << "yaw" << yaw << "弧度  " << endl;
//     cv::Point2f space_angle = cv::Point2f(pitch, yaw);
//   return space_angle;
}

// ★ 枪管系重力补偿 (cam2Gun坐标变换后调用)
// 输入: tvec_gun 枪管系三维点(m), x前 y左 z上(ROS右手系)
// 返回: 枪管系重力补偿后的pitch角(弧度)
double time_parameter = 0;//将时间定义在类内函数外用于类外提取飞行时间
double calculateTrajectory_gun(const Eigen::Vector3d& tvec_gun) {
    time_parameter = 0;
    const int iteration_times = 30; // 迭代上限次数
    float velocity = 24; // 打弹速度
    if(RmSerialData.BulletVec > 10){
        velocity = RmSerialData.BulletVec;
    }
    const float gravity = 9.8f; // 重力加速度

    // 枪管系: 水平距离和高度直接用向量分量
    double distance_xy = sqrt(tvec_gun.x()*tvec_gun.x() + tvec_gun.y()*tvec_gun.y());
    double target_z = tvec_gun.z();
    if (distance_xy < 1e-6) {
        return -50;
    }

    double pitch_angle = atan2(target_z, distance_xy);
    double iterative_height = target_z;
    double height_difference = 0;
    double impact_point_height = 0;
    for (int i = 0; i < iteration_times; ++i) {
        pitch_angle = std::atan2(iterative_height, distance_xy);

        if (std::abs(pitch_angle) > M_PI*0.165) {
            std::cerr << "异常:补偿角度超过30度!" << std::endl;
            cout<<"pitch_angle"<<pitch_angle<<endl;
            return -50;
        }

        // Flight time is horizontal distance divided by horizontal velocity.
        // tvec_gun is already in the configured infantry gun frame:
        // x forward, y left, z up.
        time_parameter = distance_xy/(velocity*std::cos(pitch_angle));
        if (time_parameter < 0) {
            std::cerr << "异常:飞行时间负数!" << std::endl;
            time_parameter = 0;
            return -50;
        }

        impact_point_height = velocity*std::sin(pitch_angle)*time_parameter-0.5*gravity*time_parameter*time_parameter;
        height_difference = target_z - impact_point_height;

        if (std::abs(height_difference) < 0.0001) break;
        iterative_height += height_difference;
    }

    if (std::abs(height_difference) > 0.01 || std::abs(pitch_angle) > M_PI*0.165) {
        std::cerr << "异常:高度差过大或角度超出限制!" << std::endl;
        return -50;
    }

    return pitch_angle; // 枪管系补偿后pitch
}

cv::Point2f solveGunPointAndGravity(const Eigen::Vector3d& target_gun) {
    cv::Point2f space_angle = {0, 0};
    const double gun_yaw = std::atan2(target_gun.y(), target_gun.x());
    const double gun_pitch_raw = std::atan2(
        target_gun.z(), std::hypot(target_gun.x(), target_gun.y()));
    if (std::abs(gun_pitch_raw) > PI * 0.165 ||
        std::abs(gun_yaw) > PI * 0.165) {
        ROS_WARN_THROTTLE(
            1.0, "[Fu] Reject Buff gun point: pitch=%.2f deg yaw=%.2f deg",
            gun_pitch_raw * 180.0 / PI, gun_yaw * 180.0 / PI);
        return space_angle;
    }

    const double compensated_pitch = calculateTrajectory_gun(target_gun);
    if (compensated_pitch == -50) return space_angle;
    space_angle.x = compensated_pitch;
    space_angle.y = gun_yaw;
    return space_angle;
}

// Map -> IMU uses the inverse of the selected IMU -> map quaternion. The
// existing imu2Gun() retains the configured gun2Imu translation and rotation.
cv::Point2f solveMapPointAndGravity(const Eigen::Vector3d& target_map) {
    if (!g_has_frame_imu_pose || g_coorConverter == nullptr) {
        return cv::Point2f(0, 0);
    }
    const Eigen::Matrix3d quaternion_rotation =
        g_frame_imu_pose.imu_to_map.toRotationMatrix();
    const Eigen::Matrix3d imu_to_map_rotation = g_quaternion_is_imu_to_map
        ? quaternion_rotation : quaternion_rotation.transpose();
    const Eigen::Vector3d target_imu = imu_to_map_rotation.transpose() *
        (target_map - g_coorConverter->imu2Map_trans_mat);
    return solveGunPointAndGravity(g_coorConverter->imu2Gun(target_imu));
}



// 总处理函数: PnP解算 + 严格按CoordinateTransformer的cam2Gun变换 + 重力补偿。
// Predict the fan center in the physical PnP plane. The phase predictor is
// still image based, but the predicted point is produced by rotating around R
// in 3D instead of translating all four PnP pixels by one image offset.
cv::Point2f PnPSolver_predicted_center_and_gravity(
    const cv::Point2f& top, const cv::Point2f& left,
    const cv::Point2f& bottom, const cv::Point2f& right,
    const cv::Point2f& r_center_pixel, const cv::Point2f& fan_center_pixel,
    const cv::Point2f& predicted_fan_pixel, const double& /*car_pitch*/) {
    cv::Mat rvec;
    cv::Mat tvec;
    if (g_coorConverter == nullptr ||
        !solvePnP(top, left, bottom, right, rvec, tvec)) {
        // fprintf(stderr, "[DBG] pred_cg: solvePnP failed or null converter\n");
        return cv::Point2f(0, 0);
    }
    // fprintf(stderr, "[DBG] pred_cg: solvePnP ok rvec.t=%d size=%dx%d\n", rvec.type(), rvec.rows, rvec.cols);

    const cv::Point2f observed_radius = fan_center_pixel - r_center_pixel;
    const cv::Point2f predicted_radius = predicted_fan_pixel - r_center_pixel;
    const double observed_radius_squared = observed_radius.dot(observed_radius);
    const double predicted_radius_squared = predicted_radius.dot(predicted_radius);
    if (!std::isfinite(observed_radius_squared) ||
        !std::isfinite(predicted_radius_squared) ||
        observed_radius_squared < 1.0 || predicted_radius_squared < 1.0) {
        return cv::Point2f(0, 0);
    }

    const double image_phase_delta = std::atan2(
        observed_radius.x * predicted_radius.y -
            observed_radius.y * predicted_radius.x,
        observed_radius.dot(predicted_radius));
    if (!std::isfinite(image_phase_delta)) return cv::Point2f(0, 0);

    cv::Mat rotation_cv;
    cv::Rodrigues(rvec, rotation_cv);
    // fprintf(stderr, "[DBG] pred_cg: Rodrigues ok rot.t=%d size=%dx%d\n", rotation_cv.type(), rotation_cv.rows, rotation_cv.cols);
    const BuffPredictionGeometry::Vec3 plane_normal{
        rotation_cv.at<double>(0, 2), rotation_cv.at<double>(1, 2),
        rotation_cv.at<double>(2, 2)};
    const BuffPredictionGeometry::Vec3 plane_point{
        tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)};

    const auto pixel_to_plane = [this, &plane_point, &plane_normal](
                                    const cv::Point2f& pixel,
                                    BuffPredictionGeometry::Vec3& point) {
        cv::Mat src(1, 1, CV_64FC2);
        src.at<cv::Vec2d>(0, 0) = cv::Vec2d(pixel.x, pixel.y);
        cv::Mat dst;
        cv::undistortPoints(src, dst, camera_matrix_, dist_coeffs_);
        if (dst.total() != 1) return false;
        // Sanity: verify output type matches expectation
        if (dst.type() != CV_64FC2) {
            ROS_ERROR_THROTTLE(1.0, "[BUFF_PNP] undistortPoints output type %d != CV_64FC2", dst.type());
            return false;
        }
        cv::Vec2d pt = dst.at<cv::Vec2d>(0, 0);
        if (!std::isfinite(pt[0]) || !std::isfinite(pt[1])) return false;
        return BuffPredictionGeometry::rayPlaneIntersection(
            {pt[0], pt[1], 1.0}, plane_point,
            plane_normal, point);
    };

    BuffPredictionGeometry::Vec3 r_center_cam;
    BuffPredictionGeometry::Vec3 fan_center_cam;
    if (!pixel_to_plane(r_center_pixel, r_center_cam) ||
        !pixel_to_plane(fan_center_pixel, fan_center_cam)) {
        // fprintf(stderr, "[DBG] pred_cg: pixel_to_plane failed\n");
        return cv::Point2f(0, 0);
    }
    // fprintf(stderr, "[DBG] pred_cg: pixel_to_plane ok\n");

    constexpr double kPhaseSignProbeRadians = 0.30;
    BuffPredictionGeometry::Vec3 positive_offset;
    if (!BuffPredictionGeometry::rotateAroundAxis(
            fan_center_cam - r_center_cam, plane_normal,
            kPhaseSignProbeRadians, positive_offset)) {
        // fprintf(stderr, "[DBG] pred_cg: rotateAroundAxis failed\n");
        return cv::Point2f(0, 0);
    }
    // fprintf(stderr, "[DBG] pred_cg: rotateAroundAxis ok\n");

    // fprintf(stderr, "[DBG] pred_cg: before projectPoints\n");
    std::vector<cv::Point2d> positive_probe_pixel;
    cv::projectPoints(
        std::vector<cv::Point3d>{{
            r_center_cam.x + positive_offset.x,
            r_center_cam.y + positive_offset.y,
            r_center_cam.z + positive_offset.z}},
        cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
        camera_matrix_, dist_coeffs_, positive_probe_pixel);
    // fprintf(stderr, "[DBG] pred_cg: projectPoints ok sz=%zu\n", positive_probe_pixel.size());
    if (positive_probe_pixel.size() != 1) return cv::Point2f(0, 0);

    const cv::Point2d positive_probe_radius =
        positive_probe_pixel[0] - cv::Point2d(r_center_pixel);
    const double positive_image_phase = std::atan2(
        observed_radius.x * positive_probe_radius.y -
            observed_radius.y * positive_probe_radius.x,
        observed_radius.dot(positive_probe_radius));
    double physical_phase_delta = 0.0;
    if (std::isfinite(positive_image_phase) &&
        std::abs(positive_image_phase) >= 0.015) {
        // Probe signal is strong, use it directly.
        if (!BuffPredictionGeometry::physicalPhaseForImagePhase(
                image_phase_delta, positive_image_phase,
                physical_phase_delta)) {
            return cv::Point2f(0, 0);
        }
    } else if (g_buff_rotation_sign != 0) {
        // Probe is ambiguous (e.g. edge-on view), fall back to the
        // persistent rotation direction from Kalman / fallback paths.
        if (!std::isfinite(image_phase_delta)) return cv::Point2f(0, 0);
        physical_phase_delta = g_buff_rotation_sign > 0
            ? image_phase_delta : -image_phase_delta;
    } else {
        return cv::Point2f(0, 0);
    }

    // Sanity check: physical phase delta must be within physically
    // possible bounds. Max omega ~2.0 rad/s x delta_t ~0.5s = 1.0 rad.
    // A 2.5 rad (143 deg) limit is generous.
    if (!std::isfinite(physical_phase_delta) ||
        std::abs(physical_phase_delta) > 2.5) {
        return cv::Point2f(0, 0);
    }
    BuffPredictionGeometry::Vec3 predicted_offset;
    if (!BuffPredictionGeometry::rotateAroundAxis(
            fan_center_cam - r_center_cam, plane_normal,
            physical_phase_delta, predicted_offset)) {
        return cv::Point2f(0, 0);
    }

    const Eigen::Vector3d predicted_center_cam(
        (r_center_cam.x + predicted_offset.x) * 0.001,
        (r_center_cam.y + predicted_offset.y) * 0.001,
        (r_center_cam.z + predicted_offset.z) * 0.001);
    auto result = solveGunPointAndGravity(
        g_coorConverter->cam2Gun(predicted_center_cam));
    // fprintf(stderr, "[DBG] pred_cg: solveGunPointAndGravity ok result=(%.4f,%.4f)\n", result.x, result.y);
    return result;
}

cv::Point2f PnPSolver_and_gravity(const cv::Point2f & top, const cv::Point2f & left,
                                   const cv::Point2f & bottom, const cv::Point2f & right,
                                   const double &car_pitch)
{
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Point2f space_angle = {0, 0};    //储存空间角
//   cout<<"top: "  <<top <<endl;
//   cout<<"left: "  <<left <<endl;
//   cout<<"bottom: "  <<bottom <<endl;
//   cout<<"right: "  <<right <<endl;
//   bool a = solvePnP(top, left, bottom, right, rvec, tvec);
//   cout<<"a: "  <<a <<endl;
  if (solvePnP(top, left, bottom, right, rvec, tvec))//解算位姿态
  {
    
    // Strict coordinate chain for the configured infantry transform:
    // OpenCV camera (x right, y down, z forward) -> IMU -> gun
    // cam2Gun() applies the configured cam2Imu and gun2Imu rotations/translations.
    Eigen::Vector3d tvec_cam(
        tvec.at<double>(0, 0) * 0.001,  // OpenCV x(右)
        tvec.at<double>(1, 0) * 0.001,  // OpenCV y(下)
        tvec.at<double>(2, 0) * 0.001   // OpenCV z(前)
    );
    Eigen::Vector3d tvec_gun = g_coorConverter->cam2Gun(tvec_cam);

    // 枪管系 pitch/yaw: x前 y左 z上
    double gun_yaw   = atan2(tvec_gun.y(), tvec_gun.x());
    double gun_pitch_raw = atan2(tvec_gun.z(), sqrt(tvec_gun.x()*tvec_gun.x() + tvec_gun.y()*tvec_gun.y()));

    if (abs(gun_pitch_raw) > PI*0.165 || abs(gun_yaw) > PI*0.165){//异常检测
        cout << "异常:枪管系角度超限!" << endl;
        cout << "pitch:" << gun_pitch_raw*180/PI << " yaw:" << gun_yaw*180/PI << endl;
        cout << "tvec_cam(mm):" << tvec_cam.transpose()*1000 << endl;
        cout << "tvec_gun(m): " << tvec_gun.transpose() << endl;
        return space_angle; // {0,0}
    }

    // 重力补偿 (枪管系)
    double compensated_pitch = calculateTrajectory_gun(tvec_gun);
    if (compensated_pitch == -50){//异常检测
        cout << "补偿异常！" << endl;
        return space_angle; // {0,0}
    }

    space_angle.x = compensated_pitch;
    space_angle.y = gun_yaw;

        ////////////
    // cout << "光心1：" << camera_matrix_.at<double>(0, 2) << endl;
    // cout << "光心1：" << camera_matrix_.at<double>(1, 2) << endl;
    ///////////

    return space_angle;
  }
else{
    cout << "PnP位姿解算失败!" << endl;
    return space_angle;
  }

}



private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

  /*1704.45313471055	0.460601854648428	665.913826347723
    0	1704.60254015387	487.291315275975
    0	0	1



    -0.0817259940211909	0.392083347020012	-0.140959043006811



-0.000578920794543104	0.00187156396493549*/

  // Unit: mm
  static constexpr double MAX_REPROJECTION_ERROR_PX = 12.0;

  // Four Buff edge centers in [top, left, bottom, right] order.
  std::vector<cv::Point3f> blade_points_;
  double last_reprojection_error_ = 0.0;
};


