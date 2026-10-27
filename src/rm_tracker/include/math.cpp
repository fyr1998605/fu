#include "math.hpp"

namespace math {

// 将3D笛卡尔坐标系中的速度转换为云台坐标系中的角速度
math::YpdCoord get_ypd_v(const Eigen::Vector3d& xyz, const Eigen::Vector3d& xyz_v) {
    double x = xyz(0, 0), y = xyz(1, 0), z = xyz(2, 0);
    double x_dot = xyz_v(0, 0), y_dot = xyz_v(1, 0), z_dot = xyz_v(2, 0);
    double yaw_v = -y * x_dot / (x * x + y * y) + x * y_dot / (x * x + y * y);
    double pitch_v = z_dot * std::sqrt(x * x + y * y) / (x * x + y * y + z * z)
        - x * z * x_dot / (std::sqrt(x * x + y * y) * (x * x + y * y + z * z))
        - y * z * y_dot / (std::sqrt(x * x + y * y) * (x * x + y * y + z * z));
    double dis_v = x * x_dot / (std::sqrt(x * x + y * y + z * z))
        + y * y_dot / (std::sqrt(x * x + y * y + z * z))
        + z * z_dot / (std::sqrt(x * x + y * y + z * z));
    return math::YpdCoord(yaw_v, pitch_v, dis_v);
}

// 将3D笛卡尔坐标系中的速度转换为相机坐标系中的角速度
math::YpdCoord camera_get_ypd_v(const Eigen::Vector3d& xyz, const Eigen::Vector3d& xyz_v) {
    double x = xyz(0, 0), y = xyz(1, 0), z = xyz(2, 0);
    double x_dot = xyz_v(0, 0), y_dot = xyz_v(1, 0), z_dot = xyz_v(2, 0);
    double yaw_v = -x * z_dot / (z * z + x * x) + z * x_dot / (z * z + x * x);
    double pitch_v = y_dot * std::sqrt(z * z + x * x) / (x * x + y * y + z * z)
        - z * y * z_dot / (std::sqrt(z * z + x * x) * (x * x + y * y + z * z))
        - x * y * x_dot / (std::sqrt(z * z + x * x) * (x * x + y * y + z * z));
    double dis_v = x * x_dot / (std::sqrt(x * x + y * y + z * z))
        + y * y_dot / (std::sqrt(x * x + y * y + z * z))
        + z * z_dot / (std::sqrt(x * x + y * y + z * z));
    return math::YpdCoord(yaw_v, pitch_v, dis_v);
}

// 对点集应用相机畸变模型，模拟真实相机的畸变效果
auto distort_points(
    const std::vector<cv::Point2d>& src,
    std::vector<cv::Point2d>& dst,
    const cv::Mat& camera_mat,
    const cv::Mat& distortion_mat) -> void {
    double fx = camera_mat.at<double>(0, 0);
    double fy = camera_mat.at<double>(1, 1);
    double cx = camera_mat.at<double>(0, 2);
    double cy = camera_mat.at<double>(1, 2);
    std::vector<cv::Point3d> src2;
    for (auto i : src) {
        src2.emplace_back((i.x - cx) / fx, (i.y - cy) / fy, 0);
    }
    cv::Mat rot_vec(3, 1, cv::DataType<double>::type,
                    cv::Scalar(0));  // Rotation vector
    cv::Mat trans_vec(3, 1, cv::DataType<double>::type,
                      cv::Scalar(0));  // Translation vector
    std::vector<cv::Point2d> dst2;
    cv::projectPoints(src2, rot_vec, trans_vec, camera_mat, distortion_mat, dst);
}

} 
