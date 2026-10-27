
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <queue>
#include <Eigen/Dense>
#include "rm_msgs/RmSerial.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
// #include <thread_safe_queue.hpp> 
#include "img_tools.hpp"
// #include <fmt/format.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include "calibration_handeye.hpp"


class ImuImageData{
public:
    // 当前图像对应的四元数 -> 存在时间不对齐需要插值法优化
    // 采集图像和对应的IMU四元数,并且存储对应数据的时间戳
    ImuImageData(const sensor_msgs::ImageConstPtr img, const rm_msgs::RmSerialConstPtr serial) {
        // 解析串口话题
        imu_timestamp = serial->header.stamp;
        rm_msgs::RmSerial serialData = *serial;     
        double roll = 0,pitch = 0,yaw = 0;
        roll = serialData.Roll;
        pitch = serialData.Pitch;
        yaw = serialData.Yaw;
        tf2::Quaternion Q;  // 欧拉角 -> 四元数
        Q.setRPY(roll, pitch, yaw);
        q.w() = Q.w();  // 获取四元数
        q.x() = Q.x();
        q.y() = Q.y();
        q.z() = Q.z();

        // 解析图像话题
        image_timestamp = img->header.stamp;
        ros::Duration time_compensation_(this->m_timeCompensation);
        image_timestamp = image_timestamp - time_compensation_;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        this->frame = cv_ptr->image.clone();
    }
private:
    ros::Time image_timestamp;  // Image图像时间
    ros::Time imu_timestamp;    // IMU四元数时间
    double m_timeCompensation = 0.0085;

public:
    cv::Mat frame;
    Eigen::Quaterniond q;
};

class Calibrate_handeye{
public: 
Calibrate_handeye(const std::string & config_path) : config_path_(config_path) {
    setparam(config_path_);
}

std::vector<cv::Point3d> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3d> centers_3d;

  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      float x = 0;
      float y = (-j + 0.5 * pattern_size.width) * center_distance;
      float z = (-i + 0.5 * pattern_size.height) * center_distance;
      centers_3d.push_back({x, y, z});
    }
  }

  return centers_3d;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

void setparam(const std::string & file_path) {
    try{
        // 1. 文件存在性检查
        if (file_path.empty()) {
            ROS_ERROR("Empty config file path provided");
            return;
        }

        // 2.检查文件是否打开
        std::ifstream file_check(file_path);
        if (!file_check.is_open()) {
            ROS_ERROR("Failed to open config file: %s. Reason: %s", 
                    file_path.c_str(), strerror(errno));
            return;
        }
        ROS_INFO("Loading parameters from: %s", file_path.c_str());

        // 3. 加载 YAML 文件并添加详细错误处理
        YAML::Node config;
        try {
            config = YAML::LoadFile(file_path);
        } catch (const YAML::BadFile& e) {
            ROS_ERROR("Bad YAML file: %s", e.what());
            return;
        } catch (const YAML::ParserException& e) {
            ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
            return;
        }

        // 加载棋盘参数
        if (config["handeye"]) {
            // 如果初始参数这样设置 那么手眼标定的时候标定板要横着放
            YAML::Node handeye = config["handeye"];
            m_pattern_cols = handeye["pattern_cols"].as<int>(6);                   // 标定板列数(水平方向圆点数量)
            m_pattern_rows = handeye["pattern_rows"].as<int>(4);                   // 标定板行数(垂直方向圆点数量)
            m_center_distance_mm = handeye["center_distance_mm"].as<double>(30);   // 相邻圆点中心的距离(mm)
            R_gimbal2imubody_data = handeye["R_gimbal2imubody_data"].as<std::vector<double>>();
        }

        // 加载相机参数
        if (config["Camera"]){
            YAML::Node camera = config["Camera"];
            // 加载相机内参矩阵
            if (camera["camera_matrix"]) {
                std::vector<double> cam_matrix_array = camera["camera_matrix"].as<std::vector<double>>();
                if (cam_matrix_array.size() == 9) {
                    // 创建3*3相机内参矩阵
                    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                    cam_matrix_array[0], cam_matrix_array[1], cam_matrix_array[2],
                    cam_matrix_array[3], cam_matrix_array[4], cam_matrix_array[5],
                    cam_matrix_array[6], cam_matrix_array[7], cam_matrix_array[8]);
                }
                // debug
                // for (int i = 0; i < cam_matrix_array.size(); i++) {
                //     ROS_INFO("cam_matrix_array%d:%.3f",i,cam_matrix_array[i]);
                // }
            }
            else {
                ROS_WARN("No 'camera_matrix' section in config file");
            }
            
            // 加载畸变系数矩阵
            if (camera["dist_coeffs"]) {
                std::vector<double> dist_coeffs_array = camera["dist_coeffs"].as<std::vector<double>>();
                if (dist_coeffs_array.size() == 5) {
                    dist_coeffs_ = (cv::Mat_<double>(1, 5) << 
                    dist_coeffs_array[0], dist_coeffs_array[1], dist_coeffs_array[2], 
                    dist_coeffs_array[3], dist_coeffs_array[4]);
                }
            }
            else {
                ROS_WARN("No 'dist_coeffs' section in config file");
            }
        }
    }
     catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parsing error: %s", e.what());
    } 
    catch (const std::exception& e) {
        ROS_ERROR("Error loading parameters: %s", e.what());
    } 
    catch (...) {
        ROS_ERROR("Unknown error occurred while loading config file");
    }
}

void load(std::queue<ImuImageData> queue_,std::vector<cv::Mat>& R_world2gimbal_list,std::vector<cv::Mat>& t_world2gimbal_list,std::vector<cv::Mat>& rvecs,std::vector<cv::Mat>& tvecs) {
    cv::Size pattern_size(this->m_pattern_cols, this->m_pattern_rows);
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
    // cv::Matx33d camera_matrix(camera_matrix_data.data());
    // cv::Mat distort_coeffs(distort_coeffs_data);

    for (int i = 1; queue_.size(); i++) {
        // 读取图片和对应的四元数
        ImuImageData data = queue_.front();
        queue_.pop();  // 弹出队首
        cv::Mat img = data.frame;
        Eigen::Quaterniond imu_q = data.q;
        // 计算云台的欧拉角
        Eigen::Matrix3d R_imubody2imuabs = imu_q.toRotationMatrix();
        Eigen::Matrix3d R_gimbal2world =
            R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
        Eigen::Vector3d ypr = eulers(R_gimbal2world, 2, 1, 0) * 57.3;  // degree
        
        // 在图片上显示云台的欧拉角，用来检验R_gimbal2imubody是否正确
        auto drawing = img.clone();
        // tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
        // tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
        // tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});
        
        // 识别标定板
        std::vector<cv::Point2f> centers_2d;
        auto success = cv::findCirclesGrid(img, pattern_size, centers_2d);  // 默认是对称圆点图案

        // 显示识别结果
        cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
        cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
        cv::imshow("Press any to continue", drawing);
        cv::waitKey(0);

        // 输出识别结果
        // fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
        if (!success) continue;

        // 计算所需的数据
        Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
        cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::Mat R_world2gimbal_cv;
        cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
        cv::Mat rvec, tvec;
        auto centers_3d_ = centers_3d(pattern_size, m_center_distance_mm);
        cv::solvePnP(centers_3d_, centers_2d, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

        // 记录所需的数据
        R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
        t_world2gimbal_list.emplace_back(t_world2gimbal);
        rvecs.emplace_back(rvec);
        tvecs.emplace_back(tvec);
    }
}

void print_terminal(
  const std::vector<double> & R_gimbal2imubody_data,    // ginbal -> imu 旋转矩阵 
  const cv::Mat & R_camera2gimbal,                      // cam -> gimbal 旋转矩阵
  const cv::Mat & t_camera2gimbal,                      // cam -> imu    平移矩阵
  const Eigen::Vector3d & camera_ypr,                   // 相机相对于理想暗转姿态的欧拉角偏角
  double distance,                                      // 标定板到世界坐标系原点的水平距离
  const Eigen::Vector3d & board_ypr                     // 标定板相对于竖直摆放时的欧拉角偏角
){
    // ==================== 核心标定结果 ====================
    std::cout << "\n[手眼标定结果]\n";
    std::cout << "========================================\n";
    
    // 1. 相机相对于云台的外参
    std::cout << "1. 相机->云台外参:\n";
    std::cout << "   旋转矩阵 R_camera2gimbal:\n";
    for (int i = 0; i < 3; i++) {
        std::cout << "   ";
        for (int j = 0; j < 3; j++) {
            printf("%8.5f ", R_camera2gimbal.at<double>(i, j));
        }
        std::cout << std::endl;
    }
    
    std::cout << "   平移向量 t_camera2gimbal (米):\n";
    printf("   [%8.4f, %8.4f, %8.4f]^T\n\n",
           t_camera2gimbal.at<double>(0, 0),
           t_camera2gimbal.at<double>(1, 0),
           t_camera2gimbal.at<double>(2, 0));
    
    // 2. 安装误差
    std::cout << "2. 相机安装误差 (度):\n";
    printf("   偏航角(Yaw):   %7.3f°\n", camera_ypr[0]);
    printf("   俯仰角(Pitch): %7.3f°\n", camera_ypr[1]);
    printf("   翻滚角(Roll):  %7.3f°\n\n", camera_ypr[2]);
    
    // 3. 四元数形式(便于ROS使用)
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R_camera2gimbal, R_eigen);
    Eigen::Quaterniond q(R_eigen);
    
    std::cout << "3. 四元数表示 (w,x,y,z):\n";
    printf("   %.6f, %.6f, %.6f, %.6f\n\n",
           q.w(), q.x(), q.y(), q.z());
    
    // 4. RPY形式(欧拉角)
    Eigen::Vector3d euler_angles = R_eigen.eulerAngles(2, 1, 0); // ZYX顺序，即yaw, pitch, roll
    // 转换为角度
    double roll_deg = euler_angles[2] * 180.0 / M_PI;
    double pitch_deg = euler_angles[1] * 180.0 / M_PI;
    double yaw_deg = euler_angles[0] * 180.0 / M_PI;
    
    std::cout << "4. RPY欧拉角表示 (度):\n";
    printf("   翻滚角(Roll, X轴):  %7.3f°\n", roll_deg);
    printf("   俯仰角(Pitch, Y轴): %7.3f°\n", pitch_deg);
    printf("   偏航角(Yaw, Z轴):   %7.3f°\n\n", yaw_deg);
    
    // ==================== 标定板信息 ====================
    std::cout << "[标定板信息]\n";
    std::cout << "========================================\n";
    printf("水平距离: %.3f 米\n", distance);
    std::cout << "标定板姿态 (度):\n";
    printf("   Yaw:   %7.3f°\n", board_ypr[0]);
    printf("   Pitch: %7.3f°\n", board_ypr[1]);
    printf("   Roll:  %7.3f°\n\n", board_ypr[2]);
    
    // ==================== 可用于配置文件的格式 ====================
    std::cout << "[配置文件格式]\n";
    std::cout << "========================================\n";
    std::cout << "camera_to_gimbal_transform:\n";
    std::cout << "  # 旋转矩阵 (3x3)\n";
    std::cout << "  rotation_matrix: [\n";
    for (int i = 0; i < 3; i++) {
        std::cout << "    [";
        for (int j = 0; j < 3; j++) {
            printf("%.6f", R_camera2gimbal.at<double>(i, j));
            if (j < 2) std::cout << ", ";
        }
        std::cout << "]";
        if (i < 2) std::cout << ",";
        std::cout << "\n";
    }
    std::cout << "  ]\n\n";
    
    std::cout << "  # 四元数 (w,x,y,z)\n";
    std::cout << "  quaternion: ["
              << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]\n\n";
    
    std::cout << "  # RPY欧拉角 (度)\n";
    std::cout << "  rpy_degrees: ["
              << roll_deg << ", " << pitch_deg << ", " << yaw_deg << "]\n\n";
    
    std::cout << "  # RPY欧拉角 (弧度)\n";
    std::cout << "  rpy_radians: ["
              << euler_angles[2] << ", " << euler_angles[1] << ", " << euler_angles[0] << "]\n\n";
    
    std::cout << "  # 平移向量 (x,y,z) 单位:米\n";
    std::cout << "  translation: ["
              << t_camera2gimbal.at<double>(0, 0) << ", "
              << t_camera2gimbal.at<double>(1, 0) << ", "
              << t_camera2gimbal.at<double>(2, 0) << "]\n";
    std::cout << "========================================\n";
}

void calibrateCamera(std::vector<cv::Mat> rvecs,std::vector<cv::Mat> tvecs,std::vector<cv::Mat> R_world2gimbal_list, std::vector<cv::Mat> t_world2gimbal_list) {
    // 手眼标定
    cv::Mat R_gimbal2camera, t_gimbal2camera;
    cv::Mat R_world2board, t_world2board;
      cv::calibrateRobotWorldHandEye(
        rvecs,                        // 输入：标定板到相机的旋转向量
        tvecs,                        // 输入：标定板到相机的平移向量
        R_world2gimbal_list,          // 输入：世界到云台的旋转矩阵列表
        t_world2gimbal_list,          // 输入：世界到云台的平移向量列表
        R_world2board,                // 输出：世界到标定板的旋转矩阵
        t_world2board,                // 输出：世界到标定板的平移向量
        R_gimbal2camera,              // 输出：云台到相机的旋转矩阵
        t_gimbal2camera               // 输出：云台到相机的平移向量
    );

    t_gimbal2camera /= 1e3;  // mm to m
    t_world2board /= 1e3;    // mm to m

    // 计算相机->云台的逆变换(云台->相机)
    cv::Mat R_camera2gimbal, t_camera2gimbal;
    cv::transpose(R_gimbal2camera, R_camera2gimbal);        // 旋转部分
    t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;   // 平移部分

    // 计算标定板->世界的逆变换(世界->标定板)
    cv::Mat R_board2world, t_board2world;
    cv::transpose(R_world2board, R_board2world);            // 旋转部分
    t_board2world = -R_board2world * t_world2board;         // 平移部分

    
    // 计算标定板到世界坐标系原点的水平距离
    auto x = t_board2world.at<double>(0);
    auto y = t_board2world.at<double>(1);
    auto distance = std::sqrt(x * x + y * y);

    // 计算相机同理想情况的偏角
    Eigen::Matrix3d R_camera2gimbal_eigen;
    cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
    // Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    Eigen::Matrix3d R_gimbal2ideal;
    R_gimbal2ideal << 0, -1, 0,
                      0, 0, -1,
                      1, 0, 0;
    Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
    Eigen::Vector3d camera_ypr = eulers(R_camera2ideal, 1, 0, 2) * 57.3;  // degree

    // 计算标定板同竖直摆放时的偏角
    Eigen::Matrix3d R_board2world_eigen;
    cv::cv2eigen(R_board2world, R_board2world_eigen);
    Eigen::Vector3d board_ypr = eulers(R_board2world_eigen, 2, 1, 0) * 57.3;  // degree

    // 打印数据到终端
    print_terminal(R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, camera_ypr, distance, board_ypr);
}

// 后续再将角度整理到工具类之中
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false)
{
  Eigen::Quaterniond q(R);
  return eulers(q, axis0, axis1, axis2, extrinsic);
}

Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
  if (!extrinsic) std::swap(axis0, axis2);

  auto i = axis0, j = axis1, k = axis2;
  auto is_proper = (i == k);
  if (is_proper) k = 3 - i - j;
  auto sign = (i - j) * (j - k) * (k - i) / 2;

  double a, b, c, d;
  Eigen::Vector4d xyzw = q.coeffs();
  if (is_proper) {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  } else {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }

  Eigen::Vector3d eulers;
  auto n2 = a * a + b * b + c * c + d * d;
  eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

  auto half_sum = std::atan2(b, a);
  auto half_diff = std::atan2(-d, c);

  auto eps = 1e-7;
  auto safe1 = std::abs(eulers[1]) >= eps;
  auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
  auto safe = safe1 && safe2;
  if (safe) {
    eulers[0] = half_sum + half_diff;
    eulers[2] = half_sum - half_diff;
  } else {
    if (!extrinsic) {
      eulers[0] = 0;
      if (!safe1) eulers[2] = 2 * half_sum;
      if (!safe2) eulers[2] = -2 * half_diff;
    } else {
      eulers[2] = 0;
      if (!safe1) eulers[0] = 2 * half_sum;
      if (!safe2) eulers[0] = 2 * half_diff;
    }
  }

  for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

  if (!is_proper) {
    eulers[2] *= sign;
    eulers[1] -= CV_PI / 2;
  }

  if (!extrinsic) std::swap(eulers[0], eulers[2]);

  return eulers;
}

double limit_rad(double angle)
{
    while (angle > CV_PI)
        angle -= 2 * CV_PI;
    while (angle <= -CV_PI)
        angle += 2 * CV_PI;
    return angle;
}

private:
    std::string config_path_;
    int m_pattern_cols;           // 标定板列数(水平方向圆点数量)
    int m_pattern_rows;           // 标定板行数(垂直方向圆点数量)
    double m_center_distance_mm;  // 相邻圆点中心的距离(mm)
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<double> R_gimbal2imubody_data;
};


class CameraImuSync{
public: 
    // ros::Time serial_time;
    // ros::Time serial_time_;

    // 回调函数快进快出不做过多处理
    // 只获取数据
    void serialCallback(const rm_msgs::RmSerialConstPtr serial)
    {
        queue_serial.push(serial);
    }

    // ros::Time image_time;
    // ros::Time image_time_;
    void imageCallback(const sensor_msgs::ImageConstPtr img)
    {
        // // 在这里记录时间 放入队列中
        queue_image.push(img);                                    
    }

    void captureData() 
    {
        // 将缓冲队列通过回调函数采集到的数据封装到队列之中
        // 如果图像和IMU都有数据就放入到缓冲队列之中
        // 如果没有数据或者只有一个有数据就清除缓冲队列
        // 如果有就处理完数据后放入队列之中
        // 好像没有这个接口,等会再写
        if (queue_image.size() == 1 && queue_serial.size() == 1) {
            // 将数据封装成数据包
            ImuImageData data = ImuImageData(queue_image.back(),queue_serial.back());
            queue_.push(data);
        }
        else {
            // queue_image.clear();
            // queue_serial.clear();
            
            // 交换方式清空队列
            std::queue<sensor_msgs::ImageConstPtr> empty_image;
            std::swap(queue_image, empty_image);
            std::queue<rm_msgs::RmSerialConstPtr> empty_serial;
            std::swap(queue_serial, empty_serial);
        }
    }

    std::queue<ImuImageData> getDataQueue() {
        return queue_;
    }

private:
    rm_msgs::RmSerial serialData;
    double m_timeCompensation = 0.075;
    cv::Mat frame;

    // ThreadSafeQueue<sensor_msgs::ImageConstPtr> queue_image;     // 采集图像数据的缓冲队列
    // ThreadSafeQueue<sensor_msgs::ImageConstPtr> queue_serial;    // 采集IMU数据的缓冲队列
    std::queue<sensor_msgs::ImageConstPtr> queue_image;     // 采集图像数据的缓冲队列
    std::queue<rm_msgs::RmSerialConstPtr> queue_serial;    // 采集IMU数据的缓冲队列

    std::queue<ImuImageData> queue_; 
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "handeye_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 创建私有节点句柄

    // 从参数服务器获取配置文件路径
    std::string config_path;
    private_nh.param<std::string>("armor_identify_config", config_path, "");

    // 创建数据采集器
    CameraImuSync camimusync;
    ros::Subscriber serial = nh.subscribe("/RmSerialData", 1, &CameraImuSync::serialCallback, &camimusync);
    ros::Subscriber cameraSub = nh.subscribe("/hikrobot_camera/rgb", 1, &CameraImuSync::imageCallback, &camimusync);

    // 获取数据(imu和image)
    while(ros::ok()) {
        auto key = cv::waitKey(1);
        if (key == 'q')
            break;
        else if (key != 's')
            continue;
        // 按住一次按钮,清空一次话题队列,调用回调函数
        ros::spinOnce();
        camimusync.captureData();
    }

    // 创建手眼标定器
    Calibrate_handeye cal_handeye = Calibrate_handeye(config_path);

    // 从数据采集器中加载标定所需的数据
    std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
    std::vector<cv::Mat> rvecs, tvecs;
    cal_handeye.load(camimusync.getDataQueue(),R_world2gimbal_list,t_world2gimbal_list,rvecs,tvecs);

    // 手眼标定并将数据打印到终端上
    cal_handeye.calibrateCamera(
        rvecs,
        tvecs,
        R_world2gimbal_list,
        t_world2gimbal_list
    );
}