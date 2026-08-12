#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "rm_msgs/RmSerial.h"
#include "rm_msgs/Armor.h"
#include "rm_msgs/ArmorArray.h"
#include "Yolov8.h"
#include "Light.h"
#include "Image.h"
#include "visual.hpp"
#include "new_detector.h"
#include <time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/core/eigen.hpp>
#include "CoorConverter.hpp"
#include <angles/angles.h>
#include <std_msgs/Float64.h>
#include <yaml-cpp/yaml.h>

// 角度归一化
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif

long long begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//首时间
long long end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//尾时间

constexpr double LIGHTBAR_LENGTH = 56e-3;    // m 灯条长度
constexpr double BIG_ARMOR_WIDTH = 230e-3;   // m 大装甲板宽度
constexpr double SMALL_ARMOR_WIDTH = 135e-3; // m 小装甲板宽度

const std::vector<cv::Point3d> BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},   // 右上
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},  // 左上
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}, // 左下
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}}; // 右下

const std::vector<cv::Point3d> SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

// 限制角度到-PI~PI
double limit_rad(double angle)
{
    while (angle > CV_PI)
        angle -= 2 * CV_PI;
    while (angle <= -CV_PI)
        angle += 2 * CV_PI;
    return angle;
}

class RmIdentify
{
public:
    RmIdentify(ros::NodeHandle &nh, new_Identify::Config config,const std::string& config_path);
    ~RmIdentify();
    void doserial(const rm_msgs::RmSerialConstPtr serail);
    void doimage(const sensor_msgs::ImageConstPtr img);
    void ArmorChoose();
    void ArmorTask();
    void PubDebugImage(Mat image);
    std::vector<cv::Point2d> reproject_armor(const Eigen::Vector3d &xyz_map, double yaw, char type, char id) const;
    double armor_reprojection_error(Eigen::Vector3d xyz_map, double yaw, char id, vector<Eigen::Vector3d> points, char type,const double &inclined) const;
    void optimize_yaw(double &map_yaw, Eigen::Vector3d map_xyz, char id, vector<Eigen::Vector3d> points) const;
    std::vector<cv::Point2d> projectPointsToImage(const std::vector<cv::Point3d>& objectPoints,const cv::Mat& frame, bool can_show) const;
    void projectCameraPointsToImage(Eigen::Vector3d center,double yaw, cv::Mat frame_,char number);
    void setParam(const std::string &file_path);
    std::vector<double> fitPlaneOpenCV_SVD(const std::vector<Point3d>& points);

private:
    char shootFlag;
    cv::Mat frame_;
    rm_msgs::RmSerial serialData;
    Light light;
    Image image;
    Visual::VisualTF vtf;

    ros::Publisher armorsPub;
    ros::Publisher imagePub;
    new_Identify::detector detectter;
    CoordinateTransformer *coorConverter;       //坐标变换器 
    std::string config_path_;                   //存储配置路径
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // 调试
    ros::Publisher debugpub4;                   //debug4发布者  
    ros::Publisher debugpub5;                   //debug5发布者
    std_msgs::Float64 debugdate4;               //debug4数据
    std_msgs::Float64 debugdate5;               //debug5数据
    ros::Time current_time = ros::Time::now();
    ros::Time last_switch_time_ = ros::Time::now();
    double m_timeCompensation;
};

// 提示: 成员变量按照它们在类中声明的顺序进行初始化,而不是按照初始化列表的顺序初始化
// 如果成员列表初始化的变量来初始化成员列表里面的变量
// 就可能会出现看似已经初始化传入的变量了,其实后面的变量传入的变量初始化顺序在这之后,就导致这时候传入的初始化变量为空
RmIdentify::RmIdentify(ros::NodeHandle &nh, new_Identify::Config config, const std::string& config_path) : light(0, 45, 0, 86, 255, 255, 0), detectter(config), config_path_(config_path) , vtf(nh,config_path)
{
    coorConverter = new CoordinateTransformer(config_path_); // 调用默认构造
    image = Image(config_path_);
    armorsPub = nh.advertise<rm_msgs::ArmorArray>("/armor_array", 1000);
    imagePub = nh.advertise<sensor_msgs::Image>("/image_debug", 1000);
    shootFlag = 'a';

    // 调试发布
    debugpub4 = nh.advertise<std_msgs::Float64>("/debugpub4", 1000);
    debugpub5 = nh.advertise<std_msgs::Float64>("/debugpub5", 1000);

    // 配置参数
    if (!config_path_.empty()) {
        try {
            setParam(config_path_);
            ROS_INFO("Successfully loaded parameters from: %s", config_path_.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load parameters: %s", e.what());
        }
    } else {
        ROS_WARN("No configuration file path provided. Using default parameters.");
    }
}

RmIdentify::~RmIdentify()
{
    delete coorConverter;
}

ros::Time serial_time;
ros::Time serial_time_;
void RmIdentify::doserial(const rm_msgs::RmSerialConstPtr serial)
{
    serial_time = serial->header.stamp;
    serialData = *serial;
    shootFlag = serial->ShootFlag;
    coorConverter->tfupdate_imu({serialData.Roll, serialData.Pitch, serialData.Yaw}); // 更新坐标变换器
}

ros::Time now_time;
ros::Time now_time_;
void RmIdentify::doimage(const sensor_msgs::ImageConstPtr img)
{
    ros::Duration time_compensation_(m_timeCompensation); // 时间补偿量
    now_time = img->header.stamp;
    now_time = now_time - time_compensation_;

    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); // 将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    }
    catch (cv_bridge::Exception &e) // 异常处理
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    frame_ = cv_ptr->image.clone();

    // true:1:0
    detectter.detect_color = this->serialData.Camp == 'r' ? 0 : 1;
    //0是蓝色，1是红色
}

// ///////////////////////////////////////////////
// 视频播放
// #include <chrono>
// #include <thread>
// ros::Time now_time;
// ros::Time now_time_;
// cv::VideoCapture cap("/home/rm/桌面/red.avi");
// // 添加全局变量控制播放状态
// bool is_paused = false;
// bool video_initialized = false;
///////////////////////////////////////////////
void RmIdentify::ArmorChoose()
{
    // 视频播放
    // ////////////////////////////////////////////////////////////////////////////
    // now_time = ros::Time::now();
    // // // 初始化视频窗口(只执行一次)
    // // if (!video_initialized) {
    // //     cv::namedWindow("Video Playback", cv::WINDOW_AUTOSIZE);
    // //     video_initialized = true;
    // // }
    // // // 检查键盘输入
    // // int key = cv::waitKey(1) & 0xFF;
    // // if (key == 32) { // 空格键的ASCII码是32
    // //     is_paused = !is_paused; // 切换暂停状态
    // //     ROS_INFO("Video %s", is_paused ? "Paused" : "Resumed");
    // // }
    // // if (is_paused) {
    // //     cv::imshow("Video Playback", frame_); // 显示当前帧（保持显示）
    // //     return;
    // // }
    // cap >> frame_;
    // double fps = cap.get(cv::CAP_PROP_FPS); // 帧率
    // // 检查是否读取到帧，如果没读到（例如视频结束），则重置视频并重新读取
    // if (frame_.empty()) {
    //     cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 重置到视频开头
    //     cap >> frame_; // 再次尝试读取
    //     if (frame_.empty()) {
    //         ROS_WARN("Failed to reset video playback.");
    //         return; // 如果仍然失败，则返回
    //     }
    // }
    // // // 控制播放速度(只在非暂停状态下)
    // // if (!is_paused) {
    // //     cv::waitKey(1000 / fps); // 基本模拟原速播放
    // // }
    // // cv::waitKey(1000 / fps); // 基本模拟原速播放
    // ////////////////////////////////////////////////////////////////////////////

    if (serial_time_ != serial_time)
    {
        serial_time_ = serial_time;
        vtf.TfUpdate({serialData.Roll, serialData.Pitch, serialData.Yaw}, serial_time_);
    }

    // 检查是否设置了有效阵营
    // if (this->serialData.Camp) {
        // 我方为红色('r')时检测蓝色装甲板(0),否则检测红色装甲板
        // detectter.detect_color = this->serialData.Camp == 'r' ? 0:1;
        ArmorTask();
    // }   
}

void RmIdentify::PubDebugImage(Mat image)
{
    sensor_msgs::ImagePtr ros_image;
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";
    cv_image.image = image;
    ros_image = cv_image.toImageMsg();
    this->imagePub.publish(ros_image);
}

double last_yaw;
auto start = std::chrono::high_resolution_clock::now();
////////////////////////////////
// pitch轴误差测定全局变量
double z1 = 0;
double z2 = 0;
double dis1 = 0;
double dis2 = 0;
ros::Time current_time;
ros::Time last_switch_time_;
///////////////////////////////
///////////////////////////////
std::vector<Point3d> point_buffer_; // 用于存储缓存点
///////////////////////////////
void RmIdentify::ArmorTask()
{
    int array_[8] = {0}; // 统计数字相同的装甲板
    if (now_time_ == now_time)
    {
        return;
    }
    else
    {
        
    }
    now_time_ = now_time;
    cv::Mat frame;
    frame = this->frame_.clone();
    cv::Mat framedebug;
    framedebug = this->frame_.clone();

    // 装甲板数组
    // Image image(frame);
    rm_msgs::ArmorArray armors;
    armors.header.frame_id = "cam";
    armors.header.stamp = ros::Time::now();

    if (!this->frame_.empty())
    {
        std::vector<new_Identify::aArmor> aArmors = detectter.detect(frame);
        if (!aArmors.empty())
        {
            // 对每个aArmor作处理,
            for (int i = 0; i < aArmors.size(); i++)
            {
                new_Identify::aArmor aArmor = aArmors[i];
                cv::Point2f armour_points_[4];
                armour_points_[0] = aArmor.left_light.bottom;
                armour_points_[1] = aArmor.left_light.top;
                armour_points_[2] = aArmor.right_light.top;
                armour_points_[3] = aArmor.right_light.bottom;
                image.armour_points = {armour_points_[1], armour_points_[2], armour_points_[3], armour_points_[0]};

                // 识别数字的ROI
                cv::Mat numRoi;
                if (aArmor.type == 's')
                    numRoi = this->detectter.extractNumbers(frame, armour_points_, true);
                else if (aArmor.type == 'l')
                    numRoi = this->detectter.extractNumbers(frame, armour_points_, false);
                if (numRoi.empty())
                    continue;
                // 识别数字
                int armorNumber = this->detectter.classify(numRoi, aArmor);
                if (armorNumber == 9)
                    continue;
                auto points = std::vector<cv::Point2f>{aArmor.left_light.top, aArmor.left_light.bottom, aArmor.right_light.top, aArmor.left_light.bottom};
                auto bounding_rect = cv::boundingRect(points);
                Rect roi(bounding_rect);
                // Rect roi(detectter.detect_rect(aArmor));
                image.aType = aArmor.type;

                // 计算装甲板中心
                image.calculate_armour_center();
                Eigen::Vector2d armorCenter;
                armorCenter[0] = aArmor.center.x;
                armorCenter[1] = aArmor.center.y;

                // pnp
                image.get_distance();

                // 单块装甲板
                rm_msgs::Armor armor;
                armor.armorPose.position.x = image.tvec.at<double>(0, 0) / 1000;
                armor.armorPose.position.y = image.tvec.at<double>(1, 0) / 1000;
                armor.armorPose.position.z = image.tvec.at<double>(2, 0) / 1000;
                cv::Mat R;
                cv::Rodrigues(image.rvec, R);
                // 确保输入是3x3的双精度矩阵
                CV_Assert(R.type() == CV_64F && R.rows == 3 && R.cols == 3);
                // 创建tf2旋转矩阵并填充数据
                tf2::Matrix3x3 tf_rot(
                    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), // 第一行
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), // 第二行
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)  // 第三行
                );
                // 转换为四元数
                tf2::Quaternion qtn;
                tf_rot.getRotation(qtn);
                qtn.normalize();
                armor.armorPose.orientation.x = qtn.getX();
                armor.armorPose.orientation.y = qtn.getY();
                armor.armorPose.orientation.z = qtn.getZ();
                armor.armorPose.orientation.w = qtn.getW();
                armor.number = armorNumber;
                // armor.type = image.aType;
                armor.type = aArmor.type;
                armor.rectX = roi.x;
                armor.rectY = roi.y;
                armor.rectWidth = roi.width;
                armor.rectHeight = roi.height;
                Eigen::Vector2d cameraCenter;
                cameraCenter[0] = image.camera_center.x;
                cameraCenter[1] = image.camera_center.y;
                armor.armorToCenterDis = (armorCenter - cameraCenter).norm();

                // 将点坐标从相机系转换为世界系
                Eigen::Vector3d armor_position_map = coorConverter->cam2Map(
                    Eigen::Vector3d(
                        armor.armorPose.position.x,
                        armor.armorPose.position.y,
                        armor.armorPose.position.z));
                // 上传世界系下的点坐标
                armor.armorPose.position.x = armor_position_map.x();
                armor.armorPose.position.y = armor_position_map.y();
                armor.armorPose.position.z = armor_position_map.z();

                // roll轴和pitch轴误差测定
                {
                    Point3d new_point = coorConverter->Vector2Point(armor_position_map);
                    if (point_buffer_.size() == 0) {
                        point_buffer_.push_back(new_point);
                    }
                    if (norm(new_point - point_buffer_.back()) > 0.2) {
                        // 将点添加到缓冲区
                        point_buffer_.push_back(new_point);
                        // 如果缓冲区大小超过限制，移除最早的点
                        if (point_buffer_.size() > 30) {
                            point_buffer_.erase(point_buffer_.begin()); // 移除第一个元素
                        }
                        // 如果点集数量小于3就跳过
                        if (points.size() >= 3) {
                            vector<double> roll_pitch = fitPlaneOpenCV_SVD(point_buffer_);
                            debugdate4.data = roll_pitch[0]; // roll
                            debugdate5.data = -roll_pitch[1]; // pitch
                            // cout << "roll_off: " << roll_pitch[0] << endl;
                            // cout << "pitch_off: " << roll_pitch[1] << endl;
                        }
                    }
                }

                // // 测定pitch轴安装误差(C板和相机)
                // {
                //     // 3秒切换开关
                //     current_time = ros::Time::now();
                //     ros::Duration elapsed = current_time - last_switch_time_;
                //     z2 = armor_position_map.z();
                //     dis2 = sqrt(pow(armor.armorPose.position.x, 2) + pow(armor.armorPose.position.y, 2) + pow(armor.armorPose.position.z,2));
                //     // 切换开关
                //     if (elapsed.toSec() >= 3.0) {
                //         last_switch_time_ = current_time;
                //         if(abs(dis2 - dis1) > 0.5){
                //             if (dis2 > dis1) {
                //                 double theta = asin((z2 - z1) / (dis2 - dis1));
                //                 cout << "theta: " << theta << endl;
                //                 debugdate4.data = theta;
                //             }
                //             else {
                //                 double theta = asin((z1 - z2) / (dis1 - dis2));
                //                 cout << "theta: " << theta << endl;
                //                 debugdate4.data = theta;
                //             }
                //         }
                //         z1 = z2;
                //         dis1 = dis2;
                //     }
                // }

                // 获取RPY角度
                tf2::Matrix3x3 mat(qtn);
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);

                yaw = yaw + serialData.Yaw; 
                vector<Eigen::Vector3d> points_pus(4);
                points_pus[0] = Eigen::Vector3d(0,armour_points_[1].x,armour_points_[1].y); 
                points_pus[1] = Eigen::Vector3d(0,armour_points_[2].x,armour_points_[2].y); 
                points_pus[2] = Eigen::Vector3d(0,armour_points_[3].x,armour_points_[3].y); 
                points_pus[3] = Eigen::Vector3d(0,armour_points_[0].x,armour_points_[0].y); 

                // cout << "优化前的角度: " << yaw << endl;
                // debugdate4.data = yaw;  // 优化前的角度发布
                // 角度优化
                // begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
                optimize_yaw(yaw,armor_position_map,armorNumber,points_pus);
                // end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
                // double frame_time = (end_Time - begin_Time);
                // cout << frame_time << "ms" << endl;
                // cout << "优化后的角度: " << yaw << endl;
                // debugdate5.data = yaw; // 优化后的角度发布
                // debug: 发布调试信息
                debugpub4.publish(debugdate4);
                debugpub5.publish(debugdate5);
                yaw += PI; // ***(很重要)
                projectCameraPointsToImage(armor_position_map,yaw, framedebug, armorNumber);

                // 将角度转换为四元数发布
                // 前哨战装甲板pitch倾斜角: -15度 其他: 15度
                tf2::Quaternion q;
                if (armorNumber == 6)
                {
                    Eigen::Vector3d angle_temp = coorConverter->Angle_map2Cam(Eigen::Vector3d(0,-0.2618,yaw));
                    q.setRPY(angle_temp[0], angle_temp[1], angle_temp[2]);
                }
                else
                {
                    Eigen::Vector3d angle_temp = coorConverter->Angle_map2Cam(Eigen::Vector3d(0,0.2618,yaw));
                    q.setRPY(angle_temp[0], angle_temp[1], angle_temp[2]);
                }
                // 归一化四元数
                q.normalize();
                armor.armorPose.orientation.x = q.x();
                armor.armorPose.orientation.y = q.y();
                armor.armorPose.orientation.z = q.z();
                armor.armorPose.orientation.w = q.w();

                armor.armorPose.position.x = image.tvec.at<double>(0, 0) / 1000;
                armor.armorPose.position.y = image.tvec.at<double>(1, 0) / 1000;
                armor.armorPose.position.z = image.tvec.at<double>(2, 0) / 1000;

                armors.armors.push_back(armor);
                cv::line(
                    frame, aArmor.left_light.top, aArmor.left_light.bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(
                    frame, aArmor.right_light.bottom, aArmor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(
                    frame, aArmor.left_light.top, aArmor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(frame,
                         aArmor.right_light.bottom,
                         aArmor.left_light.bottom,
                         cv::Scalar(0, 255, 0),
                         1,
                         cv::LINE_AA);

                putText(frame, "number : " + std::to_string(armorNumber), Point(0, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));
            }

            int count_armor[8] = {0}; // 统计数字相同的装甲板
            for (int i = 0; i < armors.armors.size(); i++)
            {
                // 根据出现次数确定frame_id
                count_armor[armors.armors[i].number - 1] += 1;
                // cout << "识别上传数据: " << image.tvec.at<double>(0, 0) / 1000.0 << " " << tf_data.transform.translation.y << endl;
                // ROS_DEBUG("[Identify] 发布装甲板TF: %s (ID=%d, 计数=%d)",
                //      frame_id.c_str(), armorNumber, armor_count+1);
            }

            for (int i = 0; i < armors.armors.size(); i++)
            {
                count_armor[armors.armors[i].number - 1] -= 1;
                int armor_count = count_armor[armors.armors[i].number - 1];
                std::string frame_id;
                if (armor_count == 0)
                {
                    // 第一块该ID的装甲板
                    frame_id = "DETECT" + std::to_string(armors.armors[i].number);
                }
                else if (armor_count == 1)
                {
                    // 第二块该ID的装甲板
                    frame_id = "DETECT" + std::to_string(armors.armors[i].number) + "_0";
                }
                else
                {
                    // 超过两块，跳过不处理
                    // ROS_WARN("[Identify] 装甲板ID %d 超过两块，跳过", armorNumber);
                    continue;
                }
                geometry_msgs::TransformStamped tf_data;
                tf_data.header.frame_id = "cam";
                tf_data.child_frame_id = frame_id;
                tf_data.header.stamp = now_time_;
                tf_data.transform.translation.x = armors.armors[i].armorPose.position.x;
                tf_data.transform.translation.y = armors.armors[i].armorPose.position.y;
                tf_data.transform.translation.z = armors.armors[i].armorPose.position.z;
                tf_data.transform.rotation.w = armors.armors[i].armorPose.orientation.w;
                tf_data.transform.rotation.x = armors.armors[i].armorPose.orientation.x;
                tf_data.transform.rotation.y = armors.armors[i].armorPose.orientation.y;
                tf_data.transform.rotation.z = armors.armors[i].armorPose.orientation.z;
                tf2_ros::TransformBroadcaster tfPubTemp;
                tfPubTemp.sendTransform(tf_data);
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        start = end;
        circle(frame, image.camera_center, 8, Scalar(255, 0, 0), -1, 8);
        putText(frame, "time: " + std::to_string(duration.count() * 1000) + "ms", Point(0, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));
        putText(frame, "fps : " + std::to_string(1 / duration.count()), Point(0, 160), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));

        cv::imshow("gds", frame);
        cv::waitKey(1);
        this->PubDebugImage(frame);
    }

    this->armorsPub.publish(armors);
}

// 将3D装甲板投影到2D图像平面
std::vector<cv::Point2d> RmIdentify::reproject_armor(
const Eigen::Vector3d &xyz_map, double yaw, char type, char id) const{
    // 计算yaw角的三角函数
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto pitch = (id == 6) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // // 正确初始化方式
    Eigen::Matrix3d R_armor2world;
    R_armor2world << cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch,
                    sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch,
                    -sin_pitch, 0, cos_pitch;

    // 创建装甲板到世界的平移旋转矩阵
    const Eigen::Vector3d &t_armor2world = xyz_map;

    // 获取旋转矩阵和平移矩阵
    Eigen::Matrix3d R_camera2gimbal_ = coorConverter->get_RT().cam2Imu_rot_mat;
    Eigen::Vector3d t_camera2gimbal_ = coorConverter->get_RT().cam2Imu_trans_mat;
    Eigen::Matrix3d R_gimbal2world_ = coorConverter->get_RT().imu2Map_rot_mat; 

    // armor装甲板系 -> cam相机系 旋转矩阵
    Eigen::Matrix3d R_armor2camera =
        R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;

    // armor装甲板系 -> cam相机系 平移矩阵
    Eigen::Vector3d t_armor2camera =
        R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    cv::Vec3d rvec;             // 声明旋转向量
    cv::Mat R_armor2camera_cv;  // 声明OpenCV矩阵,用于存储旋转向量(Rodrigues向量)
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv); // Eigen到OpenCV的矩阵转换
    cv::Rodrigues(R_armor2camera_cv, rvec);          // 旋转矩阵转换为旋转向量
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]); // 构造平移向量

    // 执行到3D到2D投影
    std::vector<cv::Point2d> image_points;
    const auto &object_points = (type == 'l') ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    cv::projectPoints(object_points, rvec, tvec, image.cameraMatrix, image.distCoeffs, image_points);
    return image_points;
}

// 计算装甲板的重投影误差
double RmIdentify::armor_reprojection_error(
    Eigen::Vector3d xyz_map, double yaw, char id,vector<Eigen::Vector3d> points,char type,const double &inclined) const
{
    // 同济坐标变换
    auto image_points = reproject_armor(xyz_map, yaw, type, id);
    // 误差计算
    // 遍历4个角点: 计算每个角点的误差
    auto error = 0.0;
    for (int i = 0; i < 4; i++) {
        cv::Point2d eigen_point(points[i].y(), points[i].z());
        // cout << "实际图像的: " << i << " " << "x: " << eigen_point.x << "y: " << eigen_point.y << endl;
        // cout << "角点变换的: " << i << " " << "x: " << image_points[i].x << "y: " << image_points[i].y << endl;
        error += cv::norm(eigen_point - image_points[i]);
    }
    return error;
}

// 通过暴力搜索法来优化装甲板的偏航角(yaw)
void RmIdentify::optimize_yaw(double &map_yaw, Eigen::Vector3d map_xyz, char id, vector<Eigen::Vector3d> points) const
{
    // 获取
    Eigen::Vector3d gimbal_rpy = Eigen::Vector3d(
        serialData.Roll,
        serialData.Pitch,
        serialData.Yaw);

    // 设置搜索参数
    constexpr double SEARCH_RANGE = 140; // degree
    auto yaw0 = limit_rad(gimbal_rpy[2] - SEARCH_RANGE / 2 * CV_PI / 180.0);
    
    char type = 's';
    if (id == 1 || id == 8)
    {
        type = 'l';
    }
    else {
        type = 's';
    }

    // 初始化搜索变量
    auto min_error = 1e10;
    auto best_yaw = map_yaw;
    // 暴力优化搜索
    for (int i = 0; i < SEARCH_RANGE; i++)
    {
        double yaw = limit_rad(yaw0 + i * CV_PI / 180.0);
        
        auto error = armor_reprojection_error(map_xyz, yaw, id, points, type,(i - SEARCH_RANGE / 2) * CV_PI / 180.0);
        // cout << "error: " << error << endl;

        if (error < min_error)
        {
            min_error = error;
            best_yaw = yaw;
        }
    }
    // cout << "min_error" << min_error << endl;
    // 保存结果
    map_yaw = best_yaw;
    
}
 
std::vector<cv::Point2d> RmIdentify::projectPointsToImage(const std::vector<cv::Point3d>& objectPoints,const cv::Mat& frame, bool can_show) const {
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//旋转向量
    cv::Mat tvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//平移向量
    std::vector<cv::Point2d> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, camera_matrix_, dist_coeffs_, imagePoints);
    if(can_show){
        for (size_t i = 0; i < imagePoints.size(); ++i) {
            cv::circle(frame, imagePoints[i], 7, cv::Scalar(0, 255, 0), -1); 
            //std::cout << "3D Point " << i << ": " << objectPoints[i] << std::endl;
            //std::cout << "Projected 2D Point " << i << ": " << imagePoints[i] << std::endl;
        }
        cv::imshow("Draw Points", frame);
        cv::waitKey(1);
    }
    return imagePoints;
}

void RmIdentify::projectCameraPointsToImage(Eigen::Vector3d center,double yaw, cv::Mat frame_,char number) {

    double pitch_temp = 0;
    if (number == 6) {
        pitch_temp = -0.2618;
    }
    else {
        pitch_temp = 0.2618;
    }
    std::vector<Eigen::Vector3d> cornerpoint = coorConverter->armor2Corner(center,number,pitch_temp, yaw - serialData.Yaw);

    // 将角点从世界系 -> 相机系
    for (int i = 0; i < cornerpoint.size(); i++) {
        cornerpoint[i] = coorConverter->map2Cam(cornerpoint[i]);
    }

    // 调用前进行类型转换
    std::vector<cv::Point3d> cvPoints;
    cvPoints.reserve(cornerpoint.size());
    
    for (const auto& point : cornerpoint) {
        cvPoints.emplace_back(point.x(), point.y(), point.z());
    }

    // 投影每块装甲板的四个角点
    std::vector<cv::Point2d> cornerPoints2D = projectPointsToImage(cvPoints, frame_, 0);

    // 绘制四个角点
    for (const auto& point : cornerPoints2D) {
        cv::circle(frame_,point , 5, cv::Scalar(0, 0, 255), -1); // 红色角点
    }

    // 绘制装甲板边框(连接四个角点)
    for (int j = 0; j < 4; j++) {
        int next_j = (j + 1) % 4;
        cv::line(frame_, cornerPoints2D[j], cornerPoints2D[next_j], 
        cv::Scalar(255, 255, 0), 2); // 青色边框
    }

    // 绘制对角线
    cv::line(frame_, cornerPoints2D[0], cornerPoints2D[2], 
        cv::Scalar(0, 255, 255), 2); // 黄色对角线1（左上到右下）
    cv::line(frame_, cornerPoints2D[1], cornerPoints2D[3], 
        cv::Scalar(0, 255, 255), 2); // 黄色对角线2（右上到左下）

    // 最后显示图像
    cv::imshow("ppppppppp", frame_);
    cv::waitKey(1);
}

void RmIdentify::setParam(const std::string &file_path) {
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


        // 加载参数
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

            if (config["Time"]) {
                YAML::Node time = config["Time"];
                m_timeCompensation = time["timeCompensation"].as<double>(0.075);
                ROS_WARN("timeCompensation: %.3f ",m_timeCompensation);
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

// 点集求平面法向量：
std::vector<double> RmIdentify::fitPlaneOpenCV_SVD(const std::vector<Point3d>& points) {
    // 计算质心
    Point3d centroid{0, 0, 0};
    for (const auto& p : points) {
        centroid.x += p.x;
        centroid.y += p.y;
        centroid.z += p.z;
    }
    double inv_n = 1.0 / points.size();
    centroid.x *= inv_n;
    centroid.y *= inv_n;
    centroid.z *= inv_n;
    
    // 构建协方差矩阵
    cv::Mat cov_mat = cv::Mat::zeros(3, 3, CV_64F);
    for (const auto& p : points) {
        double dx = p.x - centroid.x;
        double dy = p.y - centroid.y;
        double dz = p.z - centroid.z;
        
        cov_mat.at<double>(0, 0) += dx * dx;
        cov_mat.at<double>(0, 1) += dx * dy;
        cov_mat.at<double>(0, 2) += dx * dz;
        
        cov_mat.at<double>(1, 0) += dy * dx;  // 对称，可只计算上三角
        cov_mat.at<double>(1, 1) += dy * dy;
        cov_mat.at<double>(1, 2) += dy * dz;
        
        cov_mat.at<double>(2, 0) += dz * dx;  // 对称
        cov_mat.at<double>(2, 1) += dz * dy;  // 对称
        cov_mat.at<double>(2, 2) += dz * dz;
    }
    
    // 归一化协方差矩阵（提高数值稳定性）
    cov_mat *= inv_n;
    
    // SVD分解
    cv::Mat w, u, vt;
    cv::SVD::compute(cov_mat, w, u, vt, cv::SVD::FULL_UV);
    
    // 最小特征值对应的特征向量即为法向量（vt的最后一行）
    cv::Mat normal_mat = vt.row(2);
    
    double a = normal_mat.at<double>(0);
    double b = normal_mat.at<double>(1);
    double c = normal_mat.at<double>(2);
    
    // 归一化法向量
    double norm = sqrt(a*a + b*b + c*c);
    if (norm < std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Failed to compute plane normal");
    }
    
    a /= norm;
    b /= norm;
    c /= norm;
    
    // 计算D：平面方程 Ax + By + Cz + D = 0
    double d = -(a*centroid.x + b*centroid.y + c*centroid.z);

    double roll = -asin(b);
    double pitch = atan2(a,c);
    
    return {roll,pitch};
}
