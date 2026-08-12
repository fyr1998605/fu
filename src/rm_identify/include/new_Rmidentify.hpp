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
#include "CoorConverter.hpp"
#include "math.hpp"
#include "SingleCost.hpp"
#include "ArmorData.hpp"
#include "Kalman.hpp"
#include <angles/angles.h>
#include <std_msgs/Float64.h>

//   camera_matrix: [1532.72094018318, 1.20708473070890, 677.758476120614, 0, 1532.64661551967, 516.802276390385, 0, 0, 1]
//   dist_coeffs: [-0.0513756135021033, -0.0437606955093151, 0.000102236661436325, -0.00190024505133129, 0.997225368207635]

long long begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//首时间
long long end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//尾时间
const int FIND_ANGLE_ITERATIONS = 12; // 三分法迭代次数 理想精度 < 1.

// 角度拟合 降低自由度
double fit_single_z_to_v(ArmorData armordata,const double &z_to_v_exp,const double &z_to_v_min,const double &z_to_v_max,CoordinateTransformer* coorConverter){
    // 1.构建代价函数对象
    SingleCost single_cost = SingleCost(SingleData(armordata, z_to_v_exp,coorConverter));
    // 2.初始化三分求解器
    math::Trisection solver;
    // 3.执行角度优化
    std::pair<double, double> res = solver.find(z_to_v_min, z_to_v_max, single_cost, FIND_ANGLE_ITERATIONS);
    // 4.角度归一化
    return math::reduced_angle(res.first);
}

// 限制角度到-PI~PI
double limit_rad(double angle)
{
    while (angle > CV_PI)
        angle -= 2 * CV_PI;
    while (angle <= -CV_PI)
        angle += 2 * CV_PI;
    return angle;
}

double limit_half_rad(double angle)
{
    while (angle > CV_PI / 2)
        angle -= CV_PI;
    while (angle <= -CV_PI / 2)
        angle += CV_PI;
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
    void projectCameraPointsToImage(Eigen::Vector3d center,double yaw, cv::Mat frame_, char number);
    std::vector<cv::Point2d> projectPointsToImage(const std::vector<cv::Point3d>& objectPoints, cv::Mat& frame, bool can_show);
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
    ros::Publisher debugpub4;            //debug4发布者
    ros::Publisher debugpub5;            //debug5发布者
    std_msgs::Float64 debugdate4;        //debug5数据
    std_msgs::Float64 debugdate5;        //debug5数据
    CoordinateTransformer* coorConverter;
    std::string config_path_;            //存储配置路径
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double m_timeCompensation;
    bool m_debug_show = true;            // 调试可视化开关(上线前设为 false)
};

RmIdentify::RmIdentify(ros::NodeHandle &nh, new_Identify::Config config,const std::string& config_path) : light(0, 45, 0, 86, 255, 255, 0), detectter(config), vtf(nh,config_path), config_path_(config_path)  
{   
    // 创建坐标变换器
    coorConverter =  new CoordinateTransformer(config_path_);
    armorsPub = nh.advertise<rm_msgs::ArmorArray>("/armor_array", 1000);
    imagePub = nh.advertise<sensor_msgs::Image>("/image_debug", 1000);
    shootFlag = 'a';
    debugpub4 = nh.advertise<std_msgs::Float64>("/debugpub4", 1000);
    debugpub5 = nh.advertise<std_msgs::Float64>("/debugpub5", 1000);

    image = Image(config_path_);

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

RmIdentify::~RmIdentify() {
    delete coorConverter;
}

ros::Time serial_time;
ros::Time serial_time_;
void RmIdentify::doserial(const rm_msgs::RmSerialConstPtr serial)
{
    serial_time = serial->header.stamp;
    serialData = *serial;
    shootFlag = serial->ShootFlag;
}

ros::Time now_time;
ros::Time now_time_;
ros::Time now_time_free;
void RmIdentify::doimage(const sensor_msgs::ImageConstPtr img)
{
    ros::Duration time_compensation_(m_timeCompensation); // 时间补偿量
    now_time = img->header.stamp;
    now_time = now_time - time_compensation_;
    cv_bridge::CvImagePtr cv_ptr;            // 声明一个CvImage指针的实例

    // // test: 
    // // 打印图像传输延时: 
    // now_time_free = ros::Time::now();       // 记录当前时间,用于测算图像传输延时 // 以s为单位(默认)
    // cout << "图像传输延时: " << (now_time_free - now_time) * 1000 << " " << "ms" << endl; // (打印的时候转换为ms)

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
    detectter.detect_color = this->serialData.Camp == 'r' ? 0 : 1; // true:1:0
}

// ///////////////////////////////////////////////zyz
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
    // === 帧去重: 跳过已处理的时间戳 (从 ArmorTask 提升到此, 减少无效调用) ===
    if (now_time_ == now_time)
    {
        return;
    }
    now_time_ = now_time;

    if (serial_time_ != serial_time)
    {
        serial_time_ = serial_time;
        vtf.TfUpdate({serialData.Roll, serialData.Pitch, serialData.Yaw},serial_time_);
    }

    // 根据串口 ShootFlag 切换识别模式
    if(this->shootFlag == 'a')
        ArmorTask();       // 装甲板模式
    else if(this->shootFlag == 'f')
        return;            // 大符模式: 由 Fu 节点独立处理,本节点跳过
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

std::vector<cv::Point2d> RmIdentify::projectPointsToImage(const std::vector<cv::Point3d>& objectPoints, cv::Mat& frame, bool can_show) {
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//旋转向量
    cv::Mat tvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//平移向量
    std::vector<cv::Point2d> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, this->camera_matrix_, this->dist_coeffs_, imagePoints);
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

// 降自由度调试可视化
// 重投影可视化函数
void RmIdentify::projectCameraPointsToImage(Eigen::Vector3d center,double yaw, cv::Mat frame_,char number) {
    double pitch_temp = 0;
    if (number == 6) {
        pitch_temp = -0.2618;
    }
    else {
        pitch_temp = 0.2618;
    }
    std::vector<Eigen::Vector3d> cornerpoint = coorConverter->armor2Corner(center,number,pitch_temp, yaw);
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
    // 最后显示图像 (仅调试模式)
    if (m_debug_show) {
        cv::imshow("ppppppppp", frame_);
        cv::waitKey(1);
    }
}

auto start = std::chrono::high_resolution_clock::now();
double last_yaw;
////////////////////////////////
// pitch轴误差测定全局变量
double z1 = 0;
double z2 = 0;
double dis1 = 0;
double dis2 = 0;
ros::Time current_time;
ros::Time last_switch_time_;
///////////////////////////////
std::vector<Point3d> point_buffer_; // 用于存储缓存点
//////////////////////////////
void RmIdentify::ArmorTask()
{
    int array_[8] = {0}; // 统计数字相同的装甲板
    // (帧去重已提升至 ArmorChoose, 此处不再重复检查)

    cv::Mat frame;
    frame = this->frame_.clone();
    cv::Mat framedebug;
    framedebug = this->frame_.clone();

    // PCA角点优化
    ////////////////////////////////
    cv::Mat gray_frame;
    if (!this->frame_.empty()) {
        cv::cvtColor(this->frame_, gray_frame, cv::COLOR_BGR2GRAY);
    }
    ////////////////////////////////

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

                if (armorNumber == 8) {
                    armorNumber = 6;
                }

                if (armorNumber == 9)
                    continue;

                // 判断装甲板的类型
                if (armorNumber == 1) {
                    aArmor.type = 'l';
                }
                else {
                    aArmor.type = 's';
                }


                // PCA角点优化
                /////////////////////////////////
                if (!gray_frame.empty()) {
                    // 传入当前装甲板引用和全图灰度图
                    // 这行代码会直接修改 aArmor 内部 left_light 和 right_light 的 top/bottom 坐标
                    detectter.corrector.correctCorners(aArmor, gray_frame);
                }

                // 2. 将优化后的新角点“同步”给 image 对象 (用于本次 PnP 解算)
                // 注意：这里必须严格按照 image.armour_points 要求的顺序 (通常是 TL, TR, BR, BL)
                // 你原来的顺序是 {armour_points_[1], armour_points_[2], armour_points_[3], armour_points_[0]}
                // 即 {左灯上, 右灯上, 右灯下, 左灯下}
                image.armour_points = {
                    aArmor.left_light.top,      // TL
                    aArmor.right_light.top,     // TR
                    aArmor.right_light.bottom,  // BR
                    aArmor.left_light.bottom    // BL
                };

                // 3. 同时更新本地数组 armour_points_ (用于后面 armordata 的封装)
                armour_points_[0] = aArmor.left_light.bottom;
                armour_points_[1] = aArmor.left_light.top;
                armour_points_[2] = aArmor.right_light.top;
                armour_points_[3] = aArmor.right_light.bottom;
                // ====================================================
                ////////////////////////////////
                
                // 这里ROI写的不对,但是没有影响到自瞄的正常运行,老代码
                // auto points = std::vector<cv::Point2f>{aArmor.left_light.top, aArmor.left_light.bottom, aArmor.right_light.top, aArmor.left_light.bottom};
                
                //////////////////////////
                auto points = std::vector<cv::Point2f>{aArmor.left_light.top, aArmor.left_light.bottom, aArmor.right_light.top, aArmor.right_light.bottom};
                ///////////////////

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

                // 获取RPY角度
                tf2::Matrix3x3 mat(qtn);
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);

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

                // 封装传参数据包
                ArmorData armordata;
                coorConverter->tfupdate_imu({serialData.Roll,serialData.Pitch,serialData.Yaw});
                armordata.center = coorConverter->cam2Map(
                Eigen::Vector3d(
                    armor.armorPose.position.x, 
                    armor.armorPose.position.y, 
                    armor.armorPose.position.z
                ));

                // // roll轴和pitch轴误差测定
                // {
                //     Point3d new_point = coorConverter->Vector2Point(armordata.center);
                //     if (point_buffer_.size() == 0) {
                //         point_buffer_.push_back(new_point);
                //     }
                //     // 每0.5秒存储一个点并进行一次计算
                //     if (norm(new_point - point_buffer_.back()) > 0.1) {
                //         // 将点添加到缓冲区
                //         point_buffer_.push_back(new_point);
                //         // 如果缓冲区大小超过限制，移除最aArmors早的点
                //         if (point_buffer_.size() > 30) {
                //             point_buffer_.erase(point_buffer_.begin()); // 移除第一个元素
                //         }
                //         // 如果点集数量小于3就跳过
                //         if (points.size() >= 3) {
                //             vector<double> roll_pitch = fitPlaneOpenCV_SVD(point_buffer_);
                //             debugdate4.data = roll_pitch[0]; // roll
                //             debugdate5.data = -roll_pitch[1]; // pitch
                //             // cout << "roll_off: " << roll_pitch[0] << endl;
                //             // cout << "pitch_off: " << roll_pitch[1] << endl;
                //         }
                //     }
                // }


                // // 测定pitch轴安装误差
                // {
                //     // 3秒切换开关
                //     current_time = ros::Time::now();
                //     ros::Duration elapsed = current_time - last_switch_time_;
                //     z2 = armordata.center .z();
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

                armordata.armor_type = aArmor.type;
                armordata.pus[0] = armour_points_[1];
                armordata.pus[1] = armour_points_[0];
                armordata.pus[2] = armour_points_[3];
                armordata.pus[3] = armour_points_[2];
                armordata.number = armorNumber;

                begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();

                // 装甲板角度拟合
                double z_to_v_fit = math::reduced_angle([&]() {
                    double zn_to_v = math::reduced_angle(
                        fit_single_z_to_v(
                            armordata, 
                            PI / 12, 
                            PI / 2, 
                            PI / 2 * 3,
                            coorConverter
                        )
                        - PI
                    );
                    return zn_to_v;
                }() + PI);


                end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
                double frame_time = (end_Time - begin_Time);
                // cout << frame_time << "ms" << endl;

                z_to_v_fit =  limit_half_rad(z_to_v_fit);

                z_to_v_fit += PI; // ***(很重要)
                // debugdate4.data = z_to_v_fit / 3.14 *180;
                // debugdate4.data = (z_to_v_fit + serialData.Yaw) / 3.14 * 180;
                // debugpub4.publish(debugdate4);
                // debugpub5.publish(debugdate5);  
                projectCameraPointsToImage(armordata.center,z_to_v_fit, framedebug, armorNumber);

                // // 3.打印数据
                // ROS_INFO("原角度: %f", z_to_l_exp);
                // ROS_INFO("拟合角度: %f", z_to_v_fit);

                // 将角度从欧拉角转换到四元数
                tf2::Quaternion q;
                if (armorNumber == 6) {
                    Eigen::Vector3d angle_temp = coorConverter->Angle_map2Cam(Eigen::Vector3d(0,-0.2618,z_to_v_fit + serialData.Yaw));
                    q.setRPY(angle_temp[0], angle_temp[1], angle_temp[2]);
                }
                else {
                    Eigen::Vector3d angle_temp = coorConverter->Angle_map2Cam(Eigen::Vector3d(0,0.2618,z_to_v_fit + serialData.Yaw));
                    q.setRPY(angle_temp[0], angle_temp[1], angle_temp[2]);
                }
                // 归一化四元数
                q.normalize();
                // 上传优化后的姿态(四元数)
                armor.armorPose.orientation.x = q.x();
                armor.armorPose.orientation.y = q.y();
                armor.armorPose.orientation.z = q.z();
                armor.armorPose.orientation.w = q.w();

                armors.armors.push_back(armor);
                cv::line(frame, aArmor.left_light.top, aArmor.left_light.bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(frame, aArmor.right_light.bottom, aArmor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(frame, aArmor.left_light.top, aArmor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(frame, aArmor.right_light.bottom, aArmor.left_light.bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

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
        if (m_debug_show) {
            cv::imshow("gds", frame);
            cv::waitKey(1);
        }
        this->PubDebugImage(frame);
    }
    this->armorsPub.publish(armors);
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
        }

        if (config["Time"]) {
            YAML::Node time = config["Time"];
            m_timeCompensation = time["timeCompensation"].as<double>(0.075);
            ROS_WARN("timeCompensation: %.3f ",m_timeCompensation);
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
    if (points.size() < 3) {
        throw std::invalid_argument("At least 3 points required to fit a plane");
    }
    
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
    
    return {a, b, c, d};
}