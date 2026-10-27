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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

class RmIdentify
{
public:
    RmIdentify(ros::NodeHandle &nh, Identify::Config config,const std::string& config_path);
    void doserial(const rm_msgs::RmSerialConstPtr serail);
    void doimage(const sensor_msgs::ImageConstPtr img);
    void ArmorChoose();
    void ArmorTask();
    void PubDebugImage(Mat image);
    void setParam(const std::string &file_path);


private:
    char shootFlag;
    cv::Mat frame;
    rm_msgs::RmSerial serialData;
    Light light;
    Image image;
    Visual::VisualTF vtf;
    Identify::YOLOV8 yolovArmor;
    ros::Publisher armorsPub;
    ros::Publisher imagePub;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::string config_path_;                   //存储配置路径
    bool has_serial_data_ = false;
    std::chrono::steady_clock::time_point serial_received_at_;
    ros::Time image_stamp_, processed_image_stamp_;
};

RmIdentify::RmIdentify(ros::NodeHandle &nh, Identify::Config config, const std::string& config_path): light(0, 45, 0, 86, 255, 255, 0), yolovArmor(config), config_path_(config_path) , vtf(nh,config_path)
{
    armorsPub = nh.advertise<rm_msgs::ArmorArray>("/armor_array", 1000);
    imagePub = nh.advertise<sensor_msgs::Image>("/image_debug", 1000);
    shootFlag = 'a';

    // // 配置参数
    // if (!config_path_.empty()) {
    //     try {
    //         setParam(config_path_);
    //         ROS_INFO("Successfully loaded parameters from: %s", config_path_.c_str());
    //     } catch (const std::exception& e) {
    //         ROS_ERROR("Failed to load parameters: %s", e.what());
    //     }
    // } else {
    //     ROS_WARN("No configuration file path provided. Using default parameters.");
    // }
}

void RmIdentify::doserial(const rm_msgs::RmSerialConstPtr serial)
{
    const double age = (ros::Time::now() - serial->header.stamp).toSec();
    if (serial->header.stamp.isZero() || age < 0.0 || age > 0.2 ||
        !std::isfinite(serial->Roll) || !std::isfinite(serial->Pitch) ||
        !std::isfinite(serial->Yaw)) {
        has_serial_data_ = false;
        return;
    }
    if (has_serial_data_ && serialData.header.stamp == serial->header.stamp) return;
    serialData = *serial;
    has_serial_data_ = true;
    serial_received_at_ = std::chrono::steady_clock::now();
    shootFlag = serial->ShootFlag;
}

void RmIdentify::doimage(const sensor_msgs::ImageConstPtr img)
{
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
    try
    {
        cv_ptr =  cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    }
    catch(cv_bridge::Exception& e)  //异常处理
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (cv_ptr->image.empty() || img->header.stamp.isZero()) return;
    frame = cv_ptr->image.clone();
    image_stamp_ = img->header.stamp;
}

void RmIdentify::ArmorChoose()
{
    const double serial_age = (ros::Time::now() - serialData.header.stamp).toSec();
    const double image_age = (ros::Time::now() - image_stamp_).toSec();
    if (!has_serial_data_ || image_stamp_.isZero() ||
        image_stamp_ == processed_image_stamp_ || serial_age < 0.0 ||
        serial_age > 0.2 || image_age < 0.0 || image_age > 0.2 ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
            serial_received_at_).count() > 0.2) return;
    processed_image_stamp_ = image_stamp_;
    // 更新固定坐标变换
    vtf.TfUpdate({serialData.Roll, serialData.Pitch, serialData.Yaw});
    // 选择击打任务
    if(this->shootFlag == 'a')
        ArmorTask();
    else if(this->shootFlag == 'f')
        return;
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


void RmIdentify::ArmorTask()
{   
    // 装甲板数组
    rm_msgs::ArmorArray armors;
    armors.header.frame_id = "cam";
    armors.header.stamp = image_stamp_;
    // ============新增：初始化能量机关字段清空============
    armors.blade_xs.clear();
    armors.blade_ys.clear();
    armors.has_fan_target = false;
    armors.circle_x = 0.f;
    armors.circle_y = 0.f;

    if(!this->frame.empty())
    {
        // 开始计时
        auto start = std::chrono::high_resolution_clock::now();
        // 获取神经网络返回的box
        vector<Identify::Detection> box;
        box = yolovArmor.detect(frame);
        auto end2 = std::chrono::high_resolution_clock::now();

        // 对box作处理
        if(!box.empty())
        {
            //对每个box作处理
            for(int i = 0; i < box.size();i++)
            {
                Identify::Detection detection = box[i];
                float cx = detection.box.x + detection.box.width / 2.f;
                float cy = detection.box.y + detection.box.height / 2.f;

                // =====能量机关扇叶/圆心填充逻辑=====
                if(detection.class_id == 0)
                {
                    // class0 = 扇叶，存入数组
                    armors.blade_xs.push_back(cx);
                    armors.blade_ys.push_back(cy);
                }
                else if(detection.class_id == 1)
                {
                    // class1 = 能量机关旋转圆心R
                    armors.circle_x = cx;
                    armors.circle_y = cy;
                    armors.has_fan_target = true;
                }
                // ===================================

                // 判断阵营：敌方装甲才处理，扇叶不做阵营过滤
                if(Identify::coconame[box[i].class_id][0] == this->serialData.Camp)
                    continue;
                    
                //选取感兴趣区作处理
                Rect roi(detection.box);
                Mat frame_roi = frame(roi);
                // 二值化
                Mat hsv = light.fiter_hsv(frame_roi);
                //返回装甲板四点
                image.armour_points = light.refine_the_armor(hsv,HSV);
                if(!image.armour_points.empty())
                {       
                    //判断长短装甲板
                    image.BigOrSmall();
                    // 获取用来识别数字的四点
                    cv::Point2f armour_points_[4];
                    armour_points_[0] = image.armour_points[3];
                    armour_points_[1] = image.armour_points[0];
                    armour_points_[2] = image.armour_points[1];
                    armour_points_[3] = image.armour_points[2];
                    // 识别数字的ROI
                    Mat numRoi;
                    if(image.aType == 's')
                        numRoi = this->yolovArmor.extractNumbers(frame_roi, armour_points_, true);
                    else if(image.aType == 'l')
                        numRoi = this->yolovArmor.extractNumbers(frame_roi, armour_points_, false);

                    if(numRoi.empty())
                        continue;
                    // 识别数字
                    int armorNumber = this->yolovArmor.classify(numRoi);
                    
                    //计算装甲板中心 
                    image.calculate_armour_center();
                    Eigen::Vector2d armorCenter;
                    armorCenter[0] = image.armour_center.x + roi.x;
                    armorCenter[1] = image.armour_center.y + roi.y;

                    for(int i = 0; i < image.armour_points.size(); i++){
                        image.armour_points[i].x += roi.x;
                        image.armour_points[i].y += roi.y;
                    }
                    
                    // pnp
                    image.get_distance();

                    // 单块装甲板
                    rm_msgs::Armor armor;
                    armor.armorPose.position.x = image.tvec.at<double>(0, 0) / 1000;
                    armor.armorPose.position.y = image.tvec.at<double>(1, 0) / 1000;
                    armor.armorPose.position.z = image.tvec.at<double>(2, 0) / 1000;
                    tf2::Quaternion qtn;
                    cv::Mat R;
                    cv::Rodrigues(image.rvec, R);
                    // 从旋转矩阵 R 中提取欧拉角
                    double theta_x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
                    double theta_y = atan2(-R.at<double>(2, 0), sqrt(pow(R.at<double>(2, 1), 2) + pow(R.at<double>(2, 2), 2)));
                    double theta_z = atan2(R.at<double>(1, 0), R.at<double>(0, 0)); 
                    qtn.setRPY(theta_x, theta_y, theta_z);
                    armor.armorPose.orientation.x = qtn.getX();
                    armor.armorPose.orientation.y = qtn.getY();
                    armor.armorPose.orientation.z = qtn.getZ();
                    armor.armorPose.orientation.w = qtn.getW();
                    armor.number = armorNumber;
                    armor.type = image.aType;
                    armor.rectX = roi.x;
                    armor.rectY = roi.y;
                    armor.rectWidth = roi.width;
                    armor.rectHeight = roi.height;
                    Eigen::Vector2d cameraCenter;
                    cameraCenter[0] = image.camera_center.x;
                    cameraCenter[1] = image.camera_center.y;
                    armor.armorToCenterDis = (armorCenter - cameraCenter).norm();

                    armors.armors.push_back(armor);

                    // circle(frame, Point(armorCenter[0], armorCenter[1]), 8, Scalar(255, 0, 0), -1, 8);
                    for(int i = 0; i < 4; i++)
                        line(frame, image.armour_points[i], image.armour_points[(i+2)%4] ,Scalar(255, 0, 255),4, 8);
                    line(frame, image.armour_points[0], image.armour_points[3] ,Scalar(255, 0, 255),4, 8);
                    line(frame, image.armour_points[1], image.armour_points[2] ,Scalar(255, 0, 255),4, 8);
                    putText(frame, "number: " + std::to_string(armorNumber), Point(image.armour_points[0].x, image.armour_points[0].y - 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,cv::Scalar(255,255,255), 4, 8);    

                }
            }  
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::chrono::duration<double> duration2 = end2 - start;

        circle(frame, image.camera_center, 8, Scalar(255, 0, 0), -1, 8);
        putText(frame, "time: " + std::to_string(duration.count() * 1000) + "ms", Point(0, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0,cv::Scalar(255,255,255));
        putText(frame, "time_yolo: " + std::to_string(duration2.count() * 1000) + "ms", Point(0, 120), cv::FONT_HERSHEY_SIMPLEX, 1.0,cv::Scalar(255,255,255));
        putText(frame, "fps : " + std::to_string(1 / duration.count()), Point(0, 160), cv::FONT_HERSHEY_SIMPLEX, 1.0,cv::Scalar(255,255,255));

        this->PubDebugImage(frame);
    }
    // 发布整合装甲+扇叶圆心的完整消息
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
                for (int i = 0; i < cam_matrix_array.size(); i++) {
                    ROS_INFO("cam_matrix_array%d:%.3f",i,cam_matrix_array[i]);
                }
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
