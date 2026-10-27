/**
 * Buff 识别器独立测试节点
 * 只做: 相机拿图 → BuffRecognizer::identify() → 画面显示
 * 不依赖串口 / Fu_angleprocess / buff_predicted / PnP
 *
 * 运行: roslaunch rm_tracker rm_fu_test.launch
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include "buff_recognizer.hpp"

BuffRecognizer* g_recognizer = nullptr;
bool g_initialized = false;

// 图像回调 — 逐帧识别并显示
void onImage(const sensor_msgs::ImageConstPtr& img) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        if (!g_initialized) {
            cv::Point2f init_R, init_Fan;
            double init_side;
            if (g_recognizer->init(frame, init_R, init_Fan, init_side)) {
                g_initialized = true;
                ROS_INFO("[Test] 首帧初始化成功 R(%d,%d) Fan(%d,%d) side=%.1f",
                         (int)init_R.x, (int)init_R.y,
                         (int)init_Fan.x, (int)init_Fan.y, init_side);
            } else {
                ROS_WARN_THROTTLE(2, "[Test] 等待首帧识别...");
                return;
            }
        }

        // 识别
        cv::Point2f R_center, Fan_center;
        double side;
        std::vector<cv::Point2f> corners;
        if (g_recognizer->identify(frame, R_center, Fan_center, side, corners)) {
            // 显示识别结果
            g_recognizer->visual(frame);

            // 打印6关键点映射后的坐标
            ROS_INFO_THROTTLE(1.0,
                "[Test] R:(%.0f,%.0f) Fan:(%.0f,%.0f) side:%.1f corners:[%.0f,%.0f][%.0f,%.0f][%.0f,%.0f][%.0f,%.0f]",
                R_center.x, R_center.y, Fan_center.x, Fan_center.y, side,
                corners[0].x, corners[0].y, corners[1].x, corners[1].y,
                corners[2].x, corners[2].y, corners[3].x, corners[3].y);
        } else {
            ROS_WARN_THROTTLE(1, "[Test] 识别失败");
        }

        cv::imshow("Buff Recognizer Test", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "buff_recognizer_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 读取参数
    std::string model_path, enemy_color;
    pnh.param<std::string>("buff_model_path", model_path, "Model/fu_moxi/buff_repvgg.xml");
    pnh.param<std::string>("enemy_color", enemy_color, "blue");

    ROS_INFO("[Test] 加载模型: %s, 敌方: %s", model_path.c_str(), enemy_color.c_str());
    g_recognizer = new BuffRecognizer(model_path, enemy_color);

    ros::Subscriber sub = nh.subscribe("/hikrobot_camera/rgb", 1, onImage);
    ROS_INFO("[Test] 已订阅 /hikrobot_camera/rgb, 等待图像...");

    ros::spin();
    delete g_recognizer;
    return 0;
}
