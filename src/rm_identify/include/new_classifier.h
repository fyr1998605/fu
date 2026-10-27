#include <openvino/openvino.hpp>
#include <iostream>
#include <string>
#include <time.h>
#include <fstream>
#include <vector>
#include <random>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include "Image.h"
#include "new_assert.h"
#include "Light.h"
#include "new_armor.h"

using namespace cv;
using namespace std;
using namespace dnn;

// 常量定义
extern const int light_length;
extern const int warp_height;
extern const int small_armor_width;
extern const int large_armor_width;
extern std::vector<int> class_names_;

namespace new_Identify
{

    // 检测器类定义
    class detector
    {
    public:
        cv::dnn::Net net_; // OpenCV DNN网络对象

        // 构造函数
        detector(Config config);
        // 析构函数
        ~detector();
        // 检测函数，输入帧图像，返回装甲板向量
        std::vector<aArmor> detect(cv::Mat &frame);
        // 检查是否初始化完成
        bool Initialized();
        // 分类函数（已注释掉的原函数）
        // cv::Mat pre(cv::Mat Aromorimage);
        // 对图像进行分类
        int classify(cv::Mat &image, aArmor &armor)
        {
            // 归一化图像
            image = image / 255.0;
            cv::Mat blob;
            // 创建网络输入blob
            cv::dnn::blobFromImage(image, blob);
            net_.setInput(blob);
            // 前向传播
            cv::Mat outputs = net_.forward();
            // 找到最大概率值
            float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
            // 计算softmax概率
            cv::Mat softmax_prob;
            cv::exp(outputs - max_prob, softmax_prob);
            float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
            softmax_prob /= sum;
            // 获取最高置信度和类别ID
            double confidence;
            cv::Point class_id_point;
            minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
            int label_id = class_id_point.x;
            armor.confidence = confidence; // 设置装甲板置信度

            // 格式化分类结果字符串
            std::stringstream result_ss;
            result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
                      << armor.confidence * 100.0 << "%";
            armor.classfication_result = result_ss.str();

            // 特殊处理类别5
            if (class_names_[label_id] == 5)
            {
                return 9; // 返回9表示特殊类别
            }

            // 返回分类结果
            return class_names_[label_id];
        }

        // 直方图均衡化处理
        cv::Mat OnEqualizeHist(cv::Mat num_image)
        {
            cv::Mat result_num;
            // 分离通道
            std::vector<Mat> splitBGR(num_image.channels());
            split(num_image, splitBGR);

            // 对每个通道进行直方图均衡化
            for (int i = 0; i < num_image.channels(); i++)
            {
                cv::equalizeHist(splitBGR[i], splitBGR[i]);
            }

            // 合并通道
            merge(splitBGR, result_num);

            return result_num;
        }

        // 从图像中提取数字区域
        cv::Mat extractNumbers(cv::Mat &src, cv::Point2f lights_vertices[], bool isSmall)
        {
            // 数字ROI尺寸
            //const cv::Size roi_size(28, 28);//这是LeNet模型
            const cv::Size roi_size(20, 28);//这是mlp模型

            // 计算灯条在变换后图像中的位置
            int top_light_y = (warp_height - light_length) / 2 - 1;
            int bottom_light_y = top_light_y + light_length;
            int warp_width;

            // 根据装甲板类型设置宽度
            if (isSmall)
            {
                warp_width = small_armor_width;
            }
            else
            {
                warp_width = large_armor_width;
            }

            // 目标顶点坐标
            cv::Point2f target_vertices[4] = {
                cv::Point(0, bottom_light_y),
                cv::Point(0, top_light_y),
                cv::Point(warp_width - 1, top_light_y),
                cv::Point(warp_width - 1, bottom_light_y),
            };

            cv::Mat number_image;
            // 计算透视变换矩阵
            auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
            // 应用透视变换
            cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
            // 获取ROI区域
            number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

            // 二值化处理
            cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
            cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            return number_image;
        }

        cv::Mat binary_img; // 二值化图像

        // 检测矩形区域（已注释掉的原函数）
        // cv::Rect detect_rect(aArmor & armor);
        // 检测装甲板（已注释掉的原函数）
        // vector<armor> detect(const cv::Mat & input);
        // 忽略的类别向量
        const std::vector<int> &ignore_classes_ = {};
        int detect_color;                          // 检测颜色
        std::vector<new_Identify::lLight> lights_; // 灯条向量
        std::vector<new_Identify::aArmor> armors_; // 装甲板向量

    private:
        bool initialized = false; // 初始化标志

        std::string model_path; // 模型路径

        struct LightParams l; // 灯条参数
        struct ArmorParams a; // 装甲板参数

        int binary_thres;      // 二值化阈值
        std::string onnx_path; // ONNX模型路径
        Resize resize;         // 图像缩放信息

        // 图像预处理
        cv::Mat preprocess_Image(cv::Mat img);

        // 判断两个灯条是否构成装甲板
        char isArmor(const lLight &light_1, const lLight &light_2);
        // 判断是否为有效灯条
        bool isLight(const lLight &possible_light);
        // 检查灯条是否包含在其他灯条中
        bool containLight(
            const lLight &light_1, const lLight &light_2, const std::vector<lLight> &lights);
        // 在图像中查找灯条
        std::vector<lLight> findlights(const cv::Mat &img, const cv::Mat &binary_img);
        // 匹配灯条形成装甲板
        std::vector<aArmor> matchLights(const cv::Mat &img, const std::vector<lLight> &lights);

        int kFactor; // 缩放因子
    };

}