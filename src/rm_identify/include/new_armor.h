#ifndef NEW_ARMOR_H
#define NEW_ARMOR_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace new_Identify
{
    // 灯条参数结构体
    struct LightParams
    {
        double min_ratio; // 宽高比最小值
        double max_ratio; // 宽高比最大值
        double max_angle; // 最大垂直角度
    };

    // 装甲板参数结构体
    struct ArmorParams
    {
        double min_light_ratio;           // 最小灯条比例
        double min_small_center_distance; // 小装甲板中心最小距离
        double max_small_center_distance; // 小装甲板中心最大距离
        double min_large_center_distance; // 大装甲板中心最小距离
        double max_large_center_distance; // 大装甲板中心最大距离
        double max_angle;                 // 最大水平角度
    };

    // 配置结构体
    struct Config
    {
        const LightParams &l; // 灯条参数常量引用
        const ArmorParams &a; // 装甲板参数常量引用
        int binary_thres;     // 二值化阈值
    };

    // 图像缩放结构体
    struct Resize
    {
        cv::Mat resized_image; // 缩放后的图像
        int dw;                // 宽度偏移量
        int dh;                // 高度偏移量
    };

    // 灯条类，继承自OpenCV的RotatedRect
    struct lLight : public cv::RotatedRect
    {
        lLight() = default; // 默认构造函数

        // 从轮廓构造灯条的显式构造函数
        explicit lLight(const std::vector<cv::Point> &contour)
            : cv::RotatedRect(cv::minAreaRect(contour))
        {
            // 计算轮廓中心点
            center = std::accumulate(
                contour.begin(),
                contour.end(),
                cv::Point2f(0, 0),
                [n = static_cast<float>(contour.size())](const cv::Point2f &a, const cv::Point &b)
                {
                    return a + cv::Point2f(b.x, b.y) / n;
                });

            cv::Point2f p[4];
            this->points(p); // 获取旋转矩形的四个顶点
            // 按y坐标排序顶点
            std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b)
                      { return a.y < b.y; });
            top = (p[0] + p[1]) / 2;    // 计算顶部中点
            bottom = (p[2] + p[3]) / 2; // 计算底部中点

            length = cv::norm(top - bottom); // 计算灯条长度
            width = cv::norm(p[0] - p[1]);   // 计算灯条宽度

            // 计算倾斜角度（与水平线的夹角）
            tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
            tilt_angle = tilt_angle / CV_PI * 180; // 转换为角度制
        }

        int class_id;       // 类别ID
        cv::Point2f top;    // 顶部中点
        cv::Point2f bottom; // 底部中点
        cv::Point2f center; // 中心点
        double length;      // 灯条长度
        double width;       // 灯条宽度
        float tilt_angle;   // 倾斜角度
    };

    // 装甲板结构体
    struct aArmor
    {
        aArmor() = default; // 默认构造函数

        // 从两个灯条构造装甲板的显式构造函数
        explicit aArmor(const lLight &l1, const lLight &l2)
        {
            // 按x坐标排序左右灯条
            if (l1.center.x < l2.center.x)
            {
                left_light = l1, right_light = l2;
            }
            else
            {
                left_light = l2, right_light = l1;
            }
            center = (left_light.center + right_light.center) / 2; // 计算装甲板中心
        }

        lLight left_light;  // 左侧灯条
        lLight right_light; // 右侧灯条
        cv::Point2f center; // 装甲板中心
        char type;          // 装甲板类型
        float angle;        // 角度

        cv::Mat number_img;               // 数字图像区域
        int number;                       // 数字编号
        float confidence;                 // 置信度
        std::string classfication_result; // 分类结果
    };

    // 类别名称向量常量
    const std::vector<int> class_names_ = {
        1, // 类别1
        2, // 类别2
        3, // 类别3
        4, // 类别4
        5, // 类别5
        6, // 前哨站
        7, // 哨兵
        8, // 基地
        9  // 背景（负样本）
    };

    // 类别名称向量
    const std::vector<std::string> coconame = {
        "red", // 红色
        "blue" // 蓝色
    };
}

#endif