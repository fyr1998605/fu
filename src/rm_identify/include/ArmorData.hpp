#ifndef ARMOR_DATA_HPP
#define ARMOR_DATA_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class ArmorData {
public:
    cv::Point2f pus[4];
    Eigen::Vector3d center;   // 装甲板中心
    char armor_type;
    char number;
};
#endif