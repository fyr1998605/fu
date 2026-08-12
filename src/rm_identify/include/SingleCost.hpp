#ifndef SINGLE_COST_HPP
#define SINGLE_COST_HPP

#include <iostream>
#include "ArmorData.hpp"
#include "CoorConverter.hpp"
#include "math.hpp"
#include <Eigen/Dense>
using namespace std;

const double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.;

// 将3D装甲板角点投影到2D图像像素坐标的辅助函数
std::vector<cv::Point2d> radial_armor_pts(
    const Eigen::Vector3d& pos,           // 装甲板在世界系中的位置
    char number,                            // 装甲板类型
    const double& pitch,                  // 装甲板的俯仰角度
    const double& z_to_v,                // 高度与垂直方向的比例参数
    CoordinateTransformer* const converter // 坐标转换器
) {
    // 获取装甲板三维角点
    std::vector<Eigen::Vector3d> corners =
        converter->armor2Corner(pos, number, pitch, z_to_v);
    // 循环处理4个角点
    std::vector<cv::Point2d> pts;
    for (int i = 0; i < 4; ++i) {
        pts.push_back(converter->map2pixel(corners[i]));
    }
    return pts;
}

// 计算两组点之间代价的函数, 主要用于评估预测的装甲板角点与实际检测到的角点之间的差异
double get_pts_cost(const std::vector<cv::Point2d>& cv_refs,const std::vector<cv::Point2d>& cv_pts,const double& inclined) {
    std::size_t size = cv_refs.size();
    std::vector<Eigen::Vector2d> refs;
    std::vector<Eigen::Vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i) {
        refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
        pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i) {
        std::size_t p = (i + 1u) % size;
        // i - p 构成线段。过程：先移动起点，再补长度，再旋转
        Eigen::Vector2d ref_d = refs[p] - refs[i]; // 标准
        Eigen::Vector2d pt_d = pts[p] - pts[i];
        // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
        double pixel_dis = // dis 是指方差平面内到原点的距离
            (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm())
             + std::fabs(ref_d.norm() - pt_d.norm()))
            / ref_d.norm();
        double angular_dis = ref_d.norm() * math::get_abs_angle(ref_d, pt_d) / ref_d.norm();
        // 平方可能是为了配合 sin 和 cos
        // 弧度差代价（0 度左右占比应该大）
        double cost_i = math::sq(pixel_dis * std::sin(inclined))
            + math::sq(angular_dis * std::cos(inclined)) * DETECTOR_ERROR_PIXEL_BY_SLOPE;
        // 重投影像素误差越大，越相信斜率
        cost += std::sqrt(cost_i);
    }
    return cost;
}

// 单板拟合数据
struct SingleData
{
    SingleData(const ArmorData &armordata, const double &z_to_v_exp,CoordinateTransformer* coorConverter) : armordata(armordata), z_to_v_exp(z_to_v_exp), coorConverter(coorConverter) {}
    const ArmorData armordata;  // 装甲板数据
    const double z_to_v_exp; // 期望角度
    CoordinateTransformer* const coorConverter;
};

// 单板拟合代价类
class SingleCost
{
public:
    explicit SingleCost(const SingleData &data) : data(data) {}
    double operator()(const double &yaw)
    {
        double pitch_temp = 0;
        if (this->data.armordata.number == 6) {
            pitch_temp = -0.2618;
        }
        else {
            pitch_temp = 0.2618;
        }

        std::vector<cv::Point2d> pts = radial_armor_pts(
            this->data.armordata.center,
            this->data.armordata.number,
            pitch_temp,
            yaw,
            this->data.coorConverter
            );

        for (int i = 0; i < 4; i++) {
            // cout << "实际图像的: " << "x: " << pts[i].x << "y: " << pts[i].y << endl;
            // cout << "角点变换的: " << "x: " << this->data.armordata.pus[i].x << "y: " << this->data.armordata.pus[i].y << endl;
        }

        return get_pts_cost(
            pts,
            std::vector<cv::Point2d>{
                this->data.armordata.pus[0],
                this->data.armordata.pus[1],
                this->data.armordata.pus[2],
                this->data.armordata.pus[3],
            },
            this->data.z_to_v_exp);
    }

private:
    const SingleData data;
};
#endif

