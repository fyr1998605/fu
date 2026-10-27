//
// Created by password-is-robomaster on 2023/5/8.
//
#ifndef _BUNDLE_ADJUSTMENT_H
#define _BUNDLE_ADJUSTMENT_H

#include<ceres/ceres.h>
#include<ceres/rotation.h>
#include<opencv2/opencv.hpp>
using namespace std;

// 束调整(Bundle Adjustment)
// 计算机视觉中的光束法平差实现,用于优化相机位姿

// 重投影误差计算器
class ReprojectionError
{
public:
    ReprojectionError(Eigen::Vector3d point_, Eigen::Vector2d observed_)
            : point(point_), observed(observed_)
    {
    }

    template<typename T>
    bool operator()(const T* const camera_r, const T* const camera_t, T* residuals) const
    {
        T pt1[3];  //3D点坐标系
        pt1[0] = T(point.x());
        pt1[1] = T(point.y());
        pt1[2] = T(point.z());


        T pt2[3];//对3D点进行坐标转换，换算到相机坐标系下
        ceres::AngleAxisRotatePoint(camera_r, pt1, pt2);

        pt2[0] = pt2[0] + camera_t[0];
        pt2[1] = pt2[1] + camera_t[1];
        pt2[2] = pt2[2] + camera_t[2];
        //归一化并将其转换到图像坐标系下
        const T xp = T(K[0] * (pt2[0] / pt2[2]) + K[2]);
        const T yp = T(K[1] * (pt2[1] / pt2[2]) + K[3]);
        const T u = T(observed.x());
        const T v = T(observed.y());
        //投影点与图像点的误差
        residuals[0] = u - xp;
        residuals[1] = v - yp;
        return true;
    }
    static ceres::CostFunction* Create(Eigen::Vector3d points, Eigen::Vector2d observed)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3>(
                new ReprojectionError(points, observed)));
    }

private:
    Eigen::Vector3d point;
    Eigen::Vector2d observed;
    // Camera intrinsics
    double K[4] = {1679.3126, 1665.10676,  687.532088,  538.618679}; // fx,fy,cx,cy
};

// 束调整优化器
class bundle_adjustor
{
public:
    static void PoseOptimization(const std::vector<cv::Point3f>& points_3f,
                          const std::vector<cv::Point2f>& points_2f,
                          cv::Mat &rvec, cv::Mat &t)
    {
        vector<cv::Point3d>points_3d;
        for(auto t : points_3f) points_3d.emplace_back(t.x, t.y, t.z);
        vector<cv::Point2d>points_2d;
        for(auto t : points_2f)points_2d.emplace_back(t.x, t.y);

        // Attention: cv::Mat::type
        assert(rvec.type() == CV_64F);
        assert(t.type() == CV_64F);

        double camera_rvec[3];
        camera_rvec[0] = rvec.at<double>(0,0);
        camera_rvec[1] = rvec.at<double>(1,0);
        camera_rvec[2] = rvec.at<double>(2,0);

        double camera_t[3];
        camera_t[0] = t.at<double>(0,0);
        camera_t[1] = t.at<double>(1,0);
        camera_t[2] = t.at<double>(2,0);

        ceres::Problem problem;
        ceres::LossFunction* lossfunction = NULL;
        for(uint i = 0; i < points_3d.size(); i++)//将3D点和对应的图像点加入到代码函数中
        {
            Eigen::Vector3d p3d(points_3d[i].x, points_3d[i].y, points_3d[i].z);
            Eigen::Vector2d p2d(points_2d[i].x, points_2d[i].y);

            ceres::CostFunction* costfunction = ReprojectionError::Create(p3d, p2d);
            problem.AddResidualBlock(costfunction, lossfunction, camera_rvec, camera_t);//这一步也是我们照葫芦画瓢的关键，输入正确的参数
        }

        //设置优化器参数及优化方法
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 100;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = false;
        options.logging_type = ceres::SILENT; // 禁止输出日志

        //调用求解器进行优化
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        //std::cout << summary.BriefReport() << std::endl;

        //std::cout << "After Optimizing: " << std::endl;
        double quat[4];
        ceres::AngleAxisToQuaternion(camera_rvec, quat);
        Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
        //对优化后的rt进行赋值还原
        rvec.at<double>(0,0) = camera_rvec[0];
        rvec.at<double>(1,0) = camera_rvec[1];
        rvec.at<double>(2,0) = camera_rvec[2];
        t.at<double>(0,0) = camera_t[0];
        t.at<double>(1,0) = camera_t[1];
        t.at<double>(2,0) = camera_t[2];
    }
};
#endif //TEMP_BUNDLE_ADJUSTMENT_H
