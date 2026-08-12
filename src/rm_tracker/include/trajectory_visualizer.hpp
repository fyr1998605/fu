#ifndef TRAJECTORY_VISUALIZER_HPP
#define TRAJECTORY_VISUALIZER_HPP
#include "math.hpp"
#include "CoorConverter.hpp"
#include <opencv2/opencv.hpp>
#include "GimbalPos.hpp"
#include "ros/ros.h"

namespace tools{

    const int FLASK_MAP_WIDTH = 1000;                   // 定义调试地图的水平分辨率
    const double FLASK_MAP_PETER_BY_BRIGHT = 1.;        // 默认亮度系数
    const int FLASK_MAP_MID_X = FLASK_MAP_WIDTH / 2;    // 地图的水平中心点,用于坐标变换的参考原点

    // 点绘制参数
    struct FlaskPoint {
        FlaskPoint(
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const int& radius,
            const int& thickness
        ):
            pt(pt),
            color(color),
            radius(radius),
            thickness(thickness) {}
        cv::Point2d pt;   // 圆心位置
        cv::Scalar color; // 颜色
        int radius;       // 半径
        int thickness;    // 线宽
    };

    struct FlaskLine {
        FlaskLine(
            const std::pair<cv::Point2f, cv::Point2f>& pt_pair,
            const cv::Scalar& color,
            const int& thickness
        ):
            pt_pair(pt_pair),
            color(color),
            thickness(thickness) {}
        std::pair<cv::Point2f, cv::Point2f> pt_pair;
        cv::Scalar color;
        int thickness;
    };

    // 文本绘制参数
    struct FlaskText {
        FlaskText(
            const std::string& str,
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const double& scale
        ):
            str(str),
            pt(pt),
            color(color),
            scale(scale) {}
        std::string str;  // 文本内容
        cv::Point2d pt;   // 文本位置 (左下角)
        cv::Scalar color; // 颜色
        double scale;     // 字体大小
    };

    /*
        绘制流管理器
        @brief: 
            收集绘制命令: 通过重载的<<操作符接收各种绘制元素
            批量执行绘制: 通过>>操作符将所有收集的命令绘制到图形上
            命令管理: 可以清空所有收集的绘制命令
    */
    class FlaskStream {
    public:
        FlaskStream& operator<<(const char* str);
        FlaskStream& operator<<(const std::string& str);
        FlaskStream& operator<<(const FlaskPoint& pt);
        FlaskStream& operator<<(const FlaskLine& line);
        FlaskStream& operator<<(const std::vector<FlaskLine>& lines);
        FlaskStream& operator<<(const FlaskText& text);
        FlaskStream& operator>>(cv::Mat& img);
        void clear();
    private:
        std::vector<std::string> logs;
        std::vector<FlaskPoint> pts;
        std::vector<FlaskLine> lines;
        std::vector<FlaskText> texts;
    };


    // 用于复现的瞄准参数
    // 移植代码的时候将这段代码移植到自瞄那里
    struct ShootParam {
        double v0 = 0.;                                                 // 子弹初速度
        double aim_angle = 0.;                                          // 发射仰角
        // Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();     // 枪管坐标系瞄准点 (没有什么作用)
        Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();  // 相机坐标系目标点
    };

    // 子弹命中位置信息
    struct HitPos {
        bool hit;
        Eigen::Vector3d pos;     // 子弹在世界坐标系上的位置            
    };

    // 子弹图像投影信息
    struct HitCircle {
        bool hit;
        math::CircleF circle;    // 子弹在图像上的投影圆
    };

    // 匹配代价评估
    struct CaughtCost {
        bool caught;    // 是否满足匹配条件
        double cost;    // 匹配代价(越小越好)
    };

    // 子弹弹道物理模拟器
    class ProjectileSimulator {
    public:
        ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t) : coorConverter{coorConverter},shoot_param{shoot_param} ,fire_t{fire_t} {
            // ROS_WARN("构造函数: %lld",fire_t);
        }
        // 子弹在图像平面上的投影计算
        auto get_circle_by_t(const double& t,const double k) const -> HitCircle;
        
        // 计算在指定时间t的子弹位置
        auto get_pos_by_t(const double& t,const double k) const -> HitPos;

        // 获取开火时间
        auto get_fire_t() const -> long long;
    private:
        const double g { 9.8 };
        const long long fire_t;
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;
    };

    // 子弹位置信息
    struct IdPos {
        int id;             
        Eigen::Vector3d pos;
    };

    // 子弹投影圆信息
    struct IdCircle {   
        int id;               
        math::CircleF circle; // 子弹在图像平面上的投影圆
    };

    // 子弹模拟器封装
    struct IdProj {
        int id;                    
        ProjectileSimulator proj;  // 子弹物理模拟器实例
    };
    
    // 自动瞄准误差校准(目前仅用来复现理想弹道)
    class AimCorrector {
    public:
        AimCorrector();
        
        // 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆
        auto get_circles(long long now_time, const double k) -> std::vector<IdCircle>;

        void update_aim(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);

        auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN, long long delay_time) -> void;
    private:
        std::list<IdProj> bullets;             // 活跃子弹容器模拟器          
        CoordinateTransformer* coorConverter;  // 坐标变换器
        std::string config_path_;              // 存储配置路径
        ShootParam shoot_param;      
        long long next_id = 0;           
        long long last_fire_time = 0;
    };



    cv::Scalar heightened_color(const cv::Scalar& color, const double& z);

    FlaskPoint pos_to_map_point(
        const Eigen::Vector3d& pos,
        const cv::Scalar& color,
        const int& radius,
        const int& thickness
    );

    // 绘制模拟发射的子弹
    // void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN);
}

#endif // TRAJECTORY_VISUALIZER_HPP