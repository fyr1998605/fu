#include "trajectory_visualizer.hpp"
#include <chrono>

namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t , const double k) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t,k);
        // Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);
        Eigen::Vector3d xyz_c = bullet.pos;
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->cam2pixel(bullet.pos);
        // cout << "edge_xy_u x: " << edge_xy_u.x << "edge_xy_u y" << edge_xy_u.y << endl;
        // cout << "center_xy_u x: " << center_xy_u.x << "center_xy_u y" << center_xy_u.y << endl;
        // ROS_WARN("edge_xy_u x%.3f edge_xy_u y%.3f",edge_xy_u.x,edge_xy_u.y);
        // ROS_WARN("center_xy_u x%.3f center_xy_u y%.3f",center_xy_u.x,center_xy_u.y);
        double radius = math::get_dis(edge_xy_u, center_xy_u);
        // ROS_WARN("打印小弹丸的半径: %f", radius);
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
        // return HitCircle { bullet.hit, math::CircleF(center_xy_u, radius) };
    }

    auto ProjectileSimulator::get_pos_by_t(const double& t, const double k) const -> HitPos {
        // 将毫秒转换成秒
        double t_s = static_cast<double>(t * 0.001);
        double fire_t_s = static_cast<double>(this->fire_t * 0.001);

        // ROS_WARN("t_s: %f", t_s);
        // ROS_WARN("fire_t_s: %f", fire_t_s);

        // 计算水平位移
        double w = (t_s - fire_t_s) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;

        // cout << "w: " << w << endl;
        // cout << "h: " << h << endl;
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        // cout << "bullet_xyz_i_camera: " << "x: " << bullet_xyz_i_camera.x() << "y: " << bullet_xyz_i_camera.y() << "z: " << bullet_xyz_i_camera.z() << endl;
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        if (bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm()) {
            // ROS_WARN("子弹已经命中可以删除");
        }
        else {
            // ROS_WARN("子弹还没有命中");
        }
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }

    auto ProjectileSimulator::get_fire_t() const -> long long {
        return this->fire_t;
    }

    // AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
    //     this->shoot_param = shoot_param;
    // }

    AimCorrector::AimCorrector() {}
    
    void AimCorrector::update_aim(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
        this->coorConverter = coorConverter;
    }


    auto AimCorrector::get_circles(long long now_time, const double k) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        // cout << "bullets: " << bullets.size() << endl;
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            // debug
            // ROS_WARN("now_time: %lld, fire_t: %lld, 差值: %lldms", 
            //      now_time, it->proj.get_fire_t(), it->proj.get_fire_t() - now_time);
            if (now_time < it->proj.get_fire_t()) {
                ++it; 
                continue;
            }
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time, k);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }

    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN, long long delay_time) -> void {
        const long long fire_interval = 20;  // 发射间隔：20毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 25 + COMMAND_TIMESPAN + delay_time)
                });
                this->last_fire_time = current_time;
            }
        }
    }

    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }

    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }

    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }

    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }

    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }

    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }

    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }

    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }

    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }

    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }

    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }

    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }

    // // 绘制模拟发射的子弹
    // void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
    //     FlaskStream flask_aim;
    //     FlaskStream flask_map;
    //     flask_aim.clear();
    //     flask_map.clear();

    //     AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);

    //     // 更新子弹序列
    //     // 传入当前帧的时间和当前帧的瞄准姿态
    //     aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);

    //     std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
    //     for (auto& bullet: bullets) {
    //         flask_aim << FlaskPoint(
    //             bullet.circle.center,
    //             { 0, 0, 255 },
    //             bullet.circle.r,
    //             2
    //         );
    //         flask_aim << FlaskText(
    //             std::to_string(bullet.id),
    //             { bullet.circle.center.x + 20.f, bullet.circle.center.y },
    //             { 0, 0, 255 },
    //             0.8
    //         );
    //         // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
    //     }

    //     flask_aim >> img;
    // }
}
