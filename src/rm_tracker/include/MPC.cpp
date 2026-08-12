#include "MPC.hpp"

// 半成品,未落地,代替双环pid的位置环,提供更丝滑的响应
MPC::MPC(const std::string &file_path)
{
    setParam(file_path);
    setMPCMatrix();
}
MPC::~MPC()
{}

void MPC::setParam(const std::string &file_path)
{
    try {
        // 1. 文件存在性检查
        if (file_path.empty()) {
            ROS_ERROR("Empty config file path provided");
            return;
        }
        
        //2. 检查文件是否打开
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
        
        //加载MPC参数
        if (config["MPC"]){
            YAML::Node mpc = config["MPC"];
            m_debug = mpc["debug"].as<double>(0);
            m_step = mpc["step"].as<int>(5);
            // ROS_WARN("m_debug :%d", m_debug);
            // ROS_WARN("m_step :%d",  m_step);

            for (int i = 0; i < _D_STATE_; i++) {
                m_process_error_weight[i] = mpc["process_error_weight"]["Q" + std::to_string(i)].as<double>(0);
                m_target_error_weight[i] = mpc["target_error_weight"]["F" + std::to_string(i)].as<double>(0);
                // ROS_WARN("m_process_error_weight %d:%.3f",i,m_process_error_weight[i]);
                // ROS_WARN("m_target_error_weight %d:%.3f",i,m_target_error_weight[i]);
            }
            for (int i = 0; i < _D_CTRL_; i++) {
                m_process_control_weight[i] = mpc["process_control_weight"]["R" + std::to_string(i)].as<double>(0);
                // ROS_WARN("m_process_control_weight %d:%.3f",i,m_process_control_weight[i]);
            }
        }
        else {
            ROS_WARN("No 'MPC' section in config file");
        }

        // 加载dt
        if (config["ModelObserver"]) {
            YAML::Node model = config["ModelObserver"];
            m_dt = model["dt"].as<double>(0);
            // ROS_WARN("m_dt:%.3f",m_dt);
        }
        else {
            ROS_WARN("No 'dt' section in config file");
        }

        // 加载Tracker参数
        if (config["Tracker"]) {
            YAML::Node tracker = config["Tracker"];
            m_track_center = tracker["track_center"].as<bool>(0);
            m_switch_threshold = tracker["switch_threshold"].as<double>(0);
            m_aim_center_palstance_threshold = tracker["aim_center_palstance_threshold"].as<double>(0);
            m_switch_threshold = tracker["switch_threshold"].as<double>(0);
            m_fix_on = tracker["fix_on"].as<double>(1);
            COMMAND_TIMESPAN = tracker["COMMAND_TIMESPAN"].as<double>(0.2);
            eTime = tracker["eTime"].as<double>(0.002);
            pitch_compensation = tracker["pitch_compensation"].as<double>(0);
            yaw_compensation = tracker["yaw_compensation"].as<double>(0);
            // ROS_WARN("m_track_center:%d",m_track_center);
            // ROS_WARN("m_switch_threshold:%.3f",m_switch_threshold);
            // ROS_WARN("m_aim_center_palstance_threshold:%.3f",m_aim_center_palstance_threshold);
            // ROS_WARN("m_switch_threshold:%.3f",m_switch_threshold);
            // ROS_WARN("COMMAND_TIMESPAN: %.3f", COMMAND_TIMESPAN);
            // ROS_WARN("eTime: %.3f", eTime);
            // ROS_INFO("pitch_compensation:%.3f",pitch_compensation);
            // ROS_INFO("yaw_compensation:%.3f",yaw_compensation);
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

void MPC::setMPCMatrix()
{
    // 初始化系统参数
    m_A.resize(_D_STATE_, _D_STATE_);
    m_B.resize(_D_STATE_, _D_CTRL_);
    m_Q.resize(_D_STATE_, _D_STATE_);
    m_R.resize(_D_CTRL_, _D_CTRL_);
    m_F.resize(_D_STATE_, _D_STATE_);

#if _D_STATE_ == 2
    m_A << 1, 0,
            0, 1;
    m_B << m_dt, 0,
            0,    m_dt;
#elif _D_STATE_ == 4
    m_A << 1, 0, m_dt, 0,
            0, 1, 0,    m_dt,
            0, 0, 0,    0,
            0, 0, 0,    0;
    m_B << 0, 0,
            0, 0,
            1, 0,
            0, 1;
#endif
    m_Q = m_process_error_weight.asDiagonal();
    m_R = m_process_control_weight.asDiagonal();
    m_F = m_target_error_weight.asDiagonal();

    if (m_debug)
    {
        std::cout << "[MPC] A: " << std::endl << m_A << std::endl;
        std::cout << "[MPC] B: " << std::endl << m_B << std::endl;
        std::cout << "[MPC] Q: " << std::endl << m_Q << std::endl;
        std::cout << "[MPC] R: " << std::endl << m_R << std::endl;
        std::cout << "[MPC] F: " << std::endl << m_F << std::endl;
    }

    // 初始化中间量
    Eigen::MatrixXd M;
    Eigen::MatrixXd C;
    M.resize((m_step + 1) * _D_STATE_, _D_STATE_);
    C.resize((m_step + 1) * _D_STATE_, m_step * _D_STATE_);
    M.setZero();
    C.setZero();

    Eigen::MatrixXd tmp = Eigen::Matrix<double, _D_STATE_, _D_STATE_>::Identity();
    M.block<_D_STATE_, _D_STATE_>(0, 0) = Eigen::Matrix<double, _D_STATE_, _D_STATE_>::Identity();
    for (int i = 1; i <= m_step; i++)
    {
        C.block<_D_STATE_, _D_CTRL_>(i * _D_STATE_, 0) = tmp * m_B;
        for (int j = 1; j < m_step; j++)
        {
            C.block<_D_STATE_, _D_CTRL_>(i * _D_STATE_, j * _D_CTRL_) = C.block<_D_STATE_, _D_CTRL_>((i - 1) * _D_STATE_, (j - 1) * _D_CTRL_);
        }
        tmp *= m_A;
        M.block<_D_STATE_, _D_STATE_>(i * _D_STATE_, 0) = tmp;
    }

    if (m_debug)
    {
        std::cout << "[MPC] M: " << std::endl << M << std::endl;
        std::cout << "[MPC] C: " << std::endl << C << std::endl;
    }
    
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Q_.resize((m_step + 1) *_D_STATE_, (m_step + 1) * _D_STATE_);
    R_.resize(m_step *_D_CTRL_, m_step * _D_CTRL_);
    Q_.setZero();
    R_.setZero();

    for (int i = 0; i < m_step; i++)
    {
        Q_.block<_D_STATE_, _D_STATE_>(i * _D_STATE_, i * _D_STATE_) = m_Q;
        R_.block<_D_CTRL_, _D_CTRL_>(i * _D_CTRL_, i * _D_CTRL_) = m_R;
    }
    Q_.block<_D_STATE_, _D_STATE_>(m_step * _D_STATE_, m_step * _D_STATE_) = m_F;

    if (m_debug)
    {
        std::cout << "[MPC] Q_: " << std::endl << Q_ << std::endl;
        std::cout << "[MPC] R_: " << std::endl << R_ << std::endl;
    }

    // 计算损失函数系数
    m_H = C.transpose() * Q_ * C + R_;
    m_E = C.transpose() * Q_ * M;
    m_Ed = C.transpose() * Q_;

    if (m_debug)
    {
        std::cout << "[MPC] H: " << std::endl << m_H << std::endl;
        std::cout << "[MPC] E: " << std::endl << m_E << std::endl;
        std::cout << "[MPC] Ed: " << std::endl << m_Ed << std::endl;
    }
}

GimbalPose MPC::getGimbalSpeed(std::shared_ptr<KinematicModel> status, const GimbalPose cur_pose, double bullet_speed)
{
    m_bullet_speed = bullet_speed;
    m_cur_pose = cur_pose;

    // 根据运动模型预测控制目标
    Eigen::MatrixXd predict_ctrl_target = getPredictControlTarget(status);
    // predict_ctrl_target.conservativeResize(predict_ctrl_target.rows() * predict_ctrl_target.cols(), 1);
    predict_ctrl_target.resize(predict_ctrl_target.rows() * predict_ctrl_target.cols(), 1);
    // std::cout << "[MPC] Predict_Ctrl_Target: " << std::endl << predict_ctrl_target << std::endl;

    // 求解最优控制
    Eigen::Matrix<double, 2, 1> cur_state;
    cur_state << m_cur_pose.pitch, m_cur_pose.yaw;
    Eigen::MatrixXd U = solveQuadProg(m_H, m_E * cur_state - m_Ed * predict_ctrl_target);
    GimbalPose target_ctrl(U(0, 0), U(1, 0));

    return target_ctrl;
}

void MPC::reset()
{
    m_center_tracked = false;
}

Eigen::Matrix<double, _D_STATE_, Eigen::Dynamic> MPC::getPredictControlTarget(std::shared_ptr<KinematicModel> status)
{
    // 控制目标
    Eigen::MatrixXd predict_ctrl_target;
    predict_ctrl_target.resize(_D_STATE_, m_step + 1);
    predict_ctrl_target.setZero();

    // 判断是否锁中心
    if (m_track_center && ((abs(status->palstance) - m_switch_trackmode_threshold) > m_aim_center_palstance_threshold))
    {
        m_center_tracked = true;
    }
    else
    {
        m_center_tracked = false;
    }

    for (int i = 0; i <= m_step; i++)
    {
        GimbalPose pose = getTargetPose(status, m_dt * i);
        predict_ctrl_target(0, i) = pose.pitch;
        predict_ctrl_target(1, i) = pose.yaw;
    }
    // 若状态量包含速度
    if (_D_STATE_ == 4)
    {
        for (int i = 0; i < m_step; i++)
        {
            predict_ctrl_target(2, i) = (predict_ctrl_target(0, i + 1) - predict_ctrl_target(0, i)) / m_dt;
            predict_ctrl_target(3, i) = (predict_ctrl_target(1, i + 1) - predict_ctrl_target(1, i)) / m_dt;
        }
        GimbalPose pose = getTargetPose(status, m_dt * (m_step + 1));
        predict_ctrl_target(2, m_step) = (pose.pitch - predict_ctrl_target(0, m_step)) / m_dt;
        predict_ctrl_target(3, m_step) = (pose.yaw - predict_ctrl_target(1, m_step)) / m_dt;
    }
    if (m_debug)
        std::cout << "[MPC] predict_ctrl_target: " << std::endl << predict_ctrl_target << std::endl;

    return predict_ctrl_target;
}

GimbalPose MPC::getTargetPose(std::shared_ptr<KinematicModel> status, double time_diff)
{
    double m_time_off = COMMAND_TIMESPAN + eTime + 0.025;
    double hit_time = time_diff;
    Armor abs_target_armor;
    // 根据是否锁中心确定目标位姿
    if (m_center_tracked)
    {
        Point3d point_temp1 = Point3d(status->getFacingArmor(hit_time).position.x(),status->getFacingArmor(hit_time).position.y(),status->getFacingArmor(hit_time).position.z());
        hit_time += getDistance(point_temp1) / m_bullet_speed + m_time_off;
        abs_target_armor = status->getFacingArmor(hit_time);
    }
    else
    {
        Point3d point_temp2 = Point3d(status->getFacingArmor(0).position.x(),status->getFacingArmor(0).position.y(),status->getFacingArmor(0).position.z());
        hit_time += getDistance(point_temp2) / m_bullet_speed + m_time_off;
        abs_target_armor = status->getClosestArmor(hit_time, m_switch_threshold);
    }

    // 将向量转换为点
    cv::Point3d temp_point = Point3d(abs_target_armor.position.x(),abs_target_armor.position.y(),abs_target_armor.position.z());
    return getAngle(temp_point, m_cur_pose, m_bullet_speed);
}

Eigen::MatrixXd MPC::solveQuadProg(Eigen::MatrixXd H, Eigen::MatrixXd f)
{
    // 迭代中间量，在循环中可以看作是上一次迭代的结果
    Eigen::MatrixXd x0;
    Eigen::MatrixXd r0;
    Eigen::MatrixXd p0;
    x0 = f;
    x0.setZero();
    r0 = H * x0 + f;
    p0 = r0;

    // 迭代更新量
    Eigen::MatrixXd x;
    Eigen::MatrixXd r;
    Eigen::MatrixXd p;
    Eigen::MatrixXd alpha;
    Eigen::MatrixXd beta;

    for (int i = 0; i < 1000; i++)
    {
        alpha = (r0.transpose() * r0) * (p0.transpose() * H * p0).inverse();
        x = x0 - alpha(0, 0) * p0;
        r = r0 - alpha(0, 0) * H * p0;
        beta = (r.transpose() * r) * (r0.transpose() * r0).inverse();
        p = r + beta(0, 0) * p0;

        if ((x - x0).norm() < 1e-8)
        {
            if (m_debug)
                std::cout << "[MPC] quadratic_result: " << std::endl << x.transpose() << std::endl;

            return x;
        }

        x0 = x;
        r0 = r;
        p0 = p;
    }

    std::cout << "[MPC] Failed to solve the quadratic programming in limited times" << std::endl;
    return x;
}

/**
 *@brief: 根据装甲板坐标、当前云台位姿、射速等信息计算目标位姿
    */
GimbalPose MPC::getAngle(cv::Point3d position, GimbalPose cur_pose, double bullet_speed)
{
    GimbalPose solve_angle;
    cv::Point3d gun_base = position;
    double pitch_in_gun = std::atan(gun_base.z / std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y));
    double yaw_in_gun   = std::atan(gun_base.y / gun_base.x);
    solve_angle.pitch =  pitch_in_gun;
    solve_angle.yaw   =  yaw_in_gun;

    // 重力补偿
    if (m_fix_on) {
        double dis = sqrt(pow(gun_base.x, 2) + pow(gun_base.z, 2) + pow(gun_base.y, 2));
        double angle_fix = 0;
        // debug /////////////////////////////////////////////////////////////
        // cout << "dis: " << dis << endl;
        // cout << "disxy: " << std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y) << endl;
        // cout << "position.z" << position.z << endl;
        // cout << "pitch: " << solve_angle.pitch << endl;
        //////////////////////////////////////////////////////////////////////

        if (abs(9.8 * dis * pow(cos(solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(solve_angle.pitch) <= 1)
        {
            angle_fix = 0.5 * (asin((9.8 * dis * pow(cos(solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(solve_angle.pitch)) + solve_angle.pitch);
            // cout << "重力补偿角: " << angle_fix << endl;
        }
        else
        {
            std::cout << "[AngleSolver] Wrong distance in angle_fix" << std::endl;
        }
        solve_angle.pitch += angle_fix;
    }
    // debug 判断枪口是否能够正确瞄准装甲板
    // //////////////////////////////////////////////////////////////////////
    // std::vector<Eigen::Vector3d> Postest = cal->GetPos("gun","DETECT4",1);
    // cal->TFUpdata("gun","DETECT6_gun", Postest[0], Postest[1], 1);
    
    // 安装误差导致的硬补偿
    solve_angle.pitch += pitch_compensation;
    solve_angle.yaw += yaw_compensation;
    return solve_angle;
}
