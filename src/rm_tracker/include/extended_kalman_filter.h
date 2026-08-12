#include<ros/ros.h>
#include<Eigen/Dense>
#include<geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

namespace Fiter{
    class EKF
    {
    public:
    

        using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
        using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
        using VoidMatFunc = std::function<Eigen::MatrixXd()>;

        EKF(ros::NodeHandle nh);
        Eigen::MatrixXd predict(double);
        Eigen::MatrixXd update(const Eigen::VectorXd & z,double);
        void setState(const Eigen::VectorXd & x0);
        bool isUseUkF();
        void setPri(const Eigen::VectorXd & p0);
        void setParam();
    private:
        // 节点
        ros::NodeHandle nh;
        // Process nonlinear vector function 状态函数
        VecVecFunc f;
        // Observation nonlinear vector function 观测函数
        VecVecFunc h;
        // Jacobian of f() 状态函数的雅各比矩阵
        VecMatFunc jacobian_f; 
        Eigen::MatrixXd F;
        // Jacobian of h() 观测函数的雅各布矩阵
        VecMatFunc jacobian_h;
        Eigen::MatrixXd H;
        // Process noise covariance matrix 过程噪声协方差矩阵
        VoidMatFunc update_Q;
        Eigen::MatrixXd Q;
        // Measurement noise covariance matrix 测量噪声协方差矩阵
        VecMatFunc update_R;
        Eigen::MatrixXd R;
        // Priori error estimate covariance matrix 先验误差协方差矩阵
        Eigen::MatrixXd P_pri;
        // Posteriori error estimate covariance matrix 后验误差协方差矩阵
        Eigen::MatrixXd P_post;
        // Kalman gain 卡尔曼增益
        Eigen::MatrixXd K;
        // System dimensions 
        int n;
        // N-size identity 单位矩阵
        Eigen::MatrixXd I;

        // Priori state 先验估计
        Eigen::VectorXd x_pri;
        // Posteriori state 后验估计
        Eigen::VectorXd x_post;

        double dt_;
        public:
        // x y z的过程噪声的方差
        double s2qxyz_;
        // yaw 的过程噪声方差
        double s2qyaw_;
        // r 的过程噪声方差
        double s2qr_;
        // x y z 的观测噪声的方差
        double r_xyz_factor;
        // r 的观测噪声的方差
        double r_yaw;
        
        double lastTime;

        int isUseEkfFlag;
    };

    EKF::EKF(ros::NodeHandle nh)
    {
        nh.param("s2qxyz_", s2qxyz_, 0.0000001);//0.0000001
        // s2qxyz_ = 0;
        nh.param("s2qyaw_", s2qyaw_, 100.0);//0.43
        nh.param("s2qr_", s2qr_, 800.0);
        // s2qr_ = 0;
        nh.param("r_xyz_factor", r_xyz_factor, 9000.0);//1.7
        nh.param("r_yaw", r_yaw, 0.02);
        
        // // x y z的过程噪声的方差
        // s2qxyz_ = 0.5;
        // // yaw 的过程噪声方差
        // s2qyaw_ = 100.0;
        // // r 的过程噪声方差
        // s2qr_ = 800.0;
        // // x y z 的观测噪声的方差
        // r_xyz_factor = 0.05;
        // // r 的观测噪声的方差
        // r_yaw = 0.02;
        isUseEkfFlag = 0;
        // cout << "s2qxyz_" << s2qxyz_ << endl;
        // cout << "s2qyaw_" << s2qyaw_ << endl;
        // cout << "s2qr_" << s2qr_ << endl;

        // cout << "r_xyz_factor" << r_xyz_factor << endl;
        // cout << "r_yaw" << r_yaw << endl;

        // sleep(5);

        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
        // measurement: xa, ya, za, yaw 
        // f - Process function
        lastTime = ros::Time::now().nsec;
        // 状态函数
        auto f = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_;
            x_new(2) += x(3) * dt_;
            x_new(4) += x(5) * dt_;
            x_new(6) += x(7) * dt_;
            return x_new;
        };
        // J_f - Jacobian of process function
        // 状态函数的雅各比矩阵
        auto j_f = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(9, 9);
            // clang-format off
            f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
                0,   1,   0,   0,   0,   0,   0,   0,   0,
                0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
                0,   0,   0,   1,   0,   0,   0,   0,   0,
                0,   0,   0,   0,   1,   dt_, 0,   0,   0,
                0,   0,   0,   0,   0,   1,   0,   0,   0,
                0,   0,   0,   0,   0,   0,   1,   dt_, 0,
                0,   0,   0,   0,   0,   0,   0,   1,   0,
                0,   0,   0,   0,   0,   0,   0,   0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        // 观测函数 
        auto h = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        
        // J_h - Jacobian of observation function
        // 观测函数的雅各比矩阵
        auto j_h = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 9);
            double yaw = x(6), r = x(8);
            // clang-format off
            //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
            h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
                0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
                0,   0,   0,   0,   1,   0,   0,          0,   0,
                0,   0,   0,   0,   0,   0,   1,          0,   0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_q = [this]() {
            Eigen::MatrixXd q(9, 9);
            double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
            // 这段公式是算各个变量的协方差
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
            q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
                q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
                0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
                0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
                0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
                0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
                0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
                0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
                0,      0,      0,      0,      0,      0,      0,      0,      q_r;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_r = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            
            if(isnan(x))
                x = 0.05;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
            return r;
        };
        
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 9> p0;
        p0.setIdentity();
        
        this->f = f;
        this->jacobian_f = j_f;
        this->h = h;
        this->jacobian_h = j_h;
        this->update_Q = u_q;
        this->update_R = u_r;
        this->P_post = p0;
        this->n = p0.rows();
        this->I = (Eigen::MatrixXd::Identity(n, n));
        this->x_pri = VectorXd::Zero(n);
        this->x_post = VectorXd::Zero(n);
    }

    Eigen::MatrixXd EKF::predict(double in_dt = 0.02)
    {
        this->dt_ = in_dt;
        // double nowTime = ros::Time::now().nsec;
        // this->dt_ = (nowTime - this->lastTime) * 10e-9;
        // lastTime = nowTime;
        // this->dt_ = 0.02;
        // cout << this->dt_ << endl;
        // 求雅各比矩阵
        F = jacobian_f(x_post), Q = update_Q();
        // 先验估计

        x_pri = f(x_post);
        // 先验估计的协方差矩阵
        P_pri = F * P_post * F.transpose() + Q;

        // handle the case when there will be no measurement before the next predict
        // 当没有获取到观测量时，避免EKF不连续，所以采用这一帧的先验用作后验
        x_post = x_pri;
        P_post = P_pri;

        return x_pri;
    }

    Eigen::MatrixXd EKF::update(const Eigen::VectorXd & z,double in_dt = 0.02)
    {
        // cout << "======================== Update" << endl;
        dt_ = in_dt;
        // 求雅各比矩阵
        H = jacobian_h(x_pri), R = update_R(z);
        // 更新卡尔曼增益
        K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
        // cout << "P_pri * H.transpose():" << endl <<P_pri * H.transpose() <<endl;
        // cout << "H * P_pri * H.transpose() + R:" << endl <<H * P_pri * H.transpose() + R <<endl;
        // cout << "(H * P_pri * H.transpose() + R).inverse():" << endl <<H * P_pri * H.transpose() + R <<endl;
        // cout << "H.transpose() * (H * P_pri * H.transpose() + R).inverse():" << endl <<P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse() <<endl;
        // cout << "P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse():" << endl <<P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse() <<endl;
        // cout << "P_pri:"<< endl <<P_pri <<endl;
        // cout << "H.transpose():"<< endl <<H.transpose() <<endl;
        // cout << "R:"<< endl <<R <<endl;
        // cout << "K:"<< endl <<K <<endl;
        // 后验估计
        x_post = x_pri + K * (z - h(x_pri));

        P_post = (I - K * H) * P_pri;

        return x_post;
    }

    void EKF::setPri(const Eigen::VectorXd & p0){
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        P_pri <<p0(0,0),  p0(1,0), 0,      0,      0,      0,      0,      0,      0,
                p0(1,0), 0,0,      0,      0,      0,      0,      0,      0,
                0,      0,      p0(0,0),  p0(1,0), 0,      0,      0,      0,      0,
                0,      0,      p0(1,0), 0,0,      0,      0,      0,      0,
                0,      0,      0,      0,      p0(0,0),  p0(1,0), 0,      0,      0,
                0,      0,      0,      0,      p0(1,0), 0,0,      0,      0,
                0,      0,      0,      0,      0,      0,      p0(2,0),  p0(3,0), 0,
                0,      0,      0,      0,      0,      0,      p0(3,0), 0,0,
                0,      0,      0,      0,      0,      0,      0,      0,      p0(8,0);
    }

    void EKF::setState(const Eigen::VectorXd & x0) { x_post = x0; isUseEkfFlag++;}
    bool EKF::isUseUkF()
    {
        if(this->isUseEkfFlag > 1)
            return true;
        else
            return false;
    }
    void EKF::setParam()
    {
        this->nh.getParam("s2qxyz_", s2qxyz_);
        this->nh.getParam("s2qyaw_", s2qyaw_);
        this->nh.getParam("s2qr_", s2qr_);
        this->nh.getParam("r_xyz_factor", r_xyz_factor);
        this->nh.getParam("r_yaw", r_yaw);
    }
}

//三维卡尔曼滤波器
class LinearKalmanFilter3D {
public:
    // 状态维度6：x,y,z, vx,vy,vz；测量维度3：x,y,z
    LinearKalmanFilter3D(float q_var = 1e-4, float r_var = 1e-2)
        : q_var_(q_var), r_var_(r_var) {
        kf_.init(6, 3, 0);  // 修改1：状态6维，测量3维
        
        // 6x6转移矩阵（添加z轴相关项）
        kf_.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);  // 修改2：6x6矩阵
        
        // 3x6测量矩阵（测量x,y,z位置）
        kf_.measurementMatrix = (cv::Mat_<float>(3, 6) <<  // 修改3：3x6矩阵
            1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0);

        kf_.processNoiseCov = cv::Mat::eye(6, 6, CV_32F);  // 修改4：6x6噪声矩阵
        
        // 3x3测量噪声矩阵
        kf_.measurementNoiseCov = (cv::Mat_<float>(3, 3) <<  // 修改5：3x3矩阵
            r_var_,0,0,
            0,r_var_,0,
            0,0,r_var_);

        // 6x6初始协方差（z轴不确定性更大）
        kf_.errorCovPost = (cv::Mat_<float>(6, 6) <<  // 修改6：6x6矩阵
            50,0,0,  0,  0,  0,
            0,50,0,  0,  0,  0,
            0,0,50,  0,  0,  0,
            0,0,0, 100, 0,  0,
            0,0,0,  0,100, 0,
            0,0,0,  0, 0,100);
    }

    cv::Mat predict(float dt) {
        // 重置转移矩阵（添加z轴速度项）
        kf_.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);
        kf_.transitionMatrix.at<float>(0, 3) = dt; // x += vx*dt
        kf_.transitionMatrix.at<float>(1, 4) = dt; // y += vy*dt
        kf_.transitionMatrix.at<float>(2, 5) = dt; // 修改7：z += vz*dt

        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt3 * dt;

        // 更新过程噪声协方差（添加z轴计算）
        // 位置方差项
        kf_.processNoiseCov.at<float>(0, 0) = q_var_ * dt4 / 4; // x
        kf_.processNoiseCov.at<float>(1, 1) = q_var_ * dt4 / 4; // y
        kf_.processNoiseCov.at<float>(2, 2) = q_var_ * dt4 / 4; // 修改8：z
        
        // 速度方差项
        kf_.processNoiseCov.at<float>(3, 3) = q_var_ * dt2;    // vx
        kf_.processNoiseCov.at<float>(4, 4) = q_var_ * dt2;    // vy
        kf_.processNoiseCov.at<float>(5, 5) = q_var_ * dt2;    // 修改9：vz
        
        // 位置-速度协方差
        kf_.processNoiseCov.at<float>(0, 3) = kf_.processNoiseCov.at<float>(3, 0) = q_var_ * dt3 / 2; // x-vx
        kf_.processNoiseCov.at<float>(1, 4) = kf_.processNoiseCov.at<float>(4, 1) = q_var_ * dt3 / 2; // y-vy
        kf_.processNoiseCov.at<float>(2, 5) = kf_.processNoiseCov.at<float>(5, 2) = q_var_ * dt3 / 2; // 修改10：z-vz

        return kf_.predict();
    }

    void correct(const cv::Mat& measurement) {
        cv::Mat meas;
        measurement.convertTo(meas, CV_32F);
        // 确保测量值是3维
        if (meas.rows == 3 && meas.cols == 1)
            kf_.correct(meas);
        else
            CV_Error(cv::Error::StsBadArg, "Measurement must be 3x1 vector");
    }

    // 修改11：添加z和vz参数
    void initializeState(float x, float y, float z, 
                        float vx = 0, float vy = 0, float vz = 0) {
        kf_.statePost = (cv::Mat_<float>(6, 1) << x,y,z, vx,vy,vz);
    }

private:
    cv::KalmanFilter kf_;
    float q_var_;
    float r_var_;
};