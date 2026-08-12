#ifndef TRACKER
#define TRACKER
#include <iostream>
#include <Eigen/Dense>
#include "extended_kalman_filter.h"
#include "opencv2/core.hpp"
#include <vector>
#include "ros/ros.h"
#include "Calculater.hpp"
#include "Variable.h"
#include "PubUrdf.h"
#include "calculate.h"
#include "Image.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_ros/transform_listener.h>
#include "test_compensation.hpp"
#include "FrameCounter.hpp"
#include "test_compensation.hpp" 
#include "ImuData.hpp"
#include "Basic.hpp"

// 老登遗留: 老自瞄
#define IMUINCAR_X 0.2
#define IMUINCAR_Y 0
#define IMUINCAR_Z 0.3

#define CAR2CAM_X 111
#define CAR2CAM_Y 0
#define CAR2CAM_Z 41

#define CAMERAINIMU_X 0.111
#define CAMERAINIMU_Y 0
#define CAMERAINIMU_Z 0.041

static geometry_msgs::TransformStamped ts_point,ts_camera,ts_imu,ts_car,ts_target;
inline void CarToWorld_TFinit()
{
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    ts_car.header.frame_id = "map";
    ts_car.header.stamp = ros::Time::now();
    ts_car.child_frame_id = "car";
    ts_car.transform.translation.x = 1;
    ts_car.transform.translation.y = 0;
    ts_car.transform.translation.z = 0;
    ts_car.transform.rotation.w = qtn.getW();
    ts_car.transform.rotation.x = qtn.getX();
    ts_car.transform.rotation.y = qtn.getY();
    ts_car.transform.rotation.z = qtn.getZ();
}

inline void ImuToCar_TFinit(double pitch, double yaw)
{
    tf2::Quaternion qtn;
    qtn.setRPY(0,pitch,yaw);
    ts_imu.header.frame_id = "car";
    ts_imu.header.stamp = ros::Time::now();
    ts_imu.child_frame_id = "imu";
    ts_imu.transform.translation.x = IMUINCAR_X;
    ts_imu.transform.translation.y = IMUINCAR_Y;
    ts_imu.transform.translation.z = IMUINCAR_Z;
    ts_imu.transform.rotation.w = qtn.getW();
    ts_imu.transform.rotation.x = qtn.getX();
    ts_imu.transform.rotation.y = qtn.getY();
    ts_imu.transform.rotation.z = qtn.getZ();
}

inline void CameraToImu_TFinit()
{
    Eigen::Affine3d affine(R_cap2odom);
    Eigen::Matrix3d rotation = affine.rotation();
    Eigen::Quaterniond quaternion(rotation);

    // tf2::Quaternion qtn;

    
    // qtn.setRPY(0,M_PI / 2,0);
    ts_camera.header.frame_id = "imu";
    ts_camera.header.stamp = ros::Time::now();
    ts_camera.child_frame_id = "camera";
    ts_camera.transform.translation.x = CAMERAINIMU_X;
    ts_camera.transform.translation.y = CAMERAINIMU_Y;
    ts_camera.transform.translation.z = CAMERAINIMU_Z;
    ts_camera.transform.rotation.w = quaternion.w();
    ts_camera.transform.rotation.x = quaternion.x();
    ts_camera.transform.rotation.y = quaternion.y();
    ts_camera.transform.rotation.z = quaternion.z();
}

inline void TargetToCar_TFinit(Mat tvec, double yaw)
{
    tf2::Quaternion qtn;
    qtn.setRPY(0,yaw / 180 * M_PI,0);
    ts_target.header.frame_id = "camera";
    ts_target.header.stamp = ros::Time::now();
    ts_target.child_frame_id = "armour";
    ts_target.transform.translation.x = tvec.at<double>(0,0) / 1000;
    ts_target.transform.translation.y = tvec.at<double>(1,0) / 1000;
    ts_target.transform.translation.z = tvec.at<double>(2,0) / 1000;
    ts_target.transform.rotation.w = qtn.getW();
    ts_target.transform.rotation.x = qtn.getX();
    ts_target.transform.rotation.y = qtn.getY();
    ts_target.transform.rotation.z = qtn.getZ();
}

typedef struct Forecast{
    double compensation;
    Eigen::Vector3d camTarget;
    double timeToHit; //ms
}Forecast;
typedef struct Armor{
    int id = -1;
    double yaw = 0;
    double distance = -1;
    double distanceToCenter = -1;
    double priority = 0;
    char type = 's';
    cv::Rect detectionBox;
    std::vector<cv::Point2f> points;
    Eigen::Vector3d camCenterBa;
    Eigen::Vector3d rvec;
    Eigen::Vector3d rvecBa;
    Eigen::Vector3d camCenter;
    Eigen::Vector3d gunCenter;
    Eigen::Vector3d absCenter;
	cv::Point2f heightWidth;
    Eigen::Vector3d odomCenter;
    long dTime = 0; //识别到的时间
}Armor;
typedef struct Robot{
    int id = -1;
    long lastChangeTime = 0;
    long changeTime = 0;
    long lastFrame = 0;
    Fiter::EKF* ekf = NULL;
    Eigen::Vector3d center;
    double radius = -1; // 0 long 1 short
    Armor armors[4]; 
    uchar longR[2] = {0,0};
    uchar shortR[2] = {0,0};
    bool equalRadius = false;
    double angle = 0;
    double lastYawVec = 0.0;
    double yawVec = 0;// -顺时针 +逆时针
    Armor last,_last;// _做中心运算初始化后一般不再改变，没有_是上一次识别到该id时的位置
    AverageCounter<20> rav; 
}Robot;
inline double CalcPriority(double d,double d2c){
    if(d == -1 || d2c == -1)
        return double_max;
    // return d / 10000 * R_P_LOCATION + d2c / 1640 * R_P_FROM_CENTER;
    return d2c / 1640 * R_P_FROM_CENTER;
}
inline double CalcPriority(const Armor& a){
    return CalcPriority(a.distance,a.distanceToCenter);
}
typedef struct TraceBag{
    Robot* rob = NULL;
    Armor* detect = NULL;
    bool isStatic = true;
    bool shootAble = false;
    Eigen::Vector3d aimCamPos = {0,0,0};
    Eigen::Vector3d centerCamPos = {0,0,0};
}TraceBag;
template<int range>
class Tracer{
private:
    int Range = range;
    ThreadPool& pool;
    Robot* robs;
    ros::NodeHandle& handle;
    CAL::Calculater* cal;
    Image* img;
public:
    PubURDF pubUrdf;
    const int ROBOTS = 5;

    const double LimitYawVec = 1.0;

    Tracer(ros::NodeHandle&nh,ThreadPool&p,CAL::Calculater* c,Image* image):pool(p),handle(nh),pubUrdf(nh),cal(c),img(image){
        robs = new Robot[ROBOTS]();
        for(int i = 0;i < ROBOTS;i++)
            (robs + i)->ekf = new Fiter::EKF(nh);
    }
    ~Tracer(){
        for(int i = 0;i < ROBOTS;i++){
            delete (robs + i)->ekf;
            (robs + i)->ekf = NULL;
        }
        delete[] robs;
        robs = NULL;
    }
    vector<Armor> RebuildArmor(double yaw,const Robot& rob,int count = 4,double step = PI/2,int a_id = -1){
        vector<Armor> result;
        for(int i = 0;i < count;i++){
            Armor temp;
            temp.id = a_id;
            temp.yaw = yaw + step * i;
            //重构坐标
            temp.absCenter << rob.center[0] - rob.radius * cos(yaw + step * i), rob.center[1] - rob.radius * sin(yaw + step * i), rob.center[2];
            // cal->TFUpdata("map",CAL::TempTF,temp.absCenter);
            temp.camCenter = cal->GetPos("cam");
            temp.gunCenter = cal->GetPos("gun");
            temp.odomCenter = cal->GetPos("imu");
            //计算距离
            temp.distance = temp.camCenter.norm();
            temp.distanceToCenter = cv::norm(img->camera_center - cal->ThreeToTwo(temp.camCenter));
            //计算优先级
            temp.priority = CalcPriority(temp);
            //重构角点
            auto camPTS = cal->RebuildArmorInOdom({0.0,temp.yaw,-15.0 * D2R},temp.absCenter);
            for(auto&pt:camPTS)
                temp.points.push_back(cal->ThreeToTwo(pt));
            result.push_back(temp);
        }
        //根据距离优先级从小到大排序
        //TODO:模板排序类
        bool nbreak = true;
        for(int i = 0;i < result.size();i++){
            nbreak = true;
            for(auto it = result.begin(),_it = result.begin() + 1;_it != result.end();it++,_it++)
                if(fabs(it->distance) > fabs(_it->distance)){
                    auto temp = *it;
                    *it = *_it;
                    *_it = *it;
                    nbreak = false;
                }
            if(nbreak)
                break;
        }
        return std::move(result);
    }
    static Eigen::Vector3d PredictCalc(Robot& robot,Eigen::MatrixXd& predict,double ft,bool oput = true){
        Eigen::Vector3d result;
        //cout << "Input Predict:" << predict << endl;
        // double xita =  predict(6,0) + predict(7,0)*ft,x0 = predict(0,0),y0 = predict(2,0),r = (predict(8,0));
        double xita =  predict(6,0) - robot.changeTime * PI / 2 + predict(7,0)*ft,x0 = predict(0,0),y0 = predict(2,0),r = (predict(8,0));
        // result << x0 - r * cos(xita) + predict(1,0) * ft, y0 - r * sin(xita) + predict(3,0) * ft, predict(4,0) + predict(5,0) * ft;
        result << x0 - r * cos(xita) + predict(1,0) * ft, y0 - r * sin(xita) + predict(3,0) * ft, predict(4,0);
        if(Debug && oput){
            cout << "Xita      :" <<  xita*R2D << endl;
            cout << "R         :" <<  r << endl;
            cout << "Center    :" << robot.center.x() << ' ' << robot.center.y() << ' ' << robot.center.z() << endl;
            cout << "Predict   :" << result.x() << ' ' << result.y() << ' ' << result.z() << endl;
            cout << "Radius    :" << robot.radius << endl;
            cout << "Vec       :" << sqrt(pow(predict(1,0),2) + pow(predict(3,0),2)) << endl;
            cout << "Pre-Yaw   :" << predict(6,0) << endl;
            cout << "Vec-Yaw   :" << predict(7,0) << endl;
        }
        return result;
    }
    Eigen::Vector3d AfterC(cv::Mat& mat,Robot& robot,Eigen::MatrixXd& x_pri,Armor a,ImuData& imuData,Eigen::Vector3d& prePos,double frameTime,double foreTime,bool isStatic = false){
        Eigen::Vector3d result;
        //判断是否需要更改目标(装甲板)
        double yawVec = x_pri(7 ,0);
        //非跳变情况下计算平均yaw速度
        if((abs((yawVec-robot.lastYawVec)/frameTime) <= LimitYawVec)){
            //非跳变情况下计算平均YawVec
            robot.rav.Updata(yawVec / frameTime);
        }
        double r_rav = robot.rav.GetAverageValue();

        //绘制机器人中心
        if(Debug){
            auto rcenter = cal->GetPos("cam","center");
            auto rcenter2d = cal->ThreeToTwo(rcenter);
            cv::putText(mat,std::to_string(r_rav),cv::Point2f(rcenter2d.x,rcenter2d.y - 30),cv::FONT_HERSHEY_COMPLEX,1.0,cv::Scalar(255,255,255));
            cv::circle(mat,rcenter2d,10,cv::Scalar(255,255,255),3);
        }

        bool zhuan = fabs(r_rav) > RAV_LIMIT;

        //计算预测前推角度
        //重构装甲板
        //TODO:实际要根据情况更改重构量
        // auto rebuild = RebuildArmor(x_pri(6,0) + foreYaw,robot);
        double yaw = x_pri(6,0) - (int)(x_pri(6,0)/(2*PI)) * 2*PI;
        Eigen::Vector3d preTarget;
        preTarget << robot.center[0] + robot.radius * cos(yaw), robot.center[1] + robot.radius * sin(yaw), robot.center[2];

        //识别重构
        int index = 2;
        cal->TFUpdata("map","detect",a.absCenter,{0.0, -15.0 * D2R, yaw},index++);
        cal->TFUpdata("map","forecast",prePos,{0.0, -15.0 * D2R, yaw},index++);
        if(Debug){
            auto rb_d = cal->vtf.RebuildOtherArmor(cal->GetPos("map","detect"),cal->GetPos("map","center"),{0.0, -15.0 * D2R, a.yaw});
            for(int i = 0;i < 4;i++){
                cal->TFUpdata("map","armor" + std::to_string(i),rb_d[i][0],rb_d[i][1],index);
                index++;
            }
        }
        //预测重构
        // auto rb = cal->vtf.RebuildArmor(cal->GetPos("imu","forecast"),cal->GetPos("imu","center"),{15.0 * D2R,0.0,yaw + PI});
        auto rb = cal->vtf.RebuildOtherArmor(cal->GetPos("map","forecast"),cal->GetPos("map","center"), {0.0, -15.0 * D2R, yaw});
        //存储符合打击范围(yaw)的点的索引
        vector<int> targets;
        cout << "RB-Yaw:";
        for(int i = 0;i < 4;i++){
            cal->TFUpdata("map","fore_armor" + std::to_string(i),rb[i][0],rb[i][1],index);
            index++;
            if(fabs(rb[i][1].z() - PI - imuData.GetYaw()) <= ATTACK_ANGLE)
                targets.push_back(i);
            cout << rb[i][1].z() - PI - imuData.GetYaw() << ' ';
        }
        for(int i = 0; i < 4; i++)
        {
            circle(mat, cal->ThreeToTwo(cal->vtf.GetChildInFrameData("cam", "fore_armor" + std::to_string(i)) * 1000), 16, Scalar(255, 255, 255), -1, 8);
        }
        cout << endl;
        
        //如果有多个符合条件的目标,取目标中距离最近的
        //如果没有符合条件的目标,取所有装甲板中距离最近的
        Eigen::Vector3d target(0,0,0);
        if(targets.size() > 1){
            target = rb[targets[0]][0];
            double mindis = DOUBLE_MAX; 
            for(auto& i : targets){
                if(rb[i][0].norm() < mindis){
                    mindis = rb[i][0].norm();
                    target = rb[i][0];
                }
            }
        }
        else if(!targets.size()){
            target = prePos;
            //TODO
        }
        else
            target = rb[targets[0]][0];

        cal->TFUpdata("map","TARGET",target,{0,0,0},index);
        // cal->TFUpdata("imu","target",target,{0,0,0},index++);
        // return (cal->GetPos("cam","forecast") * 1000.0);
        // vector<Armor> targets = RebuildArmor(yaw,robot,4,PI/2,robot.id);
        // for(int i = 0;i < robot.rav.GetRange();i++)
        //     cout << i << ":" << *(robot.rav.Values + i) << endl;
        // Armor& target = rebuild[atkIndex];
        // result = target.absCenter;
        result = cal->GetPos("cam","TARGET") * 1000;
        //画面重构
        if(Debug){
            cout << "TAR:" << targets.size() << endl;
            cout << "AVG:" << robot.rav.GetAverageValue() << endl;
            // pubUrdf.PubTF({robot.center,result,a.absCenter},{{0,0,0},{0,15 * D2R,x_pri(6,0)},{0,15 * D2R,a.yaw}});
            cv::line(mat,cv::Point(640,1000),cal->ThreeToTwo(result),cv::Scalar(128,128,128),2);
            // pubUrdf.PubPointStamp(x_pri(8, 0), robot.radius, abs(robot.radius - abs(x_pri(8,0))));
            cout << "-----:" << -(robot.center - prePos) << endl;
        }
        // result = target;
        robot.last = a;
        robot.lastYawVec = x_pri(7,0);
        return result;
    }
    int GetId(int a_id){
        //判断号码数 <4 red >5 blue
        int result = -1;
        if(a_id > 5 && a_id < 11)
            result = a_id - 6;
        else if(a_id < 5)
            result = a_id;
        if(result < 0 || result >= ROBOTS)
            return 0;
        return result;
    }
    bool ShootAble(TraceBag aimbag){
        if(aimbag.detect){
            auto forecast2D = cal->ThreeToTwo(aimbag.aimCamPos);
            return fabs(forecast2D.x + SHOOT_YAW_OFFSET - img->camera_center.x) < (cv::norm(aimbag.detect->heightWidth)/2)*SHOOT_RANGE_RATE;
        }
        auto forecast2D = cal->ThreeToTwo(aimbag.aimCamPos);
        return fabs(forecast2D.x + SHOOT_YAW_OFFSET - img->camera_center.x) < (cv::norm(aimbag.rob->last.heightWidth)/2)*SHOOT_RANGE_RATE;
    }
    TraceBag BuildBagFromArmor(const Armor& a){
        TraceBag result;
        auto forecast2D = cal->ThreeToTwo(a.camCenter);
        result.detect = &(const_cast<Armor&>(a));
        result.rob = NULL;
        result.aimCamPos = a.camCenter * 1000;
        result.centerCamPos = a.camCenter * 1000;
        result.shootAble = ShootAble(result);
        result.isStatic = true;
        return result;
    }
    //根据单块装甲板维护对应机器人
    TraceBag Trace(ros::NodeHandle& nh,cv::Mat mat,long frame,Armor& a,ImuData& imuData,double foreTime,double frameTime = 0.2){
        // frameTime = foreTime;
        long currentTime = GetTimestamp();
        TraceBag result;
        result.detect = &(const_cast<Armor&>(a));
        
        int index = GetId(a.id);
        if(!~index){
            if(Debug){
                cout << "Unknow ID:" << index << endl;
                cout << "CNN    ID:" << a.id << endl;
            }
            return BuildBagFromArmor(a);
        }
        if(Debug){
            cout << "Detect ID:" << index << endl;
            cout << "CNN    ID:" << a.id << endl;
        }
        //FIXME:测试
        // index = 0;
        //找寻对应机器人并且判断其信息是否有效
        Robot& robot = *(robs + index);
        robot.id = index;
        if(frame - robot.lastFrame >= range){
            //信息无效，清空卡尔曼滤波
            if (robot.ekf)
                delete robot.ekf;
            robot.ekf = new Fiter::EKF(handle);
            robot.changeTime = 0;
            if(Debug)
                cout << "Clear EKF of " << index << "!" << endl;
        }
        robot.lastFrame = frame;
        double imuYaw = imuData.GetYaw();
        auto armor_vec = cal->ThreeToTwo(robot.last.camCenter * 1000.0) - cal->ThreeToTwo(a.camCenter * 1000.0);
        //判断是否装甲板切换
        if(fabs(robot.last.yaw - a.yaw) > 15*D2R)
        //  && (robot.last.absCenter - a.absCenter).norm() >= 0.5 * robot.radius
        //  && currentTime - robot.lastChangeTime >= 50
        //  && cv::norm(armor_vec) >= (robot.last.detectionBox.width + a.detectionBox.width) / 2 * 0.3
         {
            if(robot.rav.GetAverageValue() < -1.0){
                robot.changeTime -= 1;
                robot.lastChangeTime = currentTime;
            }
            else if(robot.rav.GetAverageValue() > 1.0){
                robot.changeTime += 1;
                robot.lastChangeTime = currentTime;
            }
        }
        // a.yaw = robot.changeTime * PI / 2 + a.yaw;
        //两帧间计算平移速度,太小则不尝试初始化卡尔曼(静止停止预测)
        //未初始化且装甲板为静止则尝试初始化后返回无预测点
        //已经初始化卡尔曼但是静止情况下维护卡尔曼但是不使用预测值
        double moveSpeed = (robot.last.odomCenter - a.odomCenter ).norm() / (abs(robot.last.dTime - a.dTime)/1000.0);
        bool isStatic = (moveSpeed < (ROTATION_THRESH));
        //需要初始化扩展卡尔曼滤波
        //看上一帧的装甲板是否是同一号码 是则计算机器人中心 否则返回装甲板中心
        if(!robot.ekf->isUseUkF()){
            //id不一样则遗弃
            if((1 || robot._last.id == a.id)){
                //时间间隔太长则遗弃
                if(robot.last.dTime - currentTime >= DATA_SAVE_TIME || robot._last.dTime - currentTime >= DATA_SAVE_TIME){
                    //对于角度计算,只有超出时重置
                    if(robot._last.dTime - currentTime >= DATA_SAVE_TIME)
                        robot._last = a;
                    //上一帧装甲板无论如何都重置
                    robot.last = a;
                    return BuildBagFromArmor(a);
                }
                if(Debug)
                    cout << "-------- " << frame << " Initializing" << " --------" << endl;
                robot.center << a.absCenter.x() + BEGIN_R * cos(a.yaw),a.absCenter.y() + BEGIN_R * sin(a.yaw),a.absCenter.z();
                //更新卡尔曼状态
                while(!robot.ekf->isUseUkF()){
                    // 这里是初始化EKF，初始化10次才能去预测
                    Eigen::VectorXd state_z(9);
                    state_z << robot.center[0], 0, robot.center[1], 0, a.absCenter[2], 0, a.yaw, 0, BEGIN_R;
                    robot.ekf->setState(state_z);
                }
                //输出中心
                // cout << "Current     : " << a.id << ' '<< a.absCenter[0] << ' ' << a.absCenter[1] << ' ' << a.absCenter[2] << ' ' << a.yaw << endl;
                // cout << "Last        : " << _last.id  << ' ' << _last.absCenter[0] << ' ' << _last.absCenter[1] << ' ' << _last.absCenter[2] << ' ' << _last.yaw << endl;
                // cout << "Robot Center: " << robot.center.x() << ' ' << robot.center.y() << ' ' << robot.center.z() << endl;
                robot._last = a;
                if(Debug)
                    cout << "Operated normally!" << endl;
            }
            else{
                robot.last = a;
                robot._last = a;
                return BuildBagFromArmor(a);
            }
        }
        // cal->TFUpdata("map","center",robot.center,{0,0,0},2);
        //维护卡尔曼 这个要放在isUKF为True的时候才能用
        if(robot.ekf->isUseUkF()){
            //已经初始化卡尔曼但是静止情况下维护卡尔曼但是不使用预测值
            Eigen::VectorXd z(4);
            //yaw增益系数
            double yRate = YAW_GAIN_RATE;
            z << a.absCenter[0], a.absCenter[1], a.absCenter[2], a.yaw + robot.changeTime * PI / 2;
            if(Debug){
                robot.ekf->setParam();
                cout << "-------- " << frame << " Detecting " << index << " --------" << endl;
                cout << "Current   :" << robot.changeTime % 4 << endl;
                cout << "ChangeTime:" << robot.changeTime << endl;
                cout << "Abs-Center:" << a.absCenter << endl;
                cout << "Fore-Time :" << frameTime << endl;
                cout << "Update    :" << a.absCenter[0] << ' ' << a.absCenter[1] << ' ' << a.absCenter[2] << ' ' << a.yaw << endl;
                cout << "Move-Dis  :" << (robot.last.absCenter - a.absCenter).norm() << endl;
                cout << "Time-Span :" << abs(robot.last.dTime - a.dTime) << endl;
                cout << "Move-Speed:" << moveSpeed << endl;
                cout << "Is-Static :" << isStatic << endl;
                nh.param("yaw_rate",yRate,1.0);
            }
            auto x_pri = robot.ekf->predict(frameTime);
            auto prePos = PredictCalc(robot,x_pri,foreTime,false);
            // cv::circle(mat,cal->ThreeToTwo(cal->OdomToCam(prePos) * 1000),16,cv::Scalar(255,0,0),-1);
            //预测异常处理
            bool nSetState = false;
            if(a.yaw - x_pri(6,0) >= MAX_YAW_DIFF * frameTime){
                x_pri(6,0) = a.yaw;
                x_pri(7,0) = 0;
                nSetState = true;
            }
            if(x_pri(8,0) > MAX_R){
                x_pri(8,0) = MAX_R;
                nSetState = true;
            }
            else if(x_pri(8,0) < MIN_R){
                x_pri(8,0) = MIN_R;
                nSetState = true;
            }
            if((prePos - a.absCenter).norm() >= MAX_MOVE_DIFF || (a.absCenter - robot.center).norm() >= MAX_R){
                Eigen::Vector3d tCenter;
                tCenter << a.absCenter.x() + x_pri(8,0) * cos(a.yaw),a.absCenter.y() + x_pri(8,0) * sin(a.yaw),a.absCenter.z();
                nSetState = true;
                x_pri(0,0) = tCenter.x();
                x_pri(1,0) = 0.0;
                x_pri(2,0) = tCenter.y();
                x_pri(3,0) = 0.0;
                x_pri(4,0) = tCenter.z();
                x_pri(5,0) = 0.0;
            }
            else if(fabs(a.absCenter.z()-x_pri(4,0) >= 0.5)){
                nSetState = true;
                x_pri(4,0) = a.absCenter.z();
                x_pri(5,0) = 0.0;
            }
            if(nSetState){
                robot.ekf->setState(x_pri);
                x_pri = robot.ekf->predict(frameTime);
                // robot.ekf->setPri(x_pri);
            }

            prePos = PredictCalc(robot,x_pri,foreTime,true);
            auto x_post = robot.ekf->update(z,frameTime);

            //获取预测点
            // result = PredictCalc(robot,x_pri);

            if(Debug){
                pubUrdf.PubPointStamp(a.yaw * R2D,x_pri(6,0) * R2D,0.0);
            }

            //更新机器人信息
            robot.center << x_pri(0,0),x_pri(2,0),prePos.z();
            robot.radius = x_pri(8,0);
            robot.yawVec = x_pri(7,0);
            robot.last = a;

            x_pri(6,0) *= yRate;

            // cv::circle(mat,cal->ThreeToTwo(cal->OdomToCam(robot.center)),10,cv::Scalar(255,255,255),3);
            // if((prePos - a.absCenter).norm() >= MAX_MOVE_DIFF)
            //     return a.camCenter;
            // else
            //     return cal->OdomToCam(prePos) * 1000;
            // cal->TFUpdata("map","forecast",prePos,{0,0,0},1);
            // auto rb = cal->vtf.RebuildArmor(cal->GetPos("imu","forecast"),cal->GetPos("imu","center"),{15.0 * D2R,0.0,a.yaw + PI - imuData.GetYaw()});
            // int index = 4;
            // for(int i = 0;i < 4;i++){
            //     cal->TFUpdata("imu","armor" + std::to_string(i),rb[4][i],rb[5][i],index);
            //     index++;
            // }
            // return (cal->GetPos("cam","forecast") * 1000.0);
            cal->TFUpdata("map","center",robot.center,{0,0,0},1);
            if(isStatic){
                result.aimCamPos = a.camCenter * 1000.0;
            }
            else
                result.aimCamPos = AfterC(mat,robot,x_pri,a,imuData,prePos,frameTime,foreTime,isStatic);
            result.centerCamPos = cal->GetPos("cam","center") * 1000.0;
            auto forecast2D = cal->ThreeToTwo(result.aimCamPos); 
            result.rob = &robot;
            result.isStatic = isStatic;
            result.shootAble = ShootAble(result);
            return result;
        }
    }
    //维护所有机器人，判断可打击点位，选取最优点
    bool TraceAll(ros::NodeHandle& nh,cv::Mat mat,long frame,TraceBag& res,ImuData& imuData,double foreTime,double frameTime = 0.2){
        // frameTime = foreTime;
        res.detect = NULL;
        res.isStatic = false;
        // return true;

        for(int i = 0;i < ROBOTS;i++){
            if(frame - (robs + i)->lastFrame >= range){
                //信息无效，清空卡尔曼滤波
                if((robs + i)->ekf)
                    delete (robs + i)->ekf;
                (robs + i)->ekf = new Fiter::EKF(handle);
                continue;
            }
            if(!(robs + i)->ekf->isUseUkF())
                continue;

            //yaw增益系数
            double yRate = YAW_GAIN_RATE;
            if(Debug){
                cout << "-------- " << frame << " Lost Target --------" << endl;
                cout << "Fore-Time :" << frameTime << endl;
                nh.param("yaw_rate",yRate,1.0);
            }
            //预测 并将预测结果作为新的观测量维护卡尔曼
            (robs + i)->ekf->setParam();
            auto x_pri = (robs + i)->ekf->predict(frameTime);
            auto prePos = PredictCalc(*(robs + i),x_pri,foreTime,false);
            // cv::circle(mat,cal->ThreeToTwo(cal->OdomToCam(prePos) * 1000),16,cv::Scalar(255,0,0),-1);
            
            //预测异常处理
            bool nSetState = false;
            // if((robs + i)->last.yaw - x_pri(6,0) >= MAX_YAW_DIFF * frameTime){
            //     x_pri(6,0) = (robs + i)->last.yaw;
            //     x_pri(7,0) = 0;
            //     nSetState = true;
            // }
            if(x_pri(8,0) > MAX_R){
                x_pri(8,0) = MAX_R;
                nSetState = true;
            }
            else if(x_pri(8,0) < MIN_R){
                x_pri(8,0) = MIN_R;
                nSetState = true;
            }
            if((prePos - (robs + i)->last.absCenter).norm() >= MAX_MOVE_DIFF || (prePos-(robs + i)->center).norm() > MAX_R){
                nSetState = true;
                x_pri(0,0) = (robs + i)->last.absCenter.x() + (robs + i)->radius * cos((robs + i)->last.yaw);
                x_pri(1,0) = 0.0;
                x_pri(2,0) = (robs + i)->last.absCenter.y() + (robs + i)->radius * sin((robs + i)->last.yaw);
                x_pri(3,0) = 0.0;
                x_pri(4,0) = (robs + i)->last.absCenter.z();
                x_pri(5,0) = 0.0;
            }
            if(nSetState){
                (robs + i)->ekf->setState(x_pri);
                x_pri = (robs + i)->ekf->predict(frameTime);
                // (robs + i)->ekf->setPri(x_pri);
            }

            // Eigen::VectorXd z(4);
            // z << res.x(), res.y(), res.z(), x_pri(6,0); //根据预测结果解算状态信息
            // (robs + i)->ekf->update(z,frameTime);
            
            prePos = PredictCalc(*(robs + i),x_pri,foreTime,true);
            //更新机器人信息
            (robs + i)->center << x_pri(0,0),x_pri(2,0),prePos.z();
            (robs + i)->radius = x_pri(8,0);
            (robs + i)->yawVec = x_pri(7,0);

            x_pri(6,0) *= yRate;

            res.rob = (robs + i);
            cal->TFUpdata("map","center",(robs + i)->center,{0,0,0},1);
            res.aimCamPos = AfterC(mat,*(robs + i),x_pri,(robs + i)->last,imuData,prePos,frameTime,foreTime);
            res.centerCamPos = cal->GetPos("cam","center") * 1000.0;
            res.shootAble = ShootAble(res);
            // cal->TFUpdata("map","forecast",prePos,{0,0,0},1);
            // auto rb = cal->vtf.RebuildArmor(cal->GetPos("imu","forecast"),cal->GetPos("imu","center"),{15.0 * D2R,0.0,x_pri(6,0) + PI - imuData.GetYaw()});
            // int index = 4;
            // for(int i = 0;i < 4;i++){
            //     cal->TFUpdata("imu","armor" + std::to_string(i),rb[4][i],rb[5][i],index);
            //     index++;
            // }
            // res = (cal->GetPos("cam","forecast") * 1000.0);
            // cv::circle(mat,cal->ThreeToTwo(cal->OdomToCam((robs + i)->center)),10,cv::Scalar(255,255,255),3);
            // res = (cal->OdomToCam(prePos) * 1000.0);
            return true;
        }
        return false;
    }
};
#endif