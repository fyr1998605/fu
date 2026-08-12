#pragma once
#include "ros/ros.h"
#include "rm_msgs/Armor.h"
#include "rm_msgs/ArmorArray.h"
#include "UTools.hpp"
#include "Variable.h"
#include "extended_kalman_filter.h"
#include "Log.hpp"
#include "Calculater.hpp"
#include "Locked.hpp"
#include "ImuData.hpp"
#include "rm_msgs/Debug.h"
#include <cmath>
#include "rm_msgs/RmSerial.h"

#define RLIM 0.01
#define EDGE_MODE 0
#define SAFE_ANGLE 45.0 * D2R
#define TOWER_RADIUS 0.265
#define OVERSHOOT_LIMIT 0.15
#define OVERSHOOT_RATE 1.0
#define OVER_DISTANCE 6.0
#define RESPAWN_TIME 3000
#define CHANGE_K 0.1

using namespace Tools;
enum class TrackerState{
    Starting    = 0x00, // 启动中
    Detecting   = 0x01, // 识别
    Tracking    = 0x02, // 跟踪
    Lost        = 0x04  // 丢失
};
typedef struct NArmor{
    rm_msgs::Armor source;
    Eigen::Vector3d pos;
    Eigen::Vector3d absPose;
    Eigen::Vector3d camPose;
    Eigen::Vector3d imuPose;
}NArmor;
typedef struct Robot{
    int id = -1;
    Fiter::EKF* ekf = NULL;
    Eigen::Vector3d center;
    double radius[2] = {BEGIN_R,BEGIN_R}; // 0 long 1 short
    rm_msgs::Armor armors[4]; 
    bool equalRadius = true;
    double angle = 0;
    double dRadius = 0.0; // 半径差
    double dZ = 0.0; // 高度差

    long changeTime = 0;
    Timestamp lastChangeTime = 0;

    char type = 'i'; //i 普通步兵 b 前哨 s 平步

    int current = 0;
    AverageCounter<20> __rav;
    int __timer = 0;
    AverageCounter<10> _rav;
    int _timer = 0;
    AverageCounter<10> rav;
    int kcount = 0;
    AverageCounter<10> avgk;
    
    char direction = 'l'; // s static r right l left
    int dChange = 0;
    double lyaw = 0.0;
    double lyawv = 0.0;
    long highRPMCount = 0;
    double lPYAW = 0;
    bool hasLast = false;
    Eigen::Vector3d lastCenter;
    Eigen::Vector3d osPos;
    Eigen::Vector3d osCenter;
    double osPose;
    NArmor lastArmor;
    NArmor lastLastArmor;
    bool HaveLastPri = false;
    Eigen::MatrixXd LastPri = MatrixXd(9,1);
}Robot;
template<TimeSpan _trackTime>
class Tracker{
    private:
        TimeSpan trackTime;
        double BulletVector = 29.5;
        bool Initialized;                   //是否已经初始化
        char TrackingID;                    //跟踪中的装甲板ID
        Timestamp ComputeTime;              //处理时间(上一帧处理时间戳)
        Timestamp LastErrorTime;            //上一次错误时间戳
        Timestamp LastTrackTime;            //上一次当前号码识别时间戳
        Timestamp LastDetectTime;           //上一次识别时间戳
        Robot* Robots;                      //存储机器人各种信息
        TrackerState State;
        CAL::Calculater* cal;
        ros::NodeHandle nh;
        Locked<ImuData> imu;
        ros::Publisher AngPub;
        ros::Publisher DebugPub;
        Image img;
        Eigen::Vector2d LightCenter;
        rm_msgs::Debug DebugMsg;
        float eTime = 0.009; //曝光时间
        bool Mode = true;
        bool OverShooting = false;
        bool Aborting = false;
        bool Rolling = false;
        Timestamp RespawnTime[8] = {0};
        uint8_t LastSurviving = 255;
        uint8_t Surviving = 255;
        void Build(ros::NodeHandle nh){
            this->nh = nh;
            Timestamp ctime = GetTimestamp();
            Initialized = false;
            TrackingID = ~0;//-1
            ComputeTime = ctime;
            LastErrorTime = 0;
            LastDetectTime = 0;
            LastTrackTime = 0;
            cal = new CAL::Calculater(nh,img);
            State = TrackerState::Starting;
            AngPub = nh.advertise<geometry_msgs::Vector3>("/auto_angle", 1000);
            DebugPub = nh.advertise<rm_msgs::Debug>("/debug", 1000);
            LightCenter << img.camera_center.x,img.camera_center.y;
            eTime = nh.param("ExposureTime",9000) / 1000000.0;
        }
    public:
        static const int RobotCount = 10;
        Tracker(ros::NodeHandle nh){                          //构造函数 初始化数据
            Build(nh);
        }
        ~Tracker(){                         //析构函数 释放数据
            for(int i = 0;i < RobotCount;i++)
                if((Robots + i)->ekf){
                    delete (Robots + i)->ekf;
                    (Robots + i)->ekf = NULL;
                }
            if(Robots){
                delete[] Robots;
                Robots = NULL;
            }
            if(cal){
                delete cal;
                cal = NULL;
            }
        }
        void AnglePub(double x,double y,double z){
            geometry_msgs::Vector3 angle;
            //角度限制
            angle.x = x;
            angle.y = y;
            angle.z = z;
            AngPub.publish(angle);
        }
        void Init(){                        //初始化函数
            if(Initialized)
                return;
            Initialized = true;
            Robots = new Robot[RobotCount];
            for(int i = 0;i < RobotCount;i++){
                (Robots + i)->ekf = new Fiter::EKF(nh);
                (Robots + i)->ekf->setParam();
            }
            //参数读取
            nh.param("predict_s",PREDICT_RATE,PREDICT_RATE);
            nh.param("fa_test",FA,FA);
            nh.param("atk_ang",ATTACK_ANGLE,ATTACK_ANGLE);
            nh.param("atk_range",SHOOT_RANGE_RATE,SHOOT_RANGE_RATE);
            nh.param("atk_ts",COMMAND_TIMESPAN,COMMAND_TIMESPAN);
            nh.param("overshoot",OVER_SHOOT_RATE,OVER_SHOOT_RATE);
            nh.param("MAX_R",MAX_R,MAX_R);
            nh.param("BEGIN_R",BEGIN_R,BEGIN_R);
            nh.param("MIN_R",MIN_R,MIN_R);
            nh.param("large_lim",LONG_ARMOR_LIMIT,LONG_ARMOR_LIMIT);
            nh.param("atk_vec",BULLET_VEC,BULLET_VEC);
            nh.param("yaw_gain",YAW_PREDICT_RATE,YAW_PREDICT_RATE);
            //输出参数
            cout << "predict_s:" << PREDICT_RATE << endl;
            cout << "fa_test:" << FA << endl;
            cout << "atk_ang:" << ATTACK_ANGLE << endl;
            cout << "atk_range:" << SHOOT_RANGE_RATE << endl;
            cout << "atk_ts:" << COMMAND_TIMESPAN << endl;
            cout << "overshoot:" << OVER_SHOOT_RATE << endl;
            cout << "MAX_R:" << MAX_R << endl;
            cout << "BEGIN_R:" << BEGIN_R << endl;
            cout << "MIN_R:" << MIN_R << endl;
            cout << "large_lim:" << LONG_ARMOR_LIMIT << endl;
            cout << "atk_vec:" << BULLET_VEC << endl;
            cout << "yaw_gain:" << YAW_PREDICT_RATE << endl;
            Sleep(3000);
            return;
        }
        static Eigen::Vector3d OverShoot(Eigen::MatrixXd& predict,double ft,double rate){
            Eigen::Vector3d result;
            double  xita =  predict(6,0) + (predict(7,0)*ft*PREDICT_RATE)*YAW_PREDICT_RATE * rate
                    ,x0 = predict(0,0)
                    ,y0 = predict(2,0)
                    ,r = predict(8,0);
            result << x0 - r * cos(xita) + predict(1,0) * ft*PREDICT_RATE * rate, y0 - r * sin(xita) + predict(3,0) * ft*PREDICT_RATE * rate, predict(4,0);
            // result << x0 - r * cos(xita), y0 - r * sin(xita), predict(4,0);
            return std::move(result);
        }
        static Eigen::Vector3d OverShootCenter(Eigen::MatrixXd& predict,double ft,double rate){
            Eigen::Vector3d result;
            double  x0 = predict(0,0)
                    ,y0 = predict(2,0);
            result << x0 + predict(1,0) * ft*PREDICT_RATE * rate, y0 + predict(3,0) * ft*PREDICT_RATE * rate, predict(4,0);
            // result << x0, y0, predict(4,0);
            return std::move(result);
        }
        static Eigen::Vector3d PredictCalc(Eigen::MatrixXd& predict,double ft){
            Eigen::Vector3d result;
            double  xita =  predict(6,0) + (predict(7,0)*ft*PREDICT_RATE)*YAW_PREDICT_RATE
                    ,x0 = predict(0,0)
                    ,y0 = predict(2,0)
                    ,r = predict(8,0);
            result << x0 - r * cos(xita) + predict(1,0) * ft*PREDICT_RATE, y0 - r * sin(xita) + predict(3,0) * ft*PREDICT_RATE, predict(4,0);
            // result << x0 - r * cos(xita), y0 - r * sin(xita), predict(4,0);
            return std::move(result);
        }
        static Eigen::Vector3d PredictCalc2Center(Eigen::MatrixXd& predict,double ft){
            Eigen::Vector3d result;
            double  x0 = predict(0,0)
                    ,y0 = predict(2,0);
            result << x0 + predict(1,0) * ft*PREDICT_RATE, y0 + predict(3,0) * ft*PREDICT_RATE, predict(4,0);
            // result << x0, y0, predict(4,0);
            return std::move(result);
        }
        bool TryCalCenter(Robot&rob){
            if(rob.hasLast){
                geometry_msgs::Pose p1;
                auto qtn1 = cal->Euler2QUaternion(0.0,-15.0,rob.lastLastArmor.absPose.z());
                p1.position.x = rob.lastLastArmor.pos.x();
                p1.position.y = rob.lastLastArmor.pos.y();
                p1.position.z = rob.lastLastArmor.pos.z();
                p1.orientation.x = qtn1.x();
                p1.orientation.y = qtn1.y();
                p1.orientation.z = qtn1.z();
                p1.orientation.w = qtn1.w();
                geometry_msgs::Pose p2;
                p2.position.x = rob.lastArmor.pos.x();
                p2.position.y = rob.lastArmor.pos.y();
                p2.position.z = rob.lastArmor.pos.z();
                auto qtn2 = cal->Euler2QUaternion(0.0,-15.0,rob.lastArmor.absPose.z());
                p2.orientation.x = qtn2.x();
                p2.orientation.y = qtn2.y();
                p2.orientation.z = qtn2.z();
                p2.orientation.w = qtn2.w();
                auto tcenter = cal->CalCenter(p1,p2);
                if(!SafetyCaseVec(tcenter))
                return false;
                rob.center = tcenter;
                double radius = (rob.center - rob.lastArmor.pos).norm();
                rob.radius[0] = radius;
                rob.radius[1] = radius;
                return true;
            }
            else
                return false;
        }
        Eigen::MatrixXd ErrorCase(Robot& rob,Eigen::MatrixXd pri,double frameTime,double foreTime,Eigen::Vector3d detPos,Eigen::Vector3d detPose,bool haveDet){
            auto prePos = PredictCalc(pri,foreTime);
            auto preCenter = PredictCalc2Center(pri,foreTime);
            // cv::circle(mat,cal->ThreeToTwo(cal->OdomToCam(prePos) * 1000),16,cv::Scalar(255,0,0),-1);
            //预测异常处理
            bool nSetState = false;
            if(pri(8,0) > MAX_R){
                Log::Warn("R跳变!");
                // if(rob.radius[0] >= MIN_R && rob.radius[0] <= MAX_R)
                //     pri(8,0) = rob.radius[0];
                // else{
                //     auto tradius = (rob.lastArmor.pos - rob.center).norm();
                //     if(tradius >= MIN_R && tradius <= MAX_R)
                //         pri(8,0) = tradius;
                //     else
                //         pri(8,0) = MAX_R;
                // }
                pri(8,0) = MAX_R;
                nSetState = true;
            }
            else if(pri(8,0) < MIN_R){
                Log::Warn("R跳变!");
                // if(rob.radius[0] >= MIN_R && rob.radius[0] <= MAX_R)
                //     pri(8,0) = rob.radius[0];
                // else{
                //     auto tradius = (rob.lastArmor.pos - rob.center).norm();
                //     if(tradius >= MIN_R && tradius <= MAX_R)
                //         pri(8,0) = tradius;
                //     else
                //         pri(8,0) = MIN_R; 
                // }
                pri(8,0) = MIN_R;
                nSetState = true;
            }
            if(haveDet && (prePos - detPos).norm() >= 0.6){
                Eigen::Vector3d tCenter;
                tCenter << detPos.x() + pri(8,0) * cos(detPose.z()),detPos.y() + pri(8,0) * sin(detPose.z()),detPos.z();
                nSetState = true;
                pri(0,0) = tCenter.x();
                pri(1,0) = 0.0;
                pri(2,0) = tCenter.y();
                pri(3,0) = 0.0;
                pri(4,0) = tCenter.z();
                pri(5,0) = 0.0;
                Log::Warn("卡尔曼纠错!");
                // pri = rob.ekf->predict(frameTime);
                // robot.ekf->setPri(x_pri);
            }
            if(TrackingID == 6){
                // if(rob.direction == 'r')
                //     pri(7,0) = 0.8 * PI;
                // else if(rob.direction == 'l')
                    pri(7,0) = 0.8 * PI;
                pri(8,0) = TOWER_RADIUS;
                nSetState = true;
            }
            if(nSetState)
                rob.ekf->setState(pri);

            return std::move(pri);
        }
        Eigen::Vector3d Decision(Robot&rob,const Eigen::Vector3d& priPos,int rebuildCount,int& tfUPInd,double yaw,bool* result){
            *result = true;
            //重构
            auto rb = cal->vtf.RebuildOtherArmor(priPos,rob.center,{0.0,15.0*D2R,yaw},rob.dZ,rob.dRadius,rebuildCount);
            auto osrb = cal->vtf.RebuildOtherArmor(rob.osPos,rob.osCenter,{0.0,15.0*D2R,rob.osPose},rob.dZ,rob.dRadius,rebuildCount);
            int target = -1;
            double angDis = DOUBLE_MAX;
            //根据重构的yaw角来决定目标 [180 +- ATK_ANGLE]
            //多个目标取与180近的
            //判断旋转方向
            double rpm = rob.rav.GetAverageValue();
            std::vector<double> yaws;
            std::vector<Eigen::Vector3d> posesCam;
            auto centerpose = cal->GetPos("Center","imu",1)[1].z();
            for(int i = 0;i < rebuildCount;i++){
                int ind = i;
                cal->TFUpdata("map","armor" + std::to_string(i),rb[i][0],rb[i][1],tfUPInd++);
                cal->TFUpdata("map","os_armor" + std::to_string(i),osrb[i][0],osrb[i][1],tfUPInd++);
                double tangDis = (rb[i][1].z() - 2*PI);
                auto tpose = cal->GetPos("Center","armor" + std::to_string(ind),1);
                yaws.push_back((tpose[1].z() + PI));
            }
            if(yaws.size() != rebuildCount){
                Log::Error("Rebuild Error");
                *result = false;
                return priPos;
            }
            // cout << yaws[0] << endl;
            if(fabs(rpm) <= RLIM && rob.highRPMCount < 5){
                target = 0;
            }
            else{
                double cErr = DOUBLE_MAX;
                for(int i = 0;i < rebuildCount;i++){
                    if(fabs(yaws[i] - centerpose) < cErr){
                        cErr = fabs(yaws[i] - centerpose);
                        target = i;
                    }
                }
                // double tranges[3] = {PI - 2*PI/rebuildCount - 10.0 * D2R,PI,PI + 2*PI/rebuildCount + 10.0 * D2R};
                // if(rpm < 0){
                //     // tranges[0] = PI - ATTACK_ANGLE;
                //     tranges[1] += ATTACK_ANGLE;
                //     if(yaws[0] >= tranges[0] && yaws[0] <= tranges[1])
                //         target = 0;
                //     else if(yaws[0] >= tranges[1] && yaws[0] <= tranges[2])
                //         target = rebuildCount - 1;
                //     else
                //         target = 1;
                // }
                // else{
                //     // tranges[0] = PI / 2 + ATTACK_ANGLE;
                //     // tranges[1] = PI + ATTACK_ANGLE;
                //     tranges[1] -= ATTACK_ANGLE;
                //     if(yaws[0] >= tranges[0] && yaws[0] <= tranges[1])
                //         target = 1;
                //     else if(yaws[0] >= tranges[1] && yaws[0] <= tranges[2])
                //         target = 0;
                //     else
                //         target = rebuildCount - 1;
                // }
            }
            DebugMsg.yaws = yaws;
            target = 0;
            //决策
            //如果没有目标 用识别目标
            if(!~target || rb[target].size() < 2 || osrb[target].size() < 2){
                Log::Error("目标位姿获取失败!");
                *result = false;
                return priPos;
            }
            cal->TFUpdata("map","Target",rb[target][0],rb[target][1],tfUPInd++);
            cal->TFUpdata("map","Overshoot",osrb[target][0],osrb[target][1],tfUPInd++);
            // return rob.center;
            return std::move(osrb[target][0]);
        }
        bool SafetyCaseVec(Eigen::Vector3d pri){
            return !(isnan(pri.x())
                || isnan(pri.y())
                || isnan(pri.z()));
        }
        bool SafetyCasePri(Eigen::MatrixXd pri){
            return !(isnan(pri(0,0))
                || isnan(pri(1,0))
                || isnan(pri(2,0))
                || isnan(pri(3,0))
                || isnan(pri(4,0))
                || isnan(pri(5,0))
                || isnan(pri(6,0))
                || isnan(pri(7,0))
                || isnan(pri(8,0)));
        }
        //自瞄角度限制
        Eigen::Vector3d SafetyCaseAngle(Eigen::Vector3d ang){
            auto _imu = imu.GetWithoutLock();
            if(fabs(_imu.GetPitch() - ang.x()) >= SAFE_ANGLE || fabs(_imu.GetYaw() - ang.y()) >= SAFE_ANGLE)
                ang << _imu.GetPitch(),_imu.GetYaw(),ang.z();
            return std::move(ang);
        }
        Eigen::Vector3d EKFMaintain(Robot& rob,std::vector<Eigen::Vector3d> absPos,std::vector<Eigen::Vector3d> imuPos,double frameTime,double foreTime,bool haveDet,bool& changed,double* yaw,bool* result){
            *result = true;
            Eigen::VectorXd _updater(4);
            //是否有卡尔曼
            if(!rob.ekf){
                rob.ekf = new Fiter::EKF(nh);
                rob.ekf->setParam();
            }
            //卡尔曼是否初始化
            //没有就初始化
            if(!rob.ekf->isUseUkF()){
                //使用机器人已经计算过的半径(可能是初始半径)
                //因为不确定是哪块所以半径取平均
                double radius = (rob.radius[0] + rob.radius[1]) / 2;
                double yaw = 0.0;
                //如果有识别 重新计算中心
                if(haveDet){
                    // 计算center
                    rob.center << absPos[0].x() + radius * cos(absPos[1].z()),absPos[0].y() + radius * sin(absPos[1].z()),absPos[0].z();
                    // 获取yaw
                    yaw = absPos[1].z();
                    // if(!TryCalCenter(rob)){
                    //     *result = false;
                    //     Log::Error("卡尔曼未初始化!");
                    //     return {0,0,0};
                    // }
                }
                //否则无法继续运行
                else{
                    *result = false;
                    Log::Error("卡尔曼未初始化!");
                    return {0,0,0};
                }
                //重设初始态
                Eigen::VectorXd startState;
                while(!rob.ekf->isUseUkF()){
                    Eigen::VectorXd state_z(9);
                    state_z << rob.center[0], 0, rob.center[1], 0, rob.center[2], 0, yaw, 0, radius;
                    rob.ekf->setState(state_z);
                }
            }
            //维护卡尔曼
            if(rob.ekf->isUseUkF()){
                //维护卡尔曼
                auto pri = rob.ekf->predict(frameTime);
                if(!SafetyCasePri(pri)){
                    *result = false;
                    if(rob.ekf)
                        delete rob.ekf;
                    rob.ekf = new Fiter::EKF(nh);
                    rob.ekf->setParam();
                    Log::Error("EKF nan!");
                    if(haveDet)
                        return absPos[0];
                    else
                        return {0,0,0};
                }
                //纠错
                if(haveDet)
                    pri = ErrorCase(rob,pri,frameTime,foreTime,absPos[0],absPos[1],true);
                else
                    pri = ErrorCase(rob,pri,frameTime,foreTime,{0,0,0},{0,0,0},false);
                //预测
                Eigen::Vector3d priPos = PredictCalc(pri,foreTime);
                Eigen::Vector3d osPos = OverShoot(pri,foreTime,OVER_SHOOT_RATE);
                //维护
                if(haveDet){
                    _updater << absPos[0].x(),absPos[0].y(),absPos[0].z(),absPos[1].z();
                    rob.ekf->update(_updater,frameTime);
                }
                // else{
                //     _updater << priPos.x(),priPos.y(),priPos.z(),pri(6,0);
                //     rob.ekf->update(_updater,frameTime);
                // }

                //长短轴和高低装甲板维护
                if(changed){
                    rob.dRadius = pri(8,0) - rob.radius[0];
                    rob.dZ = pri(4,0) - rob.center.z();
                    // changed = true;
                }
                //存储
                // if(!TryCalCenter(rob))
                rob.center = PredictCalc2Center(pri,foreTime);
                rob.radius[0] = pri(8,0);
                rob.radius[1] = rob.radius[0];
                rob.osCenter = OverShootCenter(pri,foreTime,OVER_SHOOT_RATE);
                rob.osPos = osPos;
                rob.osPose = pri(6,0) + (pri(7,0)*foreTime*PREDICT_RATE)*YAW_PREDICT_RATE * OVER_SHOOT_RATE;

                DebugMsg.CenterX = pri(0,0);
                DebugMsg.CenterY = pri(2,0);
                DebugMsg.CenterZ = pri(4,0);
                DebugMsg.xV = pri(1,0);
                DebugMsg.yV = pri(3,0);
                DebugMsg.zV = pri(5,0);
                DebugMsg.EKFYaw = pri(6,0);
                DebugMsg.K = rob.avgk.GetAverageValue();

                if(!haveDet)
                    DebugMsg.Yaw = pri(6,0) - PI;
                if(rob.HaveLastPri)
                    Log::Debug("YawVec:" + std::to_string((pri(6,0)) * R2D)  + "|" + std::to_string((rob.LastPri(6,0)) * R2D) + "|" + std::to_string((pri(6,0) - rob.LastPri(6,0)) * R2D));
                
                if(rob.HaveLastPri){
                    rob.avgk.Updata(pri(6,0) - rob.LastPri(6,0));
                }
                // if(fabs(rob.avgk.GetAverageValue()) > CHANGE_K)
                //     rob.kcount = 0;
                // else{
                //     if(rob.kcount == 3)
                //         changed = true;
                //     rob.kcount += 1;
                // }
                if(fabs(rob.LastPri(6,0) - pri(6,0)) > 25.0 * D2R && (haveDet && fabs((priPos - absPos[0]).norm()) > pri(8,0) / 2))
                    changed = true;

                if(
                    rob.HaveLastPri &&
                    !changed &&
                    fabs(rob.LastPri(6,0) - pri(6,0)) <= 25.0 * D2R &&
                    (haveDet && fabs((priPos - absPos[0]).norm()) <= 0.1)
                    )
                    {
                        rob.__rav.Updata(-(pri(6,0) - rob.LastPri(6,0)) * 20);
                        rob.__timer += 1;
                        if(rob.__timer == 2){
                            rob._rav.Updata(rob.__rav.GetAverageValue());
                            rob.__timer = 0;
                            rob._timer += 1;
                            if(rob._timer == 2){
                                rob.rav.Updata(rob.__rav.GetAverageValue());
                                rob._timer = 0;
                            }
                        }  
                        // rob.rav.Updata((pri(6,0) - rob.LastPri(6,0)));
                    }   
                // if(fabs(rob.LastPri(6,0) - pri(6,0)) > 25.0 * D2R && (haveDet && fabs((priPos - absPos[0]).norm()) > pri(8,0) / 2))
                
                *yaw = pri(6,0);
                *result = true;
                rob.HaveLastPri = true;
                for(int i = 0; i < 9;i++)
                    rob.LastPri(i,0) = pri(i,0);
                rob.lyawv = pri(7,0);
                return (priPos);
            }
            //否则无法继续运行
            else{
                // yaw = rob.lastArmor.absPose.z();
                *result = false;
                Log::Error("卡尔曼未初始化!");
                if(haveDet)
                    return absPos[0];
                else
                    return {0,0,0};
            }
        }
        bool AttackAble(int ind,Timestamp ct){
            if(ind <= 0 || ind >= 9)
                return false;
            return !IsRespawned(ind,ct) && IsAlive(ind);
        }
        bool AttackAble(int ind){return AttackAble(GetTimestamp());}
        void SetSerial(const rm_msgs::RmSerialConstPtr _serial){
            auto _imu = imu.Get();
            _imu->Qtn[0] = _serial->Qw;
            _imu->Qtn[1] = _serial->Qx;
            _imu->Qtn[2] = _serial->Qy;
            _imu->Qtn[3] = _serial->Qz;
            _imu->Pose[0] = _serial->Roll;
            _imu->Pose[1] = _serial->Pitch;
            _imu->Pose[2] = _serial->Yaw;
            BulletVector = _serial->BulletVec;
            Mode = true;
            // Surviving = _serial->Aliving;
            // cout << "-----------------------" << endl;
            Timestamp ctime = GetTimestamp();
            for(int i = 0;i < 8;i++){
                uint8_t mask = 1 << i;
                if((Surviving & mask) && !(LastSurviving & mask))
                    RespawnTime[i] = ctime;
                // cout <<i << ":" << (Surviving & mask) << " " << RespawnTime[i] << endl;
            }
            LastSurviving = Surviving;
            if(BulletVector <= 10.0)
                BulletVector = BULLET_VEC;
            imu.Release();
        }
        Eigen::Vector2d CalcAngle(Eigen::Vector3d pos){
            Eigen::Vector2d result;
            // bool a = pos.x() > 0,b = pos.y() > 0;
            result << acos(pos.x() / Eigen::Vector2d(pos.x(),pos.z()).norm()),atan2(pos.y(),pos.x());
            // result.y() += 1.5*D2R;
            // if(!a && b)
            //     result.y() += PI / 2;
            // else if(!a && !b)
            //     result.y() += PI;
            // else if(a && !b)
            //     result.y() += PI * 3 / 2;
            // result.y() -= (int)(result.y()/(PI * 2)) * PI * 2;
            // result.y() -= PI;
            return result;
        }
        bool ShootAble(Eigen::Vector3d pos,double pitch,double length){
            auto _imu = imu.GetWithoutLock();
            auto cpos = cal->ThreeToTwo(pos * 1000.0);
            
            Log::Debug("Shoot:" + std::to_string(fabs(cpos.x - img.camera_center.x)) + "|" + std::to_string(length * SHOOT_RANGE_RATE)
                        + "  " + std::to_string(fabs(-_imu.GetPitch() - pitch)) + "|" + std::to_string(2.0*D2R * SHOOT_RANGE_RATE));
            return fabs(cpos.x - img.camera_center.x) <= length * SHOOT_RANGE_RATE;
            // return fabs(cpos.x - img.camera_center.x) <= length * SHOOT_RANGE_RATE && fabs(_imu.GetPitch() - pitch) <= 2.0*D2R * SHOOT_RANGE_RATE;
            // return fabs(_imu.GetYaw() - yaw) <= asin((Half_Width_LONG / 1000.0) / pos.norm()) * SHOOT_RANGE_RATE && fabs(-_imu.GetPitch() - pitch) <= 2.0*D2R * SHOOT_RANGE_RATE;
        }
        void Reset(Robot& rob){
            
            if(rob.ekf)
                delete rob.ekf;
            rob.ekf = NULL;
            rob.ekf = new Fiter::EKF(nh);
            rob.ekf->setParam();
            rob.changeTime = 0;
            rob.hasLast = false;
            rob.HaveLastPri = false;
        }
        bool IsRespawned(int number,Timestamp t){return (t - RespawnTime[number - 1] <= RESPAWN_TIME);}
        bool IsRespawned(int number){return IsRespawned(number,GetTimestamp());}
        bool IsAlive(int number){return (Surviving & (1 << (number - 1)));}
        void Track(rm_msgs::ArmorArrayConstPtr armors){    //回调函数 处理识别节点发来的话题
            // Log::Debug("Atk_Ang:" + std::to_string(ATTACK_ANGLE * R2D));
            // if(!Mode){
            //     Sleep(100);
            //     return;
            // }
            if(Log::MinimumLogLevel == Log::Level::Debug){
                nh.param("predict_s",PREDICT_RATE,PREDICT_RATE);
                nh.param("fa_test",FA,FA);
                nh.param("atk_ang",ATTACK_ANGLE,ATTACK_ANGLE);
                nh.param("atk_range",SHOOT_RANGE_RATE,SHOOT_RANGE_RATE);
                nh.param("atk_ts",COMMAND_TIMESPAN,COMMAND_TIMESPAN);
                nh.param("overshoot",OVER_SHOOT_RATE,OVER_SHOOT_RATE);
                nh.param("MAX_R",MAX_R,MAX_R);
                nh.param("BEGIN_R",BEGIN_R,BEGIN_R);
                nh.param("MIN_R",MIN_R,MIN_R);
                nh.param("large_lim",LONG_ARMOR_LIMIT,LONG_ARMOR_LIMIT);
                nh.param("atk_vec",BULLET_VEC,BULLET_VEC);
                nh.param("yaw_gain",YAW_PREDICT_RATE,YAW_PREDICT_RATE);
            }
            if(TrackingID == 6)
                trackTime = _trackTime * 2.0;
            else
                trackTime = _trackTime;
            Aborting = false;
            OverShooting = false;
            Rolling = false;
            Eigen::Vector3d AimMsg;
            DebugMsg.header.frame_id = "map";
            DebugMsg.header.stamp = ros::Time::now();
            DebugMsg.Yaw = 0;
            DebugMsg.EKFYaw = 0;
            auto timu = imu.GetWithoutLock();
            AimMsg << timu.GetPitch(),timu.GetYaw(),0;
            fcr.Count();
            Timestamp CurrentTime = GetTimestamp();
            double frameTime = (ComputeTime - CurrentTime)/1000.0;
            ftcr.Updata(CurrentTime - ComputeTime);
            //初始变量声明
            rm_msgs::Armor* detect = NULL;                   //识别的装甲板
            if(State != TrackerState::Starting){
                if(armors->armors.size())
                    State = TrackerState::Detecting;
                else{
                    if(CurrentTime - LastDetectTime > trackTime)
                        State = TrackerState::Lost;
                    else
                        State = TrackerState::Tracking;
                }
            }
            //根据信息维护跟踪器
            //如果装甲板id为-1则没有识别
            //反之就是有识别
            switch (State)
            {
            case TrackerState::Starting:
                Log::Info("------------- Start [" + std::to_string(fcr.COUNT()) + "] -------------");
                //启动中 初始化参数
                Init();
                LastDetectTime = 0;
                State = TrackerState::Lost;
                return;
            case TrackerState::Detecting:
                Log::Info("------------- Detect[" + std::to_string(fcr.COUNT()) + "] -------------");
                //有识别 优先跟踪上一次跟踪的目标
                //没有则跟踪距离光心最近的
                {
                    double minDisToCenter = DOUBLE_MAX;
                    bool matchAnthor = (double)(CurrentTime - LastTrackTime) > (trackTime * 0.25);
                    if(matchAnthor)
                        Log::Warn("Lost target over 25% track time!");
                    for(auto& armor:armors->armors){
                        if(Eigen::Vector3d({armor.armorPose.position.x,armor.armorPose.position.y,armor.armorPose.position.z}).norm() >= OVER_DISTANCE)
                            continue;
                        //优先
                        if(armor.number == 6){
                            if(armor.armorToCenterDis < minDisToCenter){
                                detect = const_cast<rm_msgs::Armor*>(&armor);
                                minDisToCenter = armor.armorToCenterDis;
                            }
                            continue;
                        }
                        //前哨存活不识别哨兵
                        // else if((armor.number == 7) || armor.number == 2)
                        //     continue;
                        if(armor.number == TrackingID){
                            if(!detect){
                                detect = const_cast<rm_msgs::Armor*>(&armor);
                                minDisToCenter = armor.armorToCenterDis;
                            }
                            else if(armor.armorToCenterDis < minDisToCenter){
                                detect = const_cast<rm_msgs::Armor*>(&armor);
                                minDisToCenter = armor.armorToCenterDis;
                            }
                        }
                        else if(matchAnthor && armor.armorToCenterDis < minDisToCenter){
                            detect = const_cast<rm_msgs::Armor*>(&armor);
                            minDisToCenter = armor.armorToCenterDis;
                        }
                    }
                    if(detect){
                        // TrackingID = 1;
                        // detect->number = 1;
                        LastDetectTime = CurrentTime;
                        //触发切换
                        //号码不对 但是有目标
                        if(detect->number != TrackingID){
                            //如果允许匹配其他号码
                            //切换跟踪号码
                            if(matchAnthor && detect->number != 9){
                                if(TrackingID >= 1 && TrackingID <= 8){
                                    int tempId = TrackingID;
                                    Log::Warn("维护ID切换!");
                                    //重设卡尔曼参数
                                    if((Robots + tempId)->ekf){
                                        delete (Robots + tempId)->ekf;
                                        (Robots + tempId)->ekf = NULL;
                                    }
                                    (Robots + tempId)->ekf = new Fiter::EKF(nh);
                                    (Robots + tempId)->ekf->setParam();
                                }
                                TrackingID = detect->number;
                            }
                            //否则不切换且更新追踪时间
                            else
                            {
                                if(detect->number == 9)
                                {
                                    detect->number = TrackingID;
                                    LastTrackTime = CurrentTime;
                                }
                                else{
                                    detect = NULL;
                                    // LastTrackTime = CurrentTime;
                                    }
                            }
                        }
                        else{
                            LastTrackTime = CurrentTime;
                        }
                    }
                    else{
                        //matchAnthor && !detect
                        if((CurrentTime - LastTrackTime) > (trackTime)){
                            State = TrackerState::Lost;
                            break;
                        }
                    }
                }
            case TrackerState::Tracking:
                if(State == TrackerState::Tracking){
                    Log::Info("------------- Track [" + std::to_string(fcr.COUNT()) + "] -------------");
                    bool matchAnthor = (double)(CurrentTime - LastTrackTime) > (trackTime * 0.25);
                    if(matchAnthor){
                        double per = round((double)(CurrentTime - LastTrackTime)/(trackTime * 1.0) * 100);
                        if(per <100)
                            Log::Warn("Lost target over " + std::to_string(per) + "% track time!");
                        else{
                            Log::Error("Target almost lost!");
                            State = TrackerState::Lost;
                            break;
                        }
                    }
                }
            case (TrackerState)(-1):
                //有无识别的共有部分
                {
                    //不可击打判断
                    Eigen::Vector3d target;
                    //char -> int 方便逻辑判断
                    int trackId = TrackingID;
                    int flag = 0;
                    //不符合的id舍弃    [1,7]
                    if(trackId < 1 || trackId > 8){
                        TrackingID = -1;
                        Log::Error("错误的ID:" + std::to_string(trackId));
                        std::cerr << "Tracker->Track:获得错误ID:" << trackId << std::endl;
                        break;
                    }
                    Robot& rob = *(this->Robots + trackId - 1);
                    DebugMsg.CurrentID = rob.current;
                    if(rob.type == 's')
                        Log::Debug("Tracking:" + std::to_string(trackId)+"(2 Armors)");
                    else if(rob.type == 'b')
                        Log::Debug("Tracking:" + std::to_string(trackId)+"(3 Armors)");
                    else
                        Log::Debug("Tracking:" + std::to_string(trackId)+"(4 Armors)");
                    //重构装甲板数量
                    int rebuildCount = 4;
                    if(rob.type == 's')
                        rebuildCount = 2;
                    else if(rob.type == 'b')
                        rebuildCount = 3;

                    int tfUPInd = 0;
                    Eigen::MatrixXd pri(9,1);
                    Eigen::Vector3d finPos = {0,0,0};
                    bool haveFin = false;
                    double compensation = timu.GetPitch();
                    if(!AttackAble(TrackingID,CurrentTime)){
                        Log::Warn("ID:" + std::to_string(TrackingID) + " was respawning or dead!");
                        Reset(rob);
                        State = TrackerState::Lost;
                        break;
                    }
                    if(detect){
                        bool changed = false;
                        flag = 1;
                        //更新机器人类型
                        if(TrackingID >= 3 && TrackingID <= 5 && detect->type == 'l')
                            rob.type = 's';
                        else if(TrackingID == 6)
                            rob.type = 'b';
                        else
                            rob.type = 'i';
                        if(rob.type == 's')
                            rebuildCount = 2;
                        else if(rob.type == 'b')
                            rebuildCount = 3;

                        auto thisPose = cal->MyQUaternion2Euler(detect->armorPose.orientation);
                        DebugMsg.Yaw = thisPose.z();
                        Eigen::Vector3d lastPose({0,0,0});

                        //上传目标的TF
                        cal->TFUpdata("cam","DETECT",cal->Point2Vector(detect->armorPose.position),thisPose,tfUPInd++);
                        //获取pos[0]和pose[1]
                        auto absPos = cal->GetPos("map","DETECT",1);
                        auto imuPos = cal->GetPos("imu", "DETECT", 1);
                        NArmor narmor;
                        narmor.source = *detect;
                        narmor.pos = absPos[0];
                        narmor.absPose = absPos[1];
                        narmor.camPose = thisPose;
                        DebugMsg.ArmorX = absPos[0].x();
                        DebugMsg.ArmorY = absPos[0].y();
                        DebugMsg.ArmorZ = absPos[0].z();
                        
                        //判断装甲板切换
                        // cout << rob.direction << endl;
                        if( rob.hasLast &&
                            CurrentTime - rob.lastChangeTime >= 200 &&
                            fabs((absPos[0] - rob.lastArmor.pos).norm()) >= 0.10 &&
                            fabs((absPos[0] - rob.lastLastArmor.pos).norm()) >= 0.10 &&
                            fabs(rob.lastArmor.absPose.z() - absPos[1].z()) >= 40.0 * D2R
                        ){
                            //TODO:计算长短、高低
                            if(absPos[0].y() - rob.lastArmor.pos.y() > 0){
                                rob.changeTime += 1;
                                if(rob.direction != 'r' && rob.dChange < 3)
                                    rob.dChange += 1;
                                else if(rob.direction == 'r')
                                    rob.dChange = 0;
                                else{
                                    rob.direction = 'r';
                                    rob.dChange = 0;
                                }
                            }
                            else if(absPos[0].y() - rob.lastArmor.pos.y() < 0){
                                rob.changeTime -= 1;
                                if(rob.direction != 'l' && rob.dChange < 3)
                                    rob.dChange += 1;
                                else if(rob.direction == 'l')
                                    rob.dChange = 0;
                                else{
                                    rob.direction = 'l';
                                    rob.dChange = 0;
                                }
                            }
                            rob.lastChangeTime = CurrentTime;
                            // changed = true;
                        }
                        Log::Debug("Dire:" + std::to_string(absPos[0].y() - rob.lastArmor.pos.y())+"|" + std::to_string(rob.dChange));
                        //计算移动
                        //yaw角度修正
                        if(absPos[1].z() < 0)
                            absPos[1].z() += 2*PI;
                        DebugMsg.Yaw = absPos[1].z();
                        if(imuPos[1].z() < 0)
                            imuPos[1].z() += 2*PI;
                        DebugMsg.ImuYaw = imuPos[1].z();
                        absPos[1].z() += PI;
// absPos[1].z() += rob.changeTime * 2*PI/rebuildCount;
                        // if(rob.changeTime % rebuildCount){
                        //     int ctemp = rob.changeTime % rebuildCount;
                        //     // if(rebuildCount == 4){
                        //     //     if(ctemp == 0)
                        //     // }
                        //     // else
                        //         absPos[0] << rob.center.x() - rob.radius[0] * cos(absPos[1].z()),rob.center.y() - rob.radius[0] * sin(absPos[1].z()),absPos[0].z();
                        // }

                        //计算前推时间 process + fly
                        compensation = cal->GetCompensation(absPos[0],BulletVector);
                        // double foreTime = COMMAND_TIMESPAN + ftcr.GetAverageValue() + eTime + Eigen::Vector2d({absPos[0].x(),absPos[0].y()}).norm() / (BulletVector * cos(compensation));
                        double foreTime = COMMAND_TIMESPAN + ftcr.GetAverageValue()/1000.0 + eTime + absPos[0].norm() / BulletVector;
                        Log::Debug("Fore:" + std::to_string(foreTime) + "|" + std::to_string(absPos[0].norm() / BulletVector));
                        //维护卡尔曼并获取预测点
                        bool hasResult = false;
                        double yaw = 0.0;
                        auto priPos = EKFMaintain(rob,absPos,imuPos,frameTime*OVERSHOOT_RATE,foreTime*(2 - OVERSHOOT_RATE),true,changed,&yaw,&hasResult);
                        if(changed){
                            if(rob.current == rebuildCount - 1)
                                rob.current = 0;
                            else
                                rob.current += 1;
                        }
                        double rpm = rob.rav.GetAverageValue();
                        if(fabs(rpm) <= RLIM)
                            Rolling = false;
                        else
                            Rolling = true;                
                        //TODO
                        if(rob.hasLast){
                            rob.lastLastArmor = rob.lastArmor;
                            rob.lastArmor = narmor;
                        }
                        else{
                            rob.lastArmor = narmor;
                            rob.hasLast = true;
                        }

                        if(!hasResult){
                            break;
                        }
                        if(Rolling)
                            finPos = Decision(rob,priPos,rebuildCount,tfUPInd,yaw,&haveFin);
                        else{
                            haveFin = true;
                            finPos = absPos[0];
                            // rob.direction = 's';
                        }
                    }
                    else{
                        flag = 1;
                        //卡尔曼是否初始化
                        //没有就退出
                        if(!rob.ekf->isUseUkF()){
                            Log::Error("卡尔曼未初始化!");
                            std::cerr << "卡尔曼未初始化!" << std::endl;
                            break;
                        }
                        //维护卡尔曼
                        else{
                            //计算前推时间 process + fly
                            double compensation = cal->GetCompensation(rob.center,BulletVector);
                            // double foreTime = COMMAND_TIMESPAN + ftcr.GetAverageValue() + eTime + Eigen::Vector2d({rob.center.x(),rob.center.y()}).norm() / (BulletVector * cos(compensation));
                            double foreTime = COMMAND_TIMESPAN + ftcr.GetAverageValue()/1000.0 + eTime + rob.center.norm() / BulletVector;
                            Log::Debug("Fore:" + std::to_string(foreTime) + "|" + std::to_string(rob.center.norm() / BulletVector));
                            //维护卡尔曼并获取预测点
                            bool hasResult = false,changed;
                            double yaw = 0.0;
                            std::vector<Eigen::Vector3d> tabsPos;
                            auto priPos = EKFMaintain(rob,tabsPos,tabsPos,frameTime*OVERSHOOT_RATE,foreTime*(2 - OVERSHOOT_RATE),false,changed,&yaw,&hasResult);
                            if(!hasResult){
                                break;
                            }
                            if(changed){
                                if(rob.current == rebuildCount - 1)
                                    rob.current = 0;
                                else
                                    rob.current += 1;
                            }
                            finPos = Decision(rob,priPos,rebuildCount,tfUPInd,yaw,&haveFin);
                        }
                    }
                    if(Rolling){
                        Log::Debug("Rolling:true\t" + std::to_string(rob.rav.GetAverageValue()) + "\t" + std::to_string(rob.highRPMCount));
                        if(rob.highRPMCount < 10)
                            rob.highRPMCount += 1;
                    }
                    else{
                        if(rob.highRPMCount > 0)
                            rob.highRPMCount -= 1;
                        Log::Debug("Rolling:false\t" + std::to_string(rob.rav.GetAverageValue())+ "\t" + std::to_string(rob.highRPMCount));
                    }
                    Log::Debug("Direction:"+std::to_string(rob.direction));
                    // auto tempPose = cal->GetPos("armor0","DETECT",1);
                    // double temppvec = (tempPose[1].z() - rob.lPYAW)/frameTime;
                    // if(fabs(temppvec) <= 90.0*D2R && fabs(temppvec) >= 2.0*D2R)
                    // {
                    //     // rob.__rav.Updata(temppvec);
                    //     // rob.__timer += 1;
                    //     // if(rob.__timer == 3){
                    //         rob._rav.Updata(temppvec);
                    //         rob.__timer = 0;
                    //         rob._timer += 1;
                    //         if(rob._timer == 3){
                    //             rob.rav.Updata(rob.__rav.GetAverageValue());
                    //             rob._timer = 0;
                    //         }
                    //     // }  
                    //     // rob.rav.Updata((pri(6,0) - rob.LastPri(6,0)));
                    
                    // }
                    // rob.lPYAW = tempPose[1].z();
                    // DebugMsg.pyaw = tempPose[1].z();
                    if(haveFin){
                        auto aimAng = CalcAngle(finPos);
                        if(TrackingID == 6){
                            compensation = cal->GetCompensation(rob.center,BulletVector);
                            aimAng = CalcAngle(rob.center);
                        }
                        else
                            compensation = cal->GetCompensation(finPos,BulletVector);
                        aimAng.x() = compensation;
                        try{
                            if(detect){
                            // if(detect){
                                // if(ShootAble(cal->GetPos("cam","Overshoot"),compensation,detect->rectWidth)){
                                if(ShootAble(cal->GetPos("cam","Overshoot"),compensation,detect->rectWidth)){
                                    flag |= 2;
                                }
                                // if(TrackingID == 6){
                                //     flag = 1;
                                //     if(ShootAble(cal->GetPos("cam","armor0"),aimAng.y(),compensation,detect->rectWidth)||ShootAble(cal->GetPos("cam","armor1"),aimAng.y(),compensation,detect->rectWidth))
                                //         flag |= 2;
                                // }
                            }
                            // else
                            //     flag = 1;
                            else{
                                if(rob.hasLast && ShootAble(cal->GetPos("cam","Overshoot"),compensation,rob.lastArmor.source.rectWidth)){
                                    flag |= 2;
                                }
                            }
                        }
                        catch(...){
                            break;
                        }
                        AimMsg << aimAng.x(),aimAng.y(),flag;
                    }
                    rob.lastCenter = rob.center;
                    cal->TFUpdata("map","Center",rob.center,{0,0,0},tfUPInd++);
                    DebugMsg.Radius = rob.radius[0];
                    DebugMsg.ChangeTime = rob.changeTime;
                    DebugMsg.Rebuild = rebuildCount;
                    DebugMsg.RPM = rob.rav.GetAverageValue();
                }
                break;
            case TrackerState::Lost:
                Log::Info("------------- Lost  [" + std::to_string(fcr.COUNT()) + "] -------------");
                //重置参数
                if(!~TrackingID)
                    break;
                Log::Debug("重设ID:" + (TrackingID));
                int tempId = TrackingID;
                if((Robots + tempId)->ekf){
                    delete (Robots + tempId)->ekf;
                    (Robots + tempId)->ekf = NULL;
                } 
                TrackingID = -1;
                break;
            }
            if(State == TrackerState::Starting)
                return;
            DebugPub.publish(DebugMsg);
            AimMsg = SafetyCaseAngle(AimMsg);
            AnglePub(AimMsg.x(),AimMsg.y(),AimMsg.z());
            ComputeTime = GetTimestamp();
            Log::Info("FPS:" + std::to_string(1000.0/ftcr.GetAverageValue()) + "\tFT:" + std::to_string(ftcr.GetAverageValue()));
        }
        TimeSpan GetTrackTime(){            //获取跟踪器 跟踪时长
            return trackTime;
        }
};