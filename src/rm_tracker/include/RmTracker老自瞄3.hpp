#pragma once
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

#define RLIM 0.2
#define EDGE_MODE 0
#define SAFE_ANGLE 20.0 * D2R
#define TOWER_RADIUS 0.265
#define OVERSHOOT_LIMIT 0.15
#define OVERSHOOT 0.5
#define OVERSHOOT_RATE 0.3
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

    size_t changeTime = 0;
    Timestamp lastChangeTime = 0;

    char type = 'i'; //i 普通步兵 b 前哨 s 平步

    AverageCounter<30> __rav;
    AverageCounter<20> _rav;
    AverageCounter<10> rav;

    long highRPMCount = 0;

    bool hasLast = false;
    Eigen::Vector3d lastCenter;
    NArmor lastArmor;
    NArmor lastLastArmor;
    bool HaveLastPri = false;
    Eigen::Vector3d LastPri;
}Robot;
template<TimeSpan trackTime>
class Tracker{
    private:
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
        bool OverShooting = false;
        bool Aborting = false;
        bool Rolling = false;
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
            for(int i = 0;i < RobotCount;i++)
                Reset(*(Robots + i));
            nh.param("predict_s",PREDICT_RATE,PREDICT_RATE);
            nh.param("fa_test",FA,FA);
            return;
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
        Eigen::MatrixXd ErrorCase(Robot& rob,Eigen::MatrixXd pri,double frameTime,double foreTime,Eigen::Vector3d detPos,Eigen::Vector3d detPose,bool haveDet){
            auto prePos = PredictCalc(pri,foreTime);
            auto preCenter = PredictCalc2Center(pri,foreTime);
            // cv::circle(mat,cal->ThreeToTwo(cal->OdomToCam(prePos) * 1000),16,cv::Scalar(255,0,0),-1);
            //预测异常处理
            bool nSetState = false;
            if(TrackingID == 6){
                pri(7,0) = 0.8 * PI;
                pri(8,0) = TOWER_RADIUS;
                nSetState = true;
            }
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
            if(haveDet){
                double mdiff = (prePos - detPos).norm();
                if(mdiff>=0.5){
                    Eigen::Vector3d tCenter;
                    tCenter << detPos.x() + pri(8,0) * cos(detPose.z()),detPos.y() + pri(8,0) * sin(detPose.z()),detPos.z();
                    nSetState = true;
                    pri(0,0) = tCenter.x();
                    pri(1,0) = 0.0;
                    pri(2,0) = tCenter.y();
                    pri(3,0) = 0.0;
                    pri(4,0) = tCenter.z();
                    pri(5,0) = 0.0;
                    // if(mdiff >= 0.8){
                    //     // Reset(rob);
                    //     Aborting = true;
                    //     Log::Error("中心严重跳变!");
                    //     // return std::move(pri);
                    // }
                    // else
                    Log::Warn("中心跳变!");
                }
            }
            if(nSetState)
                rob.ekf->setState(pri);

            return std::move(pri);
        }
        Eigen::Vector3d Decision(Robot&rob,const Eigen::Vector3d& priPos,int rebuildCount,int& tfUPInd,double yaw,bool* result){
            *result = true;
            //重构
            auto rb = cal->vtf.RebuildOtherArmor(priPos,rob.center,{0.0,15.0*D2R,yaw},rob.dZ,rob.dRadius,rebuildCount);
            std::vector<Eigen::Vector3d>* target = NULL;
            double angDis = DOUBLE_MAX;
            //根据重构的yaw角来决定目标 [180 +- ATK_ANGLE]
            //多个目标取与180近的
            //判断旋转方向
            double rpm = rob.rav.GetAverageValue();
            std::vector<double> yaws;
            std::vector<Eigen::Vector3d> posesCam;
            for(int i = 0;i < rebuildCount;i++){
                cal->TFUpdata("map","armor" + std::to_string(i),rb[i][0],rb[i][1],tfUPInd++);
                double tangDis = (rb[i][1].z() - 2*PI);
                auto tpose = cal->GetPos("imu","armor" + std::to_string(i),1);
                yaws.push_back(fabs(tpose[1].z()));
            }
            if(yaws.size() != rebuildCount){
                Log::Error("Rebuild Error");
                *result = false;
                return priPos;
            }
            if(fabs(rpm) <= RLIM && rob.highRPMCount < 5){
                target = &rb[0];
            }
            else{
                double tranges[2] = {0,0};
                if(rpm < 0){
                    // tranges[0] = PI - ATTACK_ANGLE;
                    // tranges[1] = 3 * PI / 2 - ATTACK_ANGLE;
                    tranges[0] = PI - ATTACK_ANGLE;
                    tranges[1] = 3 * PI / 2;
                }
                else{
                    // tranges[0] = PI / 2 + ATTACK_ANGLE;
                    // tranges[1] = PI + ATTACK_ANGLE;
                    tranges[0] = PI / 2;
                    tranges[1] = PI + ATTACK_ANGLE;
                }
                int tindex = -1;
                for(int i = 0;i < rebuildCount;i++){
                    if(yaws[i] < tranges[0] || yaws[i] > tranges[1])
                        continue;
                    if(!~tindex){
                        tindex = i;
                    }
                    else{
                        if(rpm < 0 && yaws[i] > yaws[tindex])
                            tindex = i;
                        if(rpm > 0 && yaws[i] < yaws[tindex])
                            tindex = i;
                    }
                }
                if(EDGE_MODE || !~tindex){
                    if(rpm < 0)
                        if(EDGE_MODE)
                            target = &rb[rebuildCount - 1];
                        else
                            target = &rb[1];
                    else
                        if(EDGE_MODE)
                            target = &rb[1];
                        else
                            target = &rb[rebuildCount - 1];
                }
                else
                    target = &rb[tindex];
                // Log::Info("Selecting:" + std::to_string(tindex));
            }
            DebugMsg.yaws = yaws;
            //决策
            //如果没有目标 用识别目标
            if(!target || target->size() < 2){
                Log::Error("目标位姿获取失败!");
                *result = false;
                return priPos;
            }
            cal->TFUpdata("map","Target",(*target)[0],(*target)[1],tfUPInd++);
            return std::move((*target)[0]);
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
            if(fabs(-_imu.GetPitch() - ang.x()) >= SAFE_ANGLE || fabs(_imu.GetYaw() - ang.y()) >= SAFE_ANGLE)
                ang << -_imu.GetPitch()-0.001,_imu.GetYaw(),0;
            return std::move(ang);
        }
        Eigen::Vector3d EKFMaintain(Robot& rob,std::vector<Eigen::Vector3d> absPos,double frameTime,double foreTime,bool haveDet,bool changed,double* yaw,bool* result){
            *result = true;
            Eigen::VectorXd _updater(4);
            //是否有卡尔曼
            if(!rob.ekf){
                Reset(rob);
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
                    //计算center
                    rob.center << absPos[0].x() + radius * cos(absPos[1].z()),absPos[0].y() + radius * sin(absPos[1].z()),absPos[0].z();
                    //获取yaw
                    yaw = absPos[1].z();
                }
                //否则无法继续运行
                else{
                    // yaw = rob.lastArmor.absPose.z();
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
                    Reset(rob);
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
                //超射判断
                if(pri(3,0) > OVERSHOOT_LIMIT){
                    Log::Warn("Move speed over " + std::to_string(OVERSHOOT_LIMIT) + " m/s entry overshoot mode!");
                    OverShooting = true;
                }
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
                rob.dRadius = pri(8,0) - rob.radius[0];
                rob.dZ = pri(4,0) - rob.center.z();
                //存储
                rob.center = PredictCalc2Center(pri,foreTime);
                rob.radius[0] = pri(8,0);
                rob.radius[1] = rob.radius[0];

                DebugMsg.CenterX = pri(0,0);
                DebugMsg.CenterY = pri(2,0);
                DebugMsg.CenterZ = pri(4,0);
                DebugMsg.xV = pri(1,0);
                DebugMsg.yV = pri(3,0);
                DebugMsg.zV = pri(5,0);
                DebugMsg.EKFYaw = pri(6,0);
                // if(
                //     rob.HaveLastPri &&
                //     fabs(rob.LastPri(6,0) - pri(6,0)) <= 60.0 * D2R
                //     )
                //     {
                //         // rob.__rav.Updata((pri(6,0) - rob.LastPri(6,0))/frameTime);
                //         // rob._rav.Updata(rob.__rav.GetAverageValue());
                //         // rob.rav.Updata(rob.__rav.GetAverageValue());
                //         rob.rav.Updata((pri(6,0) - rob.LastPri(6,0))/frameTime);
                //     }   
                
                *yaw = pri(6,0);
                *result = true;
                rob.HaveLastPri = true;
                rob.LastPri = pri;
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
            if(!BulletVector)
                BulletVector = BULLET_VEC;
            imu.Release();
        }
        Eigen::Vector2d CalcAngle(Eigen::Vector3d pos){
            Eigen::Vector2d result;
            // bool a = pos.x() > 0,b = pos.y() > 0;
            result << acos(pos.x() / Eigen::Vector2d(pos.x(),pos.z()).norm()),atan2(pos.y(),pos.x());
            result.y() -= 1*D2R;
            // if(!a && b)
            //     result.y() += PI / 2;
            // else if(!a && !b)
            //     result.y() += PI;
            // else if(a && !b)
            //     result.y() += PI * 3 / 2;
            // result.y() -= (int)(result.y()/(PI * 2)) * PI * 2;
            // result.y() -= PI;
            auto timu = imu.GetWithoutLock();
            double diff = result.y() - timu.GetYaw();
            if(OverShooting)
                result.y() = diff * (1 + OVERSHOOT) + timu.GetYaw();
            return result;
        }
        bool ShootAble(Eigen::Vector3d pos,double pitch,double length){
            auto _imu = imu.GetWithoutLock();
            auto cpos = cal->ThreeToTwo(pos * 1000.0);
            return fabs(cpos.x - img.camera_center.x) <= length * SHOOT_RANGE_RATE && abs(-_imu.GetPitch() - pitch) <= 2.0*D2R * SHOOT_RANGE_RATE;
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
        void Track(rm_msgs::ArmorArrayConstPtr armors){    //回调函数 处理识别节点发来的话题
            // Log::Debug("Atk_Ang:" + std::to_string(ATTACK_ANGLE * R2D));
            if(Log::MinimumLogLevel == Log::Level::Debug){
                nh.param("predict_s",PREDICT_RATE,PREDICT_RATE);
                nh.param("fa_test",FA,FA);
            }
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
                            if(matchAnthor){
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
                    Eigen::Vector3d target;
                    //char -> int 方便逻辑判断
                    int trackId = TrackingID;
                    int flag = 0;
                    //不符合的id舍弃    [1,7]
                    if(trackId < 1 || trackId > 8){
                        Log::Error("错误的ID:" + std::to_string(trackId));
                        std::cerr << "Tracker->Track:获得错误ID:" << trackId << std::endl;
                        break;
                    }
                    Robot& rob = *(this->Robots + trackId - 1);
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
                    double compensation = -timu.GetPitch();
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
                        if( rob.hasLast &&
                            CurrentTime - rob.lastChangeTime >= 200 &&
                            abs((absPos[0] - rob.lastArmor.pos).norm()) > 0.10 &&
                            abs((absPos[0] - rob.lastLastArmor.pos).norm()) > 0.10 &&
                            abs(rob.lastArmor.absPose.z() - absPos[1].z()) > 40.0 * D2R
                        ){
                            //TODO:计算长短、高低
                            rob.changeTime += 1;
                            rob.lastChangeTime = CurrentTime;
                            changed = true;
                        }
                        //计算移动
                        double movement = 0.0;
                        if(rob.hasLast){
                            movement = (absPos[0] - rob.lastArmor.pos).norm();
                        }
                        //yaw角度修正
                        if(absPos[1].z() < 0)
                            absPos[1].z() += 2*PI;
                        DebugMsg.Yaw = absPos[1].z();
                        if(imuPos[1].z() < 0)
                            imuPos[1].z() += 2*PI;
                        DebugMsg.ImuYaw = imuPos[1].z();
                        absPos[1].z() += PI;

                        //计算前推时间 process + fly
                        compensation = cal->GetCompensation(absPos[0],BulletVector);
                        double foreTime = COMMAND_TIMESPAN + ftcr.GetAverageValue() + Eigen::Vector2d({absPos[0].x(),absPos[0].y()}).norm() / (BulletVector * cos(compensation));
                        foreTime /= 1000.0;
                        //维护卡尔曼并获取预测点
                        bool hasResult = false;
                        double yaw = 0.0;
                        auto priPos = EKFMaintain(rob,absPos,frameTime*OVERSHOOT_RATE,foreTime*(2 - OVERSHOOT_RATE),true,changed,&yaw,&hasResult);
                        if(!hasResult){
                            break;
                        }
                        if(movement >= 0.15)
                            finPos = Decision(rob,priPos,rebuildCount,tfUPInd,yaw,&haveFin);
                        else{
                            haveFin = true;
                            finPos = absPos[0];
                        }
                        
                        if(
                            rob.hasLast &&
                            // rob.HaveLastPri &&
                            fabs((absPos[0] - rob.lastArmor.pos).norm()) <= 0.3 &&
                            fabs((absPos[0] - rob.lastLastArmor.pos).norm()) <= 0.3
                            // fabs((rob.center - rob.lastCenter).norm()) <= 0.15
                            )
                            {
                                double rValue = ((absPos[0].y() - rob.lastArmor.pos.y()) - (rob.center.y() - rob.lastCenter.y()))/frameTime;
                                // double rValue = ((absPos[1].z() - rob.LastPri(6,0)))/frameTime;
                                if(!isnan(rValue)){
                                    rob.__rav.Updata(rValue);
                                    rob._rav.Updata(rob.__rav.GetAverageValue());
                                    rob.rav.Updata(rob._rav.GetAverageValue());
                                }
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
                            double foreTime = COMMAND_TIMESPAN + ftcr.GetAverageValue() + Eigen::Vector2d({rob.center.x(),rob.center.y()}).norm() / (BulletVector * cos(compensation));
                            foreTime /= 1000.0;
                            //维护卡尔曼并获取预测点
                            bool hasResult = false;
                            double yaw = 0.0;
                            std::vector<Eigen::Vector3d> tabsPos;
                            auto priPos = EKFMaintain(rob,tabsPos,frameTime*OVERSHOOT_RATE,foreTime*(2 - OVERSHOOT_RATE),false,false,&yaw,&hasResult);
                            if(!hasResult){
                                break;
                            }
                            finPos = Decision(rob,priPos,rebuildCount,tfUPInd,yaw,&haveFin);
                        }
                    }
                    if(Rolling){
                        Log::Info("Rolling:true\t" + std::to_string(rob.rav.GetAverageValue()) + "\t" + std::to_string(rob.highRPMCount));
                        if(rob.highRPMCount < 10)
                            rob.highRPMCount += 1;
                    }
                    else{
                        if(rob.highRPMCount > 0)
                            rob.highRPMCount -= 1;
                        Log::Info("Rolling:false\t" + std::to_string(rob.rav.GetAverageValue())+ "\t" + std::to_string(rob.highRPMCount));
                    }
                    if(haveFin){
                        auto aimAng = CalcAngle(finPos);
                        // if(TrackingID == 6)
                        //     cal->GetCompensation(rob.center,BulletVector);
                        // else
                        //     cal->GetCompensation(finPos,BulletVector);
                        // aimAng.x() = compensation;
                        // try{
                        //     if(detect){
                        //         if(ShootAble(cal->GetPos("cam","Target"),compensation,detect->rectWidth)){
                        //             flag |= 2;
                        //         }
                        //     }
                        //     else{
                        //         if(rob.hasLast && ShootAble(cal->GetPos("cam","Target"),compensation,rob.lastArmor.source.rectWidth)){
                        //             flag |= 2;
                        //         }
                        //     }
                        // }
                        // catch(...){
                        //     break;
                        // }
                        AimMsg << -aimAng.x(),aimAng.y(),flag;
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
                Reset(*(Robots + tempId));
                TrackingID = -1;
                break;
            }
            if(State == TrackerState::Starting)
                return;
            DebugPub.publish(DebugMsg);
            AimMsg = SafetyCaseAngle(AimMsg);
            AnglePub(AimMsg.x(),AimMsg.y(),AimMsg.z());
            ComputeTime = GetTimestamp();
            if(Aborting){
                State = TrackerState::Lost;
                int tempId = TrackingID;
                Reset(*(Robots + tempId));
                TrackingID = -1;
            }
            Log::Info("FPS:" + std::to_string(1000.0/ftcr.GetAverageValue()) + "\tFT:" + std::to_string(ftcr.GetAverageValue()));
        }
        TimeSpan GetTrackTime(){            //获取跟踪器 跟踪时长
            return trackTime;
        }
};