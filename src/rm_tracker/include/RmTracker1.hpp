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
#include "blade_tracker.hpp"
#include <opencv2/core.hpp>   // 使用 cv::Point2f
#include <array>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// 能量机关相关常量
#define YAW_PREDICT_RATE 1.0
const double COMMAND_TIMESPAN_GLOBAL = 0.2;   // 原宏改为常量（类内用成员变量）

#define RLIM 0.2
#define EDGE_MODE 0
#define SAFE_ANGLE 20.0 * D2R
#define TOWER_RADIUS 0.265
#define OVERSHOOT_LIMIT 0.15
#define OVERSHOOT 0.5
#define OVERSHOOT_RATE 0.3
using namespace Tools;

enum class TrackerState {
    Starting = 0x00,
    Detecting = 0x01,
    Tracking = 0x02,
    Lost = 0x04
};
enum class TrackTargetMode {
    ArmorMode,
    EnergyFanMode
};

typedef struct NArmor {
    rm_msgs::Armor source;
    Eigen::Vector3d pos;
    Eigen::Vector3d absPose;
    Eigen::Vector3d camPose;
} NArmor;

typedef struct Robot {
    int id = -1;
    Fiter::EKF* ekf = NULL;
    Eigen::Vector3d center;
    double radius[2] = {BEGIN_R, BEGIN_R};
    rm_msgs::Armor armors[4];
    bool equalRadius = true;
    double angle = 0;
    double dRadius = 0.0;
    double dZ = 0.0;
    size_t changeTime = 0;
    Timestamp lastChangeTime = 0;
    char type = 'i';
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
} Robot;

// YoloResult 结构体
struct YoloResult {
    cv::Point2f center;   // 修正：使用 cv::Point2f
    double score;
    int class_id;
};
typedef std::vector<YoloResult> YoloResults;

// ====================================================================
// 模板类 Tracker (所有成员函数均在类内定义)
// ====================================================================
template<TimeSpan trackTime>
class Tracker {
public:
    // ---------- 公有成员变量 ----------
    bool local_fan_force = true;
    static const int RobotCount = 10;
    void ImageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
    // 图像回调：如果暂不需要处理图像，可留空，或保存图像供YOLO使用
    // 例如：frame_img_ = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    }

    void SetSerial(const rm_msgs::RmSerialConstPtr _serial) {
        auto _imu = imu.Get();
        _imu->Qtn[0] = _serial->Qw;
        _imu->Qtn[1] = _serial->Qx;
        _imu->Qtn[2] = _serial->Qy;
        _imu->Qtn[3] = _serial->Qz;
        _imu->Pose[0] = _serial->Roll;
        _imu->Pose[1] = _serial->Pitch;
        _imu->Pose[2] = _serial->Yaw;
        BulletVector = _serial->BulletVec;
        if (!BulletVector) BulletVector = BULLET_VEC;
        imu.Release();
    }

    // ---------- 构造函数 / 析构函数 ----------
    Tracker(ros::NodeHandle nh) { Build(nh); }

    ~Tracker() {
        for (int i = 0; i < RobotCount; i++)
            if ((Robots + i)->ekf) {
                delete (Robots + i)->ekf;
                (Robots + i)->ekf = NULL;
            }
        if (Robots) { delete[] Robots; Robots = NULL; }
        if (cal) { delete cal; cal = NULL; }
    }

    // ---------- 公有成员函数 ----------
    
    void doArmors(const rm_msgs::ArmorArrayConstPtr& armors) { Track(armors); }
    void doimage(const sensor_msgs::ImageConstPtr& img_msg) { ImageCallback(img_msg); }
    

    void AnglePub(double x, double y, double z) {
        geometry_msgs::Vector3 angle;
        angle.x = x; angle.y = y; angle.z = z;
        AngPub.publish(angle);
    }

    void Init() {
        if (Initialized) return;
        Initialized = true;
        Robots = new Robot[RobotCount];
        for (int i = 0; i < RobotCount; i++)
            Reset(*(Robots + i));
        nh.param("predict_s", PREDICT_RATE, PREDICT_RATE);
        nh.param("fa_test", FA, FA);
    }

 
    TimeSpan GetTrackTime() { return trackTime; }

    // ---------- 主跟踪函数 ----------
   

private:
    // ---------- 私有成员变量 ----------
    ros::Subscriber img_sub_;
    cv::Mat frame_img_;
    double COMMAND_TIMESPAN;
    double BulletVector = 29.5;          // 只保留一个

    bool Initialized = false;
    char TrackingID = -1;
    Timestamp ComputeTime;
    Timestamp LastErrorTime = 0;
    Timestamp LastTrackTime = 0;
    Timestamp LastDetectTime = 0;
    Robot* Robots = nullptr;
    TrackerState State = TrackerState::Starting;
    CAL::Calculater* cal = nullptr;
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

    EnhancedMultiBladeTracker fan_tracker_;
    TrackTargetMode track_mode_ = TrackTargetMode::ArmorMode;
    std::vector<cv::Point2f> detect_blade_pts_;
    cv::Point2f fan_circle_center_;
    cv::Point2f target_blade_pt_;
    bool fan_need_reset_ekf_ = false;

    // ---------- 私有成员函数 ----------
    void Build(ros::NodeHandle nh) {
        this->nh = nh;
        Timestamp ctime = GetTimestamp();
        Initialized = false;
        TrackingID = ~0;
        ComputeTime = ctime;
        LastErrorTime = 0;
        LastDetectTime = 0;
        LastTrackTime = 0;
        cal = new CAL::Calculater(nh, img);
        State = TrackerState::Starting;
        AngPub = nh.advertise<geometry_msgs::Vector3>("/auto_angle", 1000);
        DebugPub = nh.advertise<rm_msgs::Debug>("/debug", 1000);
        LightCenter << img.camera_center.x, img.camera_center.y;

        double atk_ts = 0.2;
        nh.param("atk_ts", atk_ts, 0.2);
        COMMAND_TIMESPAN = atk_ts;

        fan_tracker_ = EnhancedMultiBladeTracker(100.0f);
        img_sub_ = nh.subscribe("/camera/image", 1, &Tracker::ImageCallback, this);
    }

    static Eigen::Vector3d PredictCalc(Eigen::MatrixXd& predict, double ft) {
        Eigen::Vector3d result;
        double xita = predict(6,0) + (predict(7,0)*ft*PREDICT_RATE);
        double x0 = predict(0,0);
        double y0 = predict(2,0);
        double r = predict(8,0);
        result << x0 - r * cos(xita) + predict(1,0) * ft*PREDICT_RATE,
                  y0 - r * sin(xita) + predict(3,0) * ft*PREDICT_RATE,
                  predict(4,0);
        return result;
    }

    static Eigen::Vector3d PredictCalc2Center(Eigen::MatrixXd& predict, double ft) {
        Eigen::Vector3d result;
        double x0 = predict(0,0), y0 = predict(2,0);
        result << x0 + predict(1,0) * ft*PREDICT_RATE,
                  y0 + predict(3,0) * ft*PREDICT_RATE,
                  predict(4,0);
        return result;
    }

    Eigen::MatrixXd ErrorCase(Robot& rob, Eigen::MatrixXd pri, double frameTime,
                              double foreTime, Eigen::Vector3d detPos,
                              Eigen::Vector3d detPose, bool haveDet) {
        auto prePos = PredictCalc(pri, foreTime);
        auto preCenter = PredictCalc2Center(pri, foreTime);
        bool nSetState = false;
        if (TrackingID == 6) {
            pri(7,0) = 0.8 * PI;
            pri(8,0) = TOWER_RADIUS;
            nSetState = true;
        }
        if (pri(8,0) > MAX_R) {
            Log::Warn("R跳变!");
            pri(8,0) = MAX_R;
            nSetState = true;
        } else if (pri(8,0) < MIN_R) {
            Log::Warn("R跳变!");
            pri(8,0) = MIN_R;
            nSetState = true;
        }
        if (haveDet) {
            double mdiff = (prePos - detPos).norm();
            if (mdiff >= 0.5) {
                Eigen::Vector3d tCenter;
                tCenter << detPos.x() + pri(8,0) * cos(detPose.z()),
                           detPos.y() + pri(8,0) * sin(detPose.z()),
                           detPos.z();
                nSetState = true;
                pri(0,0) = tCenter.x();
                pri(1,0) = 0.0;
                pri(2,0) = tCenter.y();
                pri(3,0) = 0.0;
                pri(4,0) = tCenter.z();
                pri(5,0) = 0.0;
                Log::Warn("中心跳变!");
            }
        }
        if (nSetState)
            rob.ekf->setState(pri);
        return pri;
    }

    Eigen::Vector3d Decision(Robot& rob, const Eigen::Vector3d& priPos,
                             int rebuildCount, int& tfUPInd, double yaw, bool* result) {
        *result = true;
        auto rb = cal->vtf.RebuildOtherArmor(priPos, rob.center,
                                             {0.0, 15.0*D2R, yaw},
                                             rob.dZ, rob.dRadius, rebuildCount);
        std::vector<Eigen::Vector3d>* target = NULL;
        double rpm = rob.rav.GetAverageValue();
        std::vector<double> yaws;
        std::vector<Eigen::Vector3d> posesCam;
        for (int i = 0; i < rebuildCount; i++) {
            cal->TFUpdata("map", "armor" + std::to_string(i), rb[i][0], rb[i][1], tfUPInd++);
            auto tpose = cal->GetPos("imu", "armor" + std::to_string(i), 1);
            yaws.push_back(fabs(tpose[1].z()));
        }
        if (yaws.size() != rebuildCount) {
            Log::Error("Rebuild Error");
            *result = false;
            return priPos;
        }
        if (fabs(rpm) <= RLIM && rob.highRPMCount < 5) {
            target = &rb[0];
        } else {
            double tranges[2] = {0,0};
            if (rpm < 0) {
                tranges[0] = PI - ATTACK_ANGLE;
                tranges[1] = 3 * PI / 2;
            } else {
                tranges[0] = PI / 2;
                tranges[1] = PI + ATTACK_ANGLE;
            }
            int tindex = -1;
            for (int i = 0; i < rebuildCount; i++) {
                if (yaws[i] < tranges[0] || yaws[i] > tranges[1])
                    continue;
                if (!~tindex) {
                    tindex = i;
                } else {
                    if (rpm < 0 && yaws[i] > yaws[tindex])
                        tindex = i;
                    if (rpm > 0 && yaws[i] < yaws[tindex])
                        tindex = i;
                }
            }
            if (EDGE_MODE || !~tindex) {
                if (rpm < 0)
                    target = (EDGE_MODE ? &rb[rebuildCount - 1] : &rb[1]);
                else
                    target = (EDGE_MODE ? &rb[1] : &rb[rebuildCount - 1]);
            } else {
                target = &rb[tindex];
            }
        }
        DebugMsg.yaws = yaws;
        if (!target || target->size() < 2) {
            Log::Error("目标位姿获取失败!");
            *result = false;
            return priPos;
        }
        cal->TFUpdata("map", "Target", (*target)[0], (*target)[1], tfUPInd++);
        return (*target)[0];
    }

    bool SafetyCasePri(Eigen::MatrixXd pri) {
        return !(isnan(pri(0,0)) || isnan(pri(1,0)) || isnan(pri(2,0)) ||
                 isnan(pri(3,0)) || isnan(pri(4,0)) || isnan(pri(5,0)) ||
                 isnan(pri(6,0)) || isnan(pri(7,0)) || isnan(pri(8,0)));
    }

    Eigen::Vector3d SafetyCaseAngle(Eigen::Vector3d ang) {
        auto _imu = imu.GetWithoutLock();
        if (fabs(-_imu.GetPitch() - ang.x()) >= SAFE_ANGLE ||
            fabs(_imu.GetYaw() - ang.y()) >= SAFE_ANGLE)
            ang << -_imu.GetPitch() - 0.001, _imu.GetYaw(), 0;
        return ang;
    }

    Eigen::Vector3d EKFMaintain(Robot& rob, std::vector<Eigen::Vector3d> absPos,
                                double frameTime, double foreTime, bool haveDet,
                                bool changed, double* yaw, bool* result) {
        *result = true;
        Eigen::VectorXd _updater(4);
        if (!rob.ekf) Reset(rob);
        if (!rob.ekf->isUseUkF()) {
            double radius = (rob.radius[0] + rob.radius[1]) / 2;
            double yaw_ = 0.0;
            if (haveDet) {
                rob.center << absPos[0].x() + radius * cos(absPos[1].z()),
                              absPos[0].y() + radius * sin(absPos[1].z()),
                              absPos[0].z();
                yaw_ = absPos[1].z();
            } else {
                *result = false;
                Log::Error("卡尔曼未初始化!");
                return {0,0,0};
            }
            while (!rob.ekf->isUseUkF()) {
                Eigen::VectorXd state_z(9);
                state_z << rob.center[0], 0, rob.center[1], 0, rob.center[2], 0, yaw_, 0, radius;
                rob.ekf->setState(state_z);
            }
        }
        if (rob.ekf->isUseUkF()) {
            auto pri = rob.ekf->predict(frameTime);
            if (!SafetyCasePri(pri)) {
                *result = false;
                Reset(rob);
                Log::Error("EKF nan!");
                return haveDet ? absPos[0] : Eigen::Vector3d{0,0,0};
            }
            if (haveDet)
                pri = ErrorCase(rob, pri, frameTime, foreTime, absPos[0], absPos[1], true);
            else
                pri = ErrorCase(rob, pri, frameTime, foreTime, {0,0,0}, {0,0,0}, false);

            Eigen::Vector3d priPos = PredictCalc(pri, foreTime);
            if (pri(3,0) > OVERSHOOT_LIMIT) {
                Log::Warn("Move speed over " + std::to_string(OVERSHOOT_LIMIT) + " m/s entry overshoot mode!");
                OverShooting = true;
            }
            if (haveDet) {
                _updater << absPos[0].x(), absPos[0].y(), absPos[0].z(), absPos[1].z();
                rob.ekf->update(_updater, frameTime);
            }
            rob.dRadius = pri(8,0) - rob.radius[0];
            rob.dZ = pri(4,0) - rob.center.z();
            rob.center = PredictCalc2Center(pri, foreTime);
            rob.radius[0] = pri(8,0);
            rob.radius[1] = rob.radius[0];

            DebugMsg.CenterX = pri(0,0);
            DebugMsg.CenterY = pri(2,0);
            DebugMsg.CenterZ = pri(4,0);
            DebugMsg.xV = pri(1,0);
            DebugMsg.yV = pri(3,0);
            DebugMsg.zV = pri(5,0);
            DebugMsg.EKFYaw = pri(6,0);

            *yaw = pri(6,0);
            *result = true;
            rob.HaveLastPri = true;
            rob.LastPri = pri;
            return priPos;
        } else {
            *result = false;
            Log::Error("卡尔曼未初始化!");
            return haveDet ? absPos[0] : Eigen::Vector3d{0,0,0};
        }
    }

    Eigen::Vector2d CalcAngle(Eigen::Vector3d pos) {
        Eigen::Vector2d result;
        result << acos(pos.x() / Eigen::Vector2d(pos.x(), pos.z()).norm()),
                  atan2(pos.y(), pos.x());
        result.y() -= 1*D2R;
        auto timu = imu.GetWithoutLock();
        double diff = result.y() - timu.GetYaw();
        if (OverShooting)
            result.y() = diff * (1 + OVERSHOOT) + timu.GetYaw();
        return result;
    }

    bool ShootAble(Eigen::Vector3d pos, double pitch, double length) {
        auto _imu = imu.GetWithoutLock();
        auto cpos = cal->ThreeToTwo(pos * 1000.0);
        return fabs(cpos.x - img.camera_center.x) <= length * SHOOT_RANGE_RATE &&
               abs(-_imu.GetPitch() - pitch) <= 2.0*D2R * SHOOT_RANGE_RATE;
    }

    void Reset(Robot& rob) {
        if (rob.ekf) delete rob.ekf;
        rob.ekf = new Fiter::EKF(nh);
        rob.ekf->setParam();
        rob.changeTime = 0;
        rob.hasLast = false;
        rob.HaveLastPri = false;
    }

public:
    // 在类内定义 Track 函数（因为使用了 goto, 必须放在类内）
    void Track(rm_msgs::ArmorArrayConstPtr armors) {
        if (local_fan_force) {
            track_mode_ = TrackTargetMode::EnergyFanMode;
        }
        if (Log::MinimumLogLevel == Log::Level::Debug) {
            nh.param("predict_s", PREDICT_RATE, PREDICT_RATE);
            nh.param("fa_test", FA, FA);
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
        AimMsg << timu.GetPitch(), timu.GetYaw(), 0;
        fcr.Count();
        Timestamp CurrentTime = GetTimestamp();
        double frameTime = (ComputeTime - CurrentTime) / 1000.0;
        ftcr.Updata(CurrentTime - ComputeTime);
        rm_msgs::Armor* detect = NULL;

        // ---------- 能量机关模式 ----------
        if (track_mode_ == TrackTargetMode::EnergyFanMode) {
            detect_blade_pts_.clear();
            fan_circle_center_ = cv::Point2f(0.f, 0.f);
            bool valid_circle = armors->has_fan_target;
            for (int i = 0; i < (int)armors->blade_xs.size(); i++) {
                cv::Point2f pt(armors->blade_xs[i], armors->blade_ys[i]);
                detect_blade_pts_.push_back(pt);
            }
            if (valid_circle) {
                fan_circle_center_.x = armors->circle_x;
                fan_circle_center_.y = armors->circle_y;
            }
            bool fan_ok = fan_tracker_.update(
                detect_blade_pts_,
                fan_circle_center_,
                target_blade_pt_,
                fan_need_reset_ekf_
            );
            Eigen::Vector3d fan_world_pos;
            if (fan_ok && valid_circle) {
                double dx = target_blade_pt_.x - fan_circle_center_.x;
                double dy = target_blade_pt_.y - fan_circle_center_.y;
                double angle = atan2(dy, dx);
                fan_world_pos << 1.0, 0.0, 0.5;
                Eigen::Vector2d aim_xy = CalcAngle(fan_world_pos);
                AimMsg << -aim_xy.x(), aim_xy.y(), 2;
                State = TrackerState::Tracking;
            } else {
                State = TrackerState::Lost;
                Log::Warn("能量机关：无有效扇叶");
            }
            goto publish_angle_step;
        }

        // ---------- 装甲模式 ----------
        if (State != TrackerState::Starting) {
            if (armors->armors.size())
                State = TrackerState::Detecting;
            else {
                if (CurrentTime - LastDetectTime > trackTime)
                    State = TrackerState::Lost;
                else
                    State = TrackerState::Tracking;
            }
        }

        switch (State) {
            case TrackerState::Starting:
                Log::Info("------------- Start [" + std::to_string(fcr.COUNT()) + "] -------------");
                Init();
                LastDetectTime = 0;
                State = TrackerState::Lost;
                return;
            case TrackerState::Detecting:
                Log::Info("------------- Detect[" + std::to_string(fcr.COUNT()) + "] -------------");
                {
                    double minDisToCenter = DOUBLE_MAX;
                    bool matchAnthor = (double)(CurrentTime - LastTrackTime) > (trackTime * 0.25);
                    if (matchAnthor) Log::Warn("Lost target over 25% track time!");
                    for (auto& armor : armors->armors) {
                        if (armor.number == TrackingID) {
                            if (!detect) {
                                detect = const_cast<rm_msgs::Armor*>(&armor);
                                minDisToCenter = armor.armorToCenterDis;
                            } else if (armor.armorToCenterDis < minDisToCenter) {
                                detect = const_cast<rm_msgs::Armor*>(&armor);
                                minDisToCenter = armor.armorToCenterDis;
                            }
                        } else if (matchAnthor && armor.armorToCenterDis < minDisToCenter) {
                            detect = const_cast<rm_msgs::Armor*>(&armor);
                            minDisToCenter = armor.armorToCenterDis;
                        }
                    }
                    if (detect) {
                        LastDetectTime = CurrentTime;
                        if (detect->number != TrackingID) {
                            if (matchAnthor) {
                                if (TrackingID >= 1 && TrackingID <= 8) {
                                    int tempId = TrackingID;
                                    Log::Warn("维护ID切换!");
                                    if ((Robots + tempId)->ekf) {
                                        delete (Robots + tempId)->ekf;
                                        (Robots + tempId)->ekf = NULL;
                                    }
                                    (Robots + tempId)->ekf = new Fiter::EKF(nh);
                                    (Robots + tempId)->ekf->setParam();
                                }
                                TrackingID = detect->number;
                            } else {
                                if (detect->number == 9) {
                                    detect->number = TrackingID;
                                } else {
                                    detect = NULL;
                                }
                            }
                        } else {
                            LastTrackTime = CurrentTime;
                        }
                    } else {
                        if ((CurrentTime - LastTrackTime) > (trackTime)) {
                            State = TrackerState::Lost;
                            break;
                        }
                    }
                }
                // fall through
            case TrackerState::Tracking:
                if (State == TrackerState::Tracking) {
                    Log::Info("------------- Track [" + std::to_string(fcr.COUNT()) + "] -------------");
                    bool matchAnthor = (double)(CurrentTime - LastTrackTime) > (trackTime * 0.25);
                    if (matchAnthor) {
                        double per = round((double)(CurrentTime - LastTrackTime) / (trackTime * 1.0) * 100);
                        if (per < 100)
                            Log::Warn("Lost target over " + std::to_string(per) + "% track time!");
                        else {
                            Log::Error("Target almost lost!");
                            State = TrackerState::Lost;
                            break;
                        }
                    }
                }
                // fall through
            case (TrackerState)(-1): {
                int trackId = TrackingID;
                int flag = 0;
                if (trackId < 1 || trackId > 8) {
                    Log::Error("错误的ID:" + std::to_string(trackId));
                    std::cerr << "Tracker->Track:获得错误ID:" << trackId << std::endl;
                    break;
                }
                Robot& rob = *(this->Robots + trackId - 1);
                int rebuildCount = 4;
                if (rob.type == 's') rebuildCount = 2;
                else if (rob.type == 'b') rebuildCount = 3;

                int tfUPInd = 0;
                Eigen::Vector3d finPos = {0,0,0};
                bool haveFin = false;
                double compensation = -timu.GetPitch();

                if (detect) {
                    bool changed = false;
                    flag = 1;
                    if (TrackingID >= 3 && TrackingID <= 5 && detect->type == 'l')
                        rob.type = 's';
                    else if (TrackingID == 6)
                        rob.type = 'b';
                    else
                        rob.type = 'i';
                    if (rob.type == 's') rebuildCount = 2;
                    else if (rob.type == 'b') rebuildCount = 3;

                    auto thisPose = cal->MyQUaternion2Euler(detect->armorPose.orientation);
                    DebugMsg.Yaw = thisPose.z();
                    cal->TFUpdata("cam", "DETECT", cal->Point2Vector(detect->armorPose.position), thisPose, tfUPInd++);
                    auto absPos = cal->GetPos("map", "DETECT", 1);
                    auto imuPos = cal->GetPos("imu", "DETECT", 1);
                    NArmor narmor;
                    narmor.source = *detect;
                    narmor.pos = absPos[0];
                    narmor.absPose = absPos[1];
                    narmor.camPose = thisPose;
                    DebugMsg.ArmorX = absPos[0].x();
                    DebugMsg.ArmorY = absPos[0].y();
                    DebugMsg.ArmorZ = absPos[0].z();

                    if (rob.hasLast && CurrentTime - rob.lastChangeTime >= 200 &&
                        abs((absPos[0] - rob.lastArmor.pos).norm()) > 0.10 &&
                        abs((absPos[0] - rob.lastLastArmor.pos).norm()) > 0.10 &&
                        abs(rob.lastArmor.absPose.z() - absPos[1].z()) > 40.0 * D2R) {
                        rob.changeTime += 1;
                        rob.lastChangeTime = CurrentTime;
                        changed = true;
                    }
                    double movement = 0.0;
                    if (rob.hasLast) movement = (absPos[0] - rob.lastArmor.pos).norm();

                    if (absPos[1].z() < 0) absPos[1].z() += 2*PI;
                    DebugMsg.Yaw = absPos[1].z();
                    if (imuPos[1].z() < 0) imuPos[1].z() += 2*PI;
                    DebugMsg.ImuYaw = imuPos[1].z();
                    absPos[1].z() += PI;

                    compensation = cal->GetCompensation(absPos[0], BulletVector);
                    Eigen::Vector2d pos2d(absPos[0].x(), absPos[0].y());   // 修正初始化
                    double foreTime = this->COMMAND_TIMESPAN + ftcr.GetAverageValue() +
                                      pos2d.norm() / (BulletVector * cos(compensation));
                    foreTime /= 1000.0;

                    bool hasResult = false;
                    double yaw = 0.0;
                    auto priPos = EKFMaintain(rob, absPos, frameTime*OVERSHOOT_RATE,
                                              foreTime*(2 - OVERSHOOT_RATE), true, changed, &yaw, &hasResult);
                    if (!hasResult) break;

                    if (movement >= 0.15)
                        finPos = Decision(rob, priPos, rebuildCount, tfUPInd, yaw, &haveFin);
                    else {
                        haveFin = true;
                        finPos = absPos[0];
                    }

                    if (rob.hasLast &&
                        fabs((absPos[0] - rob.lastArmor.pos).norm()) <= 0.3 &&
                        fabs((absPos[0] - rob.lastLastArmor.pos).norm()) <= 0.3) {
                        double rValue = ((absPos[0].y() - rob.lastArmor.pos.y()) - (rob.center.y() - rob.lastCenter.y())) / frameTime;
                        if (!isnan(rValue)) {
                            rob.__rav.Updata(rValue);
                            rob._rav.Updata(rob.__rav.GetAverageValue());
                            rob.rav.Updata(rob._rav.GetAverageValue());
                        }
                    }
                    double rpm = rob.rav.GetAverageValue();
                    if (fabs(rpm) <= RLIM) Rolling = false;
                    else Rolling = true;

                    if (rob.hasLast) {
                        rob.lastLastArmor = rob.lastArmor;
                        rob.lastArmor = narmor;
                    } else {
                        rob.lastArmor = narmor;
                        rob.hasLast = true;
                    }
                } else {
                    flag = 1;
                    if (!rob.ekf->isUseUkF()) {
                        Log::Error("卡尔曼未初始化!");
                        std::cerr << "卡尔曼未初始化!" << std::endl;
                        break;
                    } else {
                        double compensation2 = cal->GetCompensation(rob.center, BulletVector);
                        // 注意：此处原代码使用 absPos，但 detect 为空，需用 rob.center 构造位置
                        Eigen::Vector2d pos2d(rob.center.x(), rob.center.y());
                        double foreTime = this->COMMAND_TIMESPAN + ftcr.GetAverageValue() +
                                          pos2d.norm() / (BulletVector * cos(compensation2));
                        foreTime /= 1000.0;
                        bool hasResult = false;
                        double yaw = 0.0;
                        std::vector<Eigen::Vector3d> tabsPos;
                        auto priPos = EKFMaintain(rob, tabsPos, frameTime*OVERSHOOT_RATE,
                                                  foreTime*(2 - OVERSHOOT_RATE), false, false, &yaw, &hasResult);
                        if (!hasResult) break;
                        finPos = Decision(rob, priPos, rebuildCount, tfUPInd, yaw, &haveFin);
                    }
                }

                if (Rolling) {
                    Log::Info("Rolling:true\t" + std::to_string(rob.rav.GetAverageValue()) + "\t" + std::to_string(rob.highRPMCount));
                    if (rob.highRPMCount < 10) rob.highRPMCount += 1;
                } else {
                    if (rob.highRPMCount > 0) rob.highRPMCount -= 1;
                    Log::Info("Rolling:false\t" + std::to_string(rob.rav.GetAverageValue()) + "\t" + std::to_string(rob.highRPMCount));
                }

                if (haveFin) {
                    auto aimAng = CalcAngle(finPos);
                    AimMsg << -aimAng.x(), aimAng.y(), flag;
                }
                rob.lastCenter = rob.center;
                cal->TFUpdata("map", "Center", rob.center, {0,0,0}, tfUPInd++);
                DebugMsg.Radius = rob.radius[0];
                DebugMsg.ChangeTime = rob.changeTime;
                DebugMsg.Rebuild = rebuildCount;
                DebugMsg.RPM = rob.rav.GetAverageValue();
                break;
            }
            case TrackerState::Lost:
                Log::Info("------------- Lost  [" + std::to_string(fcr.COUNT()) + "] -------------");
                fan_tracker_.reset();
                if (!~TrackingID) break;
                Log::Debug("重设ID:" + (TrackingID));
                int tempId = TrackingID;
                Reset(*(Robots + tempId));
                TrackingID = -1;
                break;
        }

        if (State == TrackerState::Starting) return;

    publish_angle_step:
        DebugPub.publish(DebugMsg);
        AimMsg = SafetyCaseAngle(AimMsg);
        AnglePub(AimMsg.x(), AimMsg.y(), AimMsg.z());
        ComputeTime = GetTimestamp();
        if (Aborting) {
            State = TrackerState::Lost;
            int tempId = TrackingID;
            Reset(*(Robots + tempId));
            TrackingID = -1;
        }
        Log::Info("FPS:" + std::to_string(1000.0/ftcr.GetAverageValue()) + "\tFT:" + std::to_string(ftcr.GetAverageValue()));
    }

}; // 类定义结束（放在最后）