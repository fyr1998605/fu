#ifndef IMUDATA_HPP
#define IMUDATA_HPP

class ImuData {
public:
    double Qtn[4] = {0};  // 四元数
    double Pose[3] = {0}; // 姿态角: Roll, Pitch, Yaw

    // 获取俯仰角
    double GetPitch() const {
        return Pose[1];
    }

    // 获取偏航角
    double GetYaw() const {
        return Pose[2];
    }

    // 获取横滚角
    double GetRoll() const {
        return Pose[0];
    }
};

#endif // IMUDATA_HPP