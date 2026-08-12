#include <ros/ros.h>
#include "Variable.h"
#include <chrono>
#include "RmTracker.hpp"  
#define TTIME 500
Tracker<TTIME>* tracker;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string config_path;
    private_nh.param<std::string>("armor_tracker_config", config_path, "");
    bool force_energy_fan = false;  // 默认听从串口切换, 可通过 launch 参数覆盖
    private_nh.param<bool>("force_energy_fan", force_energy_fan, false);

    tracker = new Tracker<TTIME>(nh, config_path);

    tracker->setLocalFanForce(force_energy_fan);
    if (force_energy_fan) {
        tracker->setTrackMode(TrackTargetMode::EnergyFanMode);
    }

    ros::Subscriber armors = nh.subscribe("/armor_array", 1000, &Tracker<TTIME>::doArmors, tracker);
    ros::Subscriber serial = nh.subscribe("/RmSerialData", 1, &Tracker<TTIME>::SetSerial, tracker);
    ros::Subscriber cameraSub = nh.subscribe("/hikrobot_camera/rgb", 1, &Tracker<TTIME>::doimage, tracker);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        tracker->Track();
        rate.sleep();
    }

    if (tracker != nullptr) {
        delete tracker;
        tracker = nullptr;
    }
    return 0;
}