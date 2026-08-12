#include <ros/ros.h>
#include "new_Rmidentify.hpp"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// debug
#include <std_msgs/Float64.h>
// long long tool_begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//首时间
// long long tool_end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//尾时间
ros::Publisher debugpub2;            //debug发布者
std_msgs::Float64 debugdate2;        //debug数据
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 传统视觉识别
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "identify");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 创建私有节点句柄

    // 从参数服务器获取配置文件路径
    std::string config_path;
    private_nh.param<std::string>("armor_identify_config", config_path, "");
    
    // 筛选灯条的参数
    new_Identify::LightParams Lp;
    
    Lp.min_ratio=0.075; Lp.max_ratio=0.4;     // 长宽比范围 
    Lp.max_angle=35.0;                      // 最大倾斜角度
    Lp.min_fill_ratio = 0.5;                // 最小占比(灯条/旋转矩阵)

    // 筛选灯条配对结果的参数
    new_Identify::ArmorParams Ap;
    Ap.min_light_ratio=0.7;                 // 两灯条的最小长度之比(短边/长边)         
    Ap.min_small_center_distance=0.8; Ap.max_small_center_distance=3.2; // 装甲板灯条中心的距离范围(大装甲板)   
    Ap.min_large_center_distance=3.2; Ap.max_large_center_distance=5.5; // 装甲板灯条中心的距离范围(小装甲板)
    Ap.max_angle=35.0;                      // 装甲板的最大倾斜角度

    // new_Identify::ArmorParams Ap;
    // Ap.min_light_ratio=0.9;                 // 两灯条的最小长度之比(短边/长边)         
    // Ap.min_small_center_distance=0.8; Ap.max_small_center_distance=3.2; // 装甲板灯条中心的距离范围(大装甲板)   
    // Ap.min_large_center_distance=3.2; Ap.max_large_center_distance=5.5; // 装甲板灯条中心的距离范围(小装甲板)
    // Ap.max_angle=35.0;                      // 装甲板的最大倾斜角度

    new_Identify::Config config_armour = {Lp , Ap , 160 };
    RmIdentify ri(nh , config_armour,config_path);
    ros::Subscriber serialSub = nh.subscribe("/RmSerialData", 1, &RmIdentify::doserial, &ri);//惯性系->陀螺仪系->相机系和枪管系
    ros::Subscriber cameraSub = nh.subscribe("/hikrobot_camera/rgb", 1, &RmIdentify::doimage, &ri);//相机系->装甲板系

    ////////////////////////////////////////////////////////////////////
    // debug 发布
    // debugpub2 = nh.advertise<std_msgs::Float64>("/debugpub2", 1000);
    ///////////////////////////////////////////////////////////////////////

    while(ros::ok())
    {
        ri.ArmorChoose();
        ros::spinOnce();
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // tool_begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
        // double frame_time = (tool_begin_Time - tool_end_Time)/1000.0;
        // tool_end_Time = tool_begin_Time;
        // debugdate2.data = frame_time * 1000;
        // debugpub2.publish(debugdate2);
        // // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    return 0;
}

//   Detector::LightParams l_params = {
//     .min_ratio = declare_parameter("light.min_ratio", 0.1),
//     .max_ratio = declare_parameter("light.max_ratio", 0.4),
//     .max_angle = declare_parameter("light.max_angle", 35.0),
//     .min_fill_ratio = declare_parameter("light.min_fill_ratio", 0.8),
//   };

//   Detector::ArmorParams a_params = {
//     .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
//     .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
//     .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
//     .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
//     .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
//     .max_angle = declare_parameter("armor.max_angle", 35.0)};

