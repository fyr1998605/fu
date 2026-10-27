#include <ros/ros.h>
#include "RmIdentify.hpp"

// 神经网络识别
// 参数配置未测试
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "identify");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 创建私有节点句柄
    
    // 从参数服务器获取配置文件路径
    std::string config_path;
    private_nh.param<std::string>("armor_identify_config", config_path, "");

    std::string input_Amodel_path = "/home/rm/ws_glut_vison2711/Model/QX_model_v1_openvino/QX_model_v1.onnx";
    Identify::Config config_armour = { 0.2,0.4,0.4,640,640, input_Amodel_path};
    RmIdentify ri(nh,config_armour,config_path);
    ros::Subscriber serialSub = nh.subscribe("/RmSerialData", 1000, &RmIdentify::doserial, &ri);
    ros::Subscriber cameraSub = nh.subscribe("/hikrobot_camera/rgb", 1000, &RmIdentify::doimage, &ri);

    while(ros::ok())
    {
        // 批量处理所有待处理回调, 确保拿到最新图像和串口数据再处理
        for (int i = 0; i < 20 && ros::ok(); i++) {
            ros::spinOnce();
        }
        ri.ArmorChoose();
    }
    return 0;
}
