#ifndef PUBURDF_H
#define PUBURDF_H
#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <vector>

// 老登遗留: 工具库: 现在未经过使用和测试
// 仅供参考
using namespace std;
class PubURDF
{
public:
    PubURDF(ros::NodeHandle &nh);
    void PubModel(Eigen::Vector3d point, double roll, double pitch, double yaw);
    void PubPointStamp(double roll, double pitch, double yaw);
    void PubPoseArray(vector<Eigen::Vector3d> euler, vector<Eigen::Vector3d> point);
    void PubTF(std::vector<Eigen::Vector3d> pos,std::vector<Eigen::Vector3d> rpy);
    ~PubURDF();
private:
    ros::Publisher PubPoint;
    ros::Publisher pubPoseArray;
    tf2_ros::TransformBroadcaster static_broadcaster;
    tf2_ros::TransformBroadcaster static_broadcaster1;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::TransformBroadcaster broadcaster1;
    tf2_ros::TransformBroadcaster broadcaster2;
    tf2_ros::TransformBroadcaster broadcaster3;

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped transformStamped0;
    geometry_msgs::TransformStamped transformStamped1;
    tf2::Quaternion quat;
    Eigen::Vector3d T;
};
PubURDF::PubURDF(ros::NodeHandle &nh)
{
    PubPoint = nh.advertise<geometry_msgs::PointStamped>("showdata", 1000);
    pubPoseArray = nh.advertise<geometry_msgs::PoseArray>("cloudPoint", 1000);

    // transformStamped0.header.frame_id = "armour"; // Parent frame ID
    // transformStamped0.child_frame_id = "zj1"; // Child frame ID
    // transformStamped0.transform.translation.x = -0.01895; // Translation X
    // transformStamped0.transform.translation.y = -0.0524956; // Translation Y
    // transformStamped0.transform.translation.z = -0.019698; // Translation Z
    // transformStamped0.transform.rotation.x = 0; // Rotation X
    // transformStamped0.transform.rotation.y = 0; // Rotation Y
    // transformStamped0.transform.rotation.z = 0; // Rotation Z
    // transformStamped0.transform.rotation.w = 1; // Rotation W        

    // transformStamped1.header.frame_id = "armour"; // Parent frame ID
    // transformStamped1.child_frame_id = "zj2"; // Child frame ID
    // transformStamped1.transform.translation.x = -0.01895; // Translation X
    // transformStamped1.transform.translation.y = 0.042504; // Translation Y
    // transformStamped1.transform.translation.z = -0.019698; // Translation Z
    // transformStamped1.transform.rotation.x = 0; // Rotation X
    // transformStamped1.transform.rotation.y = 0; // Rotation Y
    // transformStamped1.transform.rotation.z = 0; // Rotation Z
    // transformStamped1.transform.rotation.w = 1; // Rotation W

}

PubURDF::~PubURDF()
{
    
}


void PubURDF::PubModel(Eigen::Vector3d point, double roll, double pitch, double yaw)
{
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "car"; // Parent frame ID
    transformStamped.child_frame_id = "armour"; // Child frame ID
    transformStamped.transform.translation.x = point[0] + 0.2; // Translation X
    transformStamped.transform.translation.y = point[1] + 0; // Translation Y
    transformStamped.transform.translation.z = point[2] + 0.3; // Translation Z
    //quat.setRPY(roll, -15 / 180.0 * M_PI, yaw);
    //quat.setRPY(yaw, -15 / 180.0 * M_PI, roll);
    quat.setRPY(roll,-15 / 180.0 * M_PI,yaw);//串口总没数据没来及试这个
    transformStamped.transform.rotation.x = quat.getX(); // Rotation X
    transformStamped.transform.rotation.y = quat.getY(); // Rotation Y
    transformStamped.transform.rotation.z = quat.getZ(); // Rotation Z
    transformStamped.transform.rotation.w = quat.getW(); // Rotation W

    broadcaster.sendTransform(transformStamped);
    static_broadcaster.sendTransform(transformStamped0);
    static_broadcaster1.sendTransform(transformStamped1);    
}

void PubURDF::PubPointStamp(double roll, double pitch, double yaw)
{
    geometry_msgs::PointStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.point.x = roll;
    point.point.y = pitch;
    point.point.z = yaw;
    PubPoint.publish(point);
}

void PubURDF::PubPoseArray(vector<Eigen::Vector3d> euler, vector<Eigen::Vector3d> point)
{
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "imu";
    poses.header.stamp = ros::Time::now();
    for(int i = 0; i < point.size(); i++)
    {
        geometry_msgs::Pose pose;
        tf2::Quaternion qtn;
        qtn.setRPY(euler[i][0], euler[i][1], euler[i][2]);
        pose.orientation.w = qtn.w();
        pose.orientation.x = qtn.x();
        pose.orientation.y = qtn.y();
        pose.orientation.z = qtn.z();

        pose.position.x = point[i][0];
        pose.position.y = point[i][1];
        pose.position.z = point[i][2];

        poses.poses.push_back(pose);
    }
    pubPoseArray.publish(poses);
}

void PubURDF::PubTF(std::vector<Eigen::Vector3d> pos,std::vector<Eigen::Vector3d> rpy){
    tf2::Quaternion qtn;
    qtn.setRPY(rpy[0][0],rpy[0][1],rpy[0][2]);

    geometry_msgs::TransformStamped tf_pos1;
    geometry_msgs::TransformStamped tf_pos2;
    geometry_msgs::TransformStamped tf_pos3;

    tf_pos1.header.frame_id = "map";
    tf_pos1.child_frame_id = "pos1";
    tf_pos1.header.stamp = ros::Time::now();
    tf_pos1.transform.rotation.w = qtn.w();
    tf_pos1.transform.rotation.x = qtn.x();
    tf_pos1.transform.rotation.y = qtn.y();
    tf_pos1.transform.rotation.z = qtn.z();
    tf_pos1.transform.translation.x = pos[0][0];
    tf_pos1.transform.translation.y = pos[0][1];
    tf_pos1.transform.translation.z = pos[0][2];
    broadcaster1.sendTransform(tf_pos1);

    qtn.setRPY(rpy[1][0],rpy[1][1],rpy[1][2]);
    tf_pos2.header.frame_id = "map";
    tf_pos2.child_frame_id = "pos2";
    tf_pos2.header.stamp = ros::Time::now();
    tf_pos2.transform.rotation.w = qtn.w();
    tf_pos2.transform.rotation.x = qtn.x();
    tf_pos2.transform.rotation.y = qtn.y();
    tf_pos2.transform.rotation.z = qtn.z();
    tf_pos2.transform.translation.x = pos[1][0];
    tf_pos2.transform.translation.y = pos[1][1];
    tf_pos2.transform.translation.z = pos[1][2];
    broadcaster2.sendTransform(tf_pos2);

    qtn.setRPY(rpy[2][0],rpy[2][1],rpy[2][2]);
    tf_pos3.header.frame_id = "map";
    tf_pos3.child_frame_id = "pos3";
    tf_pos3.header.stamp = ros::Time::now();
    tf_pos3.transform.rotation.w = qtn.w();
    tf_pos3.transform.rotation.x = qtn.x();
    tf_pos3.transform.rotation.y = qtn.y();
    tf_pos3.transform.rotation.z = qtn.z();
    tf_pos3.transform.translation.x = pos[2][0];
    tf_pos3.transform.translation.y = pos[2][1];
    tf_pos3.transform.translation.z = pos[2][2];
    broadcaster3.sendTransform(tf_pos3);
}

#endif