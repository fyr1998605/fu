#ifndef _IMAGE_H
#define _IMAGE_H
#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
#include<Eigen/Dense>
#include<math.h>
// #include<bundle_adjustment.h>
#include"Variable.h"
#include <yaml-cpp/yaml.h>
// #include <ros/ros.h>
// #include <opencv2/core/types.hpp>

const double I_PI = 3.14159265358979323846;
#define ZERO 0.0000001

#define LightSource_Armour_High 88.0
#define LightSource_Armour_Width 79.3
// #define Compensate_X -30
#define Compensate_X 0
#define Compensate_Y 0
#define Half_Height 27.75
#define Half_Width 66.685
#define Half_Height_LONG 27.75
#define Half_Width_LONG 116.44

using namespace Eigen;
using namespace std;

class Image
{public:
	cv::Mat image;
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1);;
	cv::Mat distCoeffs = cv::Mat(1,5,CV_64FC1);
	cv::Mat tvec, rvec;
	cv::Mat tvec_ba, rvec_ba;
	std::vector<cv::Mat> tvecs,rvecs;
	cv::Point2f camera_center;
	cv::Point2f camera_center_compensate;
	cv::Point2f armour_center;
	cv::Point2f space_angle;
	cv::Point2f height_width;
	cv::Point3f pixel_angle;
	cv::Point3f space_angle_3D;
	cv::Point3f space_angle_3D_TLY;
	vector<cv::Point2f>armour_points;
	vector<cv::Point3f>space_points;
	vector<cv::Point3f>space_points_long;
	double distance;
	char aType;
	std::string config_path_;                   //存储配置路径
	 
	Image();
	Image(const std::string& config_path);
	Image(cv::Mat image);
	// Image(string name);
	void Init_Camera_Inside();
	void calculate_armour_center();
	void BigOrSmall();
	void get_distance();
	void get_distance_fu(vector<cv::Point2f> fu_points);
	void calculate_space_angle(cv::Point3d amour_3d);
	void calculate_3D_space_angle(Eigen::Vector3d amour_3d);//要求相机装中心
	void calculate_3D_space_angle_TLY(Eigen::Vector3d amour_3d,Eigen::Vector3d t_car2cam,Eigen::Matrix3d cam2imu,Eigen::Matrix3d r_car2imu);
	void calculate_pixel_angle(cv::Point2d armour);
	double Compensate(double z);
	void Cal_Abs_Yaw(Eigen::Vector3d target);
	void from3D2pixel(cv::Mat & tvec_, cv::Mat & temp);
	void setParam(const std::string &file_path);
};

Image::Image()
{
	// 哨兵的
	// camera_matrix_ = (cv::Mat_<double>(3, 3) << 2846.30182491088, -5.39713068588363, 662.330391021351, 0, 2848.15109469914, 395.390263807939, 0, 0, 1);
    // dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0258980951161089, 0.679956865435681, -0.00856193956271717,0.00494554104447204, -1.35004908184921);

	// 英雄的
	// camera_matrix_ = (cv::Mat_<double>(3, 3) << 2629.54510499450, -12.1471072839626, 842.500019888980, 0, 2633.11010902430, 373.471348562057, 0, 0, 1);
    // dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0190395160099523, 0.0743850893499158, -0.0163991873539257,0.0134699765955293, 0.285686375799103);

	// 步兵
	// camera_matrix_ = (cv::Mat_<double>(3, 3) << 1689.93945172151, -0.437279012547278, 622.527568613054, 0, 1681.02041244878, 505.871489338545, 0, 0, 1);
    // dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.122777797944065, 0.137809366427858, 0.00555887612635377,0.000136169596135679, 1.88838010156816);


	// camera_matrix_ = (cv::Mat_<double>(3, 3) << 1689.93945172151, -0.437279012547278, 622.527568613054, 0, 1681.02041244878, 505.871489338545, 0, 0, 1);
	// dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.122777797944065, 0.137809366427858, 0.00555887612635377,0.000136169596135679, 1.88838010156816);

	// // 麦轮
	// camera_matrix_ = (cv::Mat_<double>(3, 3) << 1954.50869769719, 3.10746504209223, 733.959226018831, 0, 1963.05381713322, 597.084131949838, 0, 0, 1);
    // dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0272560395821776, -0.00473622566905277, 0.0151358436788667, 0.00780277780471297在, 0.799227767052554);

	// 车上的
	cv::Mat _cameraMatix = (cv::Mat_<double>(3, 3) <<
			1724.18353097744, -0.000579628569003521  , 680.697138243768 ,
			 0, 1726.83294106563,  504.398280989468,
			 0, 0, 1
		);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->cameraMatrix.at<double>(i,j) = _cameraMatix.at<double>(i, j);

	// 车上的
	this->distCoeffs = (cv::Mat_<double>(1, 5) <<
		-0.0834722705527182, 0.496852388408421    , -0.000457625756594158  ,  0.00246230944307382, -0.0130632578222697
		);

	this->camera_center_compensate.x = this->camera_center.x = cameraMatrix.at<double>(0, 2);
	this->camera_center_compensate.y = this->camera_center.y = cameraMatrix.at<double>(1, 2);
	this->camera_center_compensate.x += Compensate_X;
	this->camera_center_compensate.y += Compensate_Y;	

	// cv::Point3f p1(-Half_Width,Half_Height, 0.0);
	// cv::Point3f p2(Half_Width,Half_Height, 0.0);
	// cv::Point3f p3(Half_Width,-Half_Height, 0.0);
	// cv::Point3f p4(-Half_Width,-Half_Height, 0.0);
	// space_points.push_back(p1);
	// space_points.push_back(p2);
	// space_points.push_back(p3);
	// space_points.push_back(p4);
	// cv::Point3f p5(-Half_Width_LONG,Half_Height_LONG, 0.0);
	// cv::Point3f p6(Half_Width_LONG,Half_Height_LONG, 0.0);
	// cv::Point3f p7(Half_Width_LONG,-Half_Height_LONG, 0.0);
	// cv::Point3f p8(-Half_Width_LONG,-Half_Height_LONG, 0.0);
	// space_points_long.push_back(p5);
	// space_points_long.push_back(p6);
	// space_points_long.push_back(p7);
	// space_points_long.push_back(p8);
	// // yu 2024 4.15
	// cv::Point3f p1(0.0, -Half_Width,Half_Height);
	// cv::Point3f p2(0.0, Half_Width,Half_Height);
	// cv::Point3f p3(0.0, Half_Width,-Half_Height);
	// cv::Point3f p4(0.0, -Half_Width,-Half_Height);
	// space_points.push_back(p1);
	// space_points.push_back(p2);
	// space_points.push_back(p3);
	// space_points.push_back(p4);
	// cv::Point3f p5(0.0, -Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p6(0.0, Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p7(0.0, Half_Width_LONG,-Half_Height_LONG);
	// cv::Point3f p8(0.0, -Half_Width_LONG,-Half_Height_LONG);
	// space_points_long.push_back(p5);
	// space_points_long.push_back(p6);
	// space_points_long.push_back(p7);
	// space_points_long.push_back(p8);
	//zyz
	cv::Point3f p1(0.0, Half_Width,Half_Height);
	cv::Point3f p2(0.0, -Half_Width,Half_Height);
	cv::Point3f p3(0.0, -Half_Width,-Half_Height);
	cv::Point3f p4(0.0, Half_Width,-Half_Height);
	space_points.push_back(p1);
	space_points.push_back(p2);
	space_points.push_back(p3);
	space_points.push_back(p4);
	cv::Point3f p5(0.0, Half_Width_LONG,Half_Height_LONG);
	cv::Point3f p6(0.0, -Half_Width_LONG,Half_Height_LONG);
	cv::Point3f p7(0.0, -Half_Width_LONG,-Half_Height_LONG);
	cv::Point3f p8(0.0, Half_Width_LONG,-Half_Height_LONG);
	space_points_long.push_back(p5);
	space_points_long.push_back(p6);
	space_points_long.push_back(p7);
	space_points_long.push_back(p8);
	// // //hyd
	// cv::Point3f p1(0.0, -Half_Width,Half_Height);
	// cv::Point3f p2(0.0, Half_Width,Half_Height);
	// cv::Point3f p3(0.0, Half_Width,-Half_Height);
	// cv::Point3f p4(0.0, -Half_Width,-Half_Height);
	// space_points.push_back(p1);
	// space_points.push_back(p2);
	// space_points.push_back(p3);
	// space_points.push_back(p4);
	// cv::Point3f p5(0.0, -Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p6(0.0, Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p7(0.0, Half_Width_LONG,-Half_Height_LONG);
	// cv::Point3f p8(0.0, -Half_Width_LONG,-Half_Height_LONG);
	// space_points_long.push_back(p5);
	// space_points_long.push_back(p6);
	// space_points_long.push_back(p7);
	// space_points_long.push_back(p8);
}

Image::Image(const std::string& config_path) : config_path_(config_path)
{
	// 配置相机参数
    if (!config_path_.empty()) {
        try {
            setParam(config_path_);
            ROS_INFO("Successfully loaded parameters from: %s", config_path_.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load parameters: %s", e.what());
        }
    } else {
        ROS_WARN("No configuration file path provided. Using default parameters.");
    }

	this->camera_center_compensate.x = this->camera_center.x = cameraMatrix.at<double>(0, 2);
	this->camera_center_compensate.y = this->camera_center.y = cameraMatrix.at<double>(1, 2);
	this->camera_center_compensate.x += Compensate_X;
	this->camera_center_compensate.y += Compensate_Y;	

	// cv::Point3f p1(-Half_Width,Half_Height, 0.0);
	// cv::Point3f p2(Half_Width,Half_Height, 0.0);
	// cv::Point3f p3(Half_Width,-Half_Height, 0.0);
	// cv::Point3f p4(-Half_Width,-Half_Height, 0.0);
	// space_points.push_back(p1);
	// space_points.push_back(p2);
	// space_points.push_back(p3);
	// space_points.push_back(p4);
	// cv::Point3f p5(-Half_Width_LONG,Half_Height_LONG, 0.0);
	// cv::Point3f p6(Half_Width_LONG,Half_Height_LONG, 0.0);
	// cv::Point3f p7(Half_Width_LONG,-Half_Height_LONG, 0.0);
	// cv::Point3f p8(-Half_Width_LONG,-Half_Height_LONG, 0.0);
	// space_points_long.push_back(p5);
	// space_points_long.push_back(p6);
	// space_points_long.push_back(p7);
	// space_points_long.push_back(p8);
	// // yu 2024 4.15
	// cv::Point3f p1(0.0, -Half_Width,Half_Height);
	// cv::Point3f p2(0.0, Half_Width,Half_Height);
	// cv::Point3f p3(0.0, Half_Width,-Half_Height);
	// cv::Point3f p4(0.0, -Half_Width,-Half_Height);
	// space_points.push_back(p1);
	// space_points.push_back(p2);
	// space_points.push_back(p3);
	// space_points.push_back(p4);
	// cv::Point3f p5(0.0, -Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p6(0.0, Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p7(0.0, Half_Width_LONG,-Half_Height_LONG);
	// cv::Point3f p8(0.0, -Half_Width_LONG,-Half_Height_LONG);
	// space_points_long.push_back(p5);
	// space_points_long.push_back(p6);
	// space_points_long.push_back(p7);
	// space_points_long.push_back(p8);
	//zyz
	cv::Point3f p1(0.0, Half_Width,Half_Height);
	cv::Point3f p2(0.0, -Half_Width,Half_Height);
	cv::Point3f p3(0.0, -Half_Width,-Half_Height);
	cv::Point3f p4(0.0, Half_Width,-Half_Height);
	space_points.push_back(p1);
	space_points.push_back(p2);
	space_points.push_back(p3);
	space_points.push_back(p4);
	cv::Point3f p5(0.0, Half_Width_LONG,Half_Height_LONG);
	cv::Point3f p6(0.0, -Half_Width_LONG,Half_Height_LONG);
	cv::Point3f p7(0.0, -Half_Width_LONG,-Half_Height_LONG);
	cv::Point3f p8(0.0, Half_Width_LONG,-Half_Height_LONG);
	space_points_long.push_back(p5);
	space_points_long.push_back(p6);
	space_points_long.push_back(p7);
	space_points_long.push_back(p8);
	// // //hyd
	// cv::Point3f p1(0.0, -Half_Width,Half_Height);
	// cv::Point3f p2(0.0, Half_Width,Half_Height);
	// cv::Point3f p3(0.0, Half_Width,-Half_Height);
	// cv::Point3f p4(0.0, -Half_Width,-Half_Height);
	// space_points.push_back(p1);
	// space_points.push_back(p2);
	// space_points.push_back(p3);
	// space_points.push_back(p4);
	// cv::Point3f p5(0.0, -Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p6(0.0, Half_Width_LONG,Half_Height_LONG);
	// cv::Point3f p7(0.0, Half_Width_LONG,-Half_Height_LONG);
	// cv::Point3f p8(0.0, -Half_Width_LONG,-Half_Height_LONG);
	// space_points_long.push_back(p5);
	// space_points_long.push_back(p6);
	// space_points_long.push_back(p7);
	// space_points_long.push_back(p8);
}

Image::Image(cv::Mat image)
{
	this->image = image;
}

// Image::Image(string file)
// {
// 	this->image = cv::imread(file);
// }

void Image::calculate_armour_center()
{
	MatrixXf line_1_x(2, 2), line_2_x(2, 2);
	MatrixXf line_1_y(2, 1), line_2_y(2, 1);
	MatrixXf line_1_ab(2, 1), line_2_ab(2, 1);

	line_1_x <<
		this->armour_points[0].x, 1,
		this->armour_points[2].x, 1;

	line_2_x <<
		this->armour_points[1].x, 1,
		this->armour_points[3].x, 1;

	line_1_y <<
		this->armour_points[0].y,
		this->armour_points[2].y;

	line_2_y <<
		this->armour_points[1].y,
		this->armour_points[3].y;

	line_1_ab = line_1_x.inverse() * line_1_y;
	line_2_ab = line_2_x.inverse() * line_2_y;
	
	this->armour_center.x = (line_2_ab(1, 0) - line_1_ab(1, 0)) / (line_1_ab(0, 0) - line_2_ab(0, 0));
	this->armour_center.y = line_1_ab(0, 0) * armour_center.x + line_1_ab(1, 0);
	this->armour_center.x = abs(armour_center.x);
}

void Image::Cal_Abs_Yaw(Eigen::Vector3d target)
{

}

double Image::Compensate(double z)
{
	int x = z;
    int x2 = x * x;
    int x3 = x2 * x;
    double count = x * (-1.79465470e-02) + x2 * (3.70087126e-06) + x3 * (-1.72268656e-10) - 35.13797191;

    if(z < 6245) count = 0;
	return count;
}

void Image::get_distance_fu(vector<cv::Point2f> fu_points)
{
	double x, y, z;
	double pitch, yaw;
	vector<cv::Point3f>space_points;

	int fu_point_width_up = 300;
	int fu_point_width_down= 363;
	int fu_point_hight = 324;

	cv::Point3f p1(fu_point_width_up / 2, fu_point_hight / 2, 0.0);
	cv::Point3f p2(-fu_point_width_up / 2, fu_point_hight / 2, 0.0);
	cv::Point3f p3(-fu_point_width_down / 2, -fu_point_hight / 2, 0.0);
	cv::Point3f p4(fu_point_width_down / 2, -fu_point_hight / 2, 0.0);
	space_points.push_back(p1);
	space_points.push_back(p2);
	space_points.push_back(p3);
	space_points.push_back(p4);
	cv::solvePnP(space_points, fu_points, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec);
	
	x = this->tvec.at<double>(0, 0);
	y = this->tvec.at<double>(1, 0);
	z = this->tvec.at<double>(2, 0);
	this->distance = sqrt(x * x + y * y + z * z);
}

void Image::BigOrSmall()
{
	cv::Point2f cp1 = armour_points[0],cp2 = armour_points[1],cp3 = armour_points[2];
	// cp1-cp2 cp1-cp3 cp2-cp3 去掉最长的 计算灯条长宽
	double len_cp1_cp2 = cv::norm(cp1-cp2),len_cp1_cp3 = cv::norm(cp1-cp3),len_cp2_cp3 = cv::norm(cp2-cp3);
	double len_max = MAX(len_cp2_cp3,MAX(len_cp1_cp2,len_cp1_cp3));
	if(len_max == len_cp1_cp2){
		height_width.x = MIN(len_cp1_cp3,len_cp2_cp3);
		height_width.y = MAX(len_cp1_cp3,len_cp2_cp3);
	}
	else if(len_max == len_cp1_cp3){
		height_width.x = MIN(len_cp1_cp2,len_cp2_cp3);
		height_width.y = MAX(len_cp1_cp2,len_cp2_cp3);
	}
	else if(len_max == len_cp2_cp3){
		height_width.x = MIN(len_cp1_cp2,len_cp1_cp3);
		height_width.y = MAX(len_cp1_cp2,len_cp1_cp3);
	}
	//异常处理部分
	else{
		double len_dis[3] = {0};
		len_dis[0] = len_max - len_cp1_cp2;
		len_dis[1] = len_max - len_cp1_cp3;
		len_dis[2] = len_max - len_cp2_cp3;
		int ind_min = -1;
		double dis_min = 0.0;
		for(int i = 0;i < 3;i++)
			if(!~ind_min || len_dis[i] < dis_min){
				ind_min = i;
				dis_min = len_dis[i];
			}
		switch (ind_min)
		{
		case 0:
			height_width.x = MIN(len_cp1_cp3,len_cp2_cp3);
			height_width.y = MAX(len_cp1_cp3,len_cp2_cp3);
			break;
		case 1:
			height_width.x = MIN(len_cp1_cp2,len_cp2_cp3);
			height_width.y = MAX(len_cp1_cp2,len_cp2_cp3);
			break;
		case 2:
			height_width.x = MIN(len_cp1_cp2,len_cp1_cp3);
			height_width.y = MAX(len_cp1_cp2,len_cp1_cp3);
			break;
		default:
			throw len_max;
			break;
		}
	}
	//TODO:通过长宽比判断长短装甲板
	double whRatio = height_width.y / height_width.x;
	if(whRatio > LONG_ARMOR_LIMIT)
		aType = 'l';
	else
		aType = 's';
}

void Image::get_distance()
{
	double x, y, z;
	double pitch, yaw;
	{

		if(aType == 'l'){
			cv::solvePnP(space_points_long, this->armour_points, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec ,false ,cv::SOLVEPNP_IPPE);
			// 设置 criteria
			cv::TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, FLT_EPSILON);
			//迭代法(使用 solvePnPRefineLM 进行精细化优化)
			cv::solvePnPRefineLM(space_points_long, this->armour_points, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, criteria);
			this->tvec_ba = this->tvec.clone();
			this->rvec_ba = this->rvec.clone();
			// bundle_adjustor::PoseOptimization(space_points_long, this->armour_points, this->rvec_ba, this->tvec_ba);
		}
		else{
			cv::solvePnP(space_points, this->armour_points, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec ,false ,cv::SOLVEPNP_IPPE);
			// 设置 criteria
			cv::TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, FLT_EPSILON);
			//迭代法(使用 solvePnPRefineLM 进行精细化优化)
			cv::solvePnPRefineLM(space_points, this->armour_points, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, criteria);
			this->tvec_ba = this->tvec.clone();
			this->rvec_ba = this->rvec.clone();
			// bundle_adjustor::PoseOptimization(space_points, this->armour_points, this->rvec_ba, this->tvec_ba);
		}
	}

	x = this->tvec.at<double>(0, 0);
	y = this->tvec.at<double>(1, 0);
	z = this->tvec.at<double>(2, 0);
	this->distance = sqrt(x * x + y * y + z * z);
}

void Image::calculate_space_angle(cv::Point3d amour_3d)
{
	double x, y, z;
	double pitch, yaw;
	// vector<cv::Point3f>space_points;
	// cv::Point3f p1(-armor_width / 2, armor_height / 2,0.0);
	// cv::Point3f p2(armor_width / 2, armor_height / 2, 0.0);
	// cv::Point3f p3(armor_width / 2, -armor_height / 2, 0.0);
	// cv::Point3f p4(-armor_width / 2, -armor_height / 2, 0.0);
	// space_points.push_back(p1);
	// space_points.push_back(p2);
	// space_points.push_back(p3);
	// space_points.push_back(p4);
	// cv::solvePnP(space_points, this->armour_points, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec);
	// x = this->tvec.at<double>(0, 0);
	// y = this->tvec.at<double>(1, 0);
	// z = this->tvec.at<double>(2, 0);
	// //cout << x << " " << y << " " << z << endl;
	// this->distance = sqrt(x * x + y * y + z * z);
	// cv::Rodrigues(rvec, rvec);
	x = amour_3d.x;
	y = amour_3d.y;
	z = amour_3d.z;
	yaw = (atan(-x / z)) / CV_PI * 180.0;
	pitch = (atan(-y / sqrt(x * x + z * z))) / CV_PI * 180;
	this->space_angle = cv::Point2f(pitch, yaw);
}

// void Image::calculate_pixel_angle(cv::Point2d armour)
// {
// 	double pitch, yaw;
// 	double fx, fy;
// 	fx = this->cameraMatrix.at<double>(0, 0);
// 	fy = this->cameraMatrix.at<double>(1, 1);
// 	yaw = atan((this->camera_center_compensate.x - armour.x) / fx) / CV_PI * 180.0;
// 	pitch = atan((this->camera_center_compensate.y - armour.y) / fy) / CV_PI * 180.0;
// 	this->pixel_angle = cv::Point2f(pitch, yaw);
// }

void Image::calculate_pixel_angle(cv::Point2d armour)
{
	double pitch, yaw,Hero_angle;
	double fx, fy;
	fx = this->cameraMatrix.at<double>(0, 0);
	fy = this->cameraMatrix.at<double>(1, 1);
	yaw = atan((this->camera_center_compensate.x - armour.x) / fx) / CV_PI * 180.0;
	pitch = atan((this->camera_center_compensate.y - armour.y) / fy) / CV_PI * 180.0;
	Hero_angle = 0;
	this->pixel_angle = cv::Point3f(pitch, yaw,Hero_angle);
}

//HC自加
void Image::calculate_3D_space_angle(Eigen::Vector3d amour_3d)//相机装中心
{
	cv::Point3d X_armour_center(static_cast<double>(amour_3d.x()),
                                static_cast<double>(amour_3d.y()),
                                static_cast<double>(amour_3d.z()));

	double angle_x =(atan2(X_armour_center.z,X_armour_center.y))*180/I_PI;
	double angle_y =(atan2(X_armour_center.x,X_armour_center.z))*180/I_PI;
	double angle_z =((atan2(X_armour_center.y,X_armour_center.x))*180/I_PI)-180;
	if(abs(angle_z)>180)//防跳变
	{
		angle_z=angle_z+360;
	}
	
	this->space_angle_3D = cv::Point3d(angle_x,angle_y,angle_z);//装甲板中心对于相机坐标系下的三个偏角 
	std::cout << "3D_space_angle" << space_angle_3D << std::endl; 
}

void Image::calculate_3D_space_angle_TLY(Eigen::Vector3d amour_3d,Eigen::Vector3d t_car2cam,Eigen::Matrix3d cam2imu,Eigen::Matrix3d r_car2imu)
{
	Eigen::Vector3d space_3D_TLY =r_car2imu * cam2imu * (amour_3d + t_car2cam);
	std::cout << "3D_space_TLY" << space_3D_TLY << std::endl;           

	cv::Point3d TLY_armour_center(static_cast<double>(space_3D_TLY.x()),
                                static_cast<double>(space_3D_TLY.y()),
                                static_cast<double>(space_3D_TLY.z()));

	double angle_x =(atan2(TLY_armour_center.z,TLY_armour_center.y))*180/I_PI;
	double angle_y =(atan2(TLY_armour_center.x,TLY_armour_center.z))*180/I_PI;
	double angle_z =(atan2(TLY_armour_center.y,TLY_armour_center.x))*180/I_PI;
	
	this->space_angle_3D_TLY = cv::Point3d(angle_x,angle_y,angle_z);//装甲板中心对于相机坐标系下的三个偏角 
	std::cout << "3D_space_angle_TLY" << space_angle_3D_TLY << std::endl; 
}

void Image::from3D2pixel(cv::Mat & tvec_, cv::Mat & temp){
	if (tvec_.at<double>(2, 0) >= ZERO)
	{
		temp = cameraMatrix * tvec_ * (1 / tvec.at<double>(2, 0));
	}
	else
	{
		temp = cameraMatrix * tvec_ * ZERO;
	}
}

void Image::setParam(const std::string &file_path) {
	try{
        // 1. 文件存在性检查
        if (file_path.empty()) {
            ROS_ERROR("Empty config file path provided");
            return;
        }

        // 2.检查文件是否打开
        std::ifstream file_check(file_path);
        if (!file_check.is_open()) {
            ROS_ERROR("Failed to open config file: %s. Reason: %s", 
                    file_path.c_str(), strerror(errno));
            return;
        }
        ROS_INFO("Loading parameters from: %s", file_path.c_str());

        // 3. 加载 YAML 文件并添加详细错误处理
        YAML::Node config;
        try {
            config = YAML::LoadFile(file_path);
        } catch (const YAML::BadFile& e) {
            ROS_ERROR("Bad YAML file: %s", e.what());
            return;
        } catch (const YAML::ParserException& e) {
            ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
            return;
        }

        // 加载参数
        if (config["Camera"]){
            YAML::Node camera = config["Camera"];
            // 加载相机内参矩阵
            if (camera["camera_matrix"]) {
                std::vector<double> cam_matrix_array = camera["camera_matrix"].as<std::vector<double>>();
                if (cam_matrix_array.size() == 9) {
                    // 创建3*3相机内参矩阵
                    cameraMatrix = (cv::Mat_<double>(3, 3) << 
                    cam_matrix_array[0], cam_matrix_array[1], cam_matrix_array[2],
                    cam_matrix_array[3], cam_matrix_array[4], cam_matrix_array[5],
                    cam_matrix_array[6], cam_matrix_array[7], cam_matrix_array[8]);
                }
                // for (int i = 0; i < cam_matrix_array.size(); i++) {
                //     ROS_INFO("cam_matrix_array%d:%.3f",i,cam_matrix_array[i]);
                // }
            }
            else {
                ROS_WARN("No 'camera_matrix' section in config file");
            }
            
            // 加载畸变系数矩阵
            if (camera["dist_coeffs"]) {
                std::vector<double> dist_coeffs_array = camera["dist_coeffs"].as<std::vector<double>>();
                if (dist_coeffs_array.size() == 5) {
                    distCoeffs = (cv::Mat_<double>(1, 5) << 
                        dist_coeffs_array[0], dist_coeffs_array[1], dist_coeffs_array[2], 
                        dist_coeffs_array[3], dist_coeffs_array[4]);
                }
            }
            else {
                ROS_WARN("No 'dist_coeffs' section in config file");
            }
        }
    }
     catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parsing error: %s", e.what());
    } 
    catch (const std::exception& e) {
        ROS_ERROR("Error loading parameters: %s", e.what());
    } 
    catch (...) {
        ROS_ERROR("Unknown error occurred while loading config file");
    }
}

#endif 