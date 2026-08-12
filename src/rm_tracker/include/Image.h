#ifndef _IMAGE_H
#define _IMAGE_H

#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
#include<Eigen/Dense>
#include<math.h>
// #include<bundle_adjustment.h>
#include"Variable.h"

const double I_PI = 3.14159265358979323846;
#define ZERO 0.0000001
#define LightSource_Armour_High 88.0
#define LightSource_Armour_Width 79.3
#define Compensate_X -30
#define Compensate_Y 0
#define Half_Height 27.75
#define Half_Width 66.685
#define Half_Height_LONG 27.75
#define Half_Width_LONG 116.44

using namespace Eigen;
using namespace std;

// 这段代码没有使用到
// 但是Calculater.hpp文件依赖于这个文件
// 为了通过编译保留了这个文件
// 注意这里的相机内参和畸变系数并没有田间参数配置文件
// 如若后续需要使用这个文件
// 需要记得手动修改相机内参或者畸变系数
// 或者为这段函数田间参数配置的初始化

class Image
{
	public:
	cv::Mat image;
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1);;//�ڲ�
	cv::Mat distCoeffs = cv::Mat(1,5,CV_64FC1);//����
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
	Image();
	Image(cv::Mat image);
	Image(string name);
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
};

Image::Image()
{
	cv::Mat _cameraMatix = (cv::Mat_<double>(3, 3) <<
			1692.72620365811, 0.363957161762693, 643.964129877484,
			0, 1692.43288845269, 502.492890249832,
			0, 0, 1
		);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->cameraMatrix.at<double>(i,j) = _cameraMatix.at<double>(i, j);
	this->distCoeffs = (cv::Mat_<double>(1, 5) <<
			-0.111874561711212, -0.000296513404483851, 0.884391394501360, -0.00202166770416446, -1.84111324474713
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
	// yu 2024 4.15
	cv::Point3f p1(0.0, -Half_Width,Half_Height);
	cv::Point3f p2(0.0, Half_Width,Half_Height);
	cv::Point3f p3(0.0, Half_Width,-Half_Height);
	cv::Point3f p4(0.0, -Half_Width,-Half_Height);
	space_points.push_back(p1);
	space_points.push_back(p2);
	space_points.push_back(p3);
	space_points.push_back(p4);
	cv::Point3f p5(0.0, -Half_Width_LONG,Half_Height_LONG);
	cv::Point3f p6(0.0, Half_Width_LONG,Half_Height_LONG);
	cv::Point3f p7(0.0, Half_Width_LONG,-Half_Height_LONG);
	cv::Point3f p8(0.0, -Half_Width_LONG,-Half_Height_LONG);
	space_points_long.push_back(p5);
	space_points_long.push_back(p6);
	space_points_long.push_back(p7);
	space_points_long.push_back(p8);
}

Image::Image(cv::Mat image)
{
	this->image = image;
}

Image::Image(string file)
{
	this->image = cv::imread(file);
}

void Image::Init_Camera_Inside()
{
	cv::Mat _cameraMatix = (cv::Mat_<double>(3, 3) <<
			1692.72620365811, 0, 0,
			0.363957161762693, 1692.43288845269, 0,
			643.964129877484, 502.492890249832, 1
		);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->cameraMatrix.at<double>(j, i) = _cameraMatix.at<double>(i, j);
	this->distCoeffs = (cv::Mat_<double>(1, 5) <<
			-0.111874561711212, -0.000296513404483851, 0.884391394501360, -0.00202166770416446, -1.84111324474713
		);

	this->camera_center.x = cameraMatrix.at<double>(0, 2);
	this->camera_center.y = cameraMatrix.at<double>(1, 2);

}

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
			this->tvec_ba = this->tvec.clone();
			this->rvec_ba = this->rvec.clone();
			// bundle_adjustor::PoseOptimization(space_points_long, this->armour_points, this->rvec_ba, this->tvec_ba);
		}
		else{
			cv::solvePnP(space_points, this->armour_points, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec ,false ,cv::SOLVEPNP_IPPE);
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

void Image::from3D2pixel(cv::Mat & tvec_, cv::Mat & temp)
{
	if (tvec_.at<double>(2, 0) >= ZERO)
	{
		temp = cameraMatrix * tvec_ * (1 / tvec.at<double>(2, 0));
	}
	else
	{
		temp = cameraMatrix * tvec_ * ZERO;
	}
}

#endif 