#ifndef _LIGHT_H
#define _LIGHT_H

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define GRAY 0
#define SPLIT 1
#define HSV 2
#define Red 0
#define Blue 1
#define L_PI 3.1415926535897932384626

//const double L_PI = 3.14159265358979323846;
class Light 
{
public:
	Mat image;
	//Mat hsv;
	//Mat gray;
	int Lowest[3];//��С��ֵ,��ֵ��
	int Highest[3];//�����ֵ,��ֵ��
	int threshold_;
	int color;
	int flag = HSV;

	Light();

	Light(Mat img, int _flag);

	Light(int LH, int HH, int LS, int HS, int LV, int HV, int threshold_);

	Mat fiter_split(Mat frame, int fun_color);

	Mat fiter_gray(Mat frame);

	Mat fiter_hsv(Mat frame);

	Mat Fiter(Mat frame, int flag);

	void imshow_TrarckBar(Mat frame);

	void create_TrarckBar();

	float calculate_p_to_p(Point2f point_1, Point2f point_2);

	float cal_ang(Point2f point_1, Point2f point_2, Point2f point_3);

	static bool LessSort_y(const cv::Point2f& a, const cv::Point2f& b);

	static bool LessSort_x(const cv::Point2f& a, const cv::Point2f& b);

	vector<Point2f>refine_the_armor(Mat frame, int flag);
};

int LH = 0;
int LS = 0;
int LV = 0;
int LT = 0;
int HH = 255;
int HS = 255;
int HV = 255;
int HT = 255;

Light::Light()
{

}

Light::Light(Mat img,int _flag)
{
	this->image = img;
	this->flag = _flag;
}

Light::Light(int LH, int HH, int LS, int HS, int LV, int HV, int threshold_)
{
	this->flag = HSV;
	this->Lowest[0] = LH;
	this->Lowest[1] = LS;
	this->Lowest[2] = LV;

	this->Highest[0] = HH;
	this->Highest[1] = HS;
	this->Highest[2] = HV;

	this->threshold_ = threshold_;
}

Mat Light::fiter_split(Mat frame, int fun_color)
{
	vector<Mat>channels;
	Mat mark;
	split(frame, channels);
	if (fun_color == Red)
		threshold((channels[2] - channels[1]),mark, this->threshold_, 255, THRESH_BINARY);
	if (fun_color == Blue)
		threshold((channels[0] - channels[2]), mark, this->threshold_, 255, THRESH_BINARY);
	
	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
	morphologyEx(mark, mark, MORPH_OPEN, element);
	morphologyEx(mark, mark, MORPH_CLOSE, element);//��ֵ������

	return mark;
}

Mat Light::fiter_gray(Mat frame)
{
	Mat mark;
	cvtColor(frame, mark, COLOR_BGR2GRAY);
	threshold(mark, mark, this->threshold_, 255, THRESH_BINARY);

	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
	morphologyEx(mark, mark, MORPH_OPEN, element);
	morphologyEx(mark, mark, MORPH_CLOSE, element);//��ֵ������

	return mark;
}

Mat Light::fiter_hsv(Mat frame)
{
	Mat mark;
	cvtColor(frame, mark, COLOR_BGR2HSV);
	
	//imshow("2", frame); 

	inRange(mark, Scalar(Lowest[0], Lowest[1], Lowest[2]), Scalar(Highest[0], Highest[1], Highest[2]), mark);
	
	return mark;
}

Mat Light::Fiter(Mat frame, int flag)
{
	this->flag = flag;
	if (flag == HSV)
		return fiter_hsv(frame);
	else if (flag == GRAY)
		return fiter_gray(frame);
	else
		return fiter_split(frame, Red);
}

void Light::imshow_TrarckBar(Mat frame)
{
	Mat mark;
	if (this->flag == HSV)
	{
		cvtColor(frame, mark, COLOR_BGR2HSV);
		inRange(mark, Scalar(LH, LS, LV), Scalar(HH, HS, HV), mark);
		//int structElementSize = 3;
		//Mat element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
		////morphologyEx(hsv, hsv, MORPH_OPEN, element);
		//morphologyEx(hsv, hsv, MORPH_CLOSE, element);//��ֵ������
		imshow("2", mark);
	}
	else
	{ 
		cvtColor(frame, mark, COLOR_BGR2GRAY);
		threshold(mark, mark, LT, 255, THRESH_BINARY);
		int structElementSize = 3;
		//Mat element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
		//morphologyEx(frame, frame, MORPH_OPEN, element);
		//morphologyEx(frame, frame, MORPH_CLOSE, element);//��ֵ������
		imshow("2", mark);
	}
}


void onLH(int, void*)
{
	
}
void onLS(int, void*)
{
	
}
void onLV(int, void*)
{
	
}
void onHH(int, void*)
{
	
}
void onHS(int, void*)
{
	
}
void onHV(int, void*)
{
	
}
void thresh(int, void*)
{

}


void Light::create_TrarckBar()
{
	namedWindow("1", 0);
	resizeWindow("1", 500, 500);
	namedWindow("2", 0);
	resizeWindow("2", 500, 500);
	createTrackbar("LH", "1", &LH, 255, onLH);
	createTrackbar("HH", "1", &HH, 255, onHH);
	createTrackbar("LS", "1", &LS, 255, onLS);
	createTrackbar("HS", "1", &HS, 255, onHS);
	createTrackbar("LV", "1", &LV, 255, onLV);
	createTrackbar("HV", "1", &HV, 255, onHV);
	createTrackbar("LT", "1", &LT, 255, thresh);
	createTrackbar("HT", "1", &HT, 255, thresh);

}

float Light::calculate_p_to_p(Point2f point_1, Point2f point_2)
{
	float distance = fabs(sqrt((point_1.x - point_2.x) * (
		point_1.x - point_2.x) + (point_1.y - point_2.y) * (point_1.y - point_2.y)));
	return distance;
}

float Light::cal_ang(Point2f point_1, Point2f point_2, Point2f point_3)
{
	//���������������н�
	//:param point_1 : ��1����
	//:param point_2 : ��2����
	//:param point_3 : ��3����
	//:return : ��������ǵļн�ֵ������ֻ�Ƿ��ص�2�ļн�
	float B;
	float a = sqrt((point_2.x - point_3.x) * (point_2.x - point_3.x
		) + (point_2.y - point_3.y) * (point_2.y - point_3.y));
	float b = sqrt((point_1.x - point_3.x) * (point_1.x - point_3.x
		) + (point_1.y - point_3.y) * (point_1.y - point_3.y));
	float c = sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x
		) + (point_1.y - point_2.y) * (point_1.y - point_2.y));
	if (a * c != 0)
	{
		if ((b * b - a * a - c * c) / (-2 * a * c) >= -1 && (b * b - a * a - c * c) / (-2 * a * c) <= 1)
		{
			B = ((acos((b * b - a * a - c * c) / (-2 * a * c)))) * (180.0 / L_PI);
			/*cout << B << endl;*/
			if (B != NULL)
				return B;
			else
				return 0.0f;
		}
		else
			return 0.0f;
	}
	else
		return 0.0f;
}

bool Light::LessSort_y(const cv::Point2f& a, const cv::Point2f& b) { return (a.y < b.y); }

bool Light::LessSort_x(const cv::Point2f& a, const cv::Point2f& b) { return (a.x < b.x); }

vector<Point2f>Light::refine_the_armor(Mat frame, int flag)
{
	Light light;//ʵ�������һ������
	vector<vector<Point>>contours;//��������
	vector<Vec4i>hierarchy; //һ������
	Mat dstImage;//hsv�󼰶�ͼ����в�����õ��Ķ�ֵ��ͼ��
	vector<vector<Point2f>>lightbar_matching;//���������С��Ӿ��εĶ̱ߵ��е�
	vector<float>lightbar_matching_h;//���������С��Ӿ��εĳ���,ͨ����С��Ӿ��εģ����ߣ��������е���ƥ��
	vector<vector<Point2f>>final_armors;//��������ƥ��ɸѡ��õ�����С��Ӿ��εĶ̱ߵ��е�
	//����Ϊ���е���ƥ����ĸ���,����������λ��
	Point2f point_1_down;
	Point2f point_1_up;
	Point2f point_2_down;
	Point2f point_2_up;
	vector<Point2f>High_Points;//���yֵ��ĵ�
	vector<Point2f>Low_Points; //���yֵС�ĵ�
	//vector<Point2f>Left_Points;//���xֵС�ĵ�
	//vector<Point2f>Right_Points; //���xֵ��ĵ�
	vector<Point2f>target;//���������ǵõ���װ�װ�Ŀ����ĵ�
	findContours(frame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));//Ѱ��ͼƬ�����г��ֵ�����
	for (int findcontours = 0; findcontours < contours.size(); findcontours++)
	{
		RotatedRect box = minAreaRect(contours[findcontours]);//ȡ����������С��Ӿ���
		Point2f rect_points[4];
		box.points(rect_points);//�õ���С��Ӿ��ε��ĸ���
		float distance_1 = light.calculate_p_to_p(rect_points[2], rect_points[1]);
		float distance_2 = light.calculate_p_to_p(rect_points[1], rect_points[0]);
		if (distance_1 > distance_2)
		{
			lightbar_matching_h.push_back(distance_1);
			vector<Point2f>Longside_CenterPoints;
			Point2f Longside_CenterPoint_1;
			Point2f Longside_CenterPoint_2;
			//ȡ��������Ӿ��ε���ζ̱ߵ��е�
			Longside_CenterPoint_1.x = (rect_points[0].x + rect_points[1].x) / 2.0;
			Longside_CenterPoint_1.y = (rect_points[0].y + rect_points[1].y) / 2.0;
			Longside_CenterPoint_2.x = (rect_points[2].x + rect_points[3].x) / 2.0;
			Longside_CenterPoint_2.y = (rect_points[2].y + rect_points[3].y) / 2.0;
			Longside_CenterPoints.push_back(Longside_CenterPoint_1);
			Longside_CenterPoints.push_back(Longside_CenterPoint_2);
			lightbar_matching.push_back(Longside_CenterPoints);
		}
		else
		{
			lightbar_matching_h.push_back(distance_2);
			vector<Point2f>Longside_CenterPoints;
			Point2f Longside_CenterPoint_1;
			Point2f Longside_CenterPoint_2;
			//ȡ��������Ӿ��ε���ζ̱ߵ��е�
			Longside_CenterPoint_1.x = (rect_points[0].x + rect_points[3].x) / 2.0;
			Longside_CenterPoint_1.y = (rect_points[0].y + rect_points[3].y) / 2.0;
			Longside_CenterPoint_2.x = (rect_points[1].x + rect_points[2].x) / 2.0;
			Longside_CenterPoint_2.y = (rect_points[1].y + rect_points[2].y) / 2.0;
			Longside_CenterPoints.push_back(Longside_CenterPoint_1);
			Longside_CenterPoints.push_back(Longside_CenterPoint_2);
			lightbar_matching.push_back(Longside_CenterPoints);
		}
	}
	//ͨ�������ĸߺͽǶ������е���ƥ�䣬���е��������������
	for (int lightbar_match1 = 0; lightbar_match1 < lightbar_matching.size(); lightbar_match1++)
	{
		for (int lightbar_match2 = lightbar_match1 + 1; lightbar_match2 < lightbar_matching.size(); lightbar_match2++)
		{
			//�ڸ�֮���ڷ�Χ�ڵĵ�������Ƕ�ƥ��
			if ((lightbar_matching_h[lightbar_match1]) / (lightbar_matching_h[lightbar_match2]) > 0.7 && (lightbar_matching_h[lightbar_match1]) / (lightbar_matching_h[lightbar_match2]) < 1.3)
			{
				//Ҫ���е���ƥ�䣬����Ҫȷ����С��Ӿ����ĵ�ľ���λ��
				if (lightbar_matching[lightbar_match1][0].y > lightbar_matching[lightbar_match1][1].y)
				{
					point_1_down = lightbar_matching[lightbar_match1][0];
					point_1_up = lightbar_matching[lightbar_match1][1];
				}
				else
				{
					point_1_down = lightbar_matching[lightbar_match1][1];
					point_1_up = lightbar_matching[lightbar_match1][0];
				}
				if (lightbar_matching[lightbar_match2][0].y > lightbar_matching[lightbar_match2][1].y)
				{
					point_2_down = lightbar_matching[lightbar_match2][0];
					point_2_up = lightbar_matching[lightbar_match2][1];
				}
				else
				{
					point_2_down = lightbar_matching[lightbar_match2][1];
					point_2_up = lightbar_matching[lightbar_match2][0];
				}
				//���������������γɵ�װ�װ���ο��ĸ����ڱߵļнǴ�С
				float angle1 = light.cal_ang(
					point_1_up,
					point_1_down,
					point_2_down);
				float angle2 = light.cal_ang(
					point_1_down,
					point_1_up,
					point_2_up);
				float angle3 = light.cal_ang(
					point_1_down,
					point_2_down,
					point_2_up);
				float angle4 = light.cal_ang(
					point_1_up,
					point_2_up,
					point_2_down);
				//����������ɸѡ�����ĵ�����С��Ӿ��α�������
				if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
				{
					if (fabs(angle1 - 90) < 15 && fabs(angle2 - 90) < 15 && fabs(angle3 - 90) < 15 && fabs(angle4 - 90) < 15)
					{
						final_armors.push_back(
							lightbar_matching[lightbar_match1]);
						final_armors.push_back(
							lightbar_matching[lightbar_match2]);
					}
				}
			}
		}
	}
	//Ŀǰ��˼·�ǲ������ձ������˶��پ��ο�,������Ϊֻ������
	if (final_armors.size() >= 2)
	{
		//���������ο�̱ߵ��е㰴��y��һ����ֵ������������
		sort(final_armors[0].begin(), final_armors[0].end(), &LessSort_y);
		sort(final_armors[1].begin(), final_armors[1].end(), &LessSort_y);
		//��ȷ������λ�ú�ĵ�ֱ��������������
		Low_Points.push_back(final_armors[0][1]);
		Low_Points.push_back(final_armors[1][1]);
		High_Points.push_back(final_armors[0][0]);
		High_Points.push_back(final_armors[1][0]);
		//�����������еĵ㰴��x��һ����ֵ������������,���ܵĵõ����ĵ������һ���Ƿ���˳���
		sort(Low_Points.begin(), Low_Points.end(), &LessSort_x);
		sort(High_Points.begin(), High_Points.end(), &LessSort_x);
		//����ŵ�Ŀ��������
		target.push_back(High_Points[0]);
		target.push_back(High_Points[1]);
		target.push_back(Low_Points[1]);
		target.push_back(Low_Points[0]);

	}
	return target;
}

#endif