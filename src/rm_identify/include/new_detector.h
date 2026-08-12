#include <openvino/openvino.hpp>
#include <iostream>
#include<string>
#include<time.h>
#include <fstream>
#include <vector>
#include <random>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<chrono>
#include "Image.h"
#include"new_assert.h"
#include"Light.h"

using namespace cv;
using namespace std;
using namespace dnn;
#define Mutiply 1.05
namespace new_Identify{
// struct LightParams
// {
//         // width / height
//         double min_ratio;
//         double max_ratio;
//         // vertical angle
//         double max_angle;
// };


// struct ArmorParams
// {
//         double min_light_ratio;
//         //light pairs distance
//         double min_small_center_distance;
//         double max_small_center_distance;
//         double min_large_center_distance;
//         double max_large_center_distance;
//         //horizontal angle
//         double max_angle;
// };

// struct Config {
// 	// float confThreshold;
// 	// float nmsThreshold;
// 	// float scoreThreshold;
// 	// int inpWidth;
// 	// int inpHeight;
    
// 	  std::string onnx_path;
//     const LightParams & l;
//     const ArmorParams & a;
//     int  binary_thres ;
// };
struct LightParams
{
        // width / height
        double min_ratio;
        double max_ratio;
        // vertical angle
        double max_angle;

        double min_fill_ratio;
};


struct ArmorParams
{
        double min_light_ratio;
        //light pairs distance
        double min_small_center_distance;
        double max_small_center_distance;
        double min_large_center_distance;
        double max_large_center_distance;
        //horizontal angle
        double max_angle;
};

struct Config {
	// float confThreshold;
	// float nmsThreshold;
	// float scoreThreshold;
	// int inpWidth;
	// int inpHeight;
    
	  //std::string onnx_path;
    const LightParams & l;
    const ArmorParams & a;
    int  binary_thres ;
};

struct Resize
{
	cv::Mat resized_image;
	int dw;
	int dh;
};



// struct lLight :public cv::RotatedRect
// {
//     lLight()=default;
//      explicit lLight(const std::vector<cv::Point> &contour)
//   : cv::RotatedRect(cv::minAreaRect(contour)) {
//     FYT_ASSERT(contour.size() > 0);

//       center = std::accumulate(
//       contour.begin(),
//       contour.end(),
//       cv::Point2f(0, 0),
//       [n = static_cast<float>(contour.size())](const cv::Point2f &a, const cv::Point &b) {
//         return a + cv::Point2f(b.x, b.y) / n;
//       });

//     cv::Point2f p[4];
//     this->points(p);
//     std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
//     top = (p[0] + p[1]) / 2;
//     bottom = (p[2] + p[3]) / 2;

//     length = cv::norm(top - bottom);
//     width = cv::norm(p[0] - p[1]);

//     // axis = top - bottom;
//     // axis = axis / cv::norm(axis);

//     // Calculate the tilt angle
//     // The angle is the angle between the light bar and the horizontal line
//     tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
//     tilt_angle = tilt_angle / CV_PI * 180;
//   }
    
//     int class_id;
//     cv::Point2f top, bottom,center;
//     double length;
//     double width;
//     float tilt_angle;

// };

struct lLight : public cv::Rect
{
  lLight() = default;
  explicit lLight(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
  : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle)
  {
    length = cv::norm(top - bottom);
    width = area / length;
    center = (top + bottom) / 2;
  }

  int class_id;
  cv::Point2f top, bottom;
  cv::Point2f center;
  double length;
  double width;
  float tilt_angle;
  cv::Point2f axis;   // 用于存储PCA计算出的主轴方向
};


struct aArmor
{
    aArmor()=default;
    explicit aArmor(const lLight & l1,const lLight & l2)
    {
        if(l1.center.x < l2.center.x)
        {left_light=l1,right_light=l2;}
        else{
            left_light=l2,right_light=l1;
        }
        center = (left_light.center+right_light.center)/2;

    }
    //Light pairs part
    lLight left_light,right_light;
    cv::Point2f center;
    char type;
    float angle;

    //Number part
    cv::Mat number_img;
    int  number;
    float confidence;
    std::string classfication_result;
};





// struct Detection {
// 	int class_id;
// 	float confidence;
// 	cv::Rect box;
// };
const std::vector<int> class_names_ = {
    1,
    2,
    3,
    4,
    5,
    6,    // outpost  前哨站
    7,    // guard    哨兵
    8,    // base     基地
    9     // negative  背景
};

// PCA角点优化
///////////////////////////////////////////////
struct SymmetryAxis {
  cv::Point2f centroid;
  cv::Point2f direction;
  float mean_val; // Mean brightness
};

// This class is used to improve the precision of the corner points of the light bar.
// First, the PCA algorithm is used to find the symmetry axis of the light bar,
// and then along the symmetry axis to find the corner points of the light bar based on the gradient of brightness.
class LightCornerCorrector {
public:
  explicit LightCornerCorrector() noexcept {}

  // Correct the corners of the armor's lights
  void correctCorners(aArmor &armor, const cv::Mat &gray_img);

private:
  // Find the symmetry axis of the light
  SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img, const lLight &light);

  // Find the corner of the light
  cv::Point2f findCorner(const cv::Mat &gray_img,
                         const lLight &light,
                         const SymmetryAxis &axis,
                         std::string order);
};

// Maintained by Shenglin Qin, Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <numeric>

void LightCornerCorrector::correctCorners(aArmor &armor, const cv::Mat &gray_img) {
  // If the width of the light is too small, the correction is not performed
  constexpr int PASS_OPTIMIZE_WIDTH = 3;

  if (armor.left_light.width > PASS_OPTIMIZE_WIDTH) {
    // Find the symmetry axis of the light
    SymmetryAxis left_axis = findSymmetryAxis(gray_img, armor.left_light);
    armor.left_light.center = left_axis.centroid;
    armor.left_light.axis = left_axis.direction;
    // Find the corner of the light
    if (cv::Point2f t = findCorner(gray_img, armor.left_light, left_axis, "top"); t.x > 0) {
      armor.left_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.left_light, left_axis, "bottom"); b.x > 0) {
      armor.left_light.bottom = b;
    }
  }

  if (armor.right_light.width > PASS_OPTIMIZE_WIDTH) {
    // Find the symmetry axis of the light
    SymmetryAxis right_axis = findSymmetryAxis(gray_img, armor.right_light);
    armor.right_light.center = right_axis.centroid;
    armor.right_light.axis = right_axis.direction;
    // Find the corner of the light
    if (cv::Point2f t = findCorner(gray_img, armor.right_light, right_axis, "top"); t.x > 0) {
      armor.right_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.right_light, right_axis, "bottom"); b.x > 0) {
      armor.right_light.bottom = b;
    }
  }
}

SymmetryAxis LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img, const lLight &light) {
  constexpr float MAX_BRIGHTNESS = 25;
  constexpr float SCALE = 0.07;

  // Scale the bounding box
  // cv::Rect light_box = light.boundingRect();
  cv::Rect light_box = light;
  light_box.x -= light_box.width * SCALE;
  light_box.y -= light_box.height * SCALE;
  light_box.width += light_box.width * SCALE * 2;
  light_box.height += light_box.height * SCALE * 2;

  // Check boundary
  light_box.x = std::max(light_box.x, 0);
  light_box.x = std::min(light_box.x, gray_img.cols - 1);
  light_box.y = std::max(light_box.y, 0);
  light_box.y = std::min(light_box.y, gray_img.rows - 1);
  light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

  // Get normalized light image
  cv::Mat roi = gray_img(light_box);
  float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // Calculate the centroid
  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
                         cv::Point2f(light_box.x, light_box.y);

  // Initialize the PointCloud
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++) {
    for (int j = 0; j < roi.cols; j++) {
      for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
        points.emplace_back(cv::Point2f(j, i));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);

  // PCA (Principal Component Analysis)
  auto pca = cv::PCA(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Get the symmetry axis
  cv::Point2f axis =
    cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

  // Normalize the axis
  axis = axis / cv::norm(axis);

  if (axis.y > 0) {
    axis = -axis;
  }

  return SymmetryAxis{.centroid = centroid, .direction = axis, .mean_val = mean_val};
}

cv::Point2f LightCornerCorrector::findCorner(const cv::Mat &gray_img,
                                             const lLight &light,
                                             const SymmetryAxis &axis,
                                             std::string order) {
  constexpr float START = 0.8 / 2;
  constexpr float END = 1.2 / 2;

  auto inImage = [&gray_img](const cv::Point &point) -> bool {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 && point.y < gray_img.rows;
  };

  auto distance = [](float x0, float y0, float x1, float y1) -> float {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  };

  int oper = order == "top" ? 1 : -1;
  float L = light.length;
  float dx = axis.direction.x * oper;
  float dy = axis.direction.y * oper;

  std::vector<cv::Point2f> candidates;

  // Select multiple corner candidates and take the average as the final corner
  int n = light.width - 2;
  int half_n = std::round(n / 2);
  for (int i = -half_n; i <= half_n; i++) {
    float x0 = axis.centroid.x + L * START * dx + i;
    float y0 = axis.centroid.y + L * START * dy;

    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    float max_brightness_diff = 0;
    bool has_corner = false;
    // Search along the symmetry axis to find the corner that has the maximum brightness difference
    for (float x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < L * (END - START);
         x += dx, y += dy) {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!inImage(cv::Point(cur))) {
        break;
      }

      float brightness_diff = gray_img.at<uchar>(prev) - gray_img.at<uchar>(cur);
      if (brightness_diff > max_brightness_diff && gray_img.at<uchar>(prev) > axis.mean_val) {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }

      prev = cur;
    }

    if (has_corner) {
      candidates.emplace_back(corner);
    }
  }
  if (!candidates.empty()) {
    cv::Point2f result = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
    return result / static_cast<float>(candidates.size());
  }

  return cv::Point2f(-1, -1);
}
//////////////////////////////////////////////////




class detector {
public:

    cv::dnn::Net net_;
    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
	  detector(Config config );
	  ~detector();
	  std::vector<aArmor> detect(cv::Mat & frame );
    bool Initialized();
    //cv::Mat pre(cv::Mat Aromorimage);
    int classify(cv::Mat &image,aArmor &armor );
    cv::Mat OnEqualizeHist(cv::Mat num_image);
    cv::Mat extractNumbers(cv::Mat &src, cv::Point2f lights_vertices[],bool isSmall);
    cv::Mat binary_img;
    //cv::Rect detect_rect(aArmor & armor);
    //vector<armor> detect(const cv::Mat & input);
    const std::vector<int> & ignore_classes_={} ;
    int detect_color;
    std::vector<new_Identify::lLight> lights_;
    std::vector<new_Identify::aArmor> armors_;
    LightCornerCorrector corrector; // 矫正器
  
private:
  bool initialized = false;
  std::string model_path;
  struct LightParams l;
  struct ArmorParams a;
  int binary_thres;
  std::string onnx_path;
	Resize resize;
	cv::Mat preprocess_Image(cv::Mat img);
  char isArmor(const lLight & light_1, const lLight & light_2);
  bool isLight(const lLight & possible_light);
  bool containLight(
  const lLight & light_1, const lLight & light_2, const std::vector<lLight> & lights);
  std::vector<lLight> findlights(const cv::Mat & img,const cv::Mat & binary_img);
  std::vector<aArmor> matchLights(const cv::Mat & img,const std::vector<lLight> & lights);
  int kFactor;
};

bool detector::Initialized(){return this->Initialized();}




detector::detector(Config config )
{
  
    this->l=config.l;
    this->a=config.a;
    this->binary_thres = config.binary_thres;

    /*
    lenet所使用roi_size(28,28)
    mlp所使用roi_size(20,28)
    */

    this->model_path = "/home/rm/ws_glut_vison2711/Model/lenet1.onnx";  ///home/password-is-robomaster/ws_glut_vison/Model/mlp.onnx
    // this->model_path = "/home/rm/ws_glut_vison/Model/mlp_2.onnx";//韦骏飞


    // this->model_path = "/home/rm/ws_glut_vison/Model/mlp.onnx";//学姐

	  net_ = cv::dnn::readNetFromONNX(model_path);


}

detector::~detector(){}




std::vector<aArmor> detector::detect(Mat & frame) {
   
  binary_img = preprocess_Image(frame);
  lights_ = findlights(frame, binary_img);
 
  armors_ = matchLights(frame,lights_);


  return armors_;
                 
}



cv::Mat detector::preprocess_Image(const cv::Mat img)
{
    cv::Mat gray_img;
    cv::cvtColor(img,gray_img,cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    // cv::threshold(gray_img,binary_img,160,255,cv::THRESH_BINARY);160
    cv::threshold(gray_img,binary_img,220,255,cv::THRESH_BINARY);
    // cv::imshow("ab",binary_img);
    // cv::waitKey(1);
    return binary_img;
}


char detector::isArmor(const lLight & light_1, const lLight & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;

  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  char type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? 'l' : 's';
  } else {
    type = 'i';
  }
  
 

  return type;
}

// std::vector<lLight> detector::findlights(const cv::Mat & img,const cv::Mat & binary_img)
// {
    
//   using std::vector;
//   vector<vector<cv::Point>> contours;
//   vector<cv::Vec4i> hierarchy;
//   cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

//   vector<lLight> lights;
//   for (const auto &contour : contours) {
//     if (contour.size() < 6) continue;
//     auto r_rect = cv::minAreaRect(contour);
//     auto light = lLight(contour);

//     if (isLight(light)) {
//       auto rect = light.boundingRect();
//       if (//Avoid assertion failed 确保 ROI 不会超过原始图像的边界
//         0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= img.cols && 0 <= rect.y &&
//         0 <= rect.height && rect.y + rect.height <= img.rows) {
//         int sum_r = 0, sum_b = 0;
//         auto roi = img(rect);
//         // Iterate through the ROI
//         //遍历灯条区域，得到其中不同像素之和用于判断
//         for (int i = 0; i < roi.rows; i++) {
//           for (int j = 0; j < roi.cols; j++) {
//             if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
//               // if point is inside contour
//               sum_r += roi.at<cv::Vec3b>(i, j)[0];
//               sum_b += roi.at<cv::Vec3b>(i, j)[2];
//             }
//           }
//         }
//         //计算红色像素和蓝色像素的总和来推断灯光的颜色
//         // Sum of red pixels > sum of blue pixels ?
//         light.class_id = sum_r > sum_b ? 0:1;
//         lights.emplace_back(light);
//       }
//     }
//   }
  
//   std::sort(lights.begin(), lights.end(), [](const lLight &l1, const lLight &l2) {
//     return l1.center.x < l2.center.x;
//   });
//   return lights;
// }


// 用于检测灯条的C++函数,主要处理二值化图像中提取轮廓并识别灯条
std::vector<lLight> detector::findlights(const cv::Mat & rgb_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  // 从二值图像中提取轮廓,只检测最外层轮廓,压缩轮廓,只保留关键点
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 初始化
  vector<lLight> lights;
  // this->debug_lights.data.clear();

  // 遍历所有轮廓
  for (const auto & contour : contours) {
    // 跳过点数小于5的轮廓
    if (contour.size() < 5) continue;

    // 获取轮廓的轴对齐边界矩形
    auto b_rect = cv::boundingRect(contour);
    // 获取轮廓的最小面积旋转矩形
    auto r_rect = cv::minAreaRect(contour);

    

    // 创建轮廓掩码
    // 创建与边界矩形同大小的掩码
    // 将轮廓点坐标转换位掩码局部坐标
    // 填充轮廓区域为白色
    cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
    std::vector<cv::Point> mask_contour;
   
    for (const auto & p : contour) {
      mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
      // cout << "b_rect.x: " << b_rect.x << "b_rect.y: " << b_rect.y << endl;
      // cout << "mask_contour.x: " << mask_contour.back().x << "mask_contour.y: " << mask_contour.back().y << endl;
    }

    // 使用drawContours填充
    std::vector<std::vector<cv::Point>> contours_to_fill = {mask_contour};
    cv::drawContours(mask, contours_to_fill, 0, 255, cv::FILLED);

    
    // cv::fillPoly(mask, {mask_contour}, 255);

    //  cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << endl;

    

    // 计算填充率
    // 找到掩码中所有非零点(轮廓内部点)
    // 计算填充率: 实际轮廓面积/旋转矩形面积
    // 判断是否达到最小填充率阈值
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    // points / rotated rect area
    bool is_fill_rotated_rect =
      points.size() / (r_rect.size.width * r_rect.size.height) > l.min_fill_ratio;
    cv::Vec4f return_param;

    // 拟合直线(确定灯条方向)
    // 使用最小二乘法拟合直线
    cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
    
    // 计算灯条上下端点和角度
    cv::Point2f top, bottom;
    double angle_k;
    if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
      // 处理垂直线特殊情况
      top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
      bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
      angle_k = 0;
    } else {
      // 计算直线与边界矩形上下边的交点
      auto k = return_param[1] / return_param[0];
      auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
      top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
      bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
      // 计算角度(相对于垂直方向)
      angle_k = std::atan(k) / CV_PI * 180 - 90;
      if (angle_k > 90) {
        angle_k = 180 - angle_k;
      }
    }

    // 创建Light对象
    auto light = lLight(b_rect, top, bottom, points.size(), angle_k);

    // 筛选灯条并判断颜色
    if (isLight(light) && is_fill_rotated_rect) {
      auto rect = light;

      // 安全检查,确保ROI在图像范围内
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rgb_img(rect);
        // Iterate through the ROI
        //遍历ROI,只同济轮廓内部的像素
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // 判断颜色: 红色多还是蓝色多
        // Sum of red pixels > sum of blue pixels ?
        light.class_id = sum_r > sum_b ? 0 : 1;
        lights.emplace_back(light);
      }
    }
  }

  // 返回结果
  return lights;
}

bool detector::isLight(const lLight & light)
{
    // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  

  return is_light;

}
float cal_ang(Point2f point_1, Point2f point_2, Point2f point_3)
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



std::vector<aArmor> detector::matchLights(const cv::Mat & img,const std::vector<lLight> & lights)
{
  std::vector<aArmor> armors;

  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      	
      if (light_1->class_id != this->detect_color || light_2->class_id != this->detect_color
                                       ){ continue;}
  
      double max_iter_width = light_1->length * a.max_large_center_distance;
      if((light_2->center.x-light_1->center.x)>max_iter_width) {continue;}

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      char type = isArmor(*light_1, *light_2);
      if (type != 'i') {
        auto armor = aArmor(*light_1, *light_2);
         double l_angle =
        std::atan2(armor.left_light.center.y, armor.left_light.center.x) * 180 / M_PI;
        double r_angle =
        std::atan2(armor.right_light.center.y, armor.right_light.center.x) * 180 /M_PI;
        double angle = (l_angle + r_angle) / 2;
        angle += 90.0;
        armor.type = type;
        armor.angle=angle;
        armors.emplace_back(armor);
      }
    }
  
  }

  return armors;
}


// Check if there is another light in the boundingRect formed by the 2 lights
bool detector::containLight(
  const lLight & light_1, const lLight & light_2, const std::vector<lLight> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const lLight & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;
     double avg_length = (light_1.length + light_2.length) / 2.0;
     double avg_width = (light_1.width + light_2.width) / 2.0;
    // 防止数字干扰
    if (test_light.width > 2 * avg_width) {
      continue;
    }
    
    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) || bounding_rect.contains(test_light.center)
     ) {
      return true;
    }
  }

  return false;
}



int detector::classify(cv::Mat &image, aArmor &armor)
{
	//cv::Mat image = cv::imread("");
	image = image / 255.0;
	cv::Mat blob;
	cv::dnn::blobFromImage(image, blob);
	net_.setInput(blob);
	cv::Mat outputs = net_.forward();
	float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
	cv::Mat softmax_prob;
	cv::exp(outputs - max_prob, softmax_prob);
	float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
	softmax_prob /= sum;
	double confidence;
	cv::Point class_id_point;
	minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
	int label_id = class_id_point.x;
  armor.confidence = confidence;



  std::stringstream result_ss;
  result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
  armor.classfication_result = result_ss.str();
 

  if (class_names_[label_id]== 5) {
          return 9;          
      }
    
    return class_names_[label_id];
}


cv::Mat detector::OnEqualizeHist(cv::Mat num_image)
{
     cv::Mat result_num;
     std::vector<Mat> splitBGR(num_image.channels());

     
     split(num_image,splitBGR);

     for (int i=0; i <num_image.channels(); i++)
     {
       cv::equalizeHist(splitBGR[i],splitBGR[i]);//图像灰度均衡化
     }

     merge(splitBGR,result_num);
     
     return result_num;
}


cv::Mat detector::extractNumbers(cv::Mat &src, cv::Point2f lights_vertices[],bool isSmall)
{
	
	// Number ROI size
	// const cv::Size roi_size(20, 28);   // 这里是mlp
	const cv::Size roi_size(28, 28);      // 这里是LeNet
	int top_light_y = (warp_height - light_length) / 2 - 1;
	int bottom_light_y = top_light_y + light_length;
	int warp_width;
	if (isSmall)
	{
		warp_width = small_armor_width;
	}
	warp_width = large_armor_width;
	cv::Point2f target_vertices[4] = {
	  cv::Point(0, bottom_light_y),
	  cv::Point(0, top_light_y),
	  cv::Point(warp_width - 1, top_light_y),
	  cv::Point(warp_width - 1, bottom_light_y),
	};
	cv::Mat number_image;
	auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
	cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
	// Get ROI
	number_image =                                           
		number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));


  
	// Binarize
	cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
	cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);


  // cv::imshow("a",number_image);
  // cv::waitKey(1);

	return number_image;
}





}






