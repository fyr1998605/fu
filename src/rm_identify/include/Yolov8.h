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

using namespace cv;
using namespace std;
using namespace dnn;
#define Mutiply 1.05
namespace Identify{
struct Config {
	float confThreshold;
	float nmsThreshold;
	float scoreThreshold;
	int inpWidth;
	int inpHeight;
	std::string onnx_path;
};

struct Resize
{
	cv::Mat resized_image;
	int dw;
	int dh;
};

struct Detection {
	int class_id;
	float confidence;
	cv::Rect box;
};
// const std::vector<int> class_names_ = {
//     1,
//     2,
//     3,
//     4,
//     5,
//     6,    // outpost
//     7,    // guard
//     8,    // base
//     9     // negative
// };

const std::vector<int> class_names_ = {
    1,
    2,
    3,
    4,
    5,
    7    // guard
};
class YOLOV8 {
public:
    cv::dnn::Net net_;
    
    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
	YOLOV8(Config config);
	~YOLOV8();
	vector<Detection> detect(cv::Mat& frame);
    bool Initialized();
    cv::Mat pre(cv::Mat Aromorimage);
    int classify(cv::Mat &image);
    cv::Mat extractNumbers(cv::Mat &src, cv::Point2f lights_vertices[],bool isSmall);
    

private:
    bool initialized = false;
	float confThreshold;
	float nmsThreshold;
	float scoreThreshold;
	int inpWidth;
	int inpHeight;
    std::string model_path;
	float rx;   // the width ratio of original image and resized image
	float ry;   // the height ratio of original image and resized image
	std::string onnx_path;
	Resize resize;
	ov::Tensor input_tensor;
	ov::InferRequest infer_request;
	ov::CompiledModel compiled_model;
	void initialmodel();
	void preprocess_img(cv::Mat& frame);
	vector<Detection> postprocess_img(cv::Mat& frame, float* detections, ov::Shape & output_shape);
};

const vector<string> coconame = {
        // "red1",
        // "red2",
        // "red3",
        // "red4",
        // "red5",
        // "red_sentry",
        // "blue1",
        // "blue2",
        // "blue3",
        // "blue4",
        // "blue5",
        // "blue_sentry"
        "red",
        "blue"
        // "red1",
        // "blue1",
        // "red2",
        // "blue2",
        // "red3",
        // "blue3",
        // "red4",
        // "blue4"

        // "Red_station",
        // "Blue_station"
 };
//   const vector<string> coconame={
//      "car"
//   };
 const int parameter = coconame.size()+4;
bool YOLOV8::Initialized(){return this->initialized;}
YOLOV8::YOLOV8(Config config) {
	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;
    this->scoreThreshold = config.scoreThreshold;
	this->inpWidth = config.inpWidth;
	this->inpHeight = config.inpHeight;
	this->onnx_path = config.onnx_path;
	this->model_path = "/home/rm/ws_glut_vison2711/src/rm_node/model/number/mlp.onnx";
	net_ = cv::dnn::readNetFromONNX(model_path);


    this->initialmodel();
}
YOLOV8::~YOLOV8(){}
vector<Detection> YOLOV8::detect(Mat & frame) {
    preprocess_img(frame);
    //cout << "前处理成功" << endl;
    // chrono::steady_clock::time_point start = chrono::steady_clock::now();
    infer_request.infer();
    // chrono::steady_clock::time_point end = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(end - start);
    // cout << ".******************FPS = " << 1/time_used.count() << endl;
    const ov::Tensor& output_tensor = infer_request.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();
    float* detections = output_tensor.data<float>();
    //cout << "准备后处理" << endl;
    return this->postprocess_img(frame, detections, output_shape);
}

cv::Mat YOLOV8::pre(cv::Mat Aromorimage)
{
	cv::Mat number_image;
	// Light length in image
	const int light_length = 12;
	// Image size after warp
	const int warp_height = 28;
	const int small_armor_width = 32;
	const int large_armor_width = 54;
	// Number ROI size
	const cv::Size roi_size(20, 28);

	cv::resize(Aromorimage, number_image, roi_size);
	// Binarize
	cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
	cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	// cout << "ǰ�����ɹ�" << endl;
	return number_image;
}

int YOLOV8::classify(cv::Mat &image)
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
	/*cout << outputs.cols << endl;
	cout << outputs.rows << endl;*/

	//armor.confidence = confidence;
	//armor.number = class_names_[label_id];
	//number = class_names_[label_id];
	// std::stringstream result_ss;
	// cout << "number" << ": " << label_id << endl;
	// cout << std::fixed << std::setprecision(1)
	// 	<< confidence * 100.0 << "%" << endl;
	/*armor.classfication_result = result_ss.str();*/
    // if(label_id > 5)
    //     return -1;pub
    return class_names_[label_id];
}

cv::Mat YOLOV8::extractNumbers(cv::Mat &src, cv::Point2f lights_vertices[],bool isSmall)
{
	
	// Number ROI size
	const cv::Size roi_size(20, 28);
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
	// cv::imshow("number_image", number_image);
	return number_image;
}

void YOLOV8::initialmodel() {
    ov::Core core;
    std::shared_ptr<ov::Model> model = core.read_model(this->onnx_path);
    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);

    ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::RGB);
    ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::BGR).scale({ 255, 255, 255 });// .scale({ 112, 112, 112 });
    ppp.input().model().set_layout("NCHW");
    ppp.output().tensor().set_element_type(ov::element::f32);
    model = ppp.build();
    this->compiled_model = core.compile_model(model, "GPU");
    this->infer_request = compiled_model.create_infer_request();
    initialized = true;
}

void YOLOV8::preprocess_img(Mat& frame) {
    try {
        float width = frame.cols;
        float height = frame.rows;
        cv::Size new_shape = cv::Size(inpWidth, inpHeight);
        float r = float(new_shape.width / max(width, height));
        int new_unpadW = int(round(width * r));
        int new_unpadH = int(round(height * r));

        // cv::resize(frame, resize.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, cv::INTER_AREA);
        cv::resize(frame, resize.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, 1);
        resize.resized_image = resize.resized_image;
        resize.dw = new_shape.width - new_unpadW;
        resize.dh = new_shape.height - new_unpadH;
        cv::Scalar color = cv::Scalar(100, 100, 100);
        cv::copyMakeBorder(resize.resized_image, resize.resized_image, 0, resize.dh, 0, resize.dw, cv::BORDER_CONSTANT, color);

        this->rx = (float)frame.cols / (float)(resize.resized_image.cols - resize.dw);
        this->ry = (float)frame.rows / (float)(resize.resized_image.rows - resize.dh);
        float* input_data = (float*)resize.resized_image.data;
        input_tensor = ov::Tensor(compiled_model.input().get_element_type(), compiled_model.input().get_shape(), input_data);
        infer_request.set_input_tensor(input_tensor);
    }catch (const std::exception& e) {
        std::cerr << "exception: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "unknown exception" << std::endl;
    }

}
template<class T>
T Safety_Case(T a,T min,T max){
    if(a<min)
        return min;
    else if(a>max)
        return max;
    else
        return a;
}
vector<Detection> YOLOV8::postprocess_img(Mat& frame, float* detections, ov::Shape & output_shape) {
    //cout << " 正在进行后处理" << endl;
    std::vector<cv::Rect> boxes;
    vector<int> class_ids;
    vector<float> confidences;
    int out_rows = output_shape[1];
    int out_cols = output_shape[2];
    const cv::Mat det_output(out_rows, out_cols, CV_32F, (float*)detections);
    //cout << out_rows << " " << out_cols << endl;
    //cout << " 正在进行后处理1" << endl;
    for (int i = 0; i < det_output.cols; ++i) {
        //这里有问题！！！！！！！！！！！！！
        const cv::Mat classes_scores = det_output.col(i).rowRange(4, parameter);
        //上面
        //cout << " 正在进行后处理1.1" << endl;
        cv::Point class_id_point;
        double score;
        cv::minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id_point);
        //cout << " 正在进行后处理1.2" << endl;
        if (score > 0.3) {
            const float cx = det_output.at<float>(0, i);
            const float cy = det_output.at<float>(1, i);
            const float ow = det_output.at<float>(2, i);
            const float oh = det_output.at<float>(3, i);
            cv::Rect box;
            box.x = static_cast<int>((cx - 0.5 * ow) );
            box.y = static_cast<int>((cy - 0.5 * oh));
            box.width = static_cast<int>(ow);
            box.height = static_cast<int>(oh);

            boxes.push_back(box);
            class_ids.push_back(class_id_point.y);
            confidences.push_back(score);
        }
    }
    //cout << " 正在进行后处理2" << endl;
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, nms_result);
    //cout << " 正在进行后处理3" << endl;
    std::vector<Detection> output;
    for (int i = 0; i < nms_result.size(); i++)
    {
        Detection result;
        int idx = nms_result[i];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result); 
    }
    //cout<<"output_size:"<<output.size()<< endl;

    vector<Detection> out;
    for (int i = 0; i < output.size(); i++)
    {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;
        // if (classId != 0) continue;
        auto confidence = detection.confidence;

        box.x = this->rx * box.x ;
        box.y = this->ry * box.y ;
        if(box.x < 0 || box.y < 0)
            continue;
        box.width = this->rx * box.width ;
        box.height = this->ry * box.height ;
        if((box.x + box.width) > 1280 || (box.y + box.height) > 1024)
            continue;
        // box.x = Safety_Case((int)(box.x + box.width/2 - std::sqrt(box.width*box.width + box.height*box.height)/2 *Mutiply),0,1024);
        // box.y = Safety_Case((int)(box.y + box.height/2 - std::sqrt(box.width*box.width + box.height*box.height)/2 *Mutiply),0,1280);
        // box.width = Safety_Case((int)(2*std::sqrt(box.width*box.width + box.height*box.height)/2 *Mutiply),0,1024);
        // box.height = Safety_Case((int)(2*std::sqrt(box.width*box.width + box.height*box.height)/2 *Mutiply),0,1280);
        float xmax = box.x + box.width;
        float ymax = box.y + box.height;

        // detection box
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(100, 255);
        cv::Scalar color=  cv::Scalar(dis(gen),
            dis(gen),
            dis(gen));

        if(coconame[classId][0] == 'r')
            color = Scalar(0,0,255);
        else if(coconame[classId][0] == 'b')
            color = Scalar(255,0,0);    

        // cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(xmax, ymax), color, 3);

        Detection out_;
        out_.box = box;
        out_.class_id = classId;
        out_.confidence = confidence;
        out.push_back(out_);

        // Detection box text
        std::string classString = coconame[classId] + ' ' + std::to_string(confidence).substr(0, 4);
        cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);
        // cv::rectangle(frame, textBox, color, cv::FILLED);
        // cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);

        
        // cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(xmax, box.y), cv::Scalar(0, 255, 0), cv::FILLED);
        // cv::putText(frame, coconame[classId], cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    return out;
}
}
