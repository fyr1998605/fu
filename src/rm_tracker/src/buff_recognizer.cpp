#include "buff_recognizer.hpp"
#include "ros2_compat.hpp"
#include <algorithm>
#include <cmath>
#include <random>

// ============================================================================
//  YoloBuffDetector 实现 — OpenVINO YOLO11 推理
// ============================================================================

YoloBuffDetector::YoloBuffDetector(const std::string& model_path,
                                   float conf_threshold,
                                   float nms_threshold,
                                   int   enemy_class)
    : conf_threshold_(conf_threshold)
    , nms_threshold_(nms_threshold)
    , enemy_class_idx_(enemy_class)
{
    initModel(model_path);
}

// 加载 OpenVINO 模型: read_model → compile_model(CPU) → create_infer_request
void YoloBuffDetector::initModel(const std::string& model_path) {
    ROS_INFO("[BuffRecognizer] 加载模型: %s", model_path.c_str());
    model_ = core_.read_model(model_path);
    compiled_model_ = core_.compile_model(model_, "CPU");
    infer_request_ = compiled_model_.create_infer_request();
    ROS_INFO("[BuffRecognizer] 模型加载成功, 输入shape: [1,3,640,640]");
}

// 设置敌方颜色 → 转为 class 行号
// "blue"→我方red→识别class0(red行=4), "red"→我方blue→识别class1(blue行=5)
void YoloBuffDetector::setEnemyColor(const std::string& color) {
    enemy_class_idx_ = (color == "blue") ? 0 : 1;
    ROS_INFO("[BuffRecognizer] 敌方颜色: %s → 识别class: %s",
             color.c_str(), enemy_class_idx_ == 0 ? "red" : "blue");
}

// ====== ★ 核心: 9个原始关键点 → 6个标准关键点 ======
// 对边取中点, 平均掉单点噪声, 提升 PnP 稳定性
// raw[0]=R中心  raw[4,5]=顶边  raw[6,7]=左边  raw[1,8]=底边  raw[2,3]=右边
// kpt[0]=顶边中点 kpt[1]=左边中点 kpt[2]=底边中点 kpt[3]=右边中点
// kpt[4]=靶心(四边均值) kpt[5]=R中心(raw[0]直接透传)
std::vector<cv::Point2f> YoloBuffDetector::remapKeypoints(const std::vector<cv::Point2f>& raw) const {
    const cv::Point2f top    = (raw[4] + raw[5]) * 0.5f;  // 顶边中点
    const cv::Point2f left   = (raw[6] + raw[7]) * 0.5f;  // 左边中点
    const cv::Point2f bottom = (raw[1] + raw[8]) * 0.5f;  // 底边中点
    const cv::Point2f right  = (raw[2] + raw[3]) * 0.5f;  // 右边中点

    std::vector<cv::Point2f> out(NUM_REMAP_KPTS);
    out[0] = top;                                      // 顶边中点 → corners[0]/PnP
    out[1] = left;                                     // 左边中点 → corners[1]/PnP
    out[2] = bottom;                                   // 底边中点 → corners[2]/PnP
    out[3] = right;                                    // 右边中点 → corners[3]/PnP
    out[4] = (top + left + bottom + right) * 0.25f;   // ★ 靶心 = 扇叶中心 → center_Fan_Box
    out[5] = raw[0];                                   // ★ R中心(模型直出) → 精化 → center_R_Box
    return out;
}

// ====== 图像预处理: OpenCV Mat → OpenVINO NCHW Tensor ======
// 等比缩放(scale=min(640/H,640/W)) → BGR→RGB → /255归一化 → 填充 [1,3,640,640]
// 返回 factor=1/scale, 用于将模型输出坐标还原到原图
float YoloBuffDetector::fillTensor(cv::Mat& input_image) {
    const size_t channels = 3;
    const size_t height   = 640;
    const size_t width    = 640;

    const float scale = std::min(
        height / static_cast<float>(input_image.rows),
        width  / static_cast<float>(input_image.cols));

    cv::Mat blob;
    cv::Matx23f matrix{scale, 0.0, 0.0, 0.0, scale, 0.0};
    if (scale < 1.0f) {
        // 图像>640: 先缩小(省算力)再归一化
        cv::warpAffine(input_image, blob, matrix, cv::Size(width, height));
        blob.convertTo(blob, CV_32F, 1.0 / 255.0);
        cv::cvtColor(blob, blob, cv::COLOR_BGR2RGB);
    } else {
        // 图像<640: 先归一化再放大
        input_image.convertTo(blob, CV_32F, 1.0 / 255.0);
        cv::cvtColor(blob, blob, cv::COLOR_BGR2RGB);
        cv::warpAffine(blob, blob, matrix, cv::Size(width, height));
    }

    // 填充 NCHW tensor: data[c*W*H + h*W + w] = blob[h][w][c]
    auto input_tensor = infer_request_.get_input_tensor();
    input_tensor.set_shape({1, channels, height, width});
    float* data = input_tensor.data<float>();
    for (size_t c = 0; c < channels; c++) {
        for (size_t h = 0; h < height; h++) {
            for (size_t w = 0; w < width; w++) {
                data[c * width * height + h * width + w] =
                    blob.at<cv::Vec<float, 3>>(h, w)[c];
            }
        }
    }
    return 1.0f / scale;  // 坐标还原因子
}

// ====== 取置信度最高的单个检测框 (小符用) ======
// 遍历8400列, 在 class_row=4+enemy_class_idx_ 行取最高置信度 → 提取bbox+9关键点 → remap 9→6
std::vector<BuffDetection> YoloBuffDetector::getBestBox(cv::Mat& image) {
    if (image.empty()) {
        ROS_WARN("[BuffRecognizer] 输入图像为空!");
        return {};
    }

    // Step 1: 预处理 + 推理
    int64 start = cv::getTickCount();
    float factor = fillTensor(image);
    infer_request_.infer();

    // Step 2: 解析输出 tensor [33, 8400]
    // 行0-3=bbox(cx,cy,w,h), 行4-5=class(red,blue), 行6-32=9kpt×3(x,y,v)
    const ov::Tensor output      = infer_request_.get_output_tensor();
    const ov::Shape  out_shape   = output.get_shape();
    const float*     out_buffer  = output.data<const float>();
    const int        out_rows    = out_shape[1];
    const int        out_cols    = out_shape[2];
    const cv::Mat    det_output(out_rows, out_cols, CV_32F, (float*)out_buffer);

    const int class_row = 4 + enemy_class_idx_;   // red→行4, blue→行5
    const int kpt_rows  = NUM_RAW_KPTS * KPT_DIM; // 27行

    // Step 3: 找最高置信度
    int   best_idx   = -1;
    float best_conf  = 0.0f;
    for (int i = 0; i < det_output.cols; ++i) {
        float score = det_output.at<float>(class_row, i);
        if (score > best_conf) {
            best_conf = score;
            best_idx  = i;
        }
    }

    // Step 4: 置信度过滤
    std::vector<BuffDetection> results;
    if (best_conf <= conf_threshold_ || best_idx < 0) {
        ROS_WARN_THROTTLE(2.0, "[BuffRecognizer] 未检测到目标, max_conf=%.4f < thr=%.2f (class_row=%d)",
                  best_conf, conf_threshold_, class_row);
        return results;
    }

    // Step 5: 提取检测框 + 坐标还原 (模型输出归一化坐标×factor→原图像素坐标)
    BuffDetection det;
    float cx = det_output.at<float>(0, best_idx);
    float cy = det_output.at<float>(1, best_idx);
    float ow = det_output.at<float>(2, best_idx);
    float oh = det_output.at<float>(3, best_idx);
    det.rect  = cv::Rect_<float>(
        (cx - 0.5f * ow) * factor,
        (cy - 0.5f * oh) * factor,
        ow * factor, oh * factor);
    det.prob  = best_conf;
    det.label = enemy_class_idx_;

    // Step 6: ★ 提取9个原始关键点 + remap 9→6
    std::vector<cv::Point2f> raw_kpts(NUM_RAW_KPTS);
    cv::Mat kpts_mat = det_output.col(best_idx).rowRange(KPT_START, KPT_START + kpt_rows);
    for (int j = 0; j < NUM_RAW_KPTS; ++j) {
        raw_kpts[j].x = kpts_mat.at<float>(j * KPT_DIM + 0, 0) * factor;
        raw_kpts[j].y = kpts_mat.at<float>(j * KPT_DIM + 1, 0) * factor;
    }
    det.kpt = remapKeypoints(raw_kpts);
    results.push_back(det);

    // Step 7: 画面标注
    cv::rectangle(image, det.rect, cv::Scalar(255, 255, 255), 1);
    std::string label = (enemy_class_idx_ == 0 ? "red" : "blue")
                      + std::string(":") + std::to_string(best_conf).substr(0, 4);
    cv::putText(image, label, cv::Point(det.rect.x, det.rect.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

    float t = (cv::getTickCount() - start) / cv::getTickFrequency();
    ROS_INFO_THROTTLE(1.0, "[BuffRecognizer] getBestBox耗时: %.1fms, 置信度: %.3f", t * 1000, best_conf);
    return results;
}

// ====== 取多个候选框 + NMS 去重 (大符用) ======
// 收集所有置信度>阈值的候选 → OpenCV NMSBoxes 去重 → 返回所有通过筛选的框
std::vector<BuffDetection> YoloBuffDetector::getMultiBoxes(cv::Mat& image) {
    if (image.empty()) {
        ROS_WARN("[BuffRecognizer] 输入图像为空!");
        return {};
    }

    int64 start = cv::getTickCount();
    float factor = fillTensor(image);
    infer_request_.infer();

    const ov::Tensor output     = infer_request_.get_output_tensor();
    const ov::Shape  out_shape  = output.get_shape();
    const float*     out_buffer = output.data<const float>();
    const int        out_rows   = out_shape[1];
    const int        out_cols   = out_shape[2];
    const cv::Mat    det_output(out_rows, out_cols, CV_32F, (float*)out_buffer);

    const int class_row = 4 + enemy_class_idx_;
    const int kpt_rows  = NUM_RAW_KPTS * KPT_DIM;

    // 收集所有置信度>阈值的候选
    std::vector<cv::Rect>    boxes;
    std::vector<float>       confidences;
    std::vector<std::vector<cv::Point2f>> all_kpts;

    for (int i = 0; i < det_output.cols; ++i) {
        float score = det_output.at<float>(class_row, i);
        if (score > conf_threshold_) {
            float cx = det_output.at<float>(0, i);
            float cy = det_output.at<float>(1, i);
            float ow = det_output.at<float>(2, i);
            float oh = det_output.at<float>(3, i);
            boxes.push_back(cv::Rect(
                (cx - 0.5f * ow) * factor,
                (cy - 0.5f * oh) * factor,
                ow * factor, oh * factor));
            confidences.push_back(score);

            std::vector<cv::Point2f> raw_kpts(NUM_RAW_KPTS);
            cv::Mat kpts_mat = det_output.col(i).rowRange(KPT_START, KPT_START + kpt_rows);
            for (int j = 0; j < NUM_RAW_KPTS; ++j) {
                raw_kpts[j].x = kpts_mat.at<float>(j * KPT_DIM + 0, 0) * factor;
                raw_kpts[j].y = kpts_mat.at<float>(j * KPT_DIM + 1, 0) * factor;
            }
            all_kpts.push_back(remapKeypoints(raw_kpts));
        }
    }

    // NMS 去重: 按置信度排序, IoU>nms_threshold_的视为重复
    std::vector<int> nms_idx;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, nms_idx);

    std::vector<BuffDetection> results;
    for (int idx : nms_idx) {
        BuffDetection det;
        det.rect  = boxes[idx];
        det.prob  = confidences[idx];
        det.label = enemy_class_idx_;
        det.kpt   = all_kpts[idx];
        results.push_back(det);

        cv::rectangle(image, det.rect, cv::Scalar(255, 255, 255), 1);
    }

    float t = (cv::getTickCount() - start) / cv::getTickFrequency();
    ROS_DEBUG("[BuffRecognizer] 多框推理: %zu个候选, 耗时%.1fms",
              results.size(), t * 1000);
    return results;
}

// ============================================================================
//  BuffRecognizer 实现
// ============================================================================

BuffRecognizer::BuffRecognizer(const std::string& model_path,
                               const std::string& enemy_color)
    : detector_(model_path, 0.2f, 0.2f, (enemy_color == "blue" ? 0 : 1))
    //            模型路径    置信度 NMS    enemy_class: blue→0(red行), red→1(blue行)
{
    ROS_INFO("[BuffRecognizer] 初始化完成, model=%s, enemy=%s",
             model_path.c_str(), enemy_color.c_str());
}

// ====== 图像预处理: 灰度→二值化→膨胀 ======
// 用于 refineRCenter 中提取装甲板白色区域
void BuffRecognizer::handleImage(const cv::Mat& bgr_img, cv::Mat& dilated_img) {
    cv::Mat gray_img;
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);              // ① 灰度化
    cv::threshold(gray_img, dilated_img, binary_threshold_,            // ② 二值化(阈值120)
                  255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(dilate_kernel_size_,                  // ③ 膨胀(5×5核)
                                  dilate_kernel_size_));               //   连接R字母断裂
    cv::dilate(dilated_img, dilated_img, kernel);
}

// ====== ★ R中心精化 ======
// 模型R中心有2~5px抖动 → R中心差1px≈角度差0.5° → 用传统CV做二次精化
//
// 流程:
//   1) kpt[5]=raw[0] → 模型R中心初值
//   2) handleImage() → 灰度+二值化+膨胀 → 提取白色区域
//   3) 以模型R中心为圆心, radius=|靶心-R中心|×0.8 做圆形mask → 滤除远端干扰
//   4) findContours → 每轮廓取minAreaRect → 评分=长宽比+距离惩罚 → 选最优
//   5) 限幅: 位移>5px则截断, 防止CV误检导致跳变
cv::Point2f BuffRecognizer::refineRCenter(const cv::Mat& bgr_img,
                                           const std::vector<cv::Point2f>& kpts) {
    if (kpts.empty() || kpts.size() < 6) {
        ROS_WARN("[BuffRecognizer] 关键点不足, 无法精化R中心");
        return {0, 0};
    }

    // Step 1: 模型R中心初值
    cv::Point2f r_center_model = kpts[5];

    // Step 2: 传统CV预处理 → 二值图
    cv::Mat dilated_img;
    handleImage(bgr_img, dilated_img);

    // Step 3: 圆形 mask 约束 (半径=|底边中点-靶心|×0.8)
    double radius = cv::norm(kpts[2] - kpts[4]) * r_center_mask_ratio_;
    cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);
    circle(mask, r_center_model, static_cast<int>(radius), cv::Scalar(255), -1);
    cv::bitwise_and(dilated_img, mask, dilated_img);

    // Step 4: 找轮廓 + 评分筛选
    // 评分 = 长宽比(越接近正方形越好) + 距模型初值距离/惩罚因子(越近越好)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    cv::Point2f r_center_refined = r_center_model;
    double best_cost = INFINITY;
    for (const auto& cnt : contours) {
        auto rr = cv::minAreaRect(cnt);
        double ratio = rr.size.height > rr.size.width
                     ? rr.size.height / rr.size.width
                     : rr.size.width / rr.size.height;
        ratio += cv::norm(rr.center - r_center_model) / (radius / r_center_contour_div_);
        if (ratio < best_cost) {
            best_cost = ratio;
            r_center_refined = rr.center;
        }
    }

    // Step 5: 限幅保护 (≤5px)
    cv::Point2f correction = r_center_refined - r_center_model;
    float corr_norm = cv::norm(correction);
    if (corr_norm > r_center_refine_max_shift_ && corr_norm > 1e-6f) {
        r_center_refined = r_center_model
            + correction * (r_center_refine_max_shift_ / corr_norm);
    }

    ROS_DEBUG("[BuffRecognizer] R中心精化: model(%d,%d) → refined(%d,%d), shift=%.1fpx",
              (int)r_center_model.x, (int)r_center_model.y,
              (int)r_center_refined.x, (int)r_center_refined.y, corr_norm);
    return r_center_refined;
}

int BuffRecognizer::selectTemporalDetection(
    const std::vector<BuffDetection>& detections) const {
    std::vector<BuffTemporalCandidate> candidates;
    candidates.reserve(detections.size());
    for (const BuffDetection& detection : detections) {
        candidates.push_back({
            {detection.kpt[5].x, detection.kpt[5].y},
            {detection.kpt[4].x, detection.kpt[4].y},
            detection.prob});
    }

    float association_gate_px = 0.0f;
    if (has_prev_) {
        association_gate_px = BuffDetectionStabilizer::associationGateForRadius(
            static_cast<float>(last_track_radius_));
    }

    // ── Ego-motion compensation (diagnostic version, Section 4.2/Stage C) ──
    // Without compensation, gimbal motion alone can push the correct target
    // beyond the association gate (~54-68 px). A 2° yaw change ≈ 60 px at
    // fx≈1725, easily exceeding the gate.
    //
    // Diagnostic approximation: Δu ≈ fx × Δyaw,  Δv ≈ fy × (-Δpitch)
    // Formal implementation should use IMU poses, extrinsics, and
    // cv::undistortPoints / cv::projectPoints.
    //
    // Camera intrinsics and gimbal feedback are injected via setEgoMotionIntrinsics()
    // and feedGimbalFeedback() so this file does not depend on Fu.cpp globals.
    BuffPoint compensated_prev_fan = {last_Fan_center_.x, last_Fan_center_.y};
    if (has_prev_ && fx_ > 0.0f && fy_ > 0.0f &&
        std::isfinite(prev_gimbal_pitch_) && std::isfinite(prev_gimbal_yaw_) &&
        std::isfinite(current_gimbal_pitch_) && std::isfinite(current_gimbal_yaw_)) {
        const float dpitch = current_gimbal_pitch_ - prev_gimbal_pitch_;
        const float dyaw   = current_gimbal_yaw_   - prev_gimbal_yaw_;
        // Only compensate when deltas are physically plausible.
        // A single-frame gimbal change >15° (~450 px at fx≈1725) is a sensor glitch,
        // not real motion — skip compensation rather than corrupting the gate.
        if (std::abs(dpitch) < 0.262f && std::abs(dyaw) < 0.262f) {  // 15° in radians
            // Small-angle approximation for gimbal rotation:
            //   yaw positive (camera turns right) → image content shifts left  → Δu ≈ -fx·Δyaw
            //   pitch positive (camera tilts up)  → image content shifts down → Δv ≈ +fy·Δpitch
            compensated_prev_fan.x -= fx_ * dyaw;
            compensated_prev_fan.y += fy_ * dpitch;
        }
    }

    const int index = BuffDetectionStabilizer::selectCandidate(
        candidates, compensated_prev_fan, has_prev_, association_gate_px);
    return index;
}

BuffStabilizedGeometry BuffRecognizer::stabilizeDetection(
    const cv::Point2f& raw_r_center,
    const cv::Point2f& raw_fan_center,
    const std::vector<cv::Point2f>& raw_corners) {
    std::array<BuffPoint, 4> corners;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        corners[i] = {raw_corners[i].x, raw_corners[i].y};
    }
    return stabilizer_.update(
        {{raw_r_center.x, raw_r_center.y}, {raw_fan_center.x, raw_fan_center.y}, 0.0f},
        corners);
}

void BuffRecognizer::resetTemporalState() {
    const bool require_reacquisition =
        has_prev_ || stabilizer_.isAwaitingReacquisition();
    stabilizer_.reset();
    if (require_reacquisition) stabilizer_.beginReacquisition();
    has_prev_ = false;
    jump_reject_cnt_ = 0;
    last_track_radius_ = 0.0;
    last_corners_.clear();
}

// ====== 首帧初始化 ======
// YOLO推理 → kpt[5]→refineRCenter→R中心, kpt[4]→扇叶中心, |kpt[0]-kpt[2]|→边长
bool BuffRecognizer::init(const cv::Mat& frame,
                          cv::Point2f& out_R_center,
                          cv::Point2f& out_Fan_center,
                          double&      out_side) {
    if (frame.empty()) {
        ROS_WARN("[BuffRecognizer] init: 输入图像为空");
        return false;
    }

    cv::Mat frame_copy = frame.clone();
    auto results = detector_.getBestBox(frame_copy);
    int64 t_yolo_end = cv::getTickCount();
    if (results.empty()) {
        ROS_WARN("[BuffRecognizer] init: 首帧未检测到目标");
        return false;
    }

    auto& kpt = results[0].kpt;

    // ★ kpt[5] → R中心精化
    const cv::Point2f raw_r_center = refineRCenter(frame, kpt);
    int64 t_refine_end = cv::getTickCount();
    // ★ kpt[4] → 扇叶中心
    const cv::Point2f raw_fan_center = kpt[4];
    // ★ 上下边中点距离 → 扇叶边长
    out_side = cv::norm(kpt[0] - kpt[2]);
    stabilizer_.reset();
    const BuffStabilizedGeometry stabilized = stabilizeDetection(
        raw_r_center, raw_fan_center, {kpt[0], kpt[1], kpt[2], kpt[3]});
    out_R_center = {stabilized.r_center.x, stabilized.r_center.y};
    out_Fan_center = {stabilized.fan_center.x, stabilized.fan_center.y};

    // 保存状态
    last_R_center_   = out_R_center;
    last_Fan_center_ = out_Fan_center;
    last_side_       = out_side;
    last_track_radius_ = cv::norm(raw_fan_center - raw_r_center);
    last_corners_    = {
        {stabilized.corners[0].x, stabilized.corners[0].y},
        {stabilized.corners[1].x, stabilized.corners[1].y},
        {stabilized.corners[2].x, stabilized.corners[2].y},
        {stabilized.corners[3].x, stabilized.corners[3].y}};
    last_R_box_      = results[0].rect;
    has_prev_        = true;

    ROS_INFO("[BuffRecognizer] 首帧初始化成功: R(%d,%d) Fan(%d,%d) side=%.1f",
             (int)out_R_center.x, (int)out_R_center.y,
             (int)out_Fan_center.x, (int)out_Fan_center.y, out_side);
    return true;
}

// ====== ★ 逐帧识别 (核心!) ======
// 每帧调用, 完整7步:
//   ① YOLO推理 ② remap 9→6 ③ R中心精化 ④ 扇叶中心 ⑤ 边长 ⑥ PnP角点 ⑦ 时序校验
bool BuffRecognizer::identify(const cv::Mat& frame,
                              cv::Point2f& out_R_center,
                              cv::Point2f& out_Fan_center,
                              double&      out_side,
                              std::vector<cv::Point2f>& out_corners) {
    if (frame.empty()) {
        ROS_WARN("[BuffRecognizer] identify: 输入图像为空");
        return false;
    }

    // ═══ Step 1-2: YOLO 推理 → 9原始关键点 → remap → 6标准关键点 ═══
    int64 t_yolo_start = cv::getTickCount();
    cv::Mat frame_copy = frame.clone();
    auto results = detector_.getMultiBoxes(frame_copy);
    int64 t_yolo_end = cv::getTickCount();
    if (results.empty()) {
        consecutive_fail_++;
        resetTemporalState();
        ROS_DEBUG("[BuffRecognizer] 未检测到目标 (连续失败:%d)", consecutive_fail_);
        return false;
    }
    consecutive_fail_ = 0;

    const int selected_idx = selectTemporalDetection(results);
    if (selected_idx < 0) {
        // Association failed: no candidate within the spatial gate.
        // Enter reacquisition instead of silently picking a distant
        // high-confidence candidate (see Section 4.3 of the bug-fix document).
        consecutive_fail_++;
        resetTemporalState();
        ROS_WARN_THROTTLE(0.5,
            "[BuffRecognizer] 时序关联失败: 无候选在关联门限内 (candidates=%zu)", results.size());
        return false;
    }
    const BuffDetection& selected = results[selected_idx];
    const auto& kpt = selected.kpt;
    // kpt[0]=顶边中点  kpt[1]=左边中点  kpt[2]=底边中点  kpt[3]=右边中点
    // kpt[4]=靶心(扇叶中心)  kpt[5]=R中心(模型直出)

    // ═══ Step 3: kpt[5] → refineRCenter() 精化 → out_R_center ═══
    int64 t_refine_start = cv::getTickCount();
    const cv::Point2f raw_r_center = refineRCenter(frame, kpt);
    int64 t_refine_end = cv::getTickCount();

    // ═══ Step 4: kpt[4] → out_Fan_center (扇叶中心) ═══
    const cv::Point2f raw_fan_center = kpt[4];

    // ═══ Step 5: |kpt[0]-kpt[2]| → out_side (上下边距离) ═══
    out_side = cv::norm(kpt[0] - kpt[2]);

    // ═══ Step 6: kpt[0~3] → out_corners (4个PnP角点) ═══
    const std::vector<cv::Point2f> raw_corners{kpt[0], kpt[1], kpt[2], kpt[3]};

    // ═══ Step 7: 时序一致性校验 ═══
    // 相机移动时模型可能误检 → 扇叶位移>半径×0.55 或 R位移>半径×0.35 → 跳变
    const double radius = cv::norm(raw_fan_center - raw_r_center);
    const BuffTemporalCandidate measurement{
        {raw_r_center.x, raw_r_center.y}, {raw_fan_center.x, raw_fan_center.y}, selected.prob};

    if (!stabilizer_.confirmReacquisition(
            measurement, BuffDetectionStabilizer::associationGateForRadius(
                             static_cast<float>(radius)))) {
        ROS_DEBUG("[BuffRecognizer] 重捕获待确认");
        return false;
    }

    // Only accept a jump after the following frame confirms the same candidate.
    if (has_prev_) {
        float dist_fan = cv::norm(raw_fan_center - last_Fan_center_);
        float dist_r   = cv::norm(raw_r_center   - last_R_center_);
        if (dist_fan > radius * 0.55f || dist_r > radius * 0.35f) {
            const float association_gate_px =
                BuffDetectionStabilizer::associationGateForRadius(
                    static_cast<float>(std::max(radius, last_track_radius_)));
            const bool confirmed_jump = stabilizer_.confirmPersistentJump(
                measurement, association_gate_px);
            if (!confirmed_jump) {
                jump_reject_cnt_ = 1;
                ROS_DEBUG("[BuffRecognizer] 跳变待确认: fan=%.1f r=%.1f", dist_fan, dist_r);
                out_R_center = last_R_center_;
                out_Fan_center = last_Fan_center_;
                out_side = last_side_;
                out_corners = last_corners_;
                return true;
            }
            jump_reject_cnt_ = 0;
            // Geometry validation before accepting the jump (Section 4.4):
            // even if three consecutive frames agree spatially, reject the
            // jump if the new target's radius or side differs dramatically
            // from the previously tracked target.
            const double new_radius = cv::norm(raw_fan_center - raw_r_center);
            const double radius_change_ratio =
                last_track_radius_ > 1.0
                    ? std::abs(new_radius - last_track_radius_) / last_track_radius_
                    : 0.0;
            const double side_change_ratio =
                last_side_ > 1.0
                    ? std::abs(out_side - last_side_) / last_side_
                    : 0.0;
            if (radius_change_ratio > 0.50 || side_change_ratio > 0.50) {
                // The geometrically-confirmed jump candidate is too different
                // from the previous target — more likely a different blade or a
                // false positive than the same target after ego-motion.
                ROS_WARN_THROTTLE(
                    0.5,
                    "[BuffRecognizer] 跳变几何拒绝: radius Δ=%.0f%% side Δ=%.0f%% "
                    "(old_R=%.1f new_R=%.1f old_side=%.1f new_side=%.1f)",
                    radius_change_ratio * 100.0, side_change_ratio * 100.0,
                    last_track_radius_, new_radius, last_side_, out_side);
                stabilizer_.clearPendingJump();
                out_R_center = last_R_center_;
                out_Fan_center = last_Fan_center_;
                out_side = last_side_;
                out_corners = last_corners_;
                return true;
            }
            ROS_INFO("[BuffRecognizer] 跳变确认, 接受新位置: fan=%.1f r=%.1f", dist_fan, dist_r);
        } else {
            jump_reject_cnt_ = 0;
            stabilizer_.clearPendingJump();
        }
    }

    const BuffStabilizedGeometry stabilized = stabilizeDetection(
        raw_r_center, raw_fan_center, raw_corners);
    out_R_center = {stabilized.r_center.x, stabilized.r_center.y};
    out_Fan_center = {stabilized.fan_center.x, stabilized.fan_center.y};
    out_corners.clear();
    for (const BuffPoint& corner : stabilized.corners) {
        out_corners.emplace_back(corner.x, corner.y);
    }

    // 保存状态
    last_R_center_   = out_R_center;
    last_Fan_center_ = out_Fan_center;
    last_side_       = out_side;
    last_track_radius_ = radius;
    last_corners_    = out_corners;
    // ★ 检测框 EMA 平滑, 抑制 YOLO 模型帧间抖动
    if (has_prev_) {
        last_R_box_.x      += box_alpha_ * (selected.rect.x      - last_R_box_.x);
        last_R_box_.y      += box_alpha_ * (selected.rect.y      - last_R_box_.y);
        last_R_box_.width  += box_alpha_ * (selected.rect.width  - last_R_box_.width);
        last_R_box_.height += box_alpha_ * (selected.rect.height - last_R_box_.height);
    } else {
        last_R_box_ = selected.rect;
    }
    has_prev_        = true;

    // 延迟统计 (1秒打印一次, 避免刷屏)
    double t_freq = cv::getTickFrequency();
    double et_yolo   = (t_yolo_end - t_yolo_start) * 1000.0 / t_freq;
    double et_refine = (t_refine_end - t_refine_start) * 1000.0 / t_freq;
    double et_total  = et_yolo + et_refine;
    ROS_INFO_THROTTLE(1.0, "[BuffRecognizer] identify耗时: total=%.1fms (yolo=%.1fms  refine=%.1fms)", et_total, et_yolo, et_refine);
    return true;
}

// ====== 可视化 ======
// R中心(红圆) 扇叶中心(绿圆) PnP角点(黄圆) 检测框(白框) side值(文字)
void BuffRecognizer::visual(cv::Mat& frame) {
    if (!has_prev_) return;

    // R中心: 红色实心圆 + 坐标
    cv::circle(frame, last_R_center_, 4, cv::Scalar(0, 0, 255), -1);
    cv::putText(frame,
        cv::format("R:%d,%d", (int)last_R_center_.x, (int)last_R_center_.y),
        cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

    // 扇叶中心: 绿色实心圆 + 坐标
    cv::circle(frame, last_Fan_center_, 4, cv::Scalar(0, 255, 0), -1);
    cv::putText(frame,
        cv::format("Fan:%d,%d", (int)last_Fan_center_.x, (int)last_Fan_center_.y),
        cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

    // 4个PnP角点: 黄色小圆
    for (size_t i = 0; i < last_corners_.size(); ++i) {
        cv::circle(frame, last_corners_[i], 3, cv::Scalar(0, 255, 255), -1);
    }

    // 检测框: 白色边框
    cv::rectangle(frame, last_R_box_, cv::Scalar(255, 255, 255), 1);

    // side 值
    cv::putText(frame,
        cv::format("side:%.1f", last_side_),
        cv::Point(15, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
}
