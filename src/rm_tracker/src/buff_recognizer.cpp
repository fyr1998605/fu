#include "buff_recognizer.hpp"
#include <ros/ros.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <utility>

// ============================================================================
//  YoloBuffDetector 实现 — OpenVINO YOLO11 推理
// ============================================================================

YoloBuffDetector::YoloBuffDetector(const std::string& model_path,
                                   float conf_threshold,
                                   float nms_threshold,
                                   int   enemy_class,
                                   const std::string& device)
    : conf_threshold_(conf_threshold)
    , nms_threshold_(nms_threshold)
    , enemy_class_idx_(enemy_class)
{
    initModel(model_path, device);
}

// 加载 OpenVINO 模型: read_model → compile_model(device) → create_infer_request
void YoloBuffDetector::initModel(const std::string& model_path,
                                 const std::string& device) {
    ROS_INFO("[BuffRecognizer] 加载模型: %s", model_path.c_str());
    model_ = core_.read_model(model_path);
    // 单请求同步推理: 明确要求低延迟单流, 避免运行时按吞吐模式分配多流,
    // 让延迟尖峰更小、更可预测。
    const ov::AnyMap properties = {
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
        ov::num_streams(1)};
    const std::string requested = device.empty() ? "CPU" : device;
    try {
        compiled_model_ = core_.compile_model(model_, requested, properties);
    } catch (const std::exception& error) {
        if (requested == "CPU") throw;
        ROS_WARN("[BuffRecognizer] 推理设备 %s 不可用(%s), 回退 CPU",
                 requested.c_str(), error.what());
        compiled_model_ = core_.compile_model(model_, "CPU", properties);
    }
    infer_request_ = compiled_model_.create_infer_request();
    std::string actual_device;
    try {
        for (const auto& name : compiled_model_.get_property(ov::execution_devices)) {
            if (!actual_device.empty()) actual_device += ",";
            actual_device += name;
        }
    } catch (const std::exception&) {
        actual_device = "unknown";
    }
    ROS_INFO("[BuffRecognizer] 模型加载成功, 请求设备=%s, 实际设备=%s, "
             "输入shape: [1,3,640,640]",
             requested.c_str(), actual_device.c_str());
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
float YoloBuffDetector::fillTensor(const cv::Mat& input_image) {
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
            const cv::Vec3f* row = blob.ptr<cv::Vec3f>(static_cast<int>(h));
            for (size_t w = 0; w < width; w++) {
                data[c * width * height + h * width + w] =
                    row[w][c];
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
// 收集置信度>阈值的候选 → 只保留 top-K → 提取关键点 → OpenCV NMSBoxes 去重
//
// 正常帧通常只有 1~5 个候选; 但过曝、运动模糊或模型异常时可能有几千个
// 候选同时过 0.2 阈值。原实现会对每个候选提取关键点, 再对全部候选做
// O(n²) NMS, 单帧可以卡到几百毫秒。这里先按置信度截断, 把最坏耗时钳住。
std::vector<BuffDetection> YoloBuffDetector::getMultiBoxes(const cv::Mat& image) {
    if (image.empty()) {
        ROS_WARN("[BuffRecognizer] 输入图像为空!");
        return {};
    }

    int64 start = cv::getTickCount();
    const auto preprocess_start = std::chrono::steady_clock::now();
    float factor = fillTensor(image);
    const auto infer_start = std::chrono::steady_clock::now();
    infer_request_.infer();
    const auto parse_start = std::chrono::steady_clock::now();

    const ov::Tensor output     = infer_request_.get_output_tensor();
    const ov::Shape  out_shape  = output.get_shape();
    const float*     out_buffer = output.data<const float>();
    const int        out_rows   = out_shape[1];
    const int        out_cols   = out_shape[2];
    const cv::Mat    det_output(out_rows, out_cols, CV_32F, (float*)out_buffer);

    const int class_row = 4 + enemy_class_idx_;
    const int kpt_rows  = NUM_RAW_KPTS * KPT_DIM;

    // Step 1: 先只收集过阈值的列号, 按置信度取 top-K。
    std::vector<int> candidates;
    candidates.reserve(16);
    for (int i = 0; i < det_output.cols; ++i) {
        if (det_output.at<float>(class_row, i) > conf_threshold_) {
            candidates.push_back(i);
        }
    }
    const std::size_t raw_candidates = candidates.size();
    if (raw_candidates > kMaxCandidates) {
        std::partial_sort(
            candidates.begin(), candidates.begin() + kMaxCandidates,
            candidates.end(),
            [&](int a, int b) {
                return det_output.at<float>(class_row, a) >
                       det_output.at<float>(class_row, b);
            });
        candidates.resize(kMaxCandidates);
        ROS_WARN_THROTTLE(
            2.0,
            "[BuffRecognizer] 异常帧候选过多: %zu → 仅处理置信度最高的 %zu 个",
            raw_candidates, kMaxCandidates);
    }

    // Step 2: 只对保留的候选提取 bbox 和关键点。
    std::vector<cv::Rect>    boxes;
    std::vector<float>       confidences;
    std::vector<std::vector<cv::Point2f>> all_kpts;
    boxes.reserve(candidates.size());
    confidences.reserve(candidates.size());
    all_kpts.reserve(candidates.size());

    for (int i : candidates) {
        float score = det_output.at<float>(class_row, i);
        float cx = det_output.at<float>(0, i);
        float cy = det_output.at<float>(1, i);
        float ow = det_output.at<float>(2, i);
        float oh = det_output.at<float>(3, i);
        boxes.emplace_back(cv::Rect(
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

    // Step 3: NMS 去重: 按置信度排序, IoU>nms_threshold_的视为重复
    std::vector<int> nms_idx;
    if (!boxes.empty()) {
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, nms_idx);
    }

    std::vector<BuffDetection> results;
    results.reserve(nms_idx.size());
    for (int idx : nms_idx) {
        BuffDetection det;
        det.rect  = boxes[idx];
        det.prob  = confidences[idx];
        det.label = enemy_class_idx_;
        det.kpt   = all_kpts[idx];
        results.push_back(det);
    }

    const auto parse_end = std::chrono::steady_clock::now();
    const auto ms_since = [](std::chrono::steady_clock::time_point a,
                             std::chrono::steady_clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    float t = (cv::getTickCount() - start) / cv::getTickFrequency();
    ROS_DEBUG("[BuffRecognizer] 多框推理: raw=%zu kept=%zu results=%zu, 耗时%.1fms",
              raw_candidates, candidates.size(), results.size(), t * 1000);
    // 把 detector 拆成 预处理/推理/解析 三段, 卡顿时能直接看出是 OpenVINO
    // 推理本身变慢, 还是图像预处理/后处理在抖。
    ROS_INFO_THROTTLE(
        2.0,
        "[BUFF_PROFILE] pre=%.1fms infer=%.1fms parse=%.1fms raw=%zu kept=%zu",
        ms_since(preprocess_start, infer_start),
        ms_since(infer_start, parse_start),
        ms_since(parse_start, parse_end),
        raw_candidates, candidates.size());
    return results;
}

// ============================================================================
//  BuffRecognizer 实现
// ============================================================================

BuffRecognizer::BuffRecognizer(const std::string& model_path,
                               const std::string& enemy_color,
                               BuffRecognizerConfig config)
    : detector_(model_path, config.detector_conf_threshold,
                config.detector_nms_threshold,
                (enemy_color == "blue" ? 0 : 1),
                config.inference_device),
      stabilizer_(config.stabilizer_alpha),
      association_(BuffBladeAssociation::Config{
          config.association_gate_rad,
          config.association_min_radius_ratio,
          config.association_max_radius_ratio,
          config.association_speed_smoothing,
          config.association_speed_limit_radps,
          config.association_switch_confirm_frames}),
      config_(config)
{
    dilate_kernel_ = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(config_.dilate_kernel_size,
                                 config_.dilate_kernel_size));
    ROS_INFO("[BuffRecognizer] 初始化完成, model=%s, enemy=%s",
             model_path.c_str(), enemy_color.c_str());
}

void BuffRecognizer::setTargetPriority(
    const std::string& priority, const double lock_timeout_seconds) {
    target_priority_locked_ = priority != "confidence";
    association_.setTimeoutSeconds(lock_timeout_seconds);
    ROS_INFO("[BuffRecognizer] 目标优先级: %s, 锁定超时: %.3fs",
             target_priority_locked_ ? "locked" : "confidence",
             std::max(0.01, lock_timeout_seconds));
}

// ====== 图像预处理: 灰度→二值化→膨胀 ======
// 用于 refineRCenter 中提取装甲板白色区域
void BuffRecognizer::handleImage(const cv::Mat& bgr_img, cv::Mat& dilated_img) {
    cv::Mat gray_img;
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);              // ① 灰度化
    cv::threshold(gray_img, dilated_img, config_.binary_threshold,
                  255, cv::THRESH_BINARY);
    cv::dilate(dilated_img, dilated_img, dilate_kernel_);
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
    const cv::Point2f r_center_model = kpts[5];

    // Step 2: 圆形 mask 半径 (半径=|底边中点-靶心|×0.8)
    const double radius = cv::norm(kpts[2] - kpts[4]) * config_.r_center_mask_ratio;
    if (!std::isfinite(radius) || radius < 4.0) return r_center_model;

    // Step 3: 只处理 R 中心附近 ROI。mask 本来就会丢弃半径外的所有像素,
    // 对整张 1280x1024 做阈值/膨胀/找轮廓没有额外收益, 却会在高亮或噪声
    // 画面下产生大量轮廓, 把 identify 拖到几十甚至几百毫秒。
    const int half = static_cast<int>(std::ceil(radius)) +
        std::max(8, static_cast<int>(std::lround(radius * 0.1)));
    const int x0 = std::max(0, static_cast<int>(std::floor(r_center_model.x)) - half);
    const int y0 = std::max(0, static_cast<int>(std::floor(r_center_model.y)) - half);
    const int x1 = std::min(bgr_img.cols,
                            static_cast<int>(std::ceil(r_center_model.x)) + half + 1);
    const int y1 = std::min(bgr_img.rows,
                            static_cast<int>(std::ceil(r_center_model.y)) + half + 1);
    if (x1 - x0 < 8 || y1 - y0 < 8) return r_center_model;
    const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
    const cv::Mat bgr_roi = bgr_img(roi);

    // Step 4: 传统CV预处理 → 二值图
    cv::Mat dilated_img;
    handleImage(bgr_roi, dilated_img);

    // Step 5: 圆形 mask 约束 (ROI 内坐标)
    const cv::Point2f r_center_roi(r_center_model.x - roi.x,
                                   r_center_model.y - roi.y);
    cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);
    circle(mask, r_center_roi, std::max(1, cvRound(radius)), cv::Scalar(255), -1);
    cv::bitwise_and(dilated_img, mask, dilated_img);

    // Step 6: 找轮廓 + 评分筛选
    // 评分 = 长宽比(越接近正方形越好) + 距模型初值距离/惩罚因子(越近越好)
    // CHAIN_APPROX_SIMPLE 只保留线段端点, 比 NONE 省内存和遍历时间。
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilated_img, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    constexpr std::size_t kMaxRefineContours = 256;
    if (contours.size() > kMaxRefineContours) {
        // 面积只算一次, 避免排序比较器里反复遍历轮廓边界。
        std::vector<std::pair<double, std::size_t>> by_area;
        by_area.reserve(contours.size());
        for (std::size_t i = 0; i < contours.size(); ++i) {
            by_area.emplace_back(cv::contourArea(contours[i]), i);
        }
        std::nth_element(
            by_area.begin(), by_area.begin() + kMaxRefineContours, by_area.end(),
            [](const std::pair<double, std::size_t>& a,
               const std::pair<double, std::size_t>& b) {
                return a.first > b.first;
            });
        std::vector<std::vector<cv::Point>> kept;
        kept.reserve(kMaxRefineContours);
        for (std::size_t i = 0; i < kMaxRefineContours; ++i) {
            kept.push_back(std::move(contours[by_area[i].second]));
        }
        contours.swap(kept);
    }

    cv::Point2f r_center_refined = r_center_model;
    double best_cost = INFINITY;
    for (const auto& cnt : contours) {
        if (cv::contourArea(cnt) < 4.0) continue;
        auto rr = cv::minAreaRect(cnt);
        double ratio = rr.size.height > rr.size.width
                     ? rr.size.height / rr.size.width
                     : rr.size.width / rr.size.height;
        ratio += cv::norm(rr.center - r_center_roi) /
            (radius / config_.r_center_contour_distance_divisor);
        if (ratio < best_cost) {
            best_cost = ratio;
            r_center_refined = rr.center + cv::Point2f(roi.x, roi.y);
        }
    }

    // Step 7: 限幅保护 (≤5px)
    cv::Point2f correction = r_center_refined - r_center_model;
    float corr_norm = cv::norm(correction);
    if (corr_norm > config_.r_center_refine_max_shift_px && corr_norm > 1e-6f) {
        r_center_refined = r_center_model
            + correction * (config_.r_center_refine_max_shift_px / corr_norm);
    }

    ROS_DEBUG("[BuffRecognizer] R中心精化: model(%d,%d) → refined(%d,%d), "
              "roi=%dx%d contours=%zu shift=%.1fpx",
              (int)r_center_model.x, (int)r_center_model.y,
              (int)r_center_refined.x, (int)r_center_refined.y,
              roi.width, roi.height, contours.size(), corr_norm);
    return r_center_refined;
}

int BuffRecognizer::selectTemporalDetection(
    const std::vector<BuffDetection>& detections,
    const double timestamp_seconds) {
    std::vector<BuffTemporalCandidate> candidates;
    for (const auto& detection : detections) {
        candidates.push_back({{detection.kpt[5].x, detection.kpt[5].y},
                              {detection.kpt[4].x, detection.kpt[4].y}, detection.prob});
    }
    association_result_ = association_.update(
        candidates, timestamp_seconds, expected_speed_, target_priority_locked_);
    return association_result_.index;
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
        corners, association_result_.changed || !has_prev_);
}

// Initialization uses the same observation/association path as later frames.
bool BuffRecognizer::init(const cv::Mat& frame, cv::Point2f& out_R_center,
                          cv::Point2f& out_Fan_center, double& out_side,
                          double timestamp_seconds) {
    std::vector<cv::Point2f> corners;
    return identify(frame, out_R_center, out_Fan_center, out_side, corners,
                    timestamp_seconds);
}

bool BuffRecognizer::identify(const cv::Mat& frame, cv::Point2f& out_R_center,
                              cv::Point2f& out_Fan_center, double& out_side,
                              std::vector<cv::Point2f>& out_corners,
                              double timestamp_seconds) {
    beginFrame();
    if (frame.empty()) return false;
    if (timestamp_seconds < 0.0) {
        // Only standalone detector users lack the shared image-arrival clock.
        timestamp_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    // The detector no longer annotates its input, so the source frame can be
    // consumed directly. Cloning 1280x1024 here cost ~1-2ms per frame for an
    // image that was immediately discarded.
    const auto detector_start = std::chrono::steady_clock::now();
    detections_ = detector_.getMultiBoxes(frame);
    const auto detector_end = std::chrono::steady_clock::now();
    detections_.erase(std::remove_if(detections_.begin(), detections_.end(),
        [](const BuffDetection& d) {
            return d.kpt.size() != 6 || !std::isfinite(d.prob) ||
                !std::all_of(d.kpt.begin(), d.kpt.end(), [](const cv::Point2f& p) {
                    return std::isfinite(p.x) && std::isfinite(p.y);
                }) || cv::norm(d.kpt[0] - d.kpt[2]) <= 1.0;
        }), detections_.end());
    const double detector_ms = std::chrono::duration<double, std::milli>(
        detector_end - detector_start).count();
    const int index = selectTemporalDetection(detections_, timestamp_seconds);
    if (index < 0) {
        ROS_INFO_THROTTLE(2.0,
            "[BUFF_PROFILE] detector=%.1fms refine=0.0ms candidates=%zu (no association)",
            detector_ms, detections_.size());
        return false;  // No cached geometry masquerades as a new measurement.
    }
    const auto& selected = detections_[index];
    const auto& kpt = selected.kpt;
    const auto refine_start = std::chrono::steady_clock::now();
    const cv::Point2f raw_r = refineRCenter(frame, kpt);
    const auto refine_end = std::chrono::steady_clock::now();
    const double refine_ms = std::chrono::duration<double, std::milli>(
        refine_end - refine_start).count();
    const cv::Point2f raw_fan = kpt[4];
    const auto geometry = stabilizeDetection(raw_r, raw_fan,
                                             {kpt[0], kpt[1], kpt[2], kpt[3]});
    out_R_center = {geometry.r_center.x, geometry.r_center.y};
    out_Fan_center = {geometry.fan_center.x, geometry.fan_center.y};
    out_side = cv::norm(kpt[0] - kpt[2]);
    out_corners.clear();
    for (const auto& p : geometry.corners) out_corners.emplace_back(p.x, p.y);
    last_R_center_ = out_R_center;
    last_Fan_center_ = out_Fan_center;
    last_side_ = out_side;
    last_corners_ = out_corners;
    last_R_box_ = selected.rect;  // Draw the current observation without box EMA lag.
    has_prev_ = true;
    fresh_observation_ = true;
    ROS_INFO_THROTTLE(1.0, "[BUFF_TARGET] blade=%d candidates=%zu switched=%s",
                     association_.lockedId(), detections_.size(),
                     association_result_.changed ? "true" : "false");
    ROS_INFO_THROTTLE(
        2.0,
        "[BUFF_PROFILE] detector=%.1fms refine=%.1fms candidates=%zu",
        detector_ms, refine_ms, detections_.size());
    return true;
}

// ====== 可视化 ======
// R中心(红圆) 扇叶中心(绿圆) PnP角点(黄圆) 检测框(白框) side值(文字)
void BuffRecognizer::visual(cv::Mat& frame) {
    for (std::size_t i = 0; i < detections_.size(); ++i) {
        const auto& point = detections_[i].kpt[4];
        const int id = i < association_result_.candidate_ids.size()
            ? association_result_.candidate_ids[i] : -1;
        cv::circle(frame, point, 5, cv::Scalar(180, 180, 180), 1);
        cv::putText(frame, cv::format("ID:%d", id), point + cv::Point2f(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 180, 180), 1);
    }
    if (!fresh_observation_) return;

    // R中心: 红色实心圆 + 坐标
    cv::circle(frame, last_R_center_, 4, cv::Scalar(0, 0, 255), -1);
    cv::putText(frame,
        cv::format("R:%d,%d", (int)last_R_center_.x, (int)last_R_center_.y),
        cv::Point(15, 220), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

    // 扇叶中心: 绿色实心圆 + 坐标
    cv::circle(frame, last_Fan_center_, 4, cv::Scalar(0, 255, 0), -1);
    cv::putText(frame,
        cv::format("Fan:%d,%d", (int)last_Fan_center_.x, (int)last_Fan_center_.y),
        cv::Point(15, 250), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

    // 4个PnP角点: 黄色小圆
    for (size_t i = 0; i < last_corners_.size(); ++i) {
        cv::circle(frame, last_corners_[i], 3, cv::Scalar(0, 255, 255), -1);
    }

    // 检测框: 白色边框
    cv::rectangle(frame, last_R_box_, cv::Scalar(255, 255, 255), 1);

    // side 值
    cv::putText(frame,
        cv::format("side:%.1f", last_side_),
        cv::Point(15, 280), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
}
