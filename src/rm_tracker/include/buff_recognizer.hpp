#ifndef BUFF_RECOGNIZER_HPP
#define BUFF_RECOGNIZER_HPP

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <array>
#include <vector>
#include <string>

#include "buff_detection_stabilizer.hpp"

/**
 * @brief 能量机关(大/小符)识别器 — 单一 YOLO11 关键点模型, 替换旧的 YOLOV8_R + 传统CV
 *
 * 模型: YOLO11-pose, OpenVINO IR (.xml), CPU 推理
 * 输入: [1, 3, 640, 640] NCHW, BGR→RGB, /255 归一化
 * 输出: [1, 33, 8400] → 33行=bbox(4)+class(2)+kpt(9×3=27), 8400列=候选框
 * 标签: 0=red(红色方装甲), 1=blue(蓝色方装甲)
 *
 * 9个原始关键点 raw[0]~raw[8] → remapKeypoints() → 6个标准关键点 kpt[0]~kpt[5]:
 *   raw[0]=R中心, raw[4,5]=顶边, raw[6,7]=左边, raw[1,8]=底边, raw[2,3]=右边
 *   kpt[0]=(raw[4]+raw[5])/2=顶边中点 → corners[0]
 *   kpt[1]=(raw[6]+raw[7])/2=左边中点 → corners[1]
 *   kpt[2]=(raw[1]+raw[8])/2=底边中点 → corners[2]
 *   kpt[3]=(raw[2]+raw[3])/2=右边中点 → corners[3]
 *   kpt[4]=四边均值=靶心(扇叶中心)   → center_Fan_Box
 *   kpt[5]=raw[0]=R中心(模型直出)    → refineRCenter()精化 → center_R_Box
 * side = |kpt[0]-kpt[2]| (上下边中点距离, 用于PnP)
 *
 * 数据流: identify() → g_R_center/g_Fan_center/g_side/g_corners
 *         → Fu_angleprocess → buff_predicted → PnPSolver_gravity → /auto_angle
 *
 * 详细文档见 README_buff_upgrade.md
 */

// ============================================================
// 检测结果结构体
// ============================================================
struct BuffDetection {
    cv::Rect_<float> rect;               // 检测框 (像素坐标)
    int   label;                          // 类别: 0=red, 1=blue
    float prob;                           // 置信度 [0, 1]
    std::vector<cv::Point2f> kpt;        // 6个 remap 后的标准关键点
};

// ============================================================
// OpenVINO YOLO11 推理类
// ============================================================
class YoloBuffDetector {
public:
    /**
     * @param model_path    模型路径, 如 "Model/fu_moxi/buff_repvgg.xml"
     * @param conf_threshold 置信度阈值 (默认0.2)
     * @param nms_threshold  NMS IoU阈值 (默认0.2)
     * @param enemy_class    敌方类别: 0=打红方, 1=打蓝方
     */
    YoloBuffDetector(const std::string& model_path,
                     float conf_threshold = 0.2f,
                     float nms_threshold  = 0.2f,
                     int   enemy_class    = 0);

    /// 取置信度最高的单个检测框 (小符)
    std::vector<BuffDetection> getBestBox(cv::Mat& image);

    /// 取多个候选框 + NMS 去重 (大符)
    std::vector<BuffDetection> getMultiBoxes(cv::Mat& image);

    /// 设置敌方颜色: "blue"→识别class0(red), "red"→识别class1(blue)
    void setEnemyColor(const std::string& color);

private:
    // OpenVINO 推理三件套
    ov::Core             core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel    compiled_model_;
    ov::InferRequest     infer_request_;

    // 推理参数
    float conf_threshold_;
    float nms_threshold_;
    int   enemy_class_idx_;       // 0=red行(class_row=4), 1=blue行(class_row=5)

    // YOLO11 输出格式常量
    static const int NUM_RAW_KPTS  = 9;   // 模型原始关键点数
    static const int NUM_REMAP_KPTS = 6;  // remap后关键点数
    static const int NUM_CLASSES   = 2;   // 类别数
    static const int KPT_DIM       = 3;   // 每个关键点: x, y, visibility
    static const int KPT_START     = 6;   // 关键点在tensor起始行 (0-3:bbox, 4-5:class, 6+:kpts)

    /// 9原始关键点 → 6标准关键点 (对边取中点, 减少单点抖动)
    std::vector<cv::Point2f> remapKeypoints(const std::vector<cv::Point2f>& raw) const;

    /// 图像预处理: 等比缩放→640→BGR→RGB→/255→NCHW tensor, 返回坐标还原因子
    float fillTensor(cv::Mat& input_image);

    /// 加载 OpenVINO 模型: read_model → compile_model(CPU) → create_infer_request
    void initModel(const std::string& model_path);
};

// ============================================================
// Buff 识别器 — 对外接口 (替换旧 identifies 类)
// ============================================================
class BuffRecognizer {
public:
    /**
     * @param model_path  模型路径
     * @param enemy_color 敌方颜色 "red" 或 "blue" (默认"blue")
     */
    BuffRecognizer(const std::string& model_path,
                   const std::string& enemy_color = "blue");

    // "locked" keeps the first successfully associated blade while it is
    // temporarily missing; "confidence" retains the legacy re-acquisition.
    void setTargetPriority(const std::string& priority,
                           double lock_timeout_seconds = 0.25);

    // ========== 首帧初始化 ==========
    /// YOLO推理 → remap → R中心精化 → 输出 R中心/扇叶中心/边长, 保存状态
    bool init(const cv::Mat& frame,
              cv::Point2f& out_R_center,
              cv::Point2f& out_Fan_center,
              double&      out_side);

    // ========== 逐帧识别 ==========
    /**
     * @brief 核心识别函数, 每帧调用
     *
     * 流程:
     *   1) YOLO推理 → 9原始关键点
     *   2) remapKeypoints() → 6标准关键点 kpt[0~5]
     *   3) kpt[5] → refineRCenter() → out_R_center (R中心精化)
     *   4) kpt[4] → out_Fan_center (扇叶中心)
     *   5) |kpt[0]-kpt[2]| → out_side (扇叶边长)
     *   6) kpt[0~3] → out_corners (4个PnP角点)
     *   7) 时序校验: 跳变>阈值则连3帧确认, 防止相机移动误检
     */
    bool identify(const cv::Mat& frame,
                  cv::Point2f& out_R_center,
                  cv::Point2f& out_Fan_center,
                  double&      out_side,
                  std::vector<cv::Point2f>& out_corners);

    // ========== 可视化 ==========
    /// 绘制: R中心(红圆), 扇叶中心(绿圆), PnP角点(黄圆), 检测框(白框), side值
    void visual(cv::Mat& frame);

    // ========== 状态查询 ==========
    cv::Point2f lastRCenter()     const { return last_R_center_; }
    cv::Point2f lastFanCenter()   const { return last_Fan_center_; }
    double      lastSide()        const { return last_side_; }
    const std::vector<cv::Point2f>& lastCorners() const { return last_corners_; }

    // ========== 自运动补偿接口 ==========
    /// Set camera intrinsics for ego-motion compensation (called once during init).
    void setEgoMotionIntrinsics(float fx, float fy) { fx_ = fx; fy_ = fy; }
    /// Feed current gimbal feedback before each identify() call.
    /// The method stores the values and returns the frame-to-frame delta
    /// so the caller can use it for diagnostic logging if desired.
    void feedGimbalFeedback(float pitch_rad, float yaw_rad) {
        prev_gimbal_pitch_ = current_gimbal_pitch_;
        prev_gimbal_yaw_   = current_gimbal_yaw_;
        current_gimbal_pitch_ = pitch_rad;
        current_gimbal_yaw_   = yaw_rad;
    }

private:
    // ========== R中心精化 ==========
    /**
     * @brief 用传统CV精化模型直出的R中心 (模型有2~5px抖动, R中心差1px→角度差~0.5°)
     *
     * 流程: 灰度→二值化(120)→膨胀(5×5)
     *       → 以模型R中心为圆心做mask (半径=|靶心-R中心|×0.8)
     *       → findContours → 每轮廓取minAreaRect → 评分(长宽比+距离惩罚)
     *       → 选最优轮廓中心 → 限幅(≤5px)
     */
    cv::Point2f refineRCenter(const cv::Mat& bgr_img,
                               const std::vector<cv::Point2f>& kpts);

    int selectTemporalDetection(
        const std::vector<BuffDetection>& detections,
        double timestamp_seconds) const;

    BuffStabilizedGeometry stabilizeDetection(
        const cv::Point2f& raw_r_center,
        const cv::Point2f& raw_fan_center,
        const std::vector<cv::Point2f>& raw_corners);

    void resetTemporalState();
    void noteTrackingLoss(double timestamp_seconds);

    /// BGR→灰度→二值化→膨胀, 用于提取装甲板白色区域
    void handleImage(const cv::Mat& bgr_img, cv::Mat& dilated_img);

    // ========== 成员变量 ==========
    YoloBuffDetector detector_;
    BuffDetectionStabilizer stabilizer_;
    BuffTargetLock target_lock_;
    bool target_priority_locked_ = true;

    // 识别结果缓存 (最近一帧)
    cv::Point2f last_R_center_   = {0, 0};
    cv::Point2f last_Fan_center_ = {0, 0};
    double      last_side_       = 0;
    double      last_track_radius_ = 0;
    std::vector<cv::Point2f> last_corners_;
    cv::Rect_<float> last_R_box_;
    bool        has_prev_       = false;

    // R中心精化参数
    int    binary_threshold_         = 120;  // 二值化阈值
    int    dilate_kernel_size_       = 5;    // 膨胀核大小
    double r_center_mask_ratio_      = 0.8;  // mask半径比例
    double r_center_contour_div_     = 3.0;  // 距离惩罚因子
    double r_center_refine_max_shift_ = 5.0; // 精化最大位移(px)
    float  box_alpha_                = 0.3f; // 检测框EMA平滑系数

    // 时序状态
    int    last_quadrant_       = -1;
    int    quadrant_stable_cnt_ = 0;
    int    active_quadrant_     = -1;
    int    consecutive_fail_    = 0;    // 连续失败帧数
    int    jump_reject_cnt_     = 0;    // 连续跳变帧数(相机移动自适应)

    // 自运动补偿: 相机内参 + 云台反馈
    float  fx_ = 0.0f;
    float  fy_ = 0.0f;
    float  prev_gimbal_pitch_ = 0.0f;
    float  prev_gimbal_yaw_   = 0.0f;
    float  current_gimbal_pitch_ = 0.0f;
    float  current_gimbal_yaw_   = 0.0f;
};

#endif // BUFF_RECOGNIZER_HPP
