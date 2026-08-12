# Buff 识别器代码详解 — YOLO11 关键点模型

## 一、模型规格

### 模型文件

`Model/fu_moxi/buff_repvgg.xml` + `.bin`，OpenVINO IR 格式，CPU 推理。

### 输入

| 属性 | 值 |
|------|-----|
| Shape | `[1, 3, 640, 640]` (NCHW) |
| 数据类型 | float32, 值域 [0, 1] |
| 颜色空间 | RGB (需从 BGR 转换) |

### 输出 — Tensor 行布局

输出 shape `[1, 33, 8400]`，8400 列是不同尺度/位置的候选检测框，每列 33 行：

```
行 0:  bbox cx     (框中心 x, 归一化到 [0, 640])
行 1:  bbox cy     (框中心 y, 归一化到 [0, 640])
行 2:  bbox w      (框宽度)
行 3:  bbox h      (框高度)
行 4:  class 0     (red  置信度)
行 5:  class 1     (blue 置信度)
行 6-8:   kpt[0]   (x, y, visibility)
行 9-11:  kpt[1]
...
行 30-32: kpt[8]
```

对应常量（`buff_recognizer.hpp`）:

```cpp
static const int NUM_RAW_KPTS  = 9;   // 模型原始关键点数
static const int NUM_REMAP_KPTS = 6;  // remap后关键点数
static const int NUM_CLASSES   = 2;   // red / blue
static const int KPT_DIM       = 3;   // 每个关键点: x, y, visibility
static const int KPT_START     = 6;   // 关键点起始行 (0-3:bbox, 4-5:class)
```

---

## 二、标签/颜色映射

能量机关规则：你要打的是 **你方颜色** 的装甲板。串口下发的 `Camp` 是**对方**的颜色。

### 映射链路

```
ROS param enemy_color
  "blue" → enemy_class_idx_ = 0 → class_row = 4 → 读 red  置信度
  "red"  → enemy_class_idx_ = 1 → class_row = 5 → 读 blue 置信度
```

`BuffRecognizer` 构造函数中完成转换：

```cpp
detector_(model_path, 0.2f, 0.2f, (enemy_color == "blue" ? 0 : 1))
//                                              ↑ enemy_class_idx_
```

`setEnemyColor()` 可动态切换：

```cpp
enemy_class_idx_ = (color == "blue") ? 0 : 1;
```

推理时 `getBestBox()` 在 `class_row = 4 + enemy_class_idx_` 行取置信度。

---

## 三、关键点映射 — 9→6

### 模型输出的 9 个原始关键点

```
        raw[4] ●──────────● raw[5]          ← 顶边两点
               │          │
        raw[6] │  raw[0]  │ raw[2]           raw[0] = R中心 (旋转圆心)
        raw[7] │ (R中心)  │ raw[3]
               │          │
        raw[1] ●──────────● raw[8]          ← 底边两点
```

### remapKeypoints() — 对边取中点

对边取中点可以平均掉单点噪声，提升 PnP 稳定性：

```cpp
kpt[0] = (raw[4] + raw[5]) * 0.5   // 顶边中点
kpt[1] = (raw[6] + raw[7]) * 0.5   // 左边中点
kpt[2] = (raw[1] + raw[8]) * 0.5   // 底边中点
kpt[3] = (raw[2] + raw[3]) * 0.5   // 右边中点
kpt[4] = 四边均值                   // 靶心 = 扇叶中心
kpt[5] = raw[0]                     // R中心 (直接透传, 后续精化)
```

### side 计算

```cpp
g_side = |kpt[0] - kpt[2]|   // 上下边中点的像素距离
```

---

## 四、图像预处理 — fillTensor()

三步完成 BGR 图像 → OpenVINO NCHW tensor：

1. **等比缩放**: `scale = min(640/H, 640/W)`，保持宽高比，不足边自动补 0 (letterbox)
2. **归一化+颜色转换**: uint8→float32, ÷255, BGR→RGB。优化：scale<1 时先缩小再归一化（数据量小），scale≥1 时先归一化再放大
3. **填充 NCHW**: `data[c*W*H + h*W + w] = blob[h][w][c]`

返回 `factor = 1/scale`，用于将模型输出的归一化坐标还原到原图像素坐标：

```cpp
det.rect.x  = (cx - 0.5*w) * factor;
raw_kpt.x   = kpts_mat.at<float>(...) * factor;
```

---

## 五、推理后处理

### getBestBox() — 单框检测

遍历 8400 列，在 `class_row` 行找置信度最大的候选，若 > 0.2 则提取 bbox + 9 关键点，调用 `remapKeypoints()` 9→6。

步骤：`fillTensor` → `infer` → 解析 tensor → 找 `class_row` 行最大值 → 阈值过滤 → 提取 bbox×factor → 提取关键点×factor → remap → 画面标注白框+标签。

### getMultiBoxes() — 多框检测

与 `getBestBox` 的区别：收集**所有**置信度 > 阈值的候选，然后用 `cv::dnn::NMSBoxes()` 做 NMS 去重（按置信度降序排列，贪心去除 IoU > `nms_threshold_` 的重复框）。

---

## 六、R中心精化 — refineRCenter()

### 为什么要精化

YOLO 回归的 R中心有 2~5px 随机抖动。R中心差 1px → 极坐标角度差约 0.5° → 5m 距离瞄准点偏移约 4cm，可能脱靶。

### 算法 (5 步)

**Step 1 — 取模型初值**: `r_center_model = kpt[5] = raw[0]`

**Step 2 — CV 预处理** (`handleImage`): BGR → 灰度 → 二值化(阈值120) → 膨胀(5×5核)，提取装甲板白色区域。

**Step 3 — 圆形 mask 约束**: `radius = |底边中点-靶心| × 0.8`，以 `r_center_model` 为圆心画圆，AND 操作只保留圆内白色，滤除扇叶远端的干扰。

**Step 4 — 找轮廓 + 评分筛选**:
```
findContours(RETR_EXTERNAL) → 遍历轮廓 → minAreaRect
评分 = 长宽比(max/min) + |中心-初值| / (radius / 3.0)
选评分最小的轮廓中心
```
长宽比越接近 1（正方形）越好（R中心字母的外接矩形近似正方形），距离初值越近越好。

**Step 5 — 限幅保护**: 精化位移 > 5px 则截断，防止 CV 误检（如扇叶边缘被当成 R中心）导致大幅跳变。

### 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `binary_threshold_` | 120 | 二值化阈值 |
| `dilate_kernel_size_` | 5 | 膨胀核，连接"R"字母断裂 |
| `r_center_mask_ratio_` | 0.8 | mask半径比例，太小漏R中心，太大引入干扰 |
| `r_center_contour_div_` | 3.0 | 距离惩罚因子，越小越信任模型初值 |
| `r_center_refine_max_shift_` | 5.0 | 精化最大位移(px) |

---

## 七、时序一致性校验 — identify() Step 7

相机快速移动时模型可能误检（运动模糊、背景干扰），导致瞄准点跳变。

**方案**: 比较当前帧与上一帧的位移：
- `dist_fan > radius × 0.55` 或 `dist_r > radius × 0.35` → 判定为跳变
- 跳变时用上一帧结果替代（当前设为立即拒绝，可改为连续 3 帧确认）

正常帧间位移约 `radius × 0.1`（旋转），相机移动时约 `radius × 0.5`。阈值设在这两者之间。

---

## 八、参数总表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `conf_threshold` | 0.2 | YOLO 置信度阈值 |
| `nms_threshold` | 0.2 | NMS IoU 阈值 |
| `binary_threshold_` | 120 | R中心精化二值化阈值 |
| `dilate_kernel_size_` | 5 | 膨胀核大小 |
| `r_center_mask_ratio_` | 0.8 | mask半径 / 扇叶半径 |
| `r_center_contour_div_` | 3.0 | 轮廓评分距离惩罚因子 |
| `r_center_refine_max_shift_` | 5.0 | 精化最大位移(px) |

调优：
- 误检多 → 提高 `conf_threshold` (0.2→0.4)
- 漏检多 → 降低 `conf_threshold` (0.2→0.1)
- R中心抖动大 → 降低 `r_center_refine_max_shift_`，提高 `r_center_mask_ratio_`
- R中心精化偏移 → 降低 `r_center_contour_div_` 让算法更信任模型初值
