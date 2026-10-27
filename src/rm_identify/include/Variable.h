#include <limits>
#ifndef VARIABLE
#define VARIABLE

#define PI 3.141592653
#define D2R PI/180
#define R2D 180/PI
#define DOUBLE_MAX std::numeric_limits<double>::max()
//预测时长
#define PREDICT 0.5
//击打范围
#define ATTACK_ANGLE 10.0*D2R
//imshow 后等待时间
#define DELAY 1 //ms
#define DOUBLE_SIZE sizeof(double)
//转速阈值 大于则认为在小陀螺状态
#define RAV_LIMIT 1.0 // rad/s
//yaw角度增益系数
#define YAW_GAIN_RATE 1.0
//距离和与中心距离对优先系数的影响
#define R_P_LOCATION 5
#define R_P_FROM_CENTER 1.2
//x、p轴限制
#define X_LIMIT 720*D2R
#define P_LIMIT 720*D2R
//自动射击范围系数
#define SHOOT_RANGE_RATE 1.05
#define SHOOT_RANGE_P 5*D2R
#define SHOOT_YAW_OFFSET 0 //px
#define OVER_SHOOT_RATE 1.00
//丢失目标预测帧数
#define TRACE_FRAME 30
//预测开关标志
#define TRACE_FLAG true
//预测点和识别点最大容差 单位:m
#define TRACE_DIS_MAX_ERROR 3.000
//绘制光心和补偿光心
#define DRAW_CAMERA_CENTER true
#define DRAW_CAMERA_CENTER_COMPENSATED true
//旋转距离阈值 单位:mm/s
#define ROTATION_THRESH 0.150
#define MAX_MOVE_DIFF 1.8
#define MAX_YAW_DIFF 720*D2R
#define MAX_R 0.2250
#define BEGIN_R 0.150
#define MIN_R 0.2250
//时效数据保留时长(不包括卡尔曼数据) 单位:ms
#define DATA_SAVE_TIME 500
//调试模式标志
#define DEBUG_FLAG true
bool Debug = DEBUG_FLAG;
#define SHOW_IMG true
#define RECORD_SYNC_FT true
#define NO_DEBUG_PRINT_TS 5000 //ms
#define NO_DEBUG_SYNC_TS 5000 //ms
//长短装甲板长宽比界限
#define LONG_ARMOR_LIMIT 3.0
//暴力求解参数设置
#define ENUM_ERROR 0.1
#define ENUM_STEP 0.01
//补偿参数设置
#define LOCAL_G 9.80665 //mm/s^2
double FA = 4.9;
// #define FA 0.95
#define COMPENSATE_BEGIN_ANGLE -45*D2R
#define COMPENSATE_END_ANGLE 45*D2R
#define BULLET_VEC 29.5 //mm/s
#define BULLET_MASS 5 //kg

double PREDICT_RATE = 1.0;
#endif