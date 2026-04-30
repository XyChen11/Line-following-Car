#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include <stdlib.h>
#include <PID_v1.h>

// =========================
// 线阵 CCD 传感器引脚定义
// =========================
// TSL_CLK: CCD 时钟信号，由程序手动翻转，逐个读出像素。
// TSL_SI : CCD 帧开始信号，用来触发一次新的采样序列。
// TSL_AO : CCD 模拟输出，连接到 Arduino 模拟输入口。
// TSL_EN : CCD 使能引脚，高电平启用。
#define TSL_CLK   30
#define TSL_SI    31
#define TSL_AO    A1
#define TSL_EN    13

// =========================
// 四路电机驱动引脚定义
// =========================
// 每个电机使用两个方向控制脚和一个 PWM 调速脚。
// 例如 A 电机：
// AIN1/ AIN2 控制方向，PWMA 控制速度。
#define PWMA 4
#define AIN1 22
#define AIN2 23

#define PWMB 5
#define BIN1 24
#define BIN2 25

#define PWMC 6
#define CIN1 26
#define CIN2 27

#define PWMD 7
#define DIN1 28
#define DIN2 29

// =========================
// CCD 图像处理参数
// =========================
// 传感器原始输出 128 个像素，但前 15 个像素通常不稳定或一直为 0，
// 所以实际算法只保留后面的 113 个像素。
#define ALL_PIXEL_COUNT 128
#define PIXEL_COUNT     113

// 连续黑色区域宽度小于该值时认为是噪声，不当作有效黑线。
#define MIN_BLACK_WIDTH 8

// 一帧中最多记录几个黑线段，防止数组越界。
#define MAX_LINE_COUNT  5

// 保存最近几帧主线中心，用于判断中心跳变和丢线趋势。
#define HISTORY_DEPTH   5

// 小车理想对准的 CCD 像素中心。
// 因为丢弃了前 15 个像素，所以这里不是 128/2，而是实测校准值。
#define CAR_CENTER      49

// =========================
// 运动控制参数
// =========================
// PID 采样时间，单位 ms。
#define LOOP_DELAY_MS     1

// 横向漂移增益
#define VY_GAIN           -0.56   //

// 基础前进速度，归一化范围通常为 0~1。
#define BASE_VX           0.12 //基础速度

// 中心附近偏差死区，偏差绝对值小于该值时不修正。
#define DEV_DEADBAND      2  //

// 偏差变化率限制，避免 CCD 识别跳变导致 PID 输入剧烈变化。
#define DEV_SLEW_LIMIT    24

// 角速度变化率限制，避免电机转向输出突变。
#define OMEGA_RATE_LIMIT  0.30

// 角速度绝对上限。
#define OMEGA_LIMIT       1

// 是否打印完整二值化图像。
// 0: 只打印核心调试量；1: 额外打印 113 个黑白像素值。
#define DEBUG_PRINT_BINARY 0

// =========================
// 二值化像素值
// =========================
// 黑线为 0，白底为 1。
#define BLACK  0
#define WHITE  1

// =========================
// 道路类型定义
// =========================
// RecognizeRoad() 会根据当前 CCD 画面设置 road_type。
#define ROAD_STRAIGHT_CURVE      100  // 普通直道或弯道
#define ROAD_LEFT_TURN           101  // 直角左转
#define ROAD_RIGHT_TURN          102  // 直角右转
#define ROAD_T_JUNCTION_LEFT     103  // Y 路口，岔路在左
#define ROAD_T_JUNCTION_RIGHT    104  // Y 路口，岔路在右
#define ROAD_CROSS               105  // 十字路口
#define ROAD_LOST                106  // 丢线
#define ROAD_UNKNOWN             107  // 未知或异常情况

// 宽线判断阈值，保留给路口识别调参使用。
#define WIDTH_THRESHOLD 35

// =========================
// 道路识别阈值
// =========================
// 单黑线宽度达到该值，但没有达到十字路口宽度时，可能是直角弯入口。
#define SINGLE_TURN_WIDTH_THRESHOLD   45

// 单黑线非常宽时，通常说明进入十字路口或大面积交叉区域。
#define SINGLE_CROSS_WIDTH_THRESHOLD  90

// 判断十字路口时，中心点变化不能太大，否则更可能是急转弯。
#define SINGLE_STABLE_DELTA_MAX       8

// 黑线段接近图像边缘的容差，用于判断线是否“贴边”。
#define EDGE_TOUCH_MARGIN             2

// =========================
// 路口处理状态机
// =========================
// 正常情况下处于 STATE_LINE_FOLLOW，用 PID 巡线。
// 识别到路口后进入固定动作状态，持续 JUNCTION_DURATION 后回到巡线。
#define STATE_LINE_FOLLOW     201
#define STATE_LEFT_TURN       202
#define STATE_RIGHT_TURN      203
#define STATE_CROSS_STRAIGHT  204
#define STATE_T_LEFT          205
#define STATE_T_RIGHT         206

// 单个黑线段的信息。
// start/end 是二值图中的起止像素下标；
// center 是线段中心；
// width 是线段宽度。
struct LineSegment {
  int start;
  int end;
  int center;
  int width;
};

// 巡线历史上下文。
// 这些数据用于跨帧判断，例如上一帧中心位置、连续丢线次数等。
struct TrackContext {
  int last_center;                     // 上一帧有效主线中心，-1 表示无效
  int center_history[HISTORY_DEPTH];   // 最近几帧主线中心
  int history_index;                   // 环形缓冲区当前写入位置
  int lost_counter;                    // 连续丢线计数
  int last_type;                       // 上一帧道路类型
};

// =========================
// 全局状态变量声明
// =========================
// 实际定义在 linefollower.cpp 中，这里用 extern 让其他文件可以访问。
extern uint16_t pixel_buffer[PIXEL_COUNT];    // 当前帧灰度图，范围约 0~255
extern uint8_t binary_buffer[PIXEL_COUNT];    // 当前帧二值图，BLACK/WHITE
extern int dev;                               // 黑线中心相对车体中心的偏差
extern int last_dev;                          // 上一帧偏差
extern uint16_t threshold;                    // 当前帧二值化阈值
extern LineSegment lines[MAX_LINE_COUNT];     // 当前帧检测到的黑线段
extern int line_count;                        // 当前帧有效黑线段数量
extern int main_center;                       // 当前帧主黑线中心
extern int road_type;                         // 当前帧道路类型
extern TrackContext ctx;                      // 跨帧历史上下文

extern int cross_count;
extern int left_turn_count;
extern int right_turn_count;

// PID 控制器及其输入输出变量。
extern PID myPID;
extern double Kp, Ki, Kd;
extern double setpoint;
extern double input;
extern double output;

// 路口状态机变量。
extern int junction_state;
extern unsigned long junction_start_time;
extern unsigned long second_cross_start_time;
extern const unsigned long JUNCTION_DURATION;
extern const unsigned long FORBIDDEN_DURATION;

// 四电机 PWM 输出，由 DreamCar.ino 计算并写入。
extern int pwm_out[4];

// =========================
// 图像处理和道路识别函数
// =========================
void GetFrame();
void ComputeThreshold();
void Binarize();
void FindBlackLines();
int SelectMainCenter();
void UpdateHistory();
void RecognizeRoad();

// =========================
// 调试和电机输出函数
// =========================
void PrintDebug();
void setMotor(float pwm, int IN1, int IN2, int PWMpin);

#endif
