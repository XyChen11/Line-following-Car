#include "linefollower.h"
#include <PID_v1.h>

// =========================
// 全局变量定义
// =========================
// 这些变量在 linefollower.h 中以 extern 声明，在这里真正分配存储空间。
uint16_t pixel_buffer[PIXEL_COUNT];    // CCD 灰度图，范围约 0~255
uint8_t binary_buffer[PIXEL_COUNT];    // 二值化结果，BLACK 或 WHITE
int dev;                               // 当前主线中心相对 CAR_CENTER 的偏差
int last_dev;                          // 上一次计算出的偏差
uint16_t threshold;                    // 当前帧二值化阈值
LineSegment lines[MAX_LINE_COUNT];     // 当前帧检测到的黑线段
int line_count;                        // 当前帧有效黑线段数量
int main_center;                       // 当前帧选中的主黑线中心
int road_type;                         // 当前帧识别出的道路类型

// 初始化历史上下文。
// last_center 和历史中心都设为 -1，表示启动时还没有有效黑线。
TrackContext ctx = { -1, {-1, -1, -1, -1, -1}, 0, 0, ROAD_UNKNOWN };

// PID 参数。
// Kp 控制转向响应强度，Kd 抑制偏差变化带来的抖动，Ki 当前不用。
double Kp = 0.005, Ki = 0.00002, Kd = 0.000015;
double input, output, setpoint;

// PID 控制器对象。
// input  : 当前偏差 dev。
// output : PID 计算出的转向角速度 omega。
// setpoint 默认 0，表示目标偏差为 0，即黑线在车体中心。
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// 路口状态机变量。
int junction_state = STATE_LINE_FOLLOW;
unsigned long junction_start_time = 0;
unsigned long second_cross_start_time = 0;

// 路口固定动作持续时间，单位 ms。
// 例如进入左转状态后，会以固定角速度转 900ms，然后回到巡线。
const unsigned long JUNCTION_DURATION = 220;
const unsigned long FORBIDDEN_DURATION = 20;
void GetFrame() {
  // 线阵 CCD 的读法是：
  // 1. 用 SI 信号启动一帧采样；
  // 2. 每给一个 CLK 脉冲，模拟输出 AO 上会出现一个像素的亮度；
  // 3. 逐个 analogRead() 读出所有像素。

  // 启动一帧采样。SI 在 CLK 低电平期间拉高，再配合 CLK 上升沿触发。
  digitalWrite(TSL_CLK, LOW);
  digitalWrite(TSL_SI, LOW);
  delayMicroseconds(5);
  digitalWrite(TSL_SI, HIGH);
  delayMicroseconds(25);
  digitalWrite(TSL_CLK, HIGH);
  delayMicroseconds(5);
  digitalWrite(TSL_SI, LOW);
  delayMicroseconds(5);

  // 临时保存完整 128 像素。
  // 后面会丢弃前 15 个不稳定像素，只保留 PIXEL_COUNT 个有效像素。
  static uint16_t temp_buffer[ALL_PIXEL_COUNT];

  for (int i = 0; i < ALL_PIXEL_COUNT; i++) {
    digitalWrite(TSL_CLK, LOW);
    delayMicroseconds(10);

    // Arduino analogRead() 通常是 10 位 ADC，范围 0~1023。
    // 右移 2 位后压缩到 0~255，方便后续阈值处理。
    temp_buffer[i] = analogRead(TSL_AO) >> 2;

    digitalWrite(TSL_CLK, HIGH);
    delayMicroseconds(5);
  }

  // 抛弃前 15 个像素。
  for (int i = 0; i < PIXEL_COUNT; i++) {
    pixel_buffer[i] = temp_buffer[i + 15];
  }

  // 额外给一个时钟脉冲，使 CCD 输出进入下一轮准备状态。
  digitalWrite(TSL_CLK, LOW);
  delayMicroseconds(5);
  digitalWrite(TSL_CLK, HIGH);
  delayMicroseconds(5);
}


void ComputeThreshold() {
  // 动态阈值法：
  // 当前帧中最亮值和最暗值的平均值作为黑白分界。
  // 优点是能适应环境亮度变化；缺点是极端光照或噪声下需要限幅保护。
  uint16_t min_val = 255, max_val = 0;

  for (int i = 0; i < PIXEL_COUNT; i++) {
    if (pixel_buffer[i] < min_val) {
      min_val = pixel_buffer[i];
    }

    // min 和 max 必须独立判断，不能写成 else if。
    // 否则某些像素同时影响最小/最大更新时可能导致 max 更新不完整。
    if (pixel_buffer[i] > max_val) {
      max_val = pixel_buffer[i];
    }
  }

  threshold = (min_val + max_val) / 2;
}


void Binarize() {
  // 对阈值做限幅，避免画面过暗或过亮时阈值极端化。
  if (threshold < 30) threshold = 30;
  if (threshold > 130) threshold = 130;

  // 大于等于阈值认为是白底，小于阈值认为是黑线。
  for (int i = 0; i < PIXEL_COUNT; i++) {
    if (pixel_buffer[i] >= threshold) {
      binary_buffer[i] = WHITE;
    } else {
      binary_buffer[i] = BLACK;
    }
  }
}


void FindBlackLines() {
  // 在 binary_buffer 中寻找连续的 BLACK 区域。
  // 每个连续黑色区域会记录为一个 LineSegment。
  int count = 0;
  int i = 0;

  while (i < PIXEL_COUNT && count < MAX_LINE_COUNT) {
    // 先跳过白色背景。
    while (i < PIXEL_COUNT && binary_buffer[i] == WHITE) i++;
    if (i >= PIXEL_COUNT) break;

    // 当前位置是黑色，记录该黑色区域起点。
    int start = i;

    // 一直向右走到黑色区域结束。
    while (i < PIXEL_COUNT && binary_buffer[i] == BLACK) i++;
    int end = i - 1;
    int width = end - start + 1;

    // 宽度太小的黑色区域通常是噪声，直接忽略。
    if (width >= MIN_BLACK_WIDTH) {
      lines[count].start = start;
      lines[count].end = end;
      lines[count].center = (start + end) / 2;
      lines[count].width = width;
      count++;
    }
  }

  line_count = count;
}


int SelectMainCenter() {
  // 如果没有任何有效黑线，则返回 -1 表示丢线。
  if (line_count <= 0) {
    return -1;
  }

  // 多个黑线段同时出现时，优先选择最靠近车体中心的那一条。
  // 这样在路口、干扰线或分叉处能尽量保持主线稳定。
  int best_idx = 0;
  int best_dist = abs(lines[0].center - CAR_CENTER);

  for (int i = 1; i < line_count; i++) {
    int dist = abs(lines[i].center - CAR_CENTER);
    if (dist < best_dist) {
      best_dist = dist;
      best_idx = i;
    }
  }

  return lines[best_idx].center;
}


void UpdateHistory() {
  // 使用环形数组保存最近 HISTORY_DEPTH 帧主线中心。
  // main_center 为 -1 时也会写入，表示这一帧丢线。
  ctx.center_history[ctx.history_index] = main_center;
  ctx.history_index = (ctx.history_index + 1) % HISTORY_DEPTH;
}


void RecognizeRoad() {
  // 根据黑线段数量进入不同识别分支：
  // 0 条：丢线；
  // 1 条：普通直/弯道、直角弯、十字路口入口；
  // 2 条：Y 路口、分裂线或噪声；
  // 3 条及以上：通常认为是T型路口、十字路口或复杂交叉。

  if (line_count == 0) {
    // 当前帧完全没有黑线。
    // lost_counter 可用于后续扩展：连续丢线多帧后执行找线策略。
    ctx.lost_counter++;
    road_type = ROAD_LOST;
    dev = 0;
    return;
  }

  // 只要本帧重新看到了线，就清空连续丢线计数。
  ctx.lost_counter = 0;

  if (line_count == 1) {
    // 单条黑线是最常见情况，可能是直道、弯道，也可能是路口区域合并成一条宽线。
    int center = lines[0].center;
    int width = lines[0].width;
    int start = lines[0].start;
    int end = lines[0].end;

    // 偏差定义为 CAR_CENTER - center。
    // center 在右侧时 dev 为负，center 在左侧时 dev 为正。
    int current_dev = CAR_CENTER - center;

    // 当前中心相对上一帧中心的变化量。
    // 跳变较大且线段贴边，通常说明进入了直角弯入口。
    int current_delta = (ctx.last_center >= 0) ? (center - ctx.last_center) : 0;

    bool touch_left = (start <= EDGE_TOUCH_MARGIN);
    bool touch_right = (end >= (PIXEL_COUNT - 1 - EDGE_TOUCH_MARGIN));
    bool strong_jump_right = (current_delta >= 20);
    bool strong_jump_left = (current_delta <= -20);
    bool very_wide = (width >= SINGLE_CROSS_WIDTH_THRESHOLD);
    bool wide_turn = ((width >= SINGLE_TURN_WIDTH_THRESHOLD) && (width < SINGLE_CROSS_WIDTH_THRESHOLD));

    // 直角弯判断：
    // 1. 线中心相对上一帧明显跳变；
    // 2. 黑线区域贴到图像边缘；
    // 3. 线宽达到直角弯阈值，但没有宽到十字路口阈值。
    if (((strong_jump_right || strong_jump_left) && (touch_left || touch_right)) && wide_turn) {
      if (strong_jump_right) {
        road_type = ROAD_RIGHT_TURN;
        dev = current_dev;
      }
      if (strong_jump_left) {
        road_type = ROAD_LEFT_TURN;
        dev = current_dev;
      }
      return;
    }

    // 十字路口判断：
    // 单个黑色区域非常宽，并且中心变化不大，说明车可能正压在交叉区域上。
    if (very_wide && abs(current_delta) <= SINGLE_STABLE_DELTA_MAX) {
      road_type = ROAD_CROSS;
      dev = 0;
      return;
    }

    // 普通直道或弯道。
    // 直接使用黑线中心偏差交给 PID 做转向修正。
    dev = current_dev;
    last_dev = (ctx.last_center >= 0) ? -(ctx.last_center - CAR_CENTER) : dev;
    road_type = ROAD_STRAIGHT_CURVE;
    return;
  }

  if (line_count == 2) {
    // 两条黑线可能表示：
    // 1. Y 路口中主路和岔路同时出现；
    // 2. 一条线被反光切裂成两段；
    // 3. 复杂路口的一部分。

    // 先按中心位置区分左右线段。
    LineSegment *left = &lines[0];
    LineSegment *right = &lines[1];
    if (left->center > right->center) {
      left = &lines[1];
      right = &lines[0];
    }

    int dist = right->center - left->center;

    if (dist > 5) {
      // 两个黑线段分得比较开，认为更可能是路口结构。
      // 判断哪条线更接近图像中心，从而推断主路和岔路方向。
      bool left_near_center = (left->center >= 40 && left->center <= 88);
      bool right_near_center = (right->center >= 40 && right->center <= 88);

      if (left_near_center && !right_near_center) {
        // 左侧线更像主路，右侧额外出现岔路。
        road_type = ROAD_T_JUNCTION_RIGHT;
        dev = 0;
        return;
      } else if (!left_near_center && right_near_center) {
        // 右侧线更像主路，左侧额外出现岔路。
        road_type = ROAD_T_JUNCTION_LEFT;
        dev = 0;
        return;
      } else if (left_near_center && right_near_center) {
        // 两条线都靠近中心，通常是十字或复杂交叉。
        road_type = ROAD_CROSS;
        dev = 0;
        return;
      } else {
        // 两条线都离中心较远，无法可靠判断。
        road_type = ROAD_UNKNOWN;
        dev = 0;
        return;
      }
    } else {
      // 两条线距离很近，更像是一条黑线被噪声切裂。
      // 合并两个中心，当作普通巡线处理。
      int merged_center = (left->center + right->center) / 2;
      ctx.last_center = merged_center;
      dev = -(merged_center - CAR_CENTER);
      last_dev = dev;
      road_type = ROAD_STRAIGHT_CURVE;
      return;
    }
  }

  if (line_count >= 3) {
    // 三条及以上黑线段通常意味着进入十字路口或复杂区域。
    // 此时不再计算偏差，交给路口状态机执行固定动作。
    road_type = ROAD_CROSS;
    dev = 0;
    return;
  }

  // 理论上不走到这里，作为兜底处理。
  road_type = ROAD_UNKNOWN;
  dev = 0;
}

void PrintDebug() {
  // 基础调试信息：
  // dev       : 当前偏差；
  // input     : PID 输入；
  // output    : PID 输出；
  // road_type : 当前道路类型。
  Serial.print("dev:");
  Serial.print(dev);
  Serial.print(" ");
  Serial.print("input:");
  Serial.print(input);
  Serial.print(" ");
  Serial.print("output:");
  Serial.print(output);
  Serial.print(" ");
  Serial.print("road_type:");
  Serial.print(road_type);
  Serial.print(" ");
  Serial.print("cross_count:");
  Serial.print(cross_count);
  Serial.print(" ");
  Serial.print("left_turn_count:");
  Serial.print(left_turn_count);
  Serial.print(" ");
  Serial.print("right_turn_count:");
  Serial.print(right_turn_count);
  Serial.println(" ");

  // 完整二值化图像很长，打印会显著拖慢控制循环。
  // 只有需要观察 CCD 黑白分布时才在 linefollower.h 中打开 DEBUG_PRINT_BINARY。
  #if DEBUG_PRINT_BINARY
  Serial.print("\n");
  for (int i = 0; i < PIXEL_COUNT; i++) {
    Serial.print(binary_buffer[i]);
    Serial.print("-");
  }
  Serial.print("\n");
  #endif
}
