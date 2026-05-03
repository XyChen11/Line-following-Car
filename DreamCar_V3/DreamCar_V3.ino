#include <PID_v1.h>
#include "linefollower.h"

// 四个电机最终输出的 PWM 值，范围为 -255~255。
// 正数表示正转，负数表示反转，0 表示停止。
int pwm_out[4] = {0, 0, 0, 0};

// 小车运动指令。
// vx    : 前后方向速度，正数前进，负数后退。
// vy    : 左右平移速度，当前代码固定为 0，保留是为了兼容全向轮/麦克纳姆轮混控。
// omega : 旋转速度，正负号决定左右转向。
struct MotionCommand {
  float vx;
  float vy;
  float omega;
};

// 记录上一帧识别到的道路类型，用来判断“刚刚进入某种路口”。
// 例如 road_type 连续多帧都是左转，只在第一次从非左转变成左转时触发转向状态。
static int last_road_type = ROAD_UNKNOWN;

// =========================
// 路口计数与防重复触发变量
// =========================
// 十字路口计数，ROAD_CROSS = 105
int cross_count = 0;
// 左直角路口计数，ROAD_LEFT_TURN = 101
int left_turn_count = 0;
// 右直角路口计数，ROAD_RIGHT_TURN = 102
int right_turn_count = 0;
// 统一路口锁。
// false：允许识别并触发一个新路口；
// true ：当前路口已经触发过，暂时不能再次触发。
static bool junction_locked = false;
// 连续识别为普通道路的帧数。
// 用于判断小车是否已经离开当前路口。
static int normal_road_count = 0;
// 连续多少帧识别为普通道路后，才解锁下一个路口。
#define JUNCTION_RELEASE_FRAMES 15

int junction_delete = 0;

// 主流程拆分出的函数声明。
// Arduino IDE 会自动生成部分函数原型，但这里显式声明能让结构更清楚。
void UpdateVision();
void UpdateTrackContext();
void UpdateJunctionCounter();
double ComputeLineFollowOmega();
void BeginJunction(int next_state);
void HandleLineFollowState(MotionCommand &cmd);
void RunTimedManeuver(MotionCommand &cmd, float omega);
MotionCommand ComputeMotionCommand();
void ApplyMotorOutput(const MotionCommand &cmd);
void PrintDebugPeriodically();

void setup() {
  // 初始化线阵 CCD 传感器引脚。
  pinMode(TSL_CLK, OUTPUT);
  pinMode(TSL_SI, OUTPUT);
  pinMode(TSL_AO, INPUT);
  pinMode(TSL_EN, OUTPUT);
  digitalWrite(TSL_EN, HIGH);

  // 初始化 PID 控制器。
  // 输出限制为 -1~1，方便直接作为归一化角速度 omega 使用。
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1, 1);
  myPID.SetSampleTime(LOOP_DELAY_MS);

  // 初始化四路电机驱动引脚。
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);

  // 串口用于调试输出。
  Serial.begin(115200);
}

void loop() {
  // 1. 采集并处理 CCD 图像，得到黑线、偏差、道路类型。
  UpdateVision();

  // 2. 更新历史上下文，供下一帧识别跳变、丢线等情况。
  UpdateTrackContext();

  //3. 计数经过路口
  UpdateJunctionCounter();

  // 3. 根据道路类型和路口状态机生成运动指令。
  MotionCommand cmd = ComputeMotionCommand();

  // 4. 将运动指令换算为四个电机的 PWM 并输出。
  ApplyMotorOutput(cmd);

  // 5. 周期性打印调试信息，避免每一圈都打印导致主循环过慢。
  PrintDebug();
}

void UpdateVision() {
  // 读取一帧 CCD 灰度数据。
  GetFrame();

  // 根据当前帧的最大/最小亮度计算二值化阈值。
  ComputeThreshold();

  // 将灰度图转成黑/白图。
  Binarize();

  // 在二值图中寻找连续黑色区域。
  FindBlackLines();

  // 根据黑线数量、宽度、位置变化识别道路类型。
  RecognizeRoad();

  // 在多个黑线段中选择最接近车体中心的主线中心。
  main_center = SelectMainCenter();

  // 保存主线中心历史，用于后续帧判断中心跳变和丢线。
  //UpdateHistory();
}

void UpdateTrackContext() {
  // 只有找到有效主线时才更新 last_center。
  // 这样丢线时仍然可以保留最后一次看到线的位置。
  if (main_center >= 0) {
    ctx.last_center = main_center;
  }

  // 记录上一帧道路类型，供识别逻辑和状态机参考。
  ctx.last_type = road_type;
}

void UpdateJunctionCounter() {
  // 只有在正常巡线状态下，才允许判断是否离开路口。
  // 如果还在执行左转/右转/直行动作，就不解锁。
  if (junction_state != STATE_LINE_FOLLOW) {
    return;
  }
  // 如果已经触发过某个路口，需要等待小车重新进入普通道路。
  if (junction_locked) {
    if (road_type == ROAD_STRAIGHT_CURVE) {
      normal_road_count++;

      if (normal_road_count >= JUNCTION_RELEASE_FRAMES) {
        junction_locked = false;
        normal_road_count = 0;
      }
    } else {
      // 只要还不是普通道路，就认为还没有完全离开路口。
      normal_road_count = 0;
    }
  }
}

double ComputeLineFollowOmega() {
  static double filtered_dev = 0;
  static double last_omega = 0;

  // 丢线或没有主中心时，不做 PID 转向。
  if (road_type == ROAD_LOST || main_center < 0) {
    return 0;
  }

  // 对偏差设置死区：很小的偏差直接认为是 0，减少车身左右抖动。
  if (abs(dev) <= DEV_DEADBAND) {
    dev = 0;
  }

  // 对偏差做一阶滤波和变化率限制，避免传感器突变导致转向突变。
  // 注意：当前 PID 实际仍使用 raw dev，这是保留原行为；
  // filtered_dev 目前主要作为后续切换到滤波输入时的准备。
  // double alpha = (abs(raw_dev) > 18) ? 0.20 : 0.40;
  // double target_dev = alpha * filtered_dev + (1.0 - alpha) * raw_dev;
  // double dev_delta = target_dev - filtered_dev;
  // if (dev_delta > DEV_SLEW_LIMIT) target_dev = filtered_dev + DEV_SLEW_LIMIT;
  // if (dev_delta < -DEV_SLEW_LIMIT) target_dev = filtered_dev - DEV_SLEW_LIMIT;
  // filtered_dev = target_dev;

  // PID 输入为当前横向偏差，输出为归一化角速度 omega。
  input = dev;
  myPID.Compute();

  // 对 omega 也做变化率限制，减少转向电机输出突跳。
  double omega = output;
  double omega_delta = omega - last_omega;
  if (omega_delta > OMEGA_RATE_LIMIT) omega = last_omega + OMEGA_RATE_LIMIT;
  if (omega_delta < -OMEGA_RATE_LIMIT) omega = last_omega - OMEGA_RATE_LIMIT;
  last_omega = omega;

  return constrain(omega, -1.0, 1.0);
}

void BeginJunction(int next_state) {
  // 进入路口动作状态，并记录进入时间。
  // 后续 RunTimedManeuver() 会用这个时间判断动作是否结束。
  junction_state = next_state;
  junction_start_time = millis();
}

void HandleLineFollowState(MotionCommand &cmd) {
  // 正常巡线时，转向量由 PID 根据黑线偏差计算。
  cmd.omega = ComputeLineFollowOmega();

  // 如果路口锁已经打开，说明当前路口已经触发过。
  // 此时即使 CCD 连续多帧识别到同一个路口，也不再重复计数或重复动作。
  if (junction_locked) {
    return;
  }

  // 1. 十字路口处理
  if (road_type == ROAD_CROSS) {
    cross_count++;
    junction_locked = true;
    normal_road_count = 0;

    if (cross_count == 2) {
      second_cross_start_time = millis();
    }

    // 第 3 个十字路口左转
    if (cross_count == 3) {
      BeginJunction(STATE_LEFT_TURN);
    }
    // 第 6 个十字路口右转
    else if (cross_count == 6) {
      BeginJunction(STATE_RIGHT_TURN);
    }
    // 其他十字路口直行
    else {
      BeginJunction(STATE_CROSS_STRAIGHT);
    }
    return;
  }

  // 2. 左直角路口处理
  if (road_type == ROAD_LEFT_TURN) {
    left_turn_count++;
    junction_locked = true;
    normal_road_count = 0;

    // 第二个左转路口直行，不左转
    if (left_turn_count == 2) {
      BeginJunction(STATE_CROSS_STRAIGHT);
    }
    // 其他左转路口正常左转
    else {
      BeginJunction(STATE_LEFT_TURN);
    }
    return;
  }

  // 3. 右直角路口处理
  if (road_type == ROAD_RIGHT_TURN) {
    right_turn_count++;
    junction_locked = true;
    normal_road_count = 0;

    if (millis() - second_cross_start_time < FORBIDDEN_DURATION && right_turn_count == 1) {
      BeginJunction(STATE_LINE_FOLLOW);
      junction_delete = 1;
      right_turn_count = 0;
    }
    // // 第一个右转路口直行，不右转
    // if (right_turn_count == 1) {
    //   BeginJunction(STATE_CROSS_STRAIGHT);
    // }
    // 其他左转路口正常右转
    else {
      BeginJunction(STATE_RIGHT_TURN);
    }
    //BeginJunction(STATE_RIGHT_TURN);
    return;
  }
}

void RunTimedManeuver(MotionCommand &cmd, float omega) {
  // 路口动作采用固定时间、固定角速度策略。
  // 优点是实现简单、动作稳定；缺点是需要根据实际速度和赛道反复调时间。
  cmd.vx = BASE_VX;
  cmd.omega = omega;
  //cmd.vy = VY_GAIN * omega;

  // 动作持续超过设定时间后，回到正常巡线。
  if (millis() - junction_start_time > JUNCTION_DURATION) {
    junction_state = STATE_LINE_FOLLOW;
  }
}

MotionCommand ComputeMotionCommand() {
  // 默认指令：以基础速度前进，不平移、不旋转。
  MotionCommand cmd = {BASE_VX, 0, 0};

  // 根据当前状态机决定使用 PID 巡线，还是执行固定路口动作。
  switch (junction_state) {
    case STATE_LINE_FOLLOW:
      HandleLineFollowState(cmd);
      break;

    case STATE_LEFT_TURN:
      RunTimedManeuver(cmd, -0.9);
      break;

    case STATE_RIGHT_TURN:
      RunTimedManeuver(cmd, 0.9);
      break;

    case STATE_CROSS_STRAIGHT:
      RunTimedManeuver(cmd, 0);
      break;

    case STATE_T_LEFT:
      RunTimedManeuver(cmd, -0.9);
      break;

    case STATE_T_RIGHT:
      RunTimedManeuver(cmd, 0.9);
      break;
  }

  // 更新上一帧道路类型，用于下一次 loop 判断类型是否刚刚变化。
  last_road_type = road_type;

  // 防止任何速度分量超过归一化范围。
  cmd.vx = constrain(cmd.vx, -1.0, 1.0);
  cmd.vy = constrain(cmd.vy, -1.0, 1.0);
  cmd.omega = constrain(cmd.omega, -1.0, 1.0);

  return cmd;
}

void ApplyMotorOutput(const MotionCommand &cmd) {
  // 四轮运动学混控。
  // 每个 target_norm 的范围理想情况下为 -1~1，之后会统一缩放到 PWM。
  float target_norm[4];
  target_norm[0] = cmd.vx + VY_GAIN * cmd.omega + cmd.omega;
  target_norm[1] = -(cmd.vx - VY_GAIN * cmd.omega + cmd.omega);
  target_norm[2] = -(cmd.vx + VY_GAIN * cmd.omega - cmd.omega);
  target_norm[3] = cmd.vx - VY_GAIN * cmd.omega - cmd.omega;

  // 如果某个轮子的绝对值超过 1，则整体等比例缩小。
  // 这样可以保持四个轮子之间的速度比例，同时避免 PWM 溢出。
  float maxV = 0;
  for (int i = 0; i < 4; i++) {
    if (abs(target_norm[i]) > maxV) {
      maxV = abs(target_norm[i]);
    }
  }

  if (maxV > 1.0) {
    for (int i = 0; i < 4; i++) {
      target_norm[i] /= maxV;
    }
  }

  // 将归一化速度转换成 PWM 输出。
  for (int i = 0; i < 4; i++) {
    pwm_out[i] = (int)(target_norm[i] * 255);
  }

  // 输出到四个电机。
  setMotor(pwm_out[0], AIN1, AIN2, PWMA);
  setMotor(pwm_out[1], BIN1, BIN2, PWMB);
  setMotor(pwm_out[2], CIN1, CIN2, PWMC);
  setMotor(pwm_out[3], DIN1, DIN2, PWMD);
}

void PrintDebugPeriodically() {
  // 串口打印很耗时，所以限制为每 50ms 打印一次。
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time >= 50) {
    PrintDebug();
    last_print_time = millis();
  }
}

void setMotor(float pwm, int IN1, int IN2, int PWMpin) {
  // 限制 PWM 范围，防止异常输入。
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    // 正转：IN1 高、IN2 低。
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWMpin, pwm);
  } else if (pwm < 0) {
    // 反转：IN1 低、IN2 高，PWM 使用绝对值。
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWMpin, -pwm);
  } else {
    // 停止：两个方向脚都拉低，PWM 为 0。
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(PWMpin, 0);
  }
}
