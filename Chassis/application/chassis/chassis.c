/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "dji_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include "robot_def.h"
#include "super_cap.h"
#include "user_lib.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "general_def.h"
#include "referee_UI.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define SPEED_DEADBAND_THRESHOLD 120.0f         // 角度/秒，约80rpm
// 斜坡补偿相关
#define ROBOT_WEIGHT_FORCE (CHASSIS_MASS * GRAVITY_ACCEL) // 机器人重量
#define INV_TOTAL_WHEELBASE                                                    \
  (1.0f / WHEEL_BASE) // 预计算轴距的倒数，用于将除法变为乘法
#ifndef WHEEL_RADIUS
#define WHEEL_RADIUS RADIUS_WHEEL // 使用 robot_def.h 中的定义
#endif
#define HALF_WHEEL_RADIUS (WHEEL_RADIUS / 2.0f) // 预计算一半的轮胎半径

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体
 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t
    ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap; // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb,
    *motor_rb; // left right forward back

static PIDInstance chassis_follow_pid; // 底盘跟随云台PID控制器
static float last_follow_wz = 0.0f; // 记录上一次跟随模式的wz输出，用于一阶滤波

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
// 单独轮子的速度前馈（单位: angle per second，度/秒）
static float ff_spd_lf, ff_spd_rf, ff_spd_lb, ff_spd_rb;
// 阻力补偿数值
#define FRICTION_COMP_LF 0 // 左前轮的静摩擦补偿值 (示例值)
#define FRICTION_COMP_RF 0 // 右前轮的静摩擦补偿值 (示例值)
#define FRICTION_COMP_LB 0 // 左后轮的静摩擦补偿值 (示例值)
#define FRICTION_COMP_RB 0 // 右后轮的静摩擦补偿值 (示例值)
//  速度死区 (解决零点震荡问题) (单位: rpm)
//    当电机目标转速的绝对值低于此值时, 不施加任何阻力补偿
//    可以从一个较小的值开始尝试, 如 10-30 RPM
#define FRICTION_DEADBAND_RPM 20.0f

//  最大补偿限制 (解决目标值太大发散问题) (单位: CAN command value)
//    阻力补偿的总输出不会超过此值。通常远小于电机的最大电流(16384)
//    建议从一个保守的值开始, 如 500
#define MAX_FRICTION_COMP_CMD 500

// 阻力补偿一阶连续化过渡窗口 (rpm)
#define FRICTION_LINEAR_WINDOW_RPM 30.0f

#define MAX_SLOPE_COMP_CMD 3000

// 电机功率模型参数（需按电机/减速比/摩擦标定）：
// Pin = TORQUE_COEFF * I * rpm + POWER_MODEL_K2 * rpm^2 + POWER_MODEL_K1 * I^2
// + POWER_MODEL_CONST
#define TORQUE_COEFF                                                           \
  1.99688994e-6f // 转矩系数（将电流指令×轮侧转速换算为机械功率）
#define POWER_MODEL_K1 1.23e-07f  // 电阻损耗系数
#define POWER_MODEL_K2 1.453e-07f // 阻尼/风阻损耗系数
#define POWER_MODEL_CONST 4.081f  // 静态损耗

void ChassisInit() {
  // 四个轮子的参数一样,改tx_id和反转标志位即可
  Motor_Init_Config_s chassis_motor_config = {
      .can_init_config.can_handle = &hcan1,
      .controller_param_init_config =
          {
              .speed_PID =
                  {
                      .Kp = 1.38f,         // 纯P控制，适当增大Kp补偿稳态误差
                      .Ki = 0.0f,         // 关闭积分，避免功耗问题
                      .Kd = 0.06f,         // 暂不使用D项
                      .Derivative_LPF_RC = 0.06f,
                      .IntegralLimit = 0.0f, // 无积分项，设为0
                      .Improve = PID_DerivativeFilter, // 关闭所有改进特性，纯P控制
                      .MaxOut = 20000,             // 保持较大的输出限制
                  },
              .current_PID =
                  {
                      .Kp = 0.8f,                  // 纯P控制，降低Kp避免振荡
                      .Ki = 0.0f,                  // 关闭积分，彻底避免功耗
                      .Kd = 0.0f,                  // 暂不使用D项
                      .IntegralLimit = 0,          // 无积分项，设为0
                      .Improve = PID_IMPROVE_NONE, // 关闭所有改进特性，纯P控制
                      .MaxOut = 20000,             // 保持较大的输出限制
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = SPEED_LOOP,
              .close_loop_type = SPEED_LOOP,
          },
      .motor_type = M3508,
  };
  // 设置速度前馈
  chassis_motor_config.controller_param_init_config.speed_feedforward_ptr = &ff_spd_lf;
  chassis_motor_config.controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD;
  
  chassis_motor_config.can_init_config.tx_id = 2;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE; // 修改：左前电机反转
  motor_lf = DJIMotorInit(&chassis_motor_config);

  chassis_motor_config.controller_param_init_config.speed_feedforward_ptr = &ff_spd_rf;
  chassis_motor_config.can_init_config.tx_id = 3;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL; // 修改：右前电机反转
  motor_rf = DJIMotorInit(&chassis_motor_config);

  chassis_motor_config.controller_param_init_config.speed_feedforward_ptr = &ff_spd_lb;
  chassis_motor_config.can_init_config.tx_id = 1;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL; // 保持：左后电机正常
  motor_lb = DJIMotorInit(&chassis_motor_config);

  chassis_motor_config.controller_param_init_config.speed_feedforward_ptr = &ff_spd_rb;
  chassis_motor_config.can_init_config.tx_id = 4;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE; // 保持：右后电机正常
  motor_rb = DJIMotorInit(&chassis_motor_config);

  referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

  SuperCap_Init_Config_s cap_conf = {
      .can_config = {
          .can_handle = &hcan2,
          .tx_id = 0x302, // 超级电容默认接收id
          .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
      }};
  cap = SuperCapInit(&cap_conf); // 超级电容初始化

  // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
  Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

  CANComm_Init_Config_s comm_conf = {
      .can_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x311,
              .rx_id = 0x312,
          },
      .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
      .send_data_len = sizeof(Chassis_Upload_Data_s),
  };
  chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                        // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
  chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD

  // 底盘跟随云台PID控制器初始化
  PID_Init_Config_s follow_pid_config = {
      .Kp = 42.0f,             // 保持不变，效果好
      .Ki = 4.0f,              // 从4.0降到2.5，减慢积分
      .Kd = 0.0f,              // 保持不变
      .IntegralLimit = 150.0f, // 从200增到350，增大空间
      .MaxOut = 8000.0f,       // 保持不变
      .DeadBand = 0.15f,       // 从0.32缩小到0.15
      .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, // ⬅️ 关键！
      // 变速积分参数
      .CoefA = 3.0f, // 积分削弱区间：3度
      .CoefB = 0.8f, // 完全积分阈值：0.8度内完全积分
  };
  PIDInit(&chassis_follow_pid, &follow_pid_config);
}

#define LF_CENTER                                                              \
  ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE -              \
    CENTER_GIMBAL_OFFSET_Y) *                                                  \
   DEGREE_2_RAD)
#define RF_CENTER                                                              \
  ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE -              \
    CENTER_GIMBAL_OFFSET_Y) *                                                  \
   DEGREE_2_RAD)
#define LB_CENTER                                                              \
  ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE +              \
    CENTER_GIMBAL_OFFSET_Y) *                                                  \
   DEGREE_2_RAD)
#define RB_CENTER                                                              \
  ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE +              \
    CENTER_GIMBAL_OFFSET_Y) *                                                  \
   DEGREE_2_RAD)
// 坡度补偿相关定义
#define BACK_COG_LENGTH                                                        \
  DIST_CG_REAR_AXLE // 暂未整定前后重心,所以直接使用前后轮距的一半
#define FRONT_COG_LENGTH DIST_CG_FRONT_AXLE // 可以直接在robot_def.h中修改定义
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate() {
  vt_lf = (-chassis_vx - chassis_vy) + chassis_cmd_recv.wz * LF_CENTER;
  vt_rf = (-chassis_vx + chassis_vy) - chassis_cmd_recv.wz * RF_CENTER;
  vt_lb = (chassis_vx - chassis_vy) - chassis_cmd_recv.wz * LB_CENTER;
  vt_rb = (chassis_vx + chassis_vy) + chassis_cmd_recv.wz * RB_CENTER;
}

static void SlopeCompensation() {
#ifdef CHASSIS_BOARD
  // 1. 获取sin/cos值
  float cos_pitch = arm_cos_f32(Chassis_IMU_data->Pitch);
  float sin_pitch = arm_sin_f32(Chassis_IMU_data->Pitch);

  // 2. 计算力
  float total_compensation_force = ROBOT_WEIGHT_FORCE * sin_pitch;
  float total_normal_force = ROBOT_WEIGHT_FORCE * cos_pitch;

  // 3. 计算法向力
  float front_axle_normal_force = (DIST_CG_REAR_AXLE * total_normal_force -
                                   CG_HEIGHT * total_compensation_force) *
                                  INV_TOTAL_WHEELBASE;
  float rear_axle_normal_force = (DIST_CG_FRONT_AXLE * total_normal_force +
                                  CG_HEIGHT * total_compensation_force) *
                                 INV_TOTAL_WHEELBASE;

  // 4. 安全校验
  if (front_axle_normal_force < 0)
    front_axle_normal_force = 0;
  if (rear_axle_normal_force < 0)
    rear_axle_normal_force = 0;

  float actual_total_normal_force =
      front_axle_normal_force + rear_axle_normal_force;

  // 5. 避免除零错误和增加坡度死区
  if (actual_total_normal_force < 1e-3f ||
      fabsf(Chassis_IMU_data->Pitch) < 5.0f) {
    return;
  }

  // 6. 计算最终补偿扭矩并转换为CAN指令值
  float inv_actual_total_normal_force = 1.0f / actual_total_normal_force;
  float common_factor = total_compensation_force *
                        inv_actual_total_normal_force * HALF_WHEEL_RADIUS *
                        M3508_TORQUE_TO_CURRENT_CMD_COEFF;

  float front_wheel_comp_cmd = front_axle_normal_force * common_factor;
  float rear_wheel_comp_cmd = rear_axle_normal_force * common_factor;

  // 7. 限幅
  front_wheel_comp_cmd =
      float_constrain(front_wheel_comp_cmd, 0, MAX_SLOPE_COMP_CMD);
  rear_wheel_comp_cmd =
      float_constrain(rear_wheel_comp_cmd, 0, MAX_SLOPE_COMP_CMD);

  // 8. 将扭矩补偿转换为速度前馈（简化：直接转为速度增量）
  // 注意：这里需要根据电机模型将扭矩转换为速度，当前简化处理
  // 更精确的做法是使用电机动力学模型：tau = J*alpha + b*omega
  float speed_comp_scale = 0.1f; // 扭矩到速度的缩放系数，需根据实际调试
  ff_spd_lf += front_wheel_comp_cmd * speed_comp_scale;
  ff_spd_rf += front_wheel_comp_cmd * speed_comp_scale;
  ff_spd_lb += rear_wheel_comp_cmd * speed_comp_scale;
  ff_spd_rb += rear_wheel_comp_cmd * speed_comp_scale;
#else
  // 非 CHASSIS_BOARD 构建时不做坡度补偿，避免未定义 IMU 引用
  (void)HALF_WHEEL_RADIUS;
#endif
}

// 一阶连续化的阻力补偿计算: 在零速附近按线性比例放大到满额, 超出窗口给满额
static inline float ComputeFrictionFF(float measured_speed_aps,
                                      float ref_speed_cmd,
                                      float friction_full_cmd,
                                      float linear_window_aps) {
  float direction = 0.0f;
  if (fabsf(measured_speed_aps) > 1e-6f)
    direction = (measured_speed_aps > 0.0f) ? 1.0f : -1.0f; // 以实际速度为主
  else if (fabsf(ref_speed_cmd) > 1e-3f)
    direction = (ref_speed_cmd > 0.0f) ? 1.0f : -1.0f; // 零速处回退参考方向
  if (direction == 0.0f)
    return 0.0f;

  float abs_speed = fabsf(measured_speed_aps);
  float gain = 1.0f;
  if (abs_speed < linear_window_aps)
    gain = abs_speed / linear_window_aps; // 一阶连续化：零点线性过渡

  float cmd = direction * friction_full_cmd * gain;
  return float_constrain(cmd, -MAX_FRICTION_COMP_CMD, MAX_FRICTION_COMP_CMD);
}

static void ResistanceCompensation() {
  // 摩擦/阻力补偿: 一阶连续化零点过渡 + 方向回退 + 前馈限幅
  const float linear_window_aps =
      FRICTION_LINEAR_WINDOW_RPM * RPM_2_ANGLE_PER_SEC;

  // 阻力补偿现在直接作用于速度前馈
  // 注意：FRICTION_COMP_* 原本是电流值，需要转换为速度增量
  // 简化方案：将摩擦力补偿值缩放后加到速度前馈
  float friction_to_speed_scale = 0.05f; // 摩擦补偿到速度的缩放系数，需实测调试
  
  // 左前
  ff_spd_lf += ComputeFrictionFF(motor_lf->measure.speed_aps, vt_lf,
                                 FRICTION_COMP_LF, linear_window_aps) * friction_to_speed_scale;
  // 右前
  ff_spd_rf += ComputeFrictionFF(motor_rf->measure.speed_aps, vt_rf,
                                 FRICTION_COMP_RF, linear_window_aps) * friction_to_speed_scale;
  // 左后
  ff_spd_lb += ComputeFrictionFF(motor_lb->measure.speed_aps, vt_lb,
                                 FRICTION_COMP_LB, linear_window_aps) * friction_to_speed_scale;
  // 右后
  ff_spd_rb += ComputeFrictionFF(motor_rb->measure.speed_aps, vt_rb,
                                 FRICTION_COMP_RB, linear_window_aps) * friction_to_speed_scale;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
/* 预测单个电机的输入功率 */
static inline float predict_motor_power(float I, float rpm) {
  float p = TORQUE_COEFF * I * rpm + POWER_MODEL_K2 * rpm * rpm +
            POWER_MODEL_K1 * I * I + POWER_MODEL_CONST;
  return p > 0.0f ? p : 0.0f;
}

static void LimitChassisOutput() {
  // 1) 读取功率上限（裁判系统给出的底盘功率上限，单位: W）
  /*  float max_power_w = 0.0f;
   if (referee_data)
     max_power_w = (float)referee_data->GameRobotState.chassis_power_limit;

   // 2) 预测当前指令将导致的电机输入功率：
   // 使用"当前周期将要发送"的电流目标近似：I_ref = current_PID.Ref ≈
   speed环输出
   // + 前馈（此处用 current 前馈近似）， 简化实现：用 current 前馈 ff_cur_*
   近似
   // I_ref，速度项已在下层 PID 内部生成。 转速使用实时测量 rpm = speed_aps /
   // RPM_2_ANGLE_PER_SEC。
   float rpm_lf = motor_lf->measure.speed_aps / RPM_2_ANGLE_PER_SEC;
   float rpm_rf = motor_rf->measure.speed_aps / RPM_2_ANGLE_PER_SEC;
   float rpm_lb = motor_lb->measure.speed_aps / RPM_2_ANGLE_PER_SEC;
   float rpm_rb = motor_rb->measure.speed_aps / RPM_2_ANGLE_PER_SEC;

   float Iref_lf = ff_cur_lf;
   float Iref_rf = ff_cur_rf;
   float Iref_lb = ff_cur_lb;
   float Iref_rb = ff_cur_rb;

   // 3) 逐电机根据功率模型计算预测功率（若出现负功率，按0处理）
   float p_lf = predict_motor_power(Iref_lf, rpm_lf);
   float p_rf = predict_motor_power(Iref_rf, rpm_rf);
   float p_lb = predict_motor_power(Iref_lb, rpm_lb);
   float p_rb = predict_motor_power(Iref_rb, rpm_rb);

   float p_sum = p_lf + p_rf + p_lb + p_rb;

   // 4) 若预测超过功率上限，等比例缩放"扭矩相关量"（这里用 current
   // 前馈作为扭矩代理）
   if (max_power_w > 1e-3f && p_sum > max_power_w) {
     float k = max_power_w / p_sum;
     // 将缩放作用于参考层：速度参考和电流前馈同时按比例收缩，避免下层超调
     vt_lf *= k;
     vt_rf *= k;
     vt_lb *= k;
     vt_rb *= k;
     ff_cur_lf *= k;
     ff_cur_rf *= k;
     ff_cur_lb *= k;
     ff_cur_rb *= k;
   } */

  // 5) 下发参考
  DJIMotorSetRef(motor_lf, vt_lf);
  DJIMotorSetRef(motor_rf, vt_rf);
  DJIMotorSetRef(motor_lb, vt_lb);
  DJIMotorSetRef(motor_rb, vt_rb);
}

static void ClearFeedforward() {
  ff_spd_lf = ff_spd_rf = ff_spd_lb = ff_spd_rb = 0.0f;
}

/**
 * @brief 对单个速度值应用死区处理
 * @param speed_value 输入的速度值
 * @param threshold 死区阈值
 * @return 处理后的速度值，小于阈值时返回0
 * @note 用于消除小速度指令，防止编码器噪声导致的抖动
 */
static inline float ApplySingleSpeedDeadband(float speed_value,
                                             float threshold) {
  return (fabsf(speed_value) < threshold) ? 0.0f : speed_value;
}

/**
 * @brief 对底盘四个轮子的速度指令应用死区处理
 * @param threshold 死区阈值(角度/秒)
 * @note 防止零速附近的抖动，提高控制稳定性
 *       遵循模块化设计原则，将重复操作封装为可复用函数
 */
static void ApplyChassisSpeedDeadband(float threshold) {
  vt_lf = ApplySingleSpeedDeadband(vt_lf, threshold);
  vt_rf = ApplySingleSpeedDeadband(vt_rf, threshold);
  vt_lb = ApplySingleSpeedDeadband(vt_lb, threshold);
  vt_rb = ApplySingleSpeedDeadband(vt_rb, threshold);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed() {
  // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
  // chassis_feedback_data.vx vy wz =
  //  ...
}

/* ==================================================== */

/* 机器人底盘控制核心任务 */
void ChassisTask() {
  // 后续增加没收到消息的处理(双板的情况)
  // 获取新的控制信息
#ifdef ONE_BOARD
  SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
  chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

  if (chassis_cmd_recv.chassis_mode ==
      CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
  } else { // 正常工作
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);
  }

  // 根据控制模式设定旋转速度
  switch (chassis_cmd_recv.chassis_mode) {
  case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
    chassis_cmd_recv.wz = 0;
    last_follow_wz = 0; // 重置滤波状态
    break;

  case CHASSIS_FOLLOW_GIMBAL_YAW: { // 跟随云台,使用PID+一阶滤波控制
    // PID控制: 期望值为0(对齐),反馈值为offset_angle(偏差角度)
    // 输出为底盘旋转角速度,负号是因为角度偏差和底盘旋转方向相反
    float pid_output =
        -PIDCalculate(&chassis_follow_pid, chassis_cmd_recv.offset_angle, 0.0f);

    // 使用一阶低通滤波器平滑过渡
    // K值说明：K越小滤波越强(保持原有运动状态更多)，K越大响应越快
    // 推荐值: 0.2~0.4
    // - 小陀螺切换时建议用0.2(保持更多旋转动量)
    // - 静止跟随时可以用0.3-0.4(响应更快)
    float filter_K = 0.5f; // 可根据实际调试
    chassis_cmd_recv.wz =
        LowPassFilter_Float(pid_output, filter_K, &last_follow_wz);
    break;
  }

  case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
    chassis_cmd_recv.wz = 1500;
    last_follow_wz = chassis_cmd_recv.wz; // 保存当前wz,便于切换后平滑过渡
    break;

  default:
    break;
  }

  // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
  // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
  static float sin_theta, cos_theta;
  cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  chassis_vx =
      chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
  chassis_vy =
      chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

  // 1. 根据控制模式进行正运动学解算,计算底盘输出
  MecanumCalculate();

  // 1.5. 添加速度指令死区，防止零速抖动
  // ApplyChassisSpeedDeadband(SPEED_DEADBAND_THRESHOLD);

  // 2. 清零当周期电流前馈
  //ClearFeedforward();

  // 3. 计算坡度补偿前馈（如果底盘需要爬坡则启用）
  // SlopeCompensation();  // 可选：上下坡场景启用

  // 4. 计算阻力补偿前馈（推荐启用！）
  //ResistanceCompensation(); // 启用摩擦力补偿

  // 5. 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
  LimitChassisOutput();

  // 6. 根据电机的反馈速度和IMU(如果有)计算真实速度
  // EstimateSpeed();

  // // 获取裁判系统数据
  // 建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
  // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue ,
  // 1:red chassis_feedback_data.enemy_color =
  // referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
  // //
  // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
  // chassis_feedback_data.bullet_speed =
  // referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
  // chassis_feedback_data.rest_heat =
  // referee_data->PowerHeatData.shooter_heat0;

  // 推送反馈消息
#ifdef ONE_BOARD
  PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
  CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}