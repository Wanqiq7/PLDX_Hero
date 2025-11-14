#include "chassis.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "dji_motor.h"
#include "general_def.h"
#include "math.h"
#include "message_center.h"
#include "power_controller.h"
#include "referee_UI.h"
#include "referee_task.h"
#include "robot_def.h"
#include "super_cap.h"
#include "sysid_task.h"
#include "user_lib.h"

/* ============================================================
 * 系统辨识控制开关（测试时改为1，完成后改为0）
 * ============================================================ */
#define ENABLE_CHASSIS_SYSID 0 // 0-关闭，1-启动辨识
#define SYSID_TARGET_MOTOR 0   // 0-左前lf, 1-右前rf, 2-左后lb, 3-右后rb

#if ENABLE_CHASSIS_SYSID
static Publisher_t *sysid_macro_pub = NULL;
static uint8_t sysid_macro_done = 0;

/**
 * @brief 系统辨识宏开关触发函数
 * @note 当ENABLE_CHASSIS_SYSID=1时，上电3秒后自动启动辨识
 *       辨识运行20秒后自动停止，电机完全停止
 *       测试完成后，将宏改为0，重新编译即可恢复正常
 *
 * 使用说明：
 *   1. 架起机器人（轮子悬空）
 *   2. 修改上面的宏：ENABLE_CHASSIS_SYSID = 1
 *   3. 选择目标电机：SYSID_TARGET_MOTOR = 0~3
 *   4. 编译烧录
 *   5. 配置Ozone记录变量（见文档）
 *   6. 上电等待3秒 → 自动启动辨识（持续20秒）
 *   7. 20秒后电机自动停止
 *   8. Ozone导出CSV，运行MATLAB分析
 *   9. 测试完成后，改回：ENABLE_CHASSIS_SYSID = 0
 */
static void ChassisSystemIDSwitch() {
  static uint8_t sysid_stop_sent = 0;

  // 第一阶段：启动辨识（上电3秒后）
  if (!sysid_macro_done && DWT_GetTimeline_s() > 3.0f) {
    // 首次调用，注册发布者
    if (sysid_macro_pub == NULL) {
      sysid_macro_pub =
          PubRegister("chassis_sysid_cmd", sizeof(Chassis_SysID_Ctrl_Cmd_s));
    }

    // 发布启动指令
    Chassis_SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 1,
        .target_motor = SYSID_TARGET_MOTOR,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);

    sysid_macro_done = 1; // 标记已触发
  }

  // 第二阶段：确保停止（上电25秒后，辨识应该已完成）
  if (sysid_macro_done && !sysid_stop_sent && DWT_GetTimeline_s() > 25.0f) {
    // 发送停止指令（确保辨识任务退出）
    Chassis_SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 0,
        .target_motor = SYSID_TARGET_MOTOR,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);

    sysid_stop_sent = 1; // 标记已发送停止指令
    // 注意：电机停止由sysid_task.c中的逻辑处理
  }
}
#endif
/* ============================================================ */

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define SPEED_DEADBAND_THRESHOLD 120.0f         // 角度/秒，约80rpm

/* 遥控器速度增益 - 将归一化值转换为实际速度 */
static const float RC_CMD_MAX_LINEAR_SPEED = 2.0f;  // m/s 线速度最大值
static const float RC_CMD_MAX_ANGULAR_SPEED = 2.5f; // rad/s 角速度最大值
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

/* 功率控制相关变量已移至独立的power_controller模块 */

/* ----------------力控策略相关变量---------------- */
// 力控LQR控制器（替换PID实现更优控制）
// 力控PID控制器（速度→力/扭矩）
static PIDInstance chassis_force_x_pid; // X方向速度PID（输出：力 N）
static PIDInstance chassis_force_y_pid; // Y方向速度PID（输出：力 N）
static PIDInstance chassis_torque_pid;  // 旋转速度PID（输出：扭矩 N·m）

// 速度估算相关变量
static float chassis_estimated_vx = 0.0f; // 底盘估算速度X (m/s)
static float chassis_estimated_vy = 0.0f; // 底盘估算速度Y (m/s)
static float chassis_estimated_wz = 0.0f; // 底盘估算角速度 (rad/s)

// 力控中间变量
static float force_x = 0.0f;   // X方向控制力 (N)
static float force_y = 0.0f;   // Y方向控制力 (N)
static float torque_z = 0.0f;  // Z轴控制扭矩 (N·m)
static float wheel_force[4];   // 各轮分配的力 (N)
static float wheel_current[4]; // 各轮电流指令 (A)

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘坐标系

// ⭐ 力控 + 速度内环：麦轮正运动学解算的目标轮速（rad/s）
static float target_wheel_omega[4]; // 各轮目标角速度，用于速度内环反馈

#define MAX_SLOPE_COMP_CMD 3000 // 坡度补偿最大限幅（预留功能）

void ChassisInit() {
  // 四个轮子的参数一样,改tx_id和反转标志位即可
  Motor_Init_Config_s chassis_motor_config = {
      .can_init_config = {.can_handle = &hcan1},
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = OPEN_LOOP,
              .close_loop_type = OPEN_LOOP,
              .feedforward_flag = FEEDFORWARD_NONE,
          },
      .motor_type = M3508,
  };

  chassis_motor_config.controller_param_init_config.speed_feedforward_ptr =
      NULL;

  chassis_motor_config.can_init_config.tx_id = 2;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE; // 修改：左前电机反转
  motor_lf = DJIMotorInit(&chassis_motor_config);

  chassis_motor_config.can_init_config.tx_id = 3;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL; // 修改：右前电机反转
  motor_rf = DJIMotorInit(&chassis_motor_config);

  chassis_motor_config.can_init_config.tx_id = 1;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL; // 保持：左后电机正常
  motor_lb = DJIMotorInit(&chassis_motor_config);

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

  // 功率控制器初始化（独立模块）
  PowerControllerConfig_t power_config = {
      .k1_init = 0.22f,                  // 转速损耗系数初始值
      .k2_init = 1.2f,                   // 力矩损耗系数初始值
      .k3 = 2.78f,                       // 静态功率损耗
      .rls_lambda = 0.9999f,             // RLS遗忘因子
      .torque_constant = 0.3f,           // M3508电机转矩常数 (Nm/A)
      .current_scale = 20.0f / 16384.0f, // CAN指令值转电流系数
  };
  PowerControllerInit(&power_config);

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
  chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD

  // 底盘跟随云台PID控制器初始化（⭐ 统一力控：输出角速度）
  // ⭐ 注意：输出单位为角速度（rad/s），后续通过力控PID转换为扭矩
  PID_Init_Config_s follow_pid_config = {
      .Kp = 0.25f,               // ⭐ 单位：(rad/s)/度（角度误差→角速度）
      .Ki = 0.0f,                // 积分增益（可选开启）
      .Kd = 0.0f,                // 微分增益（暂不使用）
      .Derivative_LPF_RC = 0.0f, // 不使用微分滤波
      .IntegralLimit = 0.5f,     // ⭐ 积分限幅：1 rad/s
      .MaxOut = 5.0f,            // ⭐ 最大输出：5 rad/s（底盘旋转角速度）
      .DeadBand = 1.58f,         // ⭐ 死区：0.25度（静止时更稳定）
      .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
  };
  PIDInit(&chassis_follow_pid, &follow_pid_config);

  /* ----------------力控策略PID初始化---------------- */
  // X方向力控PID初始化 (输出单位: N)
  PID_Init_Config_s force_x_pid_config = {
      .Kp = 200.0f,                // 比例增益 [N/(m/s)]
      .Ki = 0.0f,                  // 积分增益 [N/(m·s)]
      .Kd = 0.0f,                  // ⭐ 微分增益 [N·s/m]（抑制超调）
      .Derivative_LPF_RC = 0.12f,  // 微分低通滤波
      .IntegralLimit = 100.0f,     // 积分限幅 [N]
      .MaxOut = MAX_CONTROL_FORCE, // 最大输出力 [N]
      .DeadBand = 0.01f,           // 死区 [m/s]
      .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement |
                 PID_DerivativeFilter | PID_Trapezoid_Intergral |
                 PID_ChangingIntegrationRate,
  };
  PIDInit(&chassis_force_x_pid, &force_x_pid_config);

  // Y方向力控PID初始化 (输出单位: N)
  PID_Init_Config_s force_y_pid_config = {
      .Kp = 200.0f,
      .Ki = 0.0f,
      .Kd = 0.0f,
      .Derivative_LPF_RC = 0.02f,
      .IntegralLimit = 100.0f,
      .MaxOut = MAX_CONTROL_FORCE,
      .DeadBand = 0.01f,
      .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement |
                 PID_DerivativeFilter | PID_Trapezoid_Intergral,
  };
  PIDInit(&chassis_force_y_pid, &force_y_pid_config);

  // 旋转扭矩PID初始化 (输出单位: N·m) - 优化参数减少振荡
  PID_Init_Config_s torque_pid_config = {
      .Kp = 1.5f,             // 比例增益 [N·m/(rad/s)] - 从2.5降至1.5减少过冲
      .Ki = 0.0f,             // 积分增益 [N·m/rad]
      .Kd = 0.1f,             // 微分增益 [N·m·s/rad] - 增加阻尼抑制振荡
      .IntegralLimit = 50.0f, // 积分限幅 [N·m]
      .MaxOut = 5.0f,
      .DeadBand = 0.15f, // 死区 [rad/s]
      .Output_LPF_RC = 0.00087f,
      .Improve = PID_OutputFilter | PID_Derivative_On_Measurement,
  };
  PIDInit(&chassis_torque_pid, &torque_pid_config);

  /* ----------------底盘系统辨识初始化---------------- */
  // 初始化底盘轮速系统辨识任务（用于LQR参数标定）
  Chassis_SysIDTaskInit(motor_lf, motor_rf, motor_lb, motor_rb);
}

// 计算每个轮子到旋转中心的距离（单位：m）
// 使用勾股定理: r = sqrt(x^2 + y^2)
// ⭐ WHEEL_BASE和TRACK_WIDTH在robot_def.h中已经是m，无需转换
#define LF_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define RF_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define LB_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define RB_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y, 2.0f))
// 坡度补偿相关定义
#define BACK_COG_LENGTH                                                        \
  DIST_CG_REAR_AXLE // 暂未整定前后重心,所以直接使用前后轮距的一半
#define FRONT_COG_LENGTH DIST_CG_FRONT_AXLE // 可以直接在robot_def.h中修改定义

/* ==================================================== */
/* ----------------力控策略核心函数---------------- */
/* ==================================================== */

/**
 * @brief 麦轮正运动学解算 - 计算各轮目标角速度（用于速度内环反馈）
 * @note  参考robowalker的Kinematics_Inverse_Resolution思想
 *        从底盘速度(vx,vy,wz)解算出各轮的目标角速度
 *        这是力控+速度内环的关键：提供轮速目标值用于动态补偿
 */
static void MecanumKinematicsCalculate() {
  // 麦轮正运动学：从底盘速度解算各轮线速度
  // v_wheel = v_chassis + ω × r_vec
  // 对于45度麦轮，与力分配公式对应

  float vx = chassis_vx;
  float vy = chassis_vy;
  float wz = chassis_cmd_recv.wz;

  // 计算各轮线速度（m/s）- 必须与力分配公式的符号一致
  float wheel_linear_vel[4];
  wheel_linear_vel[0] = (-vx - vy) + wz * LF_CENTER; // 左前 +wz
  wheel_linear_vel[1] = (-vx + vy) - wz * RF_CENTER; // 右前 -wz
  wheel_linear_vel[2] = (vx - vy) - wz * LB_CENTER;  // 左后 -wz
  wheel_linear_vel[3] = (vx + vy) + wz * RB_CENTER;  // 右后 +wz

  // 转换为电机角速度（rad/s）
  // ⭐ 关键：线速度 → 轮子角速度 → 电机角速度（考虑减速比）
  for (int i = 0; i < 4; i++) {
    // 轮子角速度 = 线速度 / 轮子半径
    float wheel_omega = wheel_linear_vel[i] / RADIUS_WHEEL;
    // 电机角速度 = 轮子角速度 × 减速比
    target_wheel_omega[i] = wheel_omega * REDUCTION_RATIO_WHEEL;
  }
}

/**
 * @brief 估算底盘当前速度（基于电机反馈）
 * @note  采用一阶低通滤波平滑速度估算，后续可替换为卡尔曼滤波
 */
static void EstimateChassisVelocity() {
  // 麦轮逆运动学：根据四个轮速反推底盘速度
  // 将电机角速度转换为轮子线速度 (m/s)
  // ⭐ 注意：反转电机的速度反馈需要取反，以匹配底盘运动学坐标系
  float lf_linear_vel = -motor_lf->measure.speed_aps * DEGREE_2_RAD *
                        RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
  float rf_linear_vel = motor_rf->measure.speed_aps * DEGREE_2_RAD *
                        RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
  float lb_linear_vel = motor_lb->measure.speed_aps * DEGREE_2_RAD *
                        RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
  float rb_linear_vel = -motor_rb->measure.speed_aps * DEGREE_2_RAD *
                        RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;

  // 麦轮逆解算（底盘坐标系）
  // 从正解算推导逆运动学方程：
  // vx = (-v_lf - v_rf + v_lb + v_rb) / 4  (纵向速度)
  // vy = (-v_lf + v_rf - v_lb + v_rb) / 4  (横向速度)
  // wz = (v_lf - v_rf - v_lb + v_rb) / (4 * r) (旋转角速度)
  // ⭐ WHEEL_BASE和TRACK_WIDTH在robot_def.h中已经是m，直接计算即可
  float r = sqrtf(HALF_WHEEL_BASE * HALF_WHEEL_BASE +
                  HALF_TRACK_WIDTH * HALF_TRACK_WIDTH);

  float instant_vx =
      (-lf_linear_vel - rf_linear_vel + lb_linear_vel + rb_linear_vel) / 4.0f;
  float instant_vy =
      (-lf_linear_vel + rf_linear_vel - lb_linear_vel + rb_linear_vel) / 4.0f;
  float instant_wz =
      (lf_linear_vel - rf_linear_vel - lb_linear_vel + rb_linear_vel) /
      (4.0f * r);

  // ⭐ 优化1：一阶低通滤波平滑估算，过滤编码器噪声
  static float filtered_vx = 0.0f;
  static float filtered_vy = 0.0f;
  static float filtered_wz = 0.0f;

  // ⭐ 使用一阶低通滤波平滑速度估算，消除编码器噪声
  // 当前α=0.6，截止频率约30.9Hz（200Hz采样下）
  // 计算公式：fc = -200·ln(1-α)/(2π)
  // 注意：如需更快响应可提升至0.7-0.8，如需更稳定可降低至0.5
  const float LPF_ALPHA = 0.85f; // 滤波系数（从LowPassFilter_Float第2个参数）

  // LowPassFilter_Float(新值, 滤波系数, 上次值指针)
  // ⭐ 关键修复：启用滤波，防止chassis_estimated_wz震动导致底盘震荡
  chassis_estimated_vx =
      LowPassFilter_Float(instant_vx, LPF_ALPHA, &filtered_vx);
  chassis_estimated_vy =
      LowPassFilter_Float(instant_vy, LPF_ALPHA, &filtered_vy);
  chassis_estimated_wz =
      LowPassFilter_Float(instant_wz, LPF_ALPHA, &filtered_wz);
}

/**
 * @brief 速度闭环控制 → 力/扭矩输出（力控核心环节1）
 * @note  使用PID替代LQR，更易于调参
 *        参考robowalker的Output_To_Dynamics思想：速度控制器输出力而非速度
 *        ✅ 统一力控链路：所有速度目标（包括跟随角速度）都通过力控PID转换
 */
static void VelocityToForceControl() {
  // 更新速度估算
  EstimateChassisVelocity();

  // 计算目标速度（从运动学解算得到，单位：m/s和rad/s）
  // ✅ 统一处理：所有模式下wz都是角速度(rad/s)
  //    - 跟随模式：wz = 跟随PID输出的目标角速度
  //    - 小陀螺：wz = 固定角速度（如2.5 rad/s）
  //    - 不跟随：wz = 0
  float target_vx = chassis_vx;
  float target_vy = chassis_vy;
  float target_wz = chassis_cmd_recv.wz; // 统一为角速度(rad/s)

  // 速度PID → 力/扭矩（输出力而非速度！）
  // PID输出的是需要的合力(N)和合扭矩(N·m)
  force_x = PIDCalculate(&chassis_force_x_pid, chassis_estimated_vx, target_vx);
  force_y = PIDCalculate(&chassis_force_y_pid, chassis_estimated_vy, target_vy);
  float torque_feedback =
      PIDCalculate(&chassis_torque_pid, chassis_estimated_wz, target_wz);

  // ⭐ 简化版前馈：目标角速度 × 前馈系数
  // 原理：直接预估维持某个角速度所需的扭矩，减少跟随滞后
  // 调试：从3.0开始，逐步增加到满意的响应（建议范围：3~10）
  // 推荐值：6.0对应约8 N·m/(rad/s)的阻尼系数，适合大部分底盘
  const float TORQUE_FEEDFORWARD_COEFF = 7.5f; // 前馈系数 [N·m/(rad/s)]
  float torque_feedforward = TORQUE_FEEDFORWARD_COEFF * target_wz;

  // 总扭矩 = 反馈 + 前馈
  torque_z = torque_feedback + torque_feedforward;

  // 限幅保护（PID内部已有限幅，这里作为二次保护）
  force_x = float_constrain(force_x, -MAX_CONTROL_FORCE, MAX_CONTROL_FORCE);
  force_y = float_constrain(force_y, -MAX_CONTROL_FORCE, MAX_CONTROL_FORCE);
  torque_z = float_constrain(torque_z, -MAX_CONTROL_TORQUE, MAX_CONTROL_TORQUE);
}
/**
 * @brief 力的动力学逆解算（力控核心环节2）
 * @note  将底盘合力分配到各个轮子，使用独立轮距力臂（修复振荡问题）
 *       修复：使用各轮独立力臂而非统一rotation_radius，与正解算保持一致
 */
static void ForceDynamicsInverseResolution() {
  // 麦轮力分配（参考robowalker思想，修正为独立轮距）
  // 对于麦轮，每个轮子受到的力 = 平移力分量 + 旋转力矩分量
  // ⭐ 修复：必须使用与正解算一致的独立轮距力臂，避免参数不匹配导致振荡

  // 力分配矩阵（麦轮45度辊子配置）- 修正：使用独立轮距力臂
  // wheel_force[i] = f_x * cos(angle) + f_y * sin(angle) + torque /
  // r_individual ⭐ 关键修正：符号和力臂必须与正解算中的wz符号一致！
  //
  // 正解算符号参考（333-337行）：
  // vt_lf = ... + wz * LF_CENTER   → +wz，使用LF_CENTER力臂
  // vt_rf = ... - wz * RF_CENTER   → -wz，使用RF_CENTER力臂
  // vt_lb = ... - wz * LB_CENTER   → -wz，使用LB_CENTER力臂
  // vt_rb = ... + wz * RB_CENTER   → +wz，使用RB_CENTER力臂

  // 左前轮 (45度配置) → +wz，使用LF_CENTER独立力臂
  wheel_force[0] = (-force_x - force_y) / 4.0f + torque_z / (4.0f * LF_CENTER);

  // 右前轮 → -wz，使用RF_CENTER独立力臂
  wheel_force[1] = (-force_x + force_y) / 4.0f - torque_z / (4.0f * RF_CENTER);

  // 左后轮 → -wz，使用LB_CENTER独立力臂
  wheel_force[2] = (force_x - force_y) / 4.0f - torque_z / (4.0f * LB_CENTER);

  // 右后轮 → +wz，使用RB_CENTER独立力臂
  wheel_force[3] = (force_x + force_y) / 4.0f + torque_z / (4.0f * RB_CENTER);
}

/**
 * @brief 力→电流转换 + 摩擦补偿（力控核心环节3）
 * @note  参考robowalker的动摩擦补偿思想，实现零点连续化
 */
static void ForceToCurrentConversion() {
  // 依次处理四个轮子
  // 依次处理四个轮子
  DJIMotorInstance *motors[4] = {motor_lf, motor_rf, motor_lb, motor_rb};

  for (int i = 0; i < 4; i++) {
    // 1. 基础转换
    float base_current = wheel_force[i] * RADIUS_WHEEL / M3508_TORQUE_CONSTANT;

    // 2. 速度内环反馈
    float actual_omega;
    if (i == 0 || i == 3) {
      actual_omega = -motors[i]->measure.speed_aps * DEGREE_2_RAD;
    } else {
      actual_omega = motors[i]->measure.speed_aps * DEGREE_2_RAD;
    }
    float target_omega = target_wheel_omega[i];

    float speed_feedback = 0.0f;
    float omega_error = target_omega - actual_omega;

    // 二级速度误差滤波，减少噪声放大
    static float filtered_omega_error[4] = {0.0f}; // 每个轮子独立滤波状态
    const float OMEGA_ERROR_LPF_ALPHA = 0.85f;     // 二级滤波系数
    omega_error = LowPassFilter_Float(omega_error, OMEGA_ERROR_LPF_ALPHA,
                                      &filtered_omega_error[i]);

    // 过零保护逻辑...
    const float OMEGA_THRESHOLD = 15.0f;
    if (fabsf(target_omega) < OMEGA_THRESHOLD &&
        fabsf(actual_omega) < OMEGA_THRESHOLD) {
      speed_feedback = 0.0f;
    } else if (target_omega * actual_omega < 0.0f) {
      speed_feedback = 0.05f * WHEEL_SPEED_FEEDBACK_COEFF * omega_error;
    } else {
      speed_feedback = WHEEL_SPEED_FEEDBACK_COEFF * omega_error;
    }

    // 3. 摩擦补偿 - 使用平滑tanh函数消除阶跃不连续性
    float friction_comp = 0.0f;
    // 使用tanh函数实现平滑过渡，避免零速度处的阶跃
    float smooth_factor = tanhf(target_omega / FRICTION_THRESHOLD_OMEGA);
    friction_comp = smooth_factor * FRICTION_DYNAMIC_CURRENT;

    // 4. 总电流
    wheel_current[i] = base_current + speed_feedback + friction_comp;

    // 5. 限幅保护
    wheel_current[i] = float_constrain(wheel_current[i], -MAX_WHEEL_CURRENT,
                                       MAX_WHEEL_CURRENT);
  }
}

/* 机器人底盘控制核心任务 */
void ChassisTask() {
#if ENABLE_CHASSIS_SYSID
  // 系统辨识宏开关触发（ENABLE_CHASSIS_SYSID=1时有效）
  ChassisSystemIDSwitch();
#endif

  // 检查系统辨识是否激活
  if (Chassis_SysIDIsActive()) {
    // 系统辨识激活时，完全由辨识任务控制电机
    // 底盘控制任务不干预，直接返回
    return;
  }

  // 后续增加没收到消息的处理(双板的情况)
  // 获取新的控制信息
#ifdef ONE_BOARD
  SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
  chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

  // === 应用遥控器速度增益 ===
  // 从Gimbal接收到的vx/vy是归一化值(-1.0~1.0)
  // 需要乘以增益转换为实际速度(m/s)
  chassis_cmd_recv.vx *= RC_CMD_MAX_LINEAR_SPEED;
  chassis_cmd_recv.vy *= RC_CMD_MAX_LINEAR_SPEED;
  // 注意: wz(角速度)由底盘根据模式自动设定，不需要在这里处理

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

  case CHASSIS_FOLLOW_GIMBAL_YAW: { // 跟随云台,统一力控链路
    // ⭐ PID输出目标角速度（rad/s）
    // 统一进入力控PID转换为扭矩
    // 输入：角度误差（度）
    // 输出：目标角速度（rad/s）
    float follow_angular_vel = -PIDCalculate(
        &chassis_follow_pid, chassis_cmd_recv.near_center_error, 0.0f);

    // 一阶滤波器平滑
    static float last_follow_wz = 0.0f;
    chassis_cmd_recv.wz =
        LowPassFilter_Float(follow_angular_vel, 0.80f, &last_follow_wz);
    // ✅ wz统一为角速度（rad/s），后续通过chassis_torque_pid转换为扭矩
    break;
  }

  case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
    chassis_cmd_recv.wz = 2.5f;           // rad/s
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

  /* ================== 力控策略控制流程 ================== */
  // 参考robowalker的力控思想，实现从速度到电流的完整链路

  // 0. 正运动学解算：计算各轮目标角速度（用于速度内环）
  //    ⭐ robowalker的关键：提供轮速目标值用于动态补偿
  MecanumKinematicsCalculate();

  // 1. 速度闭环 → 力/扭矩 (力控核心环节1)
  //    速度控制器输出的不是速度参考值，而是需要的合力和合扭矩
  //    ✅ 统一处理：所有模式的wz（角速度）都通过chassis_torque_pid转换为扭矩
  VelocityToForceControl();

  // 2. 力的动力学逆解算 (力控核心环节2)
  //    将底盘合力分配到各个轮子
  ForceDynamicsInverseResolution();

  // 3. 力→电流转换 + 速度内环 + 摩擦补偿 (力控核心环节3)
  //    ⭐ robowalker核心：base_current + speed_feedback + friction_comp
  ForceToCurrentConversion();

  // 4. 下发电机电流指令
  //    注意：这里直接使用wheel_current数组，单位为安培(A)
  //    需要转换为CAN指令值：cmd = current / M3508_CMD_TO_CURRENT_COEFF
  DJIMotorSetRef(motor_lf, wheel_current[0] / M3508_CMD_TO_CURRENT_COEFF);
  DJIMotorSetRef(motor_rf, wheel_current[1] / M3508_CMD_TO_CURRENT_COEFF);
  DJIMotorSetRef(motor_lb, wheel_current[2] / M3508_CMD_TO_CURRENT_COEFF);
  DJIMotorSetRef(motor_rb, wheel_current[3] / M3508_CMD_TO_CURRENT_COEFF);

#if POWER_CONTROLLER_ENABLE
  // 4.5. 更新功率控制器数据（在功率限制前更新）
  // 更新裁判系统数据
  if (referee_data) {
    PowerUpdateRefereeData(
        (float)referee_data->GameRobotState.chassis_power_limit,
        (float)referee_data->PowerHeatData.buffer_energy,
        (float)referee_data->PowerHeatData.chassis_power);
  }

  // 更新超级电容数据
  uint8_t cap_online = (cap && cap->can_ins->rx_len > 0) ? 1 : 0;
  uint8_t cap_voltage = cap_online ? cap->cap_msg.vol : 0;
  PowerUpdateCapData(cap_voltage, cap_online);

  // 更新电机反馈数据（用于RLS参数辨识）
  float motor_speeds[4] = {
      motor_lf->measure.speed_aps * DEGREE_2_RAD,
      motor_rf->measure.speed_aps * DEGREE_2_RAD,
      motor_lb->measure.speed_aps * DEGREE_2_RAD,
      motor_rb->measure.speed_aps * DEGREE_2_RAD,
  };

  // 估算电机转矩（通过下发电流转换）
  const float TORQUE_CONSTANT = 0.3f; // Nm/A
  float motor_torques[4] = {
      wheel_current[0] * TORQUE_CONSTANT,
      wheel_current[1] * TORQUE_CONSTANT,
      wheel_current[2] * TORQUE_CONSTANT,
      wheel_current[3] * TORQUE_CONSTANT,
  };
  PowerUpdateMotorFeedback(motor_speeds, motor_torques);
#endif

  // 5. 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
  // LimitChassisOutput();  // 力控策略中已在ForceToCurrentConversion中完成限幅

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