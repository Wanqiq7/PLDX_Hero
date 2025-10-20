/**
 * @file sysid_task.c
 * @brief 系统辨识独立任务，以1kHz频率执行方波阶跃测试
 * @note 从gimbal.c分离出来，确保精确的采样周期
 */

#include "sysid_task.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"
#include <string.h>

/* ==================== 系统辨识方波阶跃信号参数定义 ==================== */
#define SYS_ID_STEP_AMPLITUDE 8000.0f // 阶跃信号幅值（电流指令，单位：CAN指令值，范围：-16384~16384）
#define SYS_ID_STEP_INTERVAL 2.0f     // 每个阶跃的持续时间 [s]
#define SYS_ID_TOTAL_DURATION 20.0f   // 总测试持续时间 [s]
#define SYS_ID_TARGET_MOTOR_YAW 0     // 辨识目标：0-YAW轴，1-PITCH轴

/* ==================== 私有变量 ==================== */
static Publisher_t *sysid_pub;      // 系统辨识反馈发布者
static Subscriber_t *sysid_sub;     // 系统辨识指令订阅者

static DJIMotorInstance *sysid_yaw_motor = NULL;
static DJIMotorInstance *sysid_pitch_motor = NULL;
static attitude_t *sysid_imu_data = NULL;

// 系统辨识数据实例
static SysID_Feedback_s sysid_data = {0};

// DWT时间戳，用于精确测量任务实际周期
static uint32_t step_tick_last = 0;

// 任务状态标志
static uint8_t sysid_initialized = 0;
static uint8_t sysid_active = 0;

/* ==================== 私有函数声明 ==================== */
static float SystemID_GenerateStepSignal(void);
static void SystemID_Reset(void);
static void SystemID_ConfigMotors(void);

/* ==================== 函数实现 ==================== */

/**
 * @brief 系统辨识任务初始化
 */
void SysIDTaskInit(DJIMotorInstance *yaw, DJIMotorInstance *pitch, attitude_t *imu) {
    // 保存电机和IMU指针
    sysid_yaw_motor = yaw;
    sysid_pitch_motor = pitch;
    sysid_imu_data = imu;
    
    // 注册消息中心话题
    sysid_pub = PubRegister("sysid_feedback", sizeof(SysID_Feedback_s));
    sysid_sub = SubRegister("sysid_cmd", sizeof(SysID_Ctrl_Cmd_s));
    
    // 初始化数据结构
    memset(&sysid_data, 0, sizeof(SysID_Feedback_s));
    
    sysid_initialized = 0;
    sysid_active = 0;
}

/**
 * @brief 生成方波阶跃信号用于云台电机系统辨识
 * @return float 当前时刻的方波信号值
 *
 * @note 本函数使用DWT精确测量实际任务周期，自动适配不同频率的调用
 */
static float SystemID_GenerateStepSignal(void) {
    // 1) 使用DWT精确测量实际任务周期
    float dt = DWT_GetDeltaT(&step_tick_last);

    // 异常保护：防止首次调用或异常情况下dt过大
    // 首次调用时，DWT_GetDeltaT会返回自系统启动以来的时间，需要丢弃
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.001f; // 默认1ms，对应1kHz控制频率
        // 如果是异常大值，重新同步时间戳
        if (dt > 0.05f) {
            DWT_GetDeltaT(&step_tick_last); // 再次调用以同步时间戳
        }
    }

    // 诊断：记录实际dt和计算频率
    sysid_data.call_counter++;
    sysid_data.actual_dt = dt;
    if (dt > 0.0001f) { // 避免除零
        sysid_data.task_freq = 1.0f / dt;
    }

    // 2) 累加实际运行时间
    sysid_data.time_elapsed += dt;

    // 3) 检查是否超过总测试时间
    if (sysid_data.time_elapsed >= SYS_ID_TOTAL_DURATION) {
        sysid_data.is_finished = 1; // 辨识结束，标记完成
        return 0.0f;
    }

    // 4) 计算当前应处于第几个阶跃周期（整数除法）
    uint32_t current_step_index = (uint32_t)(sysid_data.time_elapsed / SYS_ID_STEP_INTERVAL);

    // 5) 根据周期索引的奇偶性决定输出正向还是反向
    // 偶数索引(0, 2, 4, ...): 正向幅值
    // 奇数索引(1, 3, 5, ...): 反向幅值
    if (current_step_index % 2 == 0) {
        sysid_data.step_state = 0; // 正向
        return SYS_ID_STEP_AMPLITUDE;
    } else {
        sysid_data.step_state = 1; // 反向
        return -SYS_ID_STEP_AMPLITUDE;
    }
}

/**
 * @brief 系统辨识数据重置函数
 * @note 在进入阶跃辨识模式时调用，重置辨识数据和DWT时间基
 */
static void SystemID_Reset(void) {
    memset(&sysid_data, 0, sizeof(SysID_Feedback_s));
    
    // 预热DWT时间戳，避免首次调用时dt异常大
    // 方法：调用一次DWT_GetDeltaT来同步时间戳，但不使用返回值
    DWT_GetDeltaT(&step_tick_last);
}

/**
 * @brief 配置电机为开环/闭环模式
 */
static void SystemID_ConfigMotors(void) {
#if SYS_ID_TARGET_MOTOR_YAW == 0
    // 配置 YAW 轴为开环
    DJIMotorEnable(sysid_yaw_motor);
    sysid_yaw_motor->motor_settings.close_loop_type = OPEN_LOOP;
    sysid_yaw_motor->motor_settings.outer_loop_type = OPEN_LOOP;
    sysid_yaw_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;

    // 配置 PITCH 轴保持闭环不动
    DJIMotorEnable(sysid_pitch_motor);
    DJIMotorChangeFeed(sysid_pitch_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(sysid_pitch_motor, SPEED_LOOP, OTHER_FEED);
#else
    // 配置 PITCH 轴为开环
    DJIMotorEnable(sysid_pitch_motor);
    sysid_pitch_motor->motor_settings.close_loop_type = OPEN_LOOP;
    sysid_pitch_motor->motor_settings.outer_loop_type = OPEN_LOOP;
    sysid_pitch_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;

    // 配置 YAW 轴保持闭环不动
    DJIMotorEnable(sysid_yaw_motor);
    DJIMotorChangeFeed(sysid_yaw_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(sysid_yaw_motor, SPEED_LOOP, OTHER_FEED);
#endif
}

/**
 * @brief 系统辨识任务主循环
 * @note 在1kHz FreeRTOS任务中调用
 */
void SysIDTask(void) {
    // 检查初始化状态
    if (sysid_yaw_motor == NULL || sysid_pitch_motor == NULL || sysid_imu_data == NULL) {
        return; // 未初始化，直接返回
    }

    // 获取控制指令
    static SysID_Ctrl_Cmd_s sysid_cmd;
    SubGetMessage(sysid_sub, &sysid_cmd);

    // 检查是否使能辨识
    if (!sysid_cmd.enable) {
        // 如果之前在运行，现在停止了，需要重置状态
        if (sysid_initialized) {
            sysid_initialized = 0;
            sysid_active = 0;
            // 停止电机控制
#if SYS_ID_TARGET_MOTOR_YAW == 0
            // YAW轴恢复闭环
            DJIMotorEnable(sysid_yaw_motor);
            DJIMotorChangeFeed(sysid_yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(sysid_yaw_motor, SPEED_LOOP, OTHER_FEED);
#else
            // PITCH轴恢复闭环
            DJIMotorEnable(sysid_pitch_motor);
            DJIMotorChangeFeed(sysid_pitch_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(sysid_pitch_motor, SPEED_LOOP, OTHER_FEED);
#endif
        }
        return; // 未使能，直接返回
    }

    // 首次进入辨识模式：重置数据 + 配置电机
    if (!sysid_initialized) {
        SystemID_Reset();
        SystemID_ConfigMotors();
        sysid_initialized = 1;
        sysid_active = 1;
    }

    // 如果辨识完成，停止激励信号并重置标志
    if (sysid_data.is_finished) {
        sysid_initialized = 0;
        sysid_active = 0;
        
        // 停止电机或切换回闭环
#if SYS_ID_TARGET_MOTOR_YAW == 0
        sysid_yaw_motor->motor_controller.pid_ref = 0.0f;
#else
        sysid_pitch_motor->motor_controller.pid_ref = 0.0f;
#endif
        
        // 发布最终数据
        PubPushMessage(sysid_pub, &sysid_data);
        return;
    }

    // 生成方波阶跃信号（电流指令）
    sysid_data.step_input = SystemID_GenerateStepSignal();

#if SYS_ID_TARGET_MOTOR_YAW == 0
    // YAW轴：更新电流指令，采集速度反馈
    sysid_yaw_motor->motor_controller.pid_ref = sysid_data.step_input;
    sysid_data.motor_output = sysid_imu_data->Gyro[2]; // Yaw轴陀螺仪，rad/s

    // PITCH轴：保持指定角度
    DJIMotorSetRef(sysid_pitch_motor, sysid_cmd.pitch_ref);
#else
    // PITCH轴：更新电流指令，采集速度反馈
    sysid_pitch_motor->motor_controller.pid_ref = sysid_data.step_input;
    sysid_data.motor_output = sysid_imu_data->Gyro[0]; // Pitch轴陀螺仪，rad/s

    // YAW轴：保持指定角度
    DJIMotorSetRef(sysid_yaw_motor, sysid_cmd.yaw_ref);
#endif

    // 发布辨识反馈数据
    PubPushMessage(sysid_pub, &sysid_data);
}

/**
 * @brief 检查系统辨识任务是否激活
 */
uint8_t SysIDIsActive(void) {
    return sysid_active;
}

