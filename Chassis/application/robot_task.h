/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#include "HT04.h"
#include "buzzer.h"
#include "daemon.h"
#include "ins_task.h"
#include "master_process.h"
#include "motor_task.h"
#include "referee_task.h"
#include "robot.h"
#include "sysid_task.h"
#include "power_controller.h"  // 新增：功率控制模块

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId sysidTaskHandle;

#if POWER_CONTROLLER_ENABLE
osThreadId powerTaskHandle;    // 新增：功率控制任务句柄
#endif

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);
void StartSYSIDTASK(void const *argument);

#if POWER_CONTROLLER_ENABLE
void StartPOWERTASK(void const *argument);  // 新增：功率控制任务
#endif

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit() {
  osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
  insTaskHandle = osThreadCreate(
      osThread(instask),
      NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
  // // 后续修改为读取传感器数据准备好的中断处理,

  osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
  motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

  osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
  daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

  osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 2048);
  robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

  osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
  uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

  osThreadDef(sysidtask, StartSYSIDTASK, osPriorityAboveNormal, 0, 512);
  sysidTaskHandle = osThreadCreate(osThread(sysidtask), NULL);

#if POWER_CONTROLLER_ENABLE
  // 功率控制任务：优先级设置为Normal，略低于Robot任务，周期2ms (500Hz)
  // 仅在POWER_CONTROLLER_ENABLE=1时创建
  osThreadDef(powertask, StartPOWERTASK, osPriorityNormal, 0, 256);
  powerTaskHandle = osThreadCreate(osThread(powertask), NULL);
#endif

  HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument) {
  static float ins_start;
  static float ins_dt;
  INS_Init(); // 确保BMI088被正确初始化.
  LOGINFO("[freeRTOS] INS Task Start");
  for (;;) {
    // 1kHz
    ins_start = DWT_GetTimeline_ms();
    INS_Task();
    ins_dt = DWT_GetTimeline_ms() - ins_start;
    if (ins_dt > 1)
      LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
    VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
    osDelay(1);
  }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument) {
  static float motor_dt;
  static float motor_start;
  LOGINFO("[freeRTOS] MOTOR Task Start");
  for (;;) {
    motor_start = DWT_GetTimeline_ms();
    MotorControlTask();
    motor_dt = DWT_GetTimeline_ms() - motor_start;
    if (motor_dt > 5)
      LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
    osDelay(5);
  }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument) {
  static float daemon_dt;
  static float daemon_start;
  BuzzerInit();
  LOGINFO("[freeRTOS] Daemon Task Start");
  for (;;) {
    // 100Hz
    daemon_start = DWT_GetTimeline_ms();
    DaemonTask();
    BuzzerTask();
    daemon_dt = DWT_GetTimeline_ms() - daemon_start;
    if (daemon_dt > 10)
      LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
    osDelay(10);
  }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument) {
  static float robot_dt;
  static float robot_start;
  LOGINFO("[freeRTOS] ROBOT core Task Start");
  // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
  for (;;) {
    robot_start = DWT_GetTimeline_ms();
    RobotTask();
    robot_dt = DWT_GetTimeline_ms() - robot_start;
    if (robot_dt > 2)
      LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]",
               &robot_dt);
    osDelay(2); // 500Hz: 2ms周期
  }
}

__attribute__((noreturn)) void StartUITASK(void const *argument) {
  LOGINFO("[freeRTOS] UI Task Start");
  MyUIInit();
  LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
  for (;;) {
    // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
    UITask();
    osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
  }
}

__attribute__((noreturn)) void StartSYSIDTASK(void const *argument) {
  static float sysid_start;
  static float sysid_dt;
  LOGINFO("[freeRTOS] System Identification Task Start");

  // 系统辨识任务初始化在GimbalInit()中完成，这里等待初始化完成
  osDelay(100); // 等待云台初始化完成

  for (;;) {
    // 1kHz 精确循环
    sysid_start = DWT_GetTimeline_ms();
    SysIDTask();
    sysid_dt = DWT_GetTimeline_ms() - sysid_start;
    if (sysid_dt > 1)
      LOGERROR("[freeRTOS] SYSID Task is being DELAY! dt = [%f]", &sysid_dt);
    osDelay(1);
  }
}

#if POWER_CONTROLLER_ENABLE
/**
 * @brief 功率控制任务
 * @note 独立任务，负责RLS参数辨识和能量环控制
 *       优先级略低于Robot任务，周期2ms (500Hz)
 *       仅在POWER_CONTROLLER_ENABLE=1时编译
 */
__attribute__((noreturn)) void StartPOWERTASK(void const *argument) {
  static float power_start;
  static float power_dt;
  LOGINFO("[freeRTOS] Power Controller Task Start");

  // 等待底盘初始化完成
  osDelay(100);

  for (;;) {
    // 500Hz: 2ms周期
    power_start = DWT_GetTimeline_ms();
    PowerControllerTask();  // RLS更新 + 能量环控制
    power_dt = DWT_GetTimeline_ms() - power_start;
    
    if (power_dt > 2)
      LOGERROR("[freeRTOS] POWER Task is being DELAY! dt = [%f]", &power_dt);
    
    osDelay(2);
  }
}
#endif
