/**
 * @file sysid_task.h
 * @brief 系统辨识独立任务头文件
 */

#ifndef SYSID_TASK_H
#define SYSID_TASK_H

#include "dji_motor.h"
#include "ins_task.h"
#include "stdint.h"

/**
 * @brief 系统辨识任务初始化
 * @param yaw YAW电机实例指针
 * @param pitch PITCH电机实例指针
 * @param imu IMU数据指针
 */
void SysIDTaskInit(DJIMotorInstance *yaw, DJIMotorInstance *pitch, attitude_t *imu);

/**
 * @brief 系统辨识任务主循环
 * @note 在1kHz FreeRTOS任务中调用
 */
void SysIDTask(void);

/**
 * @brief 检查系统辨识任务是否激活
 * @return 1-激活，0-未激活
 */
uint8_t SysIDIsActive(void);

#endif // SYSID_TASK_H

