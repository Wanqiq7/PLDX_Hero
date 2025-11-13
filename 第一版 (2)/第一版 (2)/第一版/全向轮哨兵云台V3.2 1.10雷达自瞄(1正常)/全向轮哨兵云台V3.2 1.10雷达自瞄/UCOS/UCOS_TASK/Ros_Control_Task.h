#ifndef __ROS_CONTROL_TASK_H
#define __ROS_CONTROL_TASK_H
#include <sys.h>
#include <includes.h>
#include <os_app_hooks.h>
#include "Time1.h"
#include "RC_Protection.h"
#include "GimbalTask.h"
#include "chassisTask.h"
#include "State_Test.h"
#include "Can_Task.h"
#include "FW_Keep.h"
#include "FlickTask.h"
#include "Referee_System.h"


void Ros_Control_Task(void *p_arg);

//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	int Motor_Out;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

#endif
