#ifndef _APPTASK_H
#define _APPTASK_H
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
#include "Base_Control.h"
#include "Ros_Control_Task.h"

//任务优先级
#define START_TASK_PRIO		7
#define State_Test_TASK_PRIO			4
#define RCPROTECTION_TASK_PRIO		1
#define Gimbal_TASK_PRIO		2
#define CHASSIS_TASK_PRIO		3
#define ROS_TASK_PRIO		3
#define CAN_Send_TASK_PRIO 7
#define FW_TASK_PRIO 6
#define Flick_TASK_PRIO 4
#define Referee_System_PRIO 4
#define Base_Control_PRIO 1
//任务堆栈大小	
#define START_STK_SIZE 		512
#define State_Test_TASK_STK_SIZE 			512
#define RC_Protection_TASK_STK_SIZE 		512
#define Gimbal_STK_SIZE 		512
#define CHASSIS_STK_SIZE		512
#define ROS_CONTROL_STK_SIZE		512
#define CAN_Send_TASK_STK_SIZE	512
#define FW_Keep_TASK_STK_SIZE 		512
#define Flick_TASK_STK_SIZE 		512
#define Referee_System_TASK_STK_SIZE 		512
//#define Base_Control_TASK_STK_SIZE 		512

//任务块
extern OS_TCB KeyTaskTCB;
extern OS_TCB Led0TaskTCB;
extern OS_TCB Led1TaskTCB;
extern OS_TCB FloatTaskTCB;

//信号量
extern OS_SEM	MY_SEM;

//互斥信号量
extern OS_MUTEX TEST_MUTEX;

//任务函数
void start_task(void *p_arg);
extern void key_task(void *p_arg);
extern void led0_task(void *p_arg);
extern void led1_task(void *p_arg);
extern void float_task(void *p_arg);




typedef enum
{
	KEY0_FLAG	= (1 << 1),
	KEY1_FLAG	= (1 << 2),
	KEY_ALL_FLAGS = (0x0003),	
}KEY_FLAGS;

#endif
