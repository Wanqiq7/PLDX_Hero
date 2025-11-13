#include "AppTask.h"

//任务控制块
OS_TCB StartTaskTCB;
OS_TCB StateTestTaskTCB;
OS_TCB RC_ProtectionTCB;
OS_TCB GimbalTaskTCB;
OS_TCB ChassisTaskTCB;
OS_TCB CANsendTaskTCB;
OS_TCB FW_KeepTCB;
OS_TCB FlickTCB;
OS_TCB RefereeSystemTCB;
OS_TCB Base_ControlTCB;
OS_TCB Ros_ControlTCB;//ros控制任务

//任务堆栈
CPU_STK START_TASK_STK[START_STK_SIZE];
__align(8) CPU_STK	CHASSIS_TASK_STK[CHASSIS_STK_SIZE];
__align(8) CPU_STK	ROS_TASK_STK[ROS_CONTROL_STK_SIZE];
CPU_STK State_Test_TASK_STK[State_Test_TASK_STK_SIZE];
CPU_STK RC_Protection_TASK_STK[RC_Protection_TASK_STK_SIZE];
CPU_STK Gimbal_TASK_STK[Gimbal_STK_SIZE];
CPU_STK CAN_Send_TASK_STK[CAN_Send_TASK_STK_SIZE];
CPU_STK FW_Keep_TASK_STK[FW_Keep_TASK_STK_SIZE];
CPU_STK Flick_TASK_STK[Flick_TASK_STK_SIZE];
CPU_STK Referee_System_TASK_STK[Referee_System_TASK_STK_SIZE];
//CPU_STK Base_Control_TASK_STK[Base_Control_TASK_STK_SIZE];


OS_FLAG_GRP	KeyEventFlags;		//定义一个事件标志组


//信号量控制块
OS_SEM	MY_SEM;

//互斥信号量
OS_MUTEX	TEST_MUTEX;


//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if OS_CFG_APP_HOOKS_EN				//使用钩子函数
	App_OS_SetAllHooks();			
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	
	//创建信号量
	OSSemCreate((OS_SEM*	)&MY_SEM,
									(CPU_CHAR	*	)"MY_SEM",
									(OS_SEM_CTR)1,
									(OS_ERR		*	)&err);
								 
//	//创建互斥信号量
//	OSMutexCreate((OS_MUTEX  	* )&TEST_MUTEX,
//									(CPU_CHAR	* )"TEST_MUTEX",
//                  (OS_ERR  	* )&err);
										 
	//定时器1
	OSTmrCreate((OS_TMR		*)&tmr1,			//定时器1
								(CPU_CHAR	*)"tmr1",		//定时器名字
                (OS_TICK	 )20,				//20*10=200ms
                (OS_TICK	 )100,      //100*10=1000ms
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, 		//周期模式
                (OS_TMR_CALLBACK_PTR)tmr1_callback,	//定时器1回调函数
                (void	    *)0,				//参数为0
                (OS_ERR	  *)&err);		//返回的错误码
	
	//创建状态机
	OSTaskCreate((OS_TCB 	* )&StateTestTaskTCB,		
				 (CPU_CHAR	* )"state Test task", 		
								(OS_TASK_PTR )State_Test_task, 			
								(void		* )0,					
								(OS_PRIO	  )State_Test_TASK_PRIO,     
								(CPU_STK   * )&State_Test_TASK_STK[0],	
								(CPU_STK_SIZE)State_Test_TASK_STK_SIZE/10,	
								(CPU_STK_SIZE)State_Test_TASK_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,					
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								(OS_ERR 	* )&err);				
				 
		
	//创建CAN发送队列任务
	OSTaskCreate((OS_TCB 	* )&CANsendTaskTCB,		
				 (CPU_CHAR	* )"CAN send task", 		
								(OS_TASK_PTR )CAN_Send_Task, 			
								(void		* )0,					
								(OS_PRIO	  )CAN_Send_TASK_PRIO,     
								(CPU_STK   * )&CAN_Send_TASK_STK[0],	
								(CPU_STK_SIZE)CAN_Send_TASK_STK_SIZE/10,	
								(CPU_STK_SIZE)CAN_Send_TASK_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,					
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								(OS_ERR 	* )&err);	
								
								
	//创建遥控器保护任务
	OSTaskCreate((OS_TCB 	* )&RC_ProtectionTCB,		
				 (CPU_CHAR	* )"Remote control protection task", 		
								(OS_TASK_PTR )RC_Protection_task, 			
								(void		* )0,					
								(OS_PRIO	  )RCPROTECTION_TASK_PRIO,     
								(CPU_STK   * )&RC_Protection_TASK_STK[0],	
								(CPU_STK_SIZE)RC_Protection_TASK_STK_SIZE/10,	
								(CPU_STK_SIZE)RC_Protection_TASK_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,					
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								(OS_ERR 	* )&err);				
				 
	//创建摩擦轮保持任务
	OSTaskCreate((OS_TCB 	* )&FW_KeepTCB,		
				 (CPU_CHAR	* )"Friction Wheel Keep task", 		
								(OS_TASK_PTR )FW_Keep_Task, 			
								(void		* )0,					
								(OS_PRIO	  )FW_TASK_PRIO,     
								(CPU_STK   * )&FW_Keep_TASK_STK[0],	
								(CPU_STK_SIZE)FW_Keep_TASK_STK_SIZE/10,	
								(CPU_STK_SIZE)FW_Keep_TASK_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,					
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								(OS_ERR 	* )&err);		

								
	//创建拨弹任务
	OSTaskCreate((OS_TCB 	* )&FlickTCB,		
				 (CPU_CHAR	* )"Flick task", 		
								(OS_TASK_PTR )Flick_TASK, 			
								(void		* )0,					
								(OS_PRIO	  )Flick_TASK_PRIO,     
								(CPU_STK   * )&Flick_TASK_STK[0],	
								(CPU_STK_SIZE)Flick_TASK_STK_SIZE/10,	
								(CPU_STK_SIZE)Flick_TASK_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,					
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								(OS_ERR 	* )&err);	
								
	//创建裁判系统响应任务
	OSTaskCreate((OS_TCB 	* )&RefereeSystemTCB,		
				 (CPU_CHAR	* )"Referee system task", 		
								(OS_TASK_PTR )Referee_system_TASK, 			
								(void		* )0,					
								(OS_PRIO	  )Referee_System_PRIO,     
								(CPU_STK   * )&Referee_System_TASK_STK[0],	
								(CPU_STK_SIZE)Referee_System_TASK_STK_SIZE/10,	
								(CPU_STK_SIZE)Referee_System_TASK_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,					
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								(OS_ERR 	* )&err);	
								
	//创建云台运动任务
	OSTaskCreate((OS_TCB 	* )&GimbalTaskTCB,		
				 (CPU_CHAR	* )"Gimbal task", 				//名称字符串
								(OS_TASK_PTR )Gimbal_task, 	  //任务函数		
								(void		* )0,									//传参
								(OS_PRIO	  )Gimbal_TASK_PRIO,    //优先级 越小优先级越高	
								(CPU_STK   * )&Gimbal_TASK_STK[0],	//任务栈起始地址
								(CPU_STK_SIZE)Gimbal_STK_SIZE/10,		//任务栈警戒线
								(CPU_STK_SIZE)Gimbal_STK_SIZE,			//任务栈大小
								(OS_MSG_QTY  )0,										//消息队列大小
								(OS_TICK	  )0,											//任务时间片
								(void   	* )0,											//指向用户扩展指针
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项共有5个  对任务堆栈检查并清除任务堆栈
								(OS_ERR 	* )&err);																			//指向接收错误代码指针变量
				 
	//创建底盘任务
	OSTaskCreate((OS_TCB 	* )&ChassisTaskTCB,		
				 (CPU_CHAR	* )"chassis test task", 		
								(OS_TASK_PTR )Chassis_task, 			
								(void		* )0,					
								(OS_PRIO	  )CHASSIS_TASK_PRIO,     	
								(CPU_STK   * )&CHASSIS_TASK_STK[0],	
								(CPU_STK_SIZE)CHASSIS_STK_SIZE/10,	
								(CPU_STK_SIZE)CHASSIS_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,				
								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
								(OS_ERR 	* )&err);	
								
								
								
								
	//创建ROS任务
	OSTaskCreate((OS_TCB 	* )&Ros_ControlTCB,		
				 (CPU_CHAR	* )"Ros_Control_Task", 		
								(OS_TASK_PTR )Ros_Control_Task, 			
								(void		* )0,					
								(OS_PRIO	  )ROS_TASK_PRIO,     	
								(CPU_STK   * )&ROS_TASK_STK[0],	
								(CPU_STK_SIZE)ROS_CONTROL_STK_SIZE/10,	
								(CPU_STK_SIZE)ROS_CONTROL_STK_SIZE,		
								(OS_MSG_QTY  )0,					
								(OS_TICK	  )0,					
								(void   	* )0,				
								(OS_OPT     )(OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR), 
								(OS_ERR 	* )&err);	

//	OSTaskCreate((OS_TCB 	* )&Base_ControlTCB,		
//				 (CPU_CHAR	* )"Base Control task", 		
//								(OS_TASK_PTR )Base_Control_task, 			
//								(void		* )0,					
//								(OS_PRIO	  )Base_Control_PRIO,     	
//								(CPU_STK   * )&Base_Control_TASK_STK[0],	
//								(CPU_STK_SIZE)Base_Control_TASK_STK_SIZE/10,	
//								(CPU_STK_SIZE)Base_Control_TASK_STK_SIZE,		
//								(OS_MSG_QTY  )0,					
//								(OS_TICK	  )0,					
//								(void   	* )0,				
//								(OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
//								(OS_ERR 	* )&err);	
								
								
	
	OSTmrStart(&tmr1,&err);	//开启定时器1
	OSTaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//退出临界区
}

void CreateEnvent(void)
{
	OS_ERR err;
	//创建按键事件标志组
	OSFlagCreate((OS_FLAG_GRP*)&KeyEventFlags,		//指向事件标志组
                 (CPU_CHAR*	  )"KeyEvent Flags",	//名字
                 (OS_FLAGS	  )KEY_ALL_FLAGS,	//事件标志组初始值
                 (OS_ERR*  	  )&err);			//错误码
}





