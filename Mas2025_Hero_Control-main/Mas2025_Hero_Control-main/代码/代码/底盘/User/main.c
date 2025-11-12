#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"

int main()
{
	
 	Warming_Init();//报警初始化
	AttitudeAlgorithms_Init();
	LED_BON();//蓝灯点亮表示代码在运行
	Delay_ms(500);//延时,等待校准和模块启动
	RefereeSystem_Init();//裁判系统数据接收初始化
	LinkCheck_Init();//连接检测初始化
	Ultra_CAP_Init();//超电初始化
	CloseLoopControl_Init();//闭环控制初始化
	UI_Init();
	//UART2_Init();
		
	uint16_t Count=0;
	while(1)
	{
		Count++;
		
		CToC_SlaveSendRefereeSystemData();//向主机发送裁判系统数据
		
		if(Count%5==0)
		{
			//printf("%f,%f\n",Chassis_MotionEstimation.v_x,Chassis_MotionEstimation.v_y);
			//printf("%f,%f\n",Chassis_WheelPower,Mecanum_EstimatedPower);
		}
		
		if(Count==500)
		{
			Count=0;
			LED_BTurn();
		}
		
		Delay_us(1000);
	}
}
