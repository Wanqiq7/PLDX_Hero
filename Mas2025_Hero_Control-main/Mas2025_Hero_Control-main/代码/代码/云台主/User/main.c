#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"

int main(void)
{
	Warming_Init();//报警初始化
	LED_BON();//蓝灯点亮表示代码在运行
	AttitudeAlgorithms_Init();//姿态解算初始化
	Delay_ms(500);//延时,等待校准和模块启动
	LinkCheck_Init();//连接检测初始化
	RefereeSystem_Init();//图传链路初始化
	Visual_Init();//视觉初始化
	CloseLoopControl_Init();//闭环控制初始化
	Remote_Init();//遥控器初始化
	while(1)
	{
		IWDG_ReloadCounter();//喂狗
		
		CToC_MasterSendGimbal();
		CToC_MasterSendControl();//CToC发送遥控器控制数据
		Delay_us(200);
		CToC_MasterSendData();//CToC发送遥控器摇杆数据
		CToC_MasterSendShooter();
		
		Delay_us(1000);//CToC周期1.5ms
	}
}
