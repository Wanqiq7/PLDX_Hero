#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"

int main(void)
{
	Warming_Init();//报警初始化
	Switch_Init();
	LED_BON();//蓝灯点亮表示代码在运行
	Delay_ms(500);//延时,等待校准和模块启动
	LinkCheck_Init();//连接检测初始化
	CloseLoopControl_Init();//闭环控制初始化
	while(1)
	{
		Delay_us(1500);//CToC周期1.5ms
	}
}
