#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 

////////////////////////////////////////////////////////////////////////////////// 	





/*******************************
LED灯光初始化函数
LED_R:PH9
LED_G:PH10
LED_B:PH11
*******************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE); //使能GPIOD的时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;  //上拉输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOH,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11); //GPIOF 高电平
}


