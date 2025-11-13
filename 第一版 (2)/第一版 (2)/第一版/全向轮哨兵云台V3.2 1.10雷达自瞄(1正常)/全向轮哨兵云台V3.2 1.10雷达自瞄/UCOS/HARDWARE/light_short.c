#include "light_short.h"


/*************
镭射激光器
使用引脚：PC8
*************/

void light_short_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOD的时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;  //上拉输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_ResetBits(GPIOC,GPIO_Pin_8); //GPIOF 高电平

}

void light_short_ON(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_8); //GPIOF 高电平
}
void light_short_OFF(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_8); //GPIOF 高电平
}

