#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"

uint8_t Switch_Flag=0;

void Switch_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource11);//配置PA0与中断线的映射关系
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;//配置中断线0
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;//使能中断线
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//配置为外部中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//选择下降沿触发
	EXTI_Init(&EXTI_InitStructure);//初始化EXTI
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组为2
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;//配置NVIC通道0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能NVIC通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;//响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
}

/*
 *函数简介:按键外部中断中断函数
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line11)==SET)//检测按键外部中断触发(即检测EXTI通道0中断触发)
	{
		EXTI_ClearITPendingBit(EXTI_Line11);//清除标志位
		Switch_Flag=1;//函数体
	}
}
