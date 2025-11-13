#include "buzzer.h"
#include <stdio.h>


#include "stm32f4xx.h"
#include "stm32f4xx_it.h"


/****************
嗡鸣器初始化
映射定时器：TIM4
引脚使用:PD14
频率
占空比范围：0~20000
*****************/
void Buzzer_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTE时钟	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4); //GPIOE9复用为定时器1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;           //GPIOE9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化GPIOE
	
	
	  
	TIM_TimeBaseStructure.TIM_Prescaler=4-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	//初始化TIM4 通道1 PWM模式 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	
	
	TIM_TimeBaseInit(TIM4 ,&TIM_TimeBaseStructure);//初始化定时器1
	
	TIM_CtrlPWMOutputs(TIM4, ENABLE);    //使能PWM输出
	
	
	
}



