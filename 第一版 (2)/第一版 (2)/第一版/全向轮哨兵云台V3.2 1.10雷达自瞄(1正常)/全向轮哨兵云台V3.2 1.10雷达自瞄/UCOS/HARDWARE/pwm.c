#include "pwm.h"
#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"



/*********************************************
PWM初始化
使用TIM1
PE11
PE9
PWM频率：
PWM装载值：20000
*********************************************/
void PWM_Configuration(void)
{		 					 
	//此部分需手动修改IO口设置,不设置IO会没有输出的
//	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOE9复用为定时器1
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); //GPIOE11复用为定时器1
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); //GPIOE11复用为定时器1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOE9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化GPIOE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           //GPIOE11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化GPIOE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;           //GPIOE11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化GPIOE
	TIM_TimeBaseStructure.TIM_Prescaler=45-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	//初始化TIM4 通道1 PWM模式 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM4
	//初始化TIM4 通道2 PWM模式 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM4
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC3
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM4
	TIM_TimeBaseInit(TIM1 ,&TIM_TimeBaseStructure);//初始化定时器1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);    //使能PWM输出
}  
