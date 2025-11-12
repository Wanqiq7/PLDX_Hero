#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Remote.h"
#include "UI_Middleware.h"
#include "LinkCheck.h"
#include "Warming.h"
#include "Chassis.h"
#include "CToC.h"
#include "math.h"
#include "Ultra_CAP.h"
/**/

void UI_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM14);//选择时基单元的时钟(此处为内部时钟),TIM_ITRxExternalClockConfig()接ITRx时钟,TIM_TIxExternalClockConfig()接TIx捕获通道时钟,TIM_ETRClockMode1Config()接ETR时钟(选择外部时钟模式1),TIM_ETRClockMode2Config()接ETR时钟(选择外部时钟模式2)
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元（配置参数）
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=2000-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=1680-1;//配置分频值PSC,默认频率25Hz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStructure);//初始化TIM2
	
	TIM_ClearFlag(TIM14,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位
	
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;//配置NVIC（配置参数）
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn;//选择中断通道为TIM2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//TIM2的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;//TIM2的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	TIM_Cmd(TIM14,ENABLE);//启动定时器
}

/*
 *函数简介:TIM2定时器更新中断函数
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	static uint8_t Remote_LastStatus,Remote_ThisStatus=0;
	static uint8_t UI_InitFlag=0;

	if(TIM_GetITStatus(TIM14,TIM_IT_Update)==SET)//检测TIM2更新
	{
		TIM_ClearITPendingBit(TIM14,TIM_IT_Update);//清除标志位
		
		Remote_LastStatus=Remote_ThisStatus;//遥控器上一次连接状态
		Remote_ThisStatus=Remote_Status;//遥控器本次连接状态
		
		if((Remote_LastStatus==0 && Remote_ThisStatus==1) || Remote_RxData.Remote_Key_V==1)UI_InitFlag=1;
		
		if(UI_InitFlag==1)
		{
			static uint8_t InitCount=0;
			
			UI_Middleware_Init(InitCount);//遥控器连接瞬间,初始化UI界面
			InitCount++;
			if(InitCount>=11){InitCount=0;UI_InitFlag=0;}
		}
		else if(LinkCheck_Error==0)
		{
			UI_Middleware_Updata(Chassis_YawTheta/3.14159f*180.0f,Ultra_CAP_Energy,fabs(Gimbal_Pitch),0);
		}
//		else
//		{
//			Warming_UIShow();
//		}
	}
}
