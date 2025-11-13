#include "TIM7_FPS.h"
#include "includes.h"					//ucos 使用	
#include "time1.h"



/*******************
定时器7
宏观统计程序执行频率
*******************/
void TIM7_FPS_Test_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///使能TIM7时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc ;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器7更新中断
	TIM_Cmd(TIM7,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}





short count_bit,count_bit_2;
int test;
long seconde_side;
long count_number;
extern unsigned char Lose_Count;					//目标丢失时间计数器
extern unsigned char First_get;
extern unsigned char heat_start;
extern unsigned char heat_cont;
extern unsigned char cold_start;
extern unsigned char cold_cont;

extern unsigned char Block_up;
extern unsigned char Block_up_cont;

extern unsigned char move_cont;
extern unsigned char move_start;
extern char Load_stop;	//拨弹停止标志

extern char Muzzle1_start;		//停发计数开始
char Muzzle1_switch_count;		//停发计数

extern char Muzzle2_start;		//停发计数开始
char Muzzle2_switch_count;		//停发计数

extern char Muzzle_position;	//1:切换至第二根枪管（0号） 0:切换至第二根枪管（0号）

#include "Can_Task.h"
void TIM7_IRQHandler(void)
{
	OSIntEnter();//OS进入中断
	
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //溢出中断
	{


		test = 0;
		First_get++;
		if(First_get>=5){First_get=5;}
		Lose_Count++;	//计数器累加，在串口接受中断清零
		if(Lose_Count>200)	{Lose_Count=200;}//计数器限幅
			
		//***延时枪口热量控制，已弃用***/
			if(heat_start==1)	//热量开始计数
			{
			heat_cont++;
			}
			if(heat_cont>=100){heat_cont=100;}
			if(cold_start==1)	//冷却开始计数
			{
			cold_cont++;
			}
			if(cold_cont>100){cold_cont=100;}
			
			if(Block_up==1)		//
			{
				Block_up_cont++;
				if(Block_up_cont>10){Block_up_cont=10;}
			}
			//***延时枪口热量控制，已弃用***/
			
			
			if(move_start==1)
			{
			move_cont++;
			
				if(move_cont>200){move_cont=200;}
			}
			
			
			
			//***枪口弹丸排空延时***/
			if(Muzzle1_start==1)		//开始停转
			{
				Muzzle1_switch_count++;		//停发计数
			}
			if(Muzzle1_switch_count>=2)	//留给排空的时间，确认可以切换枪管
			{
				Muzzle1_start=0;
				Muzzle1_switch_count=0;
				Muzzle_position=1;
			}
			
			if(Muzzle2_start==1)		//开始停转
			{
				Muzzle2_switch_count++;		//停发计数
			}
			if(Muzzle2_switch_count>=2)	//留给排空的时间，确认可以切换枪管
			{
				Muzzle2_start=0;
				Muzzle2_switch_count=0;
				Muzzle_position=0;
			}
			//***枪口弹丸排空延时***/
	}

	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //清除中断标志位
	
	OSIntExit();    	//OS退出中断
}
		
		

