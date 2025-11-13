#include "Bsp_Init.h"
#include "power_control.h"
#include "bmi08x.h"
#include "bmi08x_defs.h"
#include "usart3_rc.h"
//#include "usart6.h"

/******************************
遥控器防误触检测
若5个遥感未回中，则程序无法进行
******************************/

void Remote_control_erro_test(void)
{
	while(1)
	{
		if(((RC_Ctl.rc.ch0 < 1030) && (RC_Ctl.rc.ch0 > 1018)) && 
			((RC_Ctl.rc.ch1 < 1030) && (RC_Ctl.rc.ch1 > 1018)) &&
				((RC_Ctl.rc.ch2 < 1030) && (RC_Ctl.rc.ch2 > 1018)) &&
					((RC_Ctl.rc.ch3 < 1030) && (RC_Ctl.rc.ch3 > 1018)) &&
						((RC_Ctl.keep.che < 1030) && (RC_Ctl.keep.che > 1018)))			
						{
							break;
						}
						else
						{
//							printf("遥控器遥感未回中\r\n");
						}
						delay_ms(100);
	}
}


/**********************************************************************************、
函数名：Bsp_Init()
作者：
时间：2021年9月21日
功能：
初始化delay时钟
初始化NVIC中断分组
RGB三色提示灯LED灯初始化
串口1初始化，波特率912000
串口3初始化，用于遥控器数据解析
定时器输出PWM初始化，控制弹仓开合
镭射激光初始化
TIM8输入捕获初始化，统计程序执行时间，单位us
TIM7定时器初始化，宏观统计程序执行的FPS
CAN1和CAN2初始化，速率1Mhz
看门狗初始化，500ms触发重启、
SPI1初始化，1/4分频
**********************************************************************************/
extern float speed_x,speed_y,speed_w;		//xyz轴目标速度 单位：m/s
void Bsp_Init(void)
{
	delay_init(168);  	//时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组配置	
	
	LED_Init();         //LED初始化	
	GPIO_ResetBits(GPIOH,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11); //LED RGB熄灭 低电平
	
	
	uart_init(115200);//激光雷达导航通信
	uart6_init(115200);//视觉计算机通信
	
	USART3_Configuration();	//遥控器串口通信，DBUS协议，使用DMA+IDLE。
	
	
	PWM_Configuration();//PWM定时器输出初始化，TIM1,范围0~20000，已经适配ucos3系统。
	TIM_SetCompare1(TIM1,0);//PD12 
	delay_ms(1000);
	//TIM_SetCompare1(TIM1,9100);//PD12 		//舵机角度最大值
	//TIM_SetCompare2(TIM1,1600);//PD13 G				//9100弹仓关闭，1600弹仓打开
	//TIM_SetCompare3(TIM1,1500);//PD13 G
	
	
	
	Buzzer_Init();	//嗡鸣器初始化
	TIM_SetCompare3(TIM4,17000);//PD13 G
	delay_ms(200);
	TIM_SetCompare3(TIM4,10000);//PD13 G
	delay_ms(200);
	TIM_SetCompare3(TIM4,13000);//PD13 G
	delay_ms(200);
	TIM_SetCompare3(TIM4,0);//PD13 G
	
	
	
	light_short_Init();//镭射引脚初始化
	//light_short_ON();
	//light_short_OFF();
	
	
	TIM8_Cap_Init(0XFFFFFFFF,168-1); //输入捕获，以1Mhz的频率计数，精度1us，已经适配ucos3系统。
	Run_Time_Init();	//PI7普通引脚初始化，配合输入捕获记录时间。
	
	TIM7_FPS_Test_Init(5000-1,12800-1);//FPS统计，定时1秒（5000，18000）	统计1秒内的帧率
	
	CAN1_mode_init((uint8_t)0x00, (uint8_t)0x01,(uint8_t)0x05, 5, (uint8_t)0x00);//can1通信初始化，使用单FIFO中断，已经适配ucos3系统。
	CAN2_mode_init((uint8_t)0x00, (uint8_t)0x01,(uint8_t)0x05, 5, (uint8_t)0x00);//can2通信初始化，使用单FIFO中断，已经适配ucos3系统。
	
	//Remote_control_erro_test();//遥控器防误触检测
	IWDG_Init(4,500); //看门狗给初始化，与分频数为64,重载值为500,溢出时间为1s	
	
	SCAP_Control(0,60,1,0);
//	Chassis_PowerControlParam_Init();
	
	SPI1_Init();
	
	
  //speed_x = -0.1;
	//printf("初始化完毕\r\n");
	
	
	
	
			
}




