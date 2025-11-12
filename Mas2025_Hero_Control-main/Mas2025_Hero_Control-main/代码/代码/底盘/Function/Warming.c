#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "LED.h"
#include "Buzzer.h"
#include "CAN.h"
#include "Delay.h"
#include "M3508.h"
#include "UI_Library.h"
#include "GM6020.h"

/*====================	 报警列表	====================
	遥控器未连接	················1s里红灯连闪两下
	遥控器数据错误报警	············红灯常亮
	CAN总线设备连接异常············2s内蜂鸣器以高音6响n下,n为CAN.c文件里ID列表的索引
	IST8310连接错误	················绿灯以1s为周期闪烁
	BMI088连接错误	················1s里绿灯连闪两下
	陀螺仪温度过高报警	············绿灯常亮
	电机报警状态	················电机静止
	自定义UI显示警告	············所有状态指示灯变成红或蓝
	FLASH清除警告	················白灯常亮
  ======================================================*/

/*
 *函数简介:报警初始化
 *参数说明:无
 *返回类型:无
 *备注:报警功能用于各种错误的提示
 *备注:报警有两种方式-LED和蜂鸣器
 */
void Warming_Init(void)
{
	LED_Init();
	Buzzer_Init();
}

/*
 *函数简介:报警关闭
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Warming_Stop(void)
{
	TIM_SetCompare1(TIM10,0);
}

/*
 *函数简介:报警LED清理
 *参数说明:无
 *返回类型:无
 *备注:关闭所有灯
 */
void Warming_LEDClean(void)
{
	LED_BOFF();LED_GOFF();LED_ROFF();
}

/*
 *函数简介:报警蜂鸣器清理
 *参数说明:无
 *返回类型:无
 *备注:关闭所有蜂鸣器
 */
void Warming_BuzzerClean(void)
{
	Buzzer_ON(P);
}

/*
 *函数简介:遥控器未连接报警
 *参数说明:无
 *返回类型:无
 *备注:遥控器连接检测TIM7定时更新中断调用,定时25ms
 *报警现象:1s里红灯连闪两下
 */
void Warming_RemoteNoCheck(void)
{
	static uint8_t Counter=0;
	Counter++;
	
	if(Counter==1)LED_RON();
	else if(Counter==5)LED_ROFF();
	else if(Counter==13)LED_RON();
	else if(Counter==17)LED_ROFF();
	else if(Counter==40)Counter=0;
}

/*
 *函数简介:遥控器数据错误报警
 *参数说明:无
 *返回类型:无
 *报警现象:红灯常量
 */
void Warming_RemoteDataERROR(void)
{
	LED_RON();
}

/*
 *函数简介:CAN总线设备连接异常报警
 *参数说明:无
 *返回类型:无
 *备注:闭环控制TIM6定时更新中断调用,定时2ms
 *报警现象:2s内蜂鸣器以高音6响n下,n为连接异常的设备在ID列表的索引(CAN.h文件中CAN_IDSelect变量)
 */
void Warming_LinkError(void)
{
	static uint8_t i=0;
	static uint16_t Counter=0;
	Counter++;
	
	if(i<CAN_IDSelect+1)//高音6响CAN_IDSelect下
	{
		if(Counter==100*i+1)Buzzer_ON(H6);
		else if(Counter==100*i+51)Buzzer_ON(P);
		else if(Counter==100*i+100)i++;
	}
	if(Counter==1000){Counter=0;i=0;}
}

/*
 *函数简介:IST8310连接错误
 *参数说明:无
 *返回类型:无
 *备注:IST8310初始化调用,周期25ms
 *报警现象:绿灯以1s为周期闪烁
 */
void Warming_IST8310LinkError(void)
{
	static uint8_t Counter=0;
	Counter++;
	
	if(Counter==1)LED_GON();
	else if(Counter==5)LED_GOFF();
	else if(Counter==40)Counter=0;
}

/*
 *函数简介:BMI088连接错误
 *参数说明:无
 *返回类型:无
 *备注:BMI088初始化调用,周期25ms
 *报警现象:1s里绿灯连闪两下
 */
void Warming_BMI088LinkError(void)
{
	static uint8_t Counter=0;
	Counter++;
	
	if(Counter==1)LED_GON();
	else if(Counter==5)LED_GOFF();
	else if(Counter==13)LED_GON();
	else if(Counter==17)LED_GOFF();
	else if(Counter==40)Counter=0;
}

/*
 *函数简介:陀螺仪温度过高报警
 *参数说明:无
 *返回类型:无
 *备注:IMU恒温控制调用,温度警戒50℃
 *报警现象:绿灯常亮
 */
void Warming_IMUTemperatureTooHigh(void)
{
	LED_GON();
}

/*
 *函数简介:电机报警状态
 *参数说明:无
 *返回类型:无
 *报警现象:电机静止
 */
void Warming_MotorControl(void)
{
	GM6020_CAN1SetLIDCurrent(0,0,0,0);
	M3508_CANSetLIDCurrent(0,0,0,0);
}

/*
 *函数简介:自定义UI显示警告
 *参数说明:无
 *返回类型:无
 *报警现象:所有状态指示灯变成红或蓝
 */
void Warming_UIShow(void)
{
//	static uint8_t Count=0;
//	Count=(Count+1)%2;
//	_ui_update_default_Ungroup_2_ERROR(Count);
	
}

/*
 *函数简介:FLASH清除警告
 *参数说明:无
 *返回类型:无
 *报警现象:白灯常亮
 */
void Warming_EraseFLASH(void)
{
	LED_RON();
	LED_GON();
	LED_BON();
}
