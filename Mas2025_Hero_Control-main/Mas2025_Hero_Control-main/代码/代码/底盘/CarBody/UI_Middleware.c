#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "UI_Library.h"
#include "Remote.h"
#include "CloseLoopControl.h"
#include "Delay.h"
#include "RefereeSystem.h"
#include "Chassis.h"
/*
 *函数简介:UI初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void UI_Middleware_Init(uint8_t Flag)
{
	switch(Flag)
	{
		case 0:_ui_init_hero_Ungroup_0();break;
		case 2:_ui_init_hero_Ungroup_1();break;
		case 3:_ui_init_hero_Ungroup_2();break;
		case 4:_ui_init_hero_Ungroup_3();break;
		case 5:_ui_init_hero_Ungroup_4();break;
		case 6:_ui_init_hero_Ungroup_5();break;
		case 9:_ui_init_hero_update_0();break;
	}
}

/*
 *函数简介:UI更新
 *参数说明:无
 *返回类型:无
 *备注:无
 */
uint8_t last_flag[4]={0};
uint8_t now_flag[4]={0};
void UI_Middleware_Updata(float angle_data,float V_,float pitch_data,float d_data)
{
	static uint8_t Count=0;
	Count=(Count+1)%5;
	
	if(Count==0)
	{
		last_flag[0]=now_flag[0];
		now_flag[0]=Remote_RxData.Remote_KeyPush_Shift;
		if(last_flag[0]!=now_flag[0]){
		if(Remote_RxData.Remote_KeyPush_Shift==1)_ui_update_hero_shift_point_on();
		else _ui_update_hero_shift_point_off();}
		else Count++;
	}
	if(Count==1)
	{
		last_flag[1]=now_flag[1];
		now_flag[1]=Remote_RxData.Remote_KeyPush_Ctrl;
		if(last_flag[1]!=now_flag[1]){
		if(Remote_RxData.Remote_KeyPush_Ctrl==1)_ui_update_hero_ctrl_point_on();
		else _ui_update_hero_ctrl_point_off();}
		else Count++;
	}
	if(Count==2)
	{
		last_flag[2]=now_flag[2];
		now_flag[2]=Remote_RxData.Remote_KeyPush_Q;
		if(last_flag[2]!=now_flag[2]){
		if(Remote_RxData.Remote_KeyPush_Q==1)_ui_update_hero_Q_point_on();
		else _ui_update_hero_Q_point_off();}
		else Count++;
	}
	if(Count==3)
	{
		last_flag[3]=now_flag[3];
		now_flag[3]=BushuModel_Flag;
		if(last_flag[3]!=now_flag[3]){
		if(BushuModel_Flag==1)_ui_update_hero_X_point_on();
		else _ui_update_hero_X_point_off();}
		else Count++;
	}
	if(Count==4)
	{
		_ui_update_hero_change(angle_data,V_);
	}

}

/*
 *函数简介:UI显示遥控器未连接
 *参数说明:无
 *返回类型:无
 *备注:所有指示灯变绿
 */
void UI_Middleware_RemoteNoCheck(void)
{
//	if(CloseLoopControl_ErrorFlag==1)CloseLoopControl_ErrorFlag=0;
//	_ui_update_default_Ungroup_5_NOCheck();
}
