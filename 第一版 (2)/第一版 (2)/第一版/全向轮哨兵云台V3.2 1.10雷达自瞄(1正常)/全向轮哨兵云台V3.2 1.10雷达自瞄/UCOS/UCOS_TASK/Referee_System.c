#include "Referee_System.h"
#include "AppTask.h"
#include "iwdg.h"

extern int Attack_plan;		//进攻方式：0：遥控器接管	其他：雷达接管
										//														1：原地陀螺		2：自定义进攻方式①		3：自定义进攻方式②




extern int Chassis_power;

extern char Barrel_init_state;
extern long int position2006_target;
extern int position_out;
extern int M2006_counter;

extern u16 Muzzle_heat0;		//枪口热量
extern u16 Muzzle_heat1;		//枪口热量
extern u16 Muzzle_heat_Limit;		//枪口热量上限
extern u16 Game_progress;	//比赛阶段：
										//	0：未开始		
										//	1：准备阶段		
										//	2：15秒自检			
										//	3：五秒倒计时		
										//	4：比赛中		
										//	5：比赛结算
										
										
extern u16 red_outpost_HP;			//红方前哨站hp
extern u16 blue_outpost_HP;			//蓝方前哨站hp
extern u16 admit_num;						//允许发弹量
extern u16 time;								//当前比赛进程剩余时间

extern u16	Remian_hp;					//当前血量
/**********
裁判系统任务
//待填
**********/


void Referee_system_TASK()
{
	OS_ERR err;
	//printf("创建裁判系统响应任务\r\n");
	
	while(1)
	{

		IWDG_Feed();//喂狗
//		printf("枪管0热量：%d; 枪管1热量： %d; 枪口热量上限：%d		比赛进程：%d\r\n",Muzzle_heat0,Muzzle_heat1,Muzzle_heat_Limit,Game_progress);			
//		printf("红方前哨站hp:%d; 红方前哨站hp: %d; 允许发弹量: %d		当前比赛进程剩余时间：%d\r\n",red_outpost_HP,blue_outpost_HP,admit_num,time);			
//		printf("当前血量：%d\r\n",Remian_hp);			
		//printf("%d, %d, %d\r",Barrel_init_state,M2006_counter,position2006_target);
		//printf("%d,	%d,	%d,	%d\r",Chassis_power,Game_progress,Muzzle_heat,Muzzle_heat_Limit);
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err); //延时300ms
	}
}

