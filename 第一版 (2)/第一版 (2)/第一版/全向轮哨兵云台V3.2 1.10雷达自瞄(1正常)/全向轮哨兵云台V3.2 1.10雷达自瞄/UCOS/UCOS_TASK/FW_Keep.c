#include "FW_Keep.h"
#include "AppTask.h"

#include "can1.h"
#include "can2.h"
#include "usart3_rc.h"
#include "light_short.h"

/**********
摩擦轮保持任务
//待填
**********/

double test_qq,test_qq_1,test_qq_2;
double target,initer1,initer2,initer3,initer4,initer6,det,out,d1,d2,d3;

double initer5;
double PID_D_1,PID_D_2,PID_D_3,PID_D_4,PID_D_5,PID_D_6,PID_D_7,PID_D_8,PID_D_9,PID_D_10;

double target1,target2,target3,target4,target5,target6;

double erro1,erro2,erro3,erro4,erro5,erro6;
//double initer1,initer2,initer3,initer4;
double out1,out2,out3,out4;
double now_post,last_post,post_erro,post_target,post_D1,post_D2;

int i2;

int Target_Speed_2006;
int Erro_Velocity_2006;

int M2006_counter;

float Position_KP=0.07,Position_KI=0,Position_KD=0;
float Bias,Pwm,Integral_bias,Last_Bias;

extern unsigned char Capture;
extern unsigned char First_get;

unsigned char heat_start;
unsigned char heat_cont;
unsigned char cold_start;
unsigned char cold_cont;
unsigned char cold_cant;



unsigned char Block_up;		//超扭矩后此值置1
unsigned char Block_up_cont;	//堵转计时，Block_up为1后开始计数
unsigned char Block_Confirm;	//堵转确认，堵转超过一段时间置1，拨弹盘开始反转退弹

//int shoot_speed=4500;	//摩擦轮速度

int switched_sp;	//拨弹速度

int switched_speed=5000;	//拨弹速度

extern  short Ros_Target_Speedx;		//接收来自底盘传来的树莓派指定的底盘目标速度（mm/s），将此装载至自动控制模式下的底盘目标速度变量
extern  short Ros_Target_Speedy;
extern  short Ros_Target_Speedw;

extern float xx;
extern float yy;
extern float ww;

extern float speed_x,speed_y,speed_w;		//xyz轴目标速度 单位：m/s

extern int game_state;

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

char Load_stop;	//拨弹停止标志

extern unsigned char Auto_state;					//自动模式状态标志	1为自动 0为手动

int heat_Limit;
/**********************************************************************************、
函数名：FW_V(int FW_1,int FW_2)
形式参数：
1.左摩擦轮转速，范围-8000~8000;
2.右摩擦轮转速，范围-8000~8000
作者：
时间：2022年2月3日
功能：
技术实现：
1.位置形式PID实现角度控制
2.对微分进行重装载
3.引入D值实现鲁棒控制，在左右摩擦轮存在体质差异下，实现响应曲线的近似性
**********************************************************************************/

void FW_V(int FW_1,int FW_2)
{
		target1 = FW_1;
		target2 = FW_2;
	
		PID_D_1 = Velocity_Value_3508_FW[0];
		PID_D_2 = Velocity_Value_3508_FW[1];
		

		
		erro1 = target1 - Velocity_Value_3508_FW[0] ;
		erro2 = target2 -Velocity_Value_3508_FW[1] ;
		
		if(erro1 > 150){initer1 = 0;}
		else if(erro1 < -150){initer1 = 0;}
		if(erro2 > 150){initer2 = 0;}
		else if(erro2 < -150){initer2 = 0;}
		
		
		initer1 = initer1 + erro1;
		initer2 = initer2 + erro2;


		if(initer1>10000){initer1 = 10000;}
		if(initer1<-10000){initer1 = -10000;}	
		
		if(initer2>10000){initer2 = 10000;}
		if(initer2<-10000){initer2 = -10000;}	
		
		
		out1 = erro1*26 + initer1*1.18 - (PID_D_1-PID_D_3)*4 ;
		out2 = erro2*26 + initer2*1.18 - (PID_D_2-PID_D_4)*4;
		
		
		if(out1> 17999){out1=17999;}
		if(out1< -17999){out1=-17999;}
		if(out2> 17999){out2=17999;}
		if(out2< -17999){out2=-17999;}


		PID_D_3 = Velocity_Value_3508_FW[0];
		PID_D_4 = Velocity_Value_3508_FW[1];
		
	
}


char Muzzle1_start;		//停发计数1,
char Muzzle2_start;		//停发计数2

char Muzzle_position;	//1:切换至第二根枪管（0号） 0:切换至第二根枪管（0号）

//2006匀速拨弹
void Load_Rounds(int speed)
{
	
		target5 = speed;
//	if(	(Muzzle_heat1>((int)(0.8*Muzzle_heat_Limit)))	||	(Muzzle_heat0>((int)(0.8*Muzzle_heat_Limit))))	//如果任意枪管热量达到上限
//		{
//			target5=0;		//禁止拨弹
//		}
		heat_Limit=((int)(0.8*Muzzle_heat_Limit));
		//heat_Limit=30;
	
		if(Muzzle_heat1>heat_Limit)		//枪管热量达到上限
		{
			Muzzle1_start=1;
		}
		
		if(Muzzle1_start==1)
		{
			target5=0;		//禁止拨弹
		}
		
		if(Muzzle_heat0>heat_Limit)		//枪管热量达到上限
		{
			Muzzle2_start=1;
		}
		
		if(Muzzle2_start==1)
		{
			target5=0;		//禁止拨弹
		}
	
		PID_D_7 = Velocity_Value_3508_FW[2];	
		erro5 = target5 - Velocity_Value_3508_FW[2] ;
		
		if(erro5 > 150)
		{
			initer5 = 0;
		}
		else if(erro5 < -150)
		{
			initer5 = 0;
		}
		
		
		initer5 = initer5 + erro5;
		if(initer5>10000){initer5 = 10000;}
		if(initer5<-10000){initer5 = -10000;}	
		out3 = erro5*26 + initer5*1.18 - (PID_D_7-PID_D_8)*4 ;
		if(out3> 17999){out3=17999;}
		if(out3< -17999){out3=-17999;}
		PID_D_8 = Velocity_Value_3508_FW[2];
		
}

//枪管切换2006速度环
void Barrel_switch(int speed)
{
		target6 = speed;
		PID_D_9 = Velocity_Value_3508_FW[3];	
		erro6 = target6 - Velocity_Value_3508_FW[3] ;
		if(erro6 > 150){initer6 = 0;}
		else if(erro6 < -150){initer6 = 0;}
		initer6 = initer6 + erro6;
		if(initer6>10000){initer6 = 10000;}
		if(initer6<-10000){initer6 = -10000;}	
		out4 = erro6*19 + initer6*1.18 - (PID_D_9-PID_D_10)*4 ;
		if(out4> 17999){out4=17999;}
		if(out4< -17999){out4=-17999;}
		PID_D_10 = Velocity_Value_3508_FW[3];
		
}


char qiehuan_ok;

/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID (int Encoder,int Target)
{ 	
	 
	 Bias=Encoder-Target;                                  //计算偏差
//		if(Bias>28000){Bias=28000;}
//		if(Bias<-28000){Bias=-28000;}		//偏差限幅值
	
	if(Bias>-200&&Bias<200)
	{
		qiehuan_ok=1;
	}
	else 
	{
		qiehuan_ok=0;
	}
	 Integral_bias+=Bias;	                                 //求出偏差的积分
		if(Integral_bias>5000){Integral_bias=5000;}
		if(Integral_bias<-5000){Integral_bias=-5000;}
		
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	
	if(Pwm>8000)
	{Pwm=8000;}
	if(Pwm<-8000)
	{Pwm=-8000;}		//输出限幅值
	
//	if((Bias<100)&&(Bias>-100))
//	{
//		Pwm=0;
//	}
	 return Pwm;                                           //增量输出
	
}

/**********************************************************************************、
函数名：Finght_Switch(int Switch,int short_V,int short_mode)
形式参数：
1.1/0，是否允许发弹;
2.发弹频率，0~30;
3.1/0，是否单发模式
作者：
时间：2022年2月3日
功能：
技术实现：
1.单发模型，通过计量拨弹旋转的角度实现单发拨弹
2.连发模型，套路单发模型，快速单发实现连续发弹
**********************************************************************************/

int time_test;
int mouse_press_l_test_bit;

void Finght_Switch(int Switch,int short_V,int short_mode)
{
	
	
	if(Switch == 1)
		{
			if(short_mode == 0)
			{
				if(short_mode == 0)
				{
					mouse_press_l_test_bit = 0;
					
					time_test++;
					if(short_V == 0)
					{
						post_target = 0;
					}
					
					else if(time_test < 0 )
					{
						post_target = 0;
					}
					
					else if(time_test > (510/short_V))
					{
						time_test = 0;
						post_target = post_target-32766;
					}
				}
				
				else if(short_mode == 1)
				{
					if(mouse_press_l_test_bit == 1)
					{
						post_target = post_target-32768;
					}
				}		
			}
		}	
		else
		{
			post_target = 0;
		}
		
		
		
		if(post_target > 163840){post_target = 163840;}
		else if(post_target < -163840){post_target = -163840;}  //堵转放疯
		
		
		
		now_post = Position_Value_3508_FW[2];
		
		if(now_post >= last_post)
		{
			if((now_post-last_post) >= 4095)
			{
				post_erro = -last_post  + now_post - 8191;
			}	
			else if((now_post-last_post) < 4095)
			{
				post_erro = now_post - last_post ;
			}
		}	
		else if(now_post < last_post)
		{
			if((last_post-now_post) >= 4095)
			{
				post_erro =  now_post + 8191 - last_post;
			}	
						if((last_post-now_post) < 4095)
			{
				post_erro =  now_post - last_post;
			}	
		}
	
		last_post = Position_Value_3508_FW[2];
		
		post_target = post_target - post_erro;
		
		post_D1 = post_target;	
		target3 = post_target*0.47 - (post_D1 - post_D2)*0.22 ;		
		post_D2 = post_target;
		
		if(target3 >= 12000){target3= 12000;}
		else if	(target3 < -12000){target3 = -12000;}
		
		erro3 = target3 - Velocity_Value_3508_FW[2];
		
		initer3 = initer3 + erro3*0.18;
		

						

		if(initer3>5000){initer3 = 5000;}
		if(initer3<-5000){initer3= -5000;}	
		
		if(initer4>5000){initer4 = 5000;}
		if(initer4<-5000){initer4 = -5000;}	
		out3 = erro3*8 + initer3 - (PID_D_5-PID_D_6)*8 ;
		if(out3>10000){out3 = 10000;}
		if(out3<-10000){out3= -10000;}	
		
		
}


void Shoot_Control(u8 YN)
{
	if(YN==1)
	{
		Target_Speed_2006=-2000;
	}
	
	else if(YN==0)
	{
	Target_Speed_2006=0;
	}
	
	out3=out3+0.04*(Target_Speed_2006-Velocity_Value_3508_FW[2])+1.5*(Target_Speed_2006-Velocity_Value_3508_FW[2]-Erro_Velocity_2006);
	Erro_Velocity_2006=Target_Speed_2006-Velocity_Value_3508_FW[2];

}

/********/
//Func Name		:	Fire_Start
//Performance	:	带卡弹退弹的自动开火
//Return			:	无
//Para				:	
//Author			:	
//Date				:	
void Fire_Start()
{
			
       FW_V(7000,-7000);		//开摩擦轮		 
				
//				if(qiehuan_ok!=1)
//				{
//					switched_sp=0;
//				}
//				
//				else
					switched_sp=switched_speed;
			 Load_Rounds(switched_sp);	//开始拨弹
				
			if(Torque_Value_3508_FW[2]<-7200)	//如果扭矩超过范围，则判断为卡弹
				 {
					 Block_up=1;
				 }			 
				 if(Block_up==1)//反转退弹
				 {
				  Load_Rounds(2567);	//拨弹盘反转
				 }			 
					 
				 if(Torque_Value_3508_FW[2]>3500)//卡弹解除
					 {		 
					  Block_up=0;
					 }
				 if(Block_up==0)	
				   {
					 if(cold_cant==0)
						{
							Load_Rounds(switched_sp);	//开始拨弹
						}
						else
						{
							Load_Rounds(0);	//停止拨弹冷却
						}
//						
//				if(Muzzle_heat1>=((int)(0.8*Muzzle_heat_Limit)))		//单枪管自动切换
//					{
//						cold_cant=1;					
//					}
//					if(Muzzle_heat1==0)
//					{
//						cold_cant=0;
//					}
						
				   }		
			
			
		light_short_ON();
}

int FW_Mode;
char Barrel_init_state;
long int position2006_target;
int position_out;
long int first_position2006_target;
/**********************************************************************************、
任务名：FW_Keep_Task(void)
形式参数：无
作者：
时间：2022年2月3日
功能：
模式一：摩擦轮起转，并发弹
模式二：摩擦轮主动停转后无力，停止发弹
模式三：摩擦轮起转，不发弹
丢控保护：摩擦轮主动停转后无力，主动停止发弹后拨弹盘无力
**********************************************************************************/
void FW_Keep_Task(void)
{
	OS_ERR err;
	while(1)
	{
		if(RC_Ctl.rc.s2 == 2)	//右杆下拨
		{
				USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断 
		}
		
		else if(RC_Ctl.rc.s2 == 3)	//右杆处于中间位置
		{
			if(abs(Velocity_Value_3508_FW[0]) > 15 || abs(Velocity_Value_3508_FW[1]) > 15)
			{
				FW_V(0,0);
			}		
			else
			{
				out1 = 0;
				out2 = 0;
			}

			Load_Rounds(0);//停止拨弹
		}
		
		
		else if(RC_Ctl.keep.che>=1500)	//右杆上拨
		{
			Fire_Start(); //开火	
		}		
		
		else
		{
			 if(abs(Velocity_Value_3508_FW[0]) > 15 || abs(Velocity_Value_3508_FW[1]) > 15)
			{
				FW_V(0,0);
			}		
			else
			{
				out1 = 0;
				out2 = 0;
			}
		if(abs(Velocity_Value_3508_FW[2]) > 15 ){Finght_Switch(0,8,0);}		
		else{out3 = 0;}
			
		FW_V(0,0);
		}
		
		
		//自动扳机：
		if((Capture==1&&First_get>=1)||RC_Ctl.keep.che>=1500)	//找到目标，自动发弹
		{		
			Fire_Start(); //开火	
		}
		
		else 
		{
			 if(abs(Velocity_Value_3508_FW[0]) > 15 || abs(Velocity_Value_3508_FW[1]) > 15)
			{
				FW_V(0,0);
			}		
			else
			{
				out1 = 0;
				out2 = 0;
			}
		if(abs(Velocity_Value_3508_FW[2]) > 15 ){Finght_Switch(0,8,0);}		
		else{out3 = 0;}
			
		FW_V(0,0);
		}
		
		
	if(Barrel_init_state==0)		//确认枪管初始位置
	{
		Barrel_switch(-1000);
		if(Torque_Value_3508_FW[3]>7500||Torque_Value_3508_FW[3]<-7500)
		{
			Barrel_init_state=1;
		}
	}
	else if(Barrel_init_state==1)	//到达初始位置
	{
		Barrel_switch(0);
		out4=0;
		Barrel_init_state=2;
	}
	
	
	if(Barrel_init_state==2)	//枪管切换选择
	{
		
		if(	(Muzzle_position==1)|| ((RC_Ctl.rc.s2 == 2)&&(Auto_state==0)) )		//如果枪管1热量满	,且拨弹已经停止
		{
				position2006_target=first_position2006_target-(int)(8912*15.807);		//15.9  15.807切换第二根枪管位置
		}
		else if(	(Muzzle_position==0)	||	((RC_Ctl.rc.s2 == 3)&&(Auto_state==0)) )	//如果枪管0热量满,且拨弹已经停止
		{
				position2006_target=first_position2006_target;											//切换到第一根枪管位置
		}
			position_out = -(int)Position_PID(M2006_counter,position2006_target);
			Barrel_switch(position_out);
	}
	
	
  //printf("%d, %d, %d, %d\r",Torque_Value_3508_FW[3],Barrel_init_state,M2006_counter,position2006_target);
	CAN2_Send_queue_add(0x200,0x08,(int)out1,(int)out2,0,0);//控制摩擦轮电机、枪管切换电机
	CAN1_Send_queue_add(0x1ff,0x08,(int)out3,0,0,0);//控制拨弹盘电机	
		
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err); //延时2ms 频率=1/0.002s=500hz
	} 

}

