#include "GimbalTask.h"
#include "TIM7_FPS.H"
#include "can1.h"
#include "can2.h"
#include "ChassisTask.h"
#include "AppTask.h"
#include "usart3_rc.h"
#include "usart.h"
#include "Base_Control.h"
/**********************
‘∆Ã®»ŒŒÒV1.1

1. π”√BM088I◊˜Œ™Õ”¬›“«°£
2.¥¥Ω®4∏ˆƒ£ Ω£∫
	£®1£©£∫‘∆Ã®Œﬁ¡¶°£
	£®2£©£∫‘∆Ã®∏˙ÀÊµ◊≈Ã°£
	£®3£©£∫µ◊≈Ã∏˙ÀÊ‘∆Ã®°£
	£®4£©£∫‘∆Ã®◊‘∂Ø√È◊º°£
**********************/
double angle_v_as[3];
double PID_Yaw_I2;
double PID_Yaw_I,PID_YawW_I,PID_Yaw_D_1,PID_Yaw_D_2;
double PID_Pich_I,PID_Pich_D_1,PID_Pich_D_2;
double PID_Yaw_Out,PID_Pich_Out;
double M6020_X_V[2],M6020_X_V_err[2];
double M6020_J_V[2],M6020_J_V_err[2];
double M6020_OUT[2];
double YAW_posisition_err,YAW_posisition_err_D[2];
int kk ;
double Yaw_Middl,Yaw_erro,Pich_erro;
double Poch_hignt_limit,Poch_down_limit;
double Yaw_target,Pich_target;
double target_yaw,target_pich;
double kkk;

float kp_X_com=0.95,ki_X_com,kd_X_com=0.6,Integral_bias_x;	//x÷·∏˙ÀÊ
float err_X,last_err_X;	//µ±«∞ŒÛ≤Ó
int Add_X;	

float kp_Y_com=0.72,ki_Y_com,kd_Y_com=0.9,Integral_bias_y;	//x÷·∏˙ÀÊ
float err_Y,last_err_Y;	//µ±«∞ŒÛ≤Ó
int Add_Y;	

int Pit_Add=400;

extern int Data,cx,cy,cw,ch;	//¥Æø⁄Ω” ’µƒ◊∞º◊∞Â÷––ƒ◊¯±Í÷µ


int mid_X=560;								//ª≠√Ê÷––ƒµ„		Õ˘”“‘ˆ¥Û  77   80		//590
int mid_Y=440;							//Õ˘…œ‘ˆ¥Û

int out_pitch;

unsigned char Auto_state=0;					//◊‘∂Øƒ£ Ω◊¥Ã¨±Í÷æ 1:◊‘∂Ø  0£∫ ÷∂Ø   
unsigned char Capture;				// «∑Ò ∂±µΩƒø±Í
unsigned char Lose_Count;					//ƒø±Í∂™ ß ±º‰º∆ ˝∆˜£¨¥À÷µ‘⁄∂® ±∆˜÷–∂œ£®1s£©ƒ⁄◊‘‘ˆ£¨‘⁄¥Æø⁄6÷–∂œƒ⁄÷√¡„£¨µ±¥Æø⁄6Œ¥Ω” ’µΩ ˝æ›≤ªΩ¯÷–∂œ£¨‘Ú¥À÷µ“ª÷±◊‘‘ˆ£¨”√”⁄≈–∂œƒø±Í «∑Ò∂™ ß
unsigned char com=9;
unsigned char First_get;
extern short	Position_Value_6020[2];


int Spin_Speed=300;
float Yaw_in;

extern short gyro_0,gyro_1,gyro_2;
extern short accel_0,accel_1,accel_2;

unsigned char target_online;

unsigned char aim_auto=0;

extern _Base_Control_Param Base_Control_Param;

										
char aim_test;		//◊‘√È≤‚ ‘


extern u16 Muzzle_heat0;		//«πø⁄»»¡ø
extern u16 Muzzle_heat1;		//«πø⁄»»¡ø
extern u16 Muzzle_heat_Limit;		//«πø⁄»»¡ø…œœﬁ
extern u16 Game_progress;	//±»»¸Ω◊∂Œ£∫
										//	0£∫Œ¥ø™ º		
										//	1£∫◊º±∏Ω◊∂Œ		
										//	2£∫15√Î◊‘ºÏ			
										//	3£∫ŒÂ√Îµπº∆ ±		
										//	4£∫±»»¸÷–		
										//	5£∫±»»¸Ω·À„
										
										
extern u16 red_outpost_HP;			//∫Ï∑Ω«∞…⁄’æhp
extern u16 blue_outpost_HP;			//¿∂∑Ω«∞…⁄’æhp
extern u16 admit_num;						//‘ –Ì∑¢µØ¡ø
extern u16 time;								//µ±«∞±»»¸Ω¯≥Ã £”‡ ±º‰

extern u16	Remian_hp;					//µ±«∞—™¡ø

extern char arrive;



int p_out;

int Open_MV_X()		//openMVµƒx÷·◊∑◊Ÿ
{
	err_X=cx-mid_X;	//º∆À„ŒÛ≤Ó
	Integral_bias_x+=err_X;			//ŒÛ≤Ó¿€º”ª˝∑÷
	p_out=(int)(kp_X_com*err_X);
	if(p_out>=400)
	{
		p_out=400;
	}
	
	if(p_out<=-400)
	{
		p_out=-400;
	}
	Add_X=p_out+ki_X_com*Integral_bias_x+kd_X_com*(err_X-last_err_X);
	last_err_X=err_X;
	return Add_X;
}

int Open_MV_Y()		//openMVµƒY÷·◊∑◊Ÿ
{
	err_Y=cy-out_pitch;	//º∆À„ŒÛ≤Ó
	Integral_bias_y+=err_Y;			//ŒÛ≤Ó¿€º”ª˝∑÷
	
	Add_Y=kp_Y_com*err_Y+ki_Y_com*Integral_bias_y+kd_Y_com*(err_Y-last_err_Y);
	last_err_Y=err_Y;
	return Add_Y;
}

int pitch_min;
int pitch_max;
int pitch_add_max;

unsigned char init_sta;	//pitch∆ º

/**********************************************************************************°¢
∫Ø ˝√˚£∫Motion_mode(void)
–Œ Ω≤Œ ˝£∫Œﬁ
◊˜’ﬂ£∫
 ±º‰£∫2022ƒÍ2‘¬6»’
π¶ƒ‹£∫øÿ÷∆‘∆Ã® ¿ΩÁΩ«ÀŸ∂»
**********************************************************************************/
void Motion_mode(void)
{
	
	
	if(Auto_state==1&&Capture==1)	//»Áπ˚¥¶”⁄◊‘∂Øƒ£ Ω«“’“µΩƒø±Í, Ω¯–– ”æı◊‘√È
	{
		if(cx!=0||cy!=0)	//ƒø±Í◊∞º◊∞Â¥Ê‘⁄
		{
			Yaw_target = Open_MV_X()*-10;  
		    Pich_target = Open_MV_Y()*-10;
		}
		
		
		if(Lose_Count>0&&Lose_Count<=1)
			{
				Yaw_target=0;
				Pich_target=0;
			}
		Yaw_erro = Yaw_target - (double)gyro_souce[2]+7.6207;//   Õ”¬›“«µ˜ ‘ 3.7337 0.08-0.1 +”“0.6207-0.6007
		Pich_erro = Pich_target - (double)gyro_souce[0]-3.321;//Õ”¬›“«µ˜ ‘ 3.7307  0.2-0.3  +…œ0.2257-0.2307
		PID_Yaw_D_1 = gyro_souce[2];
		PID_Pich_D_1 = gyro_souce[0];
		PID_Yaw_I = PID_Yaw_I + Yaw_erro*1.88;
		PID_Pich_I = PID_Pich_I + Pich_erro*1.18;
		if(PID_Yaw_I>19999){PID_Yaw_I = 19999;}	
		if(PID_Yaw_I<-19999){PID_Yaw_I = -19999;}
		if(Yaw_erro>9999){PID_Yaw_I = 0;}	
		if(Yaw_erro<-9999){PID_Yaw_I = 0;}	
		if(PID_Pich_I>19999){PID_Pich_I = 19999;}	
		if(PID_Pich_I<-19999){PID_Pich_I = -19999;}
		if(Pich_erro>9999){PID_Pich_I = 0;}	
		if(Pich_erro<-9999){PID_Pich_I = 0;}
		PID_Yaw_Out = Yaw_erro*17+ PID_Yaw_I*0.12 - (PID_Yaw_D_1-PID_Yaw_D_2)*2.5;
		//PID_Pich_Out = Pich_erro * 16 + PID_Pich_I*0.32 - (PID_Pich_D_1-PID_Pich_D_2)*2.0;
		PID_Pich_Out = Pich_erro * 16 + PID_Pich_I*0.32 - (PID_Pich_D_1-PID_Pich_D_2)*2.0;
		PID_Pich_D_2 = gyro_souce[0];
		PID_Yaw_D_2 = gyro_souce[2];
		if(PID_Yaw_Out>29999){PID_Yaw_Out = 29999;}	
		if(PID_Yaw_Out<-29999){PID_Yaw_Out = -29999;}		
		if(PID_Pich_Out>29999){PID_Pich_Out = 29999;}	
		if(PID_Pich_Out<-29999){PID_Pich_Out = -29999;}	
		CAN2_Send_queue_add(0x1ff,0x08,PID_Pich_Out,0,0,0);//
		CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);
	}
	
	if(Auto_state==1&&Capture==0)	//»Áπ˚Œ™◊‘∂Øƒ£ Ω≤¢∂™ ßƒø±Í£¨‘Ú‘∆Ã®ø™ º◊‘÷˜‘À∂Ø—∞’“ƒø±Í
	{
		
		pitch_min=3700;	//pitch÷·∞⁄∂Ø∑∂Œß
		pitch_max=4500;
		pitch_add_max=300;
		
//		if(Position_Value_6020[0]>=6450){Pit_Add=-700;} //pitch÷·∞⁄∂Ø∑∂Œß
//		if(Position_Value_6020[0]<=6100){Pit_Add= 700;}
		
		if(Position_Value_6020[0]>=pitch_max)			{init_sta=2;} 
		else if(Position_Value_6020[0]<=pitch_min){init_sta=1;}
		
		if(init_sta==1)	{Pit_Add=pitch_add_max;}
		if(init_sta==2)	{Pit_Add=-pitch_add_max;}
		
		if(aim_test==0)		//¿◊¥Ôøÿ÷∆ ±£¨¿◊¥ÔÕ®π˝øÿ÷∆‘∆Ã®∑ΩœÚΩ¯––µº∫Ω
		{
			Yaw_target = (-8)+(880*speed_w);  	//yaw÷·–˝◊™
			Pich_target =0; //Pit_Add;	
		}
		
		if(aim_test==1)		//◊‘√È≤‚ ‘ ±
		{
			Yaw_target = 0;  	//yaw÷·–˝◊™
			Pich_target =Pit_Add; //Pit_Add;	
		}
		
		Yaw_erro = Yaw_target - (double)gyro_souce[2]+7.6207;//   Õ”¬›“«µ˜ ‘ 3.7337 0.08-0.1 +”“0.6207-0.6007
		Pich_erro = Pich_target - (double)gyro_souce[0]-3.321;//Õ”¬›“«µ˜ ‘ 3.7307  0.2-0.3  +…œ0.2257-0.2307
		PID_Yaw_D_1 = gyro_souce[2];
		PID_Pich_D_1 = gyro_souce[0];
		PID_Yaw_I = PID_Yaw_I + Yaw_erro*1.88;
		PID_Pich_I = PID_Pich_I + Pich_erro*1.18;
		if(PID_Yaw_I>19999){PID_Yaw_I = 19999;}	
		if(PID_Yaw_I<-19999){PID_Yaw_I = -19999;}
		if(Yaw_erro>9999){PID_Yaw_I = 0;}	
		if(Yaw_erro<-9999){PID_Yaw_I = 0;}	
		if(PID_Pich_I>19999){PID_Pich_I = 19999;}	
		if(PID_Pich_I<-19999){PID_Pich_I = -19999;}
		if(Pich_erro>9999){PID_Pich_I = 0;}	
		if(Pich_erro<-9999){PID_Pich_I = 0;}
		PID_Yaw_Out = Yaw_erro*16 + PID_Yaw_I - (PID_Yaw_D_1-PID_Yaw_D_2)*1.5;
		PID_Pich_Out = Pich_erro * 16 + PID_Pich_I - (PID_Pich_D_1-PID_Pich_D_2)*1;
		PID_Pich_D_2 = gyro_souce[0];
		PID_Yaw_D_2 = gyro_souce[2];
		if(PID_Yaw_Out>29999){PID_Yaw_Out = 29999;}	
		if(PID_Yaw_Out<-29999){PID_Yaw_Out = -29999;}		
		if(PID_Pich_Out>29999){PID_Pich_Out = 29999;}	
		if(PID_Pich_Out<-29999){PID_Pich_Out = -29999;}	
		
//		if(Game_progress!=4||aim_test!=1)		//±»»¸Œ¥ø™ º£¨‘∆Ã®Œﬁ¡¶
//		{
//			PID_Pich_Out=0;
//			PID_Yaw_Out=0;
//		}
	    CAN2_Send_queue_add(0x1ff,0x08,PID_Pich_Out,0,0,0);//øÿ÷∆Àƒ∏ˆµ◊≈ÃµÁª˙
		CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);
	}
	
	if(Auto_state==0)//»Áπ ˚Œ™ ÷∂Ø£¨‘Ú‘∆Ã®”…“£øÿ∆˜øÿ÷∆
	{
		Yaw_target = -(RC_Ctl.rc.ch0-1024)*8;
		Pich_target = (RC_Ctl.rc.ch1-1024)*8;
		Yaw_erro = Yaw_target - (double)(gyro_souce[2]+com)+7.6207;//   Õ”¬›“«µ˜ ‘ 3.7337 0.08-0.1 +”“0.6207-0.6007
		Pich_erro = Pich_target - (double)gyro_souce[0]-3.321;//Õ”¬›“«µ˜ ‘ 3.7307  0.2-0.3  +…œ0.2257-0.2307
		PID_Yaw_D_1 = (gyro_souce[2]+com);
		PID_Pich_D_1 = gyro_souce[0];
		PID_Yaw_I = PID_Yaw_I + Yaw_erro*1.18;
		PID_Pich_I = PID_Pich_I + Pich_erro*1.18;
		if(PID_Yaw_I>19999){PID_Yaw_I = 19999;}	
		if(PID_Yaw_I<-19999){PID_Yaw_I = -19999;}
		if(Yaw_erro>9999){PID_Yaw_I = 0;}	
		if(Yaw_erro<-9999){PID_Yaw_I = 0;}	
		if(PID_Pich_I>19999){PID_Pich_I = 19999;}	
		if(PID_Pich_I<-19999){PID_Pich_I = -19999;}
		if(Pich_erro>9999){PID_Pich_I = 0;}	
		if(Pich_erro<-9999){PID_Pich_I = 0;}
		PID_Yaw_Out = Yaw_erro*18 + PID_Yaw_I - (PID_Yaw_D_1-PID_Yaw_D_2)*1.2;
		PID_Pich_Out = Pich_erro * 18 + PID_Pich_I - (PID_Pich_D_1-PID_Pich_D_2)*0.2;
		PID_Pich_D_2 = gyro_souce[0];
		PID_Yaw_D_2 = (gyro_souce[2]+com);
		if(PID_Yaw_Out>29999){PID_Yaw_Out = 29999;}	
		if(PID_Yaw_Out<-29999){PID_Yaw_Out = -29999;}		
		if(PID_Pich_Out>29999){PID_Pich_Out = 29999;}	
		if(PID_Pich_Out<-29999){PID_Pich_Out = -29999;}
		CAN2_Send_queue_add(0x1ff,0x08,PID_Pich_Out,0,0,0);//øÿ÷∆Àƒ∏ˆµ◊≈ÃµÁª˙
		CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);
	}
} 


void Finding_Mode()
{
Yaw_target = 300;  
		Pich_target = -Open_MV_Y()*5;

}


/********/
//Func Name		:	Target_loss_judgment
//Performance	:	ƒø±Í∂™ ß≈–∂œ
//Return			:	
//Para				:	
//Author			:	
//Date				:	
void Target_loss_judgment()
{
	if(Auto_state==1)		//»Áπ˚Œ™◊‘∂Øƒ£ Ω
		{
			if(Lose_Count<3)		//»Áπ˚º∆ ˝∆˜–°”⁄3«“£¨‘Ú≈–∂œŒ™’“µΩƒø±Í£¨ø™ º◊‘√È
				{
					Capture=1;	//ƒø±Í◊∞º◊∞Â‘⁄œﬂ
				}
				
			if(Lose_Count>=3)		//∂™ ßƒø±Í3s∫Û£¨ø™ º◊‘∂ØÀ—À˜
				{
					Capture=0;	//ƒø±Í◊∞º◊∞Â∂™ ß			
				}
	 	}
}


/********/
//Func Name		:	AutoAim_PitchAdjust
//Performance	:	∏˘æ›openmv¥´ªÿœÒÀÿ¥Û–°Ω¯––æ‡¿ÎÕ∆∂œ≤¢µ˜’˚◊‘√È ±‘∆Ã®Pitchƒø±ÍΩ«∂»“‘µ˜’˚µØµ¿£ª»±µ„£∫ ˝æ›≤ªæ´»∑£¨¡È√Ù∂»µÕ £®∏ƒΩ¯ø’º‰£©
//Return			:	
//Para				:	
//Author			:	
//Date				:	
void AutoAim_PitchAdjust()
{		
		unsigned char cw_con;	
		int cw_av,cw_sum;	//”√”⁄º∆À„œÒÀÿ¥Û–°
		cw_con++;
		cw_sum+=cw;
	
	  
	
		if(cw_con>7)	//√øΩ” ’∆ﬂ¥Œopenmv ˝æ›º∆À„ƒø±Í∆‰œÒÀÿ¥Û–°∆Ωæ˘÷µ
		{
			cw_av=cw_sum/7;
			cw_sum=0;
			cw_con=0;
			
		}
		//printf("%d\r",cw);
		
		if(cw<=6)
			out_pitch=mid_Y+5;
		else 
			out_pitch=mid_Y;

}


/**********************************************************************************°¢
»ŒŒÒ√˚£∫Gimbal_task(void *p_arg)
◊˜’ﬂ£∫
 ±º‰£∫2022ƒÍ2‘¬11»’
π¶ƒ‹£∫øÿ÷∆‘∆Ã®√È◊º
ƒ£ Ω“ª£∫‘À∂Øƒ£ Ω
ƒ£ Ω∂˛£∫‘À∂Øƒ£ Ω
ƒ£ Ω»˝£∫‘À∂Øƒ£ Ω
∂™øÿ±£ª§£∫‘∆Ã®µÁª˙Œﬁ¡¶
»ŒŒÒ∆µ¬ £∫500FPS
**********************************************************************************/
void Gimbal_task(void *p_arg)
{
	double last_world_angle;
	OS_ERR err;
	p_arg = p_arg;
	printf("‘∆Ã®»ŒŒÒ ø™ º\r\n");
	
	while(1)
	{
		//printf("%d\r",Position_Value_6020[0]);
		
		AutoAim_PitchAdjust();	//◊‘√Èœ¬µƒpitchƒø±Í÷µΩ√’˝
		Target_loss_judgment();	//ƒø±Í∂™ ß≈–∂œ
		
		
		
		
		
	 if(RC_Ctl.rc.s2 == 3) //ƒ£ Ω»˝£∫‘À∂Øƒ£ Ω
		{
			Motion_mode();
			last_world_angle = word_angle[2];		
		}
		else if(RC_Ctl.rc.s2 == 1)	//ƒ£ Ω∂˛£∫‘À∂Øƒ£ Ω
		{
			Motion_mode();
			last_world_angle = word_angle[2];
		}
		else if(RC_Ctl.rc.s2 == 2) //ƒ£ Ω“ª£∫‘À∂Øƒ£ Ω
		{
			Motion_mode();
			last_world_angle = word_angle[2];
		}
		else 	//ƒ£ ΩÀƒ£∫∂™øÿ±£ª§£¨‘∆Ã®Œﬁ¡¶
		{
			CAN1_Send_queue_add(0x1ff,0x08,0,0,0,0);//øÿ÷∆Àƒ∏ˆµ◊≈ÃµÁª˙
            
		}
		
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err); //—” ±100ms
	}
}

/**********************************************************************************°¢
∫Ø ˝√˚£∫TV_test(void)
–Œ Ω≤Œ ˝£∫Œﬁ
◊˜’ﬂ£∫
 ±º‰£∫2022ƒÍ2‘¬16»’
π¶ƒ‹£∫◊‘∂Ø√È◊ºƒ£ Ω
*********************************************************************************/
//void TV_test(void)
//{
//		Yaw_erro = -(RC_Ctl.rc.ch0-1024)*8 - gyro_souce[2] + (TV_target[0]-180)*28;//Yaw_erro = -(RC_Ctl.rc.ch0-1024)*8 - gyro_souce[2];
//	  Pich_erro = -(RC_Ctl.rc.ch1-1024)*8 - gyro_souce[0];
//		PID_Yaw_I = PID_Yaw_I + Yaw_erro;
//		PID_Pich_I = PID_Pich_I + Pich_erro*0.58;
//		PID_Yaw_Out = Yaw_erro*18 + PID_Yaw_I*0.88;
//		PID_Pich_Out = Pich_erro * 18 + PID_Pich_I;
//		if(PID_Yaw_I>9999){PID_Pich_Out = 9999;}	
//		if(PID_Yaw_I<-9999){PID_Pich_Out = -9999;}	
//		if(PID_Pich_I>9999){PID_Pich_Out = 9999;}	
//		if(PID_Pich_I<-9999){PID_Pich_Out = -9999;}	
//		if(PID_Yaw_Out>29999){PID_Yaw_Out = 29999;}	
//		if(PID_Yaw_Out<-29999){PID_Yaw_Out = -29999;}		
//		if(PID_Pich_Out>29999){PID_Pich_Out = 29999;}	
//		if(PID_Pich_Out<-29999){PID_Pich_Out = -29999;}	
//		CAN2_Send_queue_add(0x1ff,0x08,PID_Pich_Out,0,0,0);//øÿ÷∆Àƒ∏ˆµ◊≈ÃµÁª˙
//		CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);
//}
/**********************************************************************************°¢
∫Ø ˝√˚£∫world_angle_keep(double abbs)
–Œ Ω≤Œ ˝£∫
1. ”æıΩ«∂»£∫0~418000£¨∑÷±∂‘”¶0~360∂»
◊˜’ﬂ£∫
 ±º‰£∫2022ƒÍ2‘¬14»’
π¶ƒ‹£∫øÿ÷∆‘∆Ã®Ω«∂»Œ»∂®”⁄ƒ≥∏ˆ ¿ΩÁΩ«∂»
**********************************************************************************/
//double world_angle_target;
//double world_angle_erro;
//double world_angle_yaw_D1;
//double world_angle_yaw_D2;
//void world_angle_keep(double abbs)
//{
//		world_angle_target = abbs;
//			if(world_angle_target >= word_angle[2])
//		{
//			if((world_angle_target-word_angle[2]) >= 209000)
//			{
//				world_angle_erro = -word_angle[2]  + (world_angle_target - 418000);
//			}	
//			else if((world_angle_target-word_angle[2]) < 209000)
//			{
//				world_angle_erro = (world_angle_target - word_angle[2]);
//			}
//		}	
//		else if(world_angle_target < word_angle[2])
//		{
//			if((world_angle_target-word_angle[2]) >= -209000)
//			{
//				world_angle_erro =  -(word_angle[2] - world_angle_target);
//			}	
//				if((world_angle_target-word_angle[2]) < -209000)
//			{
//				world_angle_erro =  (418000 - word_angle[2] + world_angle_target);
//			}	
//		}
//	 if(world_angle_erro < -18000)
//		{
//			world_angle_erro = -18000;
//		}	
//		if(world_angle_erro > 18000)
//		{
//			world_angle_erro = 18000;
//		}
//		world_angle_yaw_D1 = world_angle_erro;
//		Yaw_target = -(RC_Ctl.rc.ch0-1024)*1 + world_angle_erro*0.2033322 - (world_angle_yaw_D1 - world_angle_yaw_D2)*0.4;
//		Pich_target = (RC_Ctl.rc.ch1-1024)*8;
//		world_angle_yaw_D2 = world_angle_erro;
//		Yaw_erro = Yaw_target - (double)gyro_souce[2]+3.7337;
//		Pich_erro = Pich_target - (double)gyro_souce[0];
//		PID_Yaw_I = PID_Yaw_I + Yaw_erro*1.18;
//		PID_Pich_I = PID_Pich_I + Pich_erro*1.18;
//		if(PID_Yaw_I>9999){PID_Yaw_I = 9999;}	
//		if(PID_Yaw_I<-9999){PID_Yaw_I = -9999;}
//		if(Yaw_erro>9999){PID_Yaw_I = 0;}	
//		if(Yaw_erro<-9999){PID_Yaw_I = 0;}	
//		PID_Yaw_D_1 = gyro_souce[2];
//		PID_Pich_D_1 = gyro_souce[0];
//		PID_Yaw_Out = Yaw_erro*15 + PID_Yaw_I - (PID_Yaw_D_1-PID_Yaw_D_2)*1.2;
//		PID_Pich_Out = Pich_erro * 18 + PID_Pich_I - (PID_Pich_D_1-PID_Pich_D_2)*0.2;
//		PID_Yaw_D_2 = gyro_souce[2];
//		PID_Pich_D_2 = gyro_souce[0];
//		if(PID_Yaw_Out>29999){PID_Yaw_Out = 29999;}	
//		if(PID_Yaw_Out<-29999){PID_Yaw_Out = -29999;}		
//		if(PID_Pich_Out>29999){PID_Pich_Out = 29999;}	
//		if(PID_Pich_Out<-29999){PID_Pich_Out = -29999;}
//		//printf("%d,%d\r",cx,cy);
//		//printf("data1=%f,data2=%f,data3=%d,",PID_Yaw_Out,PID_Yaw_I2,gyro_souce[2]+4);
//		CAN2_Send_queue_add(0x1ff,0x08,PID_Pich_Out,0,0,0);//øÿ÷∆Àƒ∏ˆµ◊≈ÃµÁª˙
//		CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);
//}