#include "ChassisTask.h"
#include "power_control.h"
#include "AppTask.h"
#include "usart1.h"
#include "timer.h"
#include "can1.h"
#include "can2.h"
#include "GimbalTask.h"
#include "usart.h"
#include "pid.h"

float M3508_out[4];
float Chassis_Remote_Control[4];
const float M3508_len2 = 460;

volatile char Chassis_Mode2;
volatile unsigned short int LEV=0;

PidTypeDef Chassis_Speed[4];
float Chassis_Speed_PID[3]= {5,0.6,0};

float PID_M3508_I[4],PID_M3508_D_1[4],PID_M3508_D_2[4],PID_M3508_D_3[4];
float M3508_erro[4];
float M3508_target[4];
float M3508_out[4];
float M3508_target_set[4];
float M3508_erro[4];
float M3508_len;
int Middle_err;
int Middle_location;
float Middle_err_D_1,Middle_err_D_2;
float Big_top_angle;
float Bit_top_X,Bit_top_Y;
float Chassis_angle_speed;
float Chassis_angle_speed_Low_pass_filtering_1;
float Chassis_angle_speed_Low_pass_filtering_2;
float Chassis_360_speed_target;
float Chassis_360_speed_erro;
float Chassis_360_speed_line;
Chassis_Remote Chassis_Remote_t;
float Chassis_Remote_X;
float Chassis_Remote_Y;

uint8_t chassis_gryo360_move;
uint8_t chassis_sideway;
uint8_t chassis_following_Gimbal;

extern unsigned char Auto_state;					//自动模式状态标志	1为自动 0为手动
extern unsigned char Capture;				//是否识别到目标 

extern unsigned char First_get;	//

int X_Speed,Y_Speed=0,Z_Speed=-2000,ch1_in,ch2_in;
extern int game_state;

float WHEEL_R=0.075;	//麦克纳姆轮半径	（单位：m）
float WHEEL_PI=3.14;	//PI常数
float WHEEL_RATIO=19;	//电机减速比
float WHEEL_K=0.345;	//以车为中心点，电机坐标xy绝对值之和（单位：m）

float speed_x=0,speed_y,speed_w;		//xyz轴目标速度 单位：m/s
float v1=0,v2=0,v3=0,v4=0;							//换算出的每个电机转速

float Real_speed_x,Real_speed_y,Real_speed_w;		//xyz轴实际速度 单位：m/s
float V1,V2,V3,V4;							//换算出的每个电机转速


extern float x, fracpart;	//fracpart（小数部分）
extern short intpart,f_to_int;  //intpart(分出的整数部分)，f_to_int：小数转为整数发送

float gyro_yaw;

extern short gyro_0,gyro_1,gyro_2;
extern short accel_0,accel_1,accel_2;

unsigned char move_mode;	//移动方式：1：移动至入口处  2：移动至增益点
unsigned char move_start;
unsigned char move_cont;

extern unsigned char aim_auto;

extern short Ros_Target_Speedx;		//接收来自底盘传来的树莓派指定的底盘目标速度（mm/s），将此装载至自动控制模式下的底盘目标速度变量
extern short Ros_Target_Speedy;
extern short Ros_Target_Speedw;

float xx;
float yy;
float ww;

//int Attack_plan;		//进攻方式：0：遥控器接管	其他：雷达接管
										//														1：原地陀螺		2：自定义进攻方式①		3：自定义进攻方式②


extern char aim_test;		//自瞄测试

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

extern char arrive;
void speed_trans()		//xyz轴速度转换为底盘各电机转速
{
//  speed_x=Ros_Target_Speedx;
//	speed_y=Ros_Target_Speedy;
//	speed_w=Ros_Target_Speedw;

//	speed_x=((float)Ros_Target_Speedx)/1000;		//将底盘传来数据进行转换（mm/s转换成m/s）
//		 speed_y=((float)Ros_Target_Speedy)/1000;
//		 speed_w=((float)Ros_Target_Speedw)/1000;
	//speed_x=0.1;
	
  v1 =speed_x-speed_y-WHEEL_K*speed_w;       //转化为每个轮子的线速度
  v2 =speed_x+speed_y-WHEEL_K*speed_w;
  v3 =-(speed_x-speed_y+WHEEL_K*speed_w);
  v4 =-(speed_x+speed_y+WHEEL_K*speed_w);
	
//  v1 =speed_x-speed_y;       //转化为每个轮子的线速度
//  v2 =speed_x+speed_y;
//  v3 =-(speed_x-speed_y);
//  v4 =-(speed_x+speed_y);	
 
  v1 =v1/(2.0f*WHEEL_R*WHEEL_PI);    //转换为轮子的速度(弧度/s)
  v2 =v2/(2.0f*WHEEL_R*WHEEL_PI);
  v3 =v3/(2.0f*WHEEL_R*WHEEL_PI);
  v4 =v4/(2.0f*WHEEL_R*WHEEL_PI);
 
  v1 =v1*WHEEL_RATIO*60;    //转换为电机速度　单位　RPM
  v2 =v2*WHEEL_RATIO*60;
  v3 =v3*WHEEL_RATIO*60;
  v4 =v4*WHEEL_RATIO*60;
	
  //send_rpm_to_chassis(v1,v2,v3,v4);	//串口发送函数
}

void trans_speed()		//各电机转速（转每分）转换为车的xyz轴速度
{
	
	V1=Velocity_Value_3508[0];
	V2=Velocity_Value_3508[3];
	V3=Velocity_Value_3508[2];
	V4=Velocity_Value_3508[1];
	
	V1=V1/(WHEEL_RATIO*60);
	V2=V2/(WHEEL_RATIO*60);
	V3=V3/(WHEEL_RATIO*60);
	V4=V4/(WHEEL_RATIO*60);
	
	V1=V1*(2.0f*WHEEL_R*WHEEL_PI);
	V2=V2*(2.0f*WHEEL_R*WHEEL_PI);
	V3=V3*(2.0f*WHEEL_R*WHEEL_PI);
	V4=V4*(2.0f*WHEEL_R*WHEEL_PI);
	
	Real_speed_x=(V1+V2-V3-V4)/4;
	Real_speed_x=(V1+V2-V3-V4)/4;
	Real_speed_y=(V2-V1-V4+V3)/4;
	Real_speed_w=(V1+V2+V3+V4)/(-4*WHEEL_K);
	
}




void Gyro_360_motion(int Center_adjustment_value ,float target)
{
	float gain = V_Gains;
	Chassis_Remote_t.X=RC_Ctl.rc.ch3;
	Chassis_Remote_t.Y=RC_Ctl.rc.ch2;
	double scaled_X = (Chassis_Remote_t.X - 1024) * gain;
	double scaled_Y = (Chassis_Remote_t.Y - 1024) * gain;
			Position_Value_6020[1]	= Position_Value_6020[1] + Center_adjustment_value;
		if(Position_Value_6020[1] > 8191)
			{	
			Position_Value_6020[1] = Position_Value_6020[1] - 8191;
			}	
		Chassis_360_speed_target =	-(target -(RC_Ctl.keep.che-1024));
		M3508_len = 40;
		Chassis_angle_speed = (float)(Velocity_Value_6020[1]*7.8) - gyro_souce[2]/10 ;
		Chassis_angle_speed_Low_pass_filtering_1 = Chassis_angle_speed;		
		Chassis_angle_speed = Chassis_angle_speed_Low_pass_filtering_1 * 0.05f + Chassis_angle_speed_Low_pass_filtering_2 * (1-0.05f);
		Chassis_angle_speed_Low_pass_filtering_2	= Chassis_angle_speed;
		//printf("data1=%f,data2=%d,\r\n",Chassis_angle_speed,-Velocity_Value_6020[1]);
		Chassis_360_speed_erro 	= Chassis_360_speed_target - Chassis_angle_speed;
		if(Chassis_360_speed_erro > 0)
				{
					if(Chassis_360_speed_erro > M3508_len){Chassis_360_speed_target = Chassis_angle_speed + M3508_len;}
					else{Chassis_360_speed_target = Chassis_angle_speed + Chassis_360_speed_erro;}			
				}else
				{
					if(Chassis_360_speed_erro < -M3508_len){Chassis_360_speed_target = Chassis_angle_speed - M3508_len;}
					else{Chassis_360_speed_target = Chassis_angle_speed + Chassis_360_speed_erro;}	
				}	
		Big_top_angle = 3.14159*((8191-Position_Value_6020[1])/22.755)/180;
		Bit_top_X = ((scaled_X) * sin(Big_top_angle)) + ((scaled_Y) * cos(Big_top_angle)); //RC_Ctl.rc.ch3 X
		Bit_top_Y = ((scaled_X) * cos(Big_top_angle)) - ((scaled_Y) * sin(Big_top_angle));//RC_Ctl.rc.ch2 Y
		M3508_target[0] = (float)(-Bit_top_Y)*10+(Bit_top_X)*10 +3000 ;//Chassis_360_speed_target*13;
		M3508_target[1] = (float)(Bit_top_Y)*10+(Bit_top_X)*10 + 3000;//Chassis_360_speed_target*13;
		M3508_target[2] = (float)(Bit_top_Y)*10-(Bit_top_X)*10 + 3000;//Chassis_360_speed_target*13;
		M3508_target[3] = (float)(-Bit_top_Y)*10-(Bit_top_X)*10 + 3000;//Chassis_360_speed_target*13;
			PID_M3508_D_1[0] = Velocity_Value_3508[0];
			PID_M3508_D_1[1] = Velocity_Value_3508[1];
			PID_M3508_D_1[2] = Velocity_Value_3508[2];
			PID_M3508_D_1[3] = Velocity_Value_3508[3];
			PID_M3508_D_3[0] = PID_M3508_D_1[0] - PID_M3508_D_2[0];
			PID_M3508_D_3[1] = PID_M3508_D_1[1] - PID_M3508_D_2[1];
			PID_M3508_D_3[2] = PID_M3508_D_1[2] - PID_M3508_D_2[2];
			PID_M3508_D_3[3] = PID_M3508_D_1[3] - PID_M3508_D_2[3];
			M3508_erro[0] = M3508_target[0] - Velocity_Value_3508[0];
			M3508_erro[1] = M3508_target[1] - Velocity_Value_3508[1];
			M3508_erro[2] = M3508_target[2] - Velocity_Value_3508[2];
			M3508_erro[3] = M3508_target[3] - Velocity_Value_3508[3];
			PID_M3508_I[0] = PID_M3508_I[0] + M3508_erro[0]*2 - PID_M3508_D_3[0]*14;
			PID_M3508_I[1] = PID_M3508_I[1] + M3508_erro[1]*2 - PID_M3508_D_3[1]*14;
			PID_M3508_I[2] = PID_M3508_I[2] + M3508_erro[2]*2 - PID_M3508_D_3[2]*14;
			PID_M3508_I[3] = PID_M3508_I[3] + M3508_erro[3]*2 - PID_M3508_D_3[3]*14;
			M3508_out[0] =  PID_M3508_I[0] ;
			M3508_out[1] =  PID_M3508_I[1] ;
			M3508_out[2] =  PID_M3508_I[2] ;
			M3508_out[3] =  PID_M3508_I[3] ;
			PID_M3508_D_2[0] = Velocity_Value_3508[0];
			PID_M3508_D_2[1] = Velocity_Value_3508[1];
			PID_M3508_D_2[2] = Velocity_Value_3508[2];
			PID_M3508_D_2[3] = Velocity_Value_3508[3];
			CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
			chassis_gryo360_move=1;
			chassis_following_Gimbal=0;
			chassis_sideway=0;
			//printf("data1 = %d,data2 = %d,data3 = %d,data4 = %d\r\n",Velocity_Value_3508[0],-Velocity_Value_3508[1],-Velocity_Value_3508[2],Velocity_Value_3508[3]);	
}

/**********************************************************************************、
函数名：Gyro_360_motion(int Center_adjustment_value ,float target)
形式参数：
1.底盘正前方向校准值，范围0~8191;
2.陀螺转速，范围0~500
作者：
时间：2022年2月3日
功能：
1.实现陀螺运动

功能实现：
1.运动向量合
2.旋转向量坡度化，减少卡位时底盘打滑摩擦摩擦产生能量热消耗
**********************************************************************************/
void Gyro_360_motion2(int Center_adjustment_value ,float target)
{
		float Chassis_360_speed_target;
		float Big_top_angle;
		float Bit_top_X,Bit_top_Y;
		float M3508_target[4];
	
			Position_Value_6020[1]	= Position_Value_6020[1] + Center_adjustment_value;
		if(Position_Value_6020[1] > 8191)
			{	
			Position_Value_6020[1] = Position_Value_6020[1] - 8191;
			}	
			
		Chassis_360_speed_target =	target ;
		Big_top_angle = 3.14159*((8191-Position_Value_6020[1])/22.755)/180;//计算世界角
		Bit_top_X = (-(RC_Ctl.rc.ch3-1024) * sin(Big_top_angle)) - ((RC_Ctl.rc.ch2-1024) * cos(Big_top_angle));
		Bit_top_Y = (-(RC_Ctl.rc.ch3-1024) * cos(Big_top_angle)) + ((RC_Ctl.rc.ch2-1024) * sin(Big_top_angle));
			
		M3508_target[0] = (Bit_top_X)*10 + (Bit_top_Y)*10 + Chassis_360_speed_target*10;
		M3508_target[1] = (-Bit_top_X)*10 +(Bit_top_Y)*10 + Chassis_360_speed_target*10;
		M3508_target[2] = (-Bit_top_X)*10 -(Bit_top_Y)*10 + Chassis_360_speed_target*10;
		M3508_target[3] = (Bit_top_X)*10 - (Bit_top_Y)*10 + Chassis_360_speed_target*10;		
			
		M3508_out[0] = Chassis_Speed[0].Cal_PID(&Chassis_Speed[0],Velocity_Value_3508[0],M3508_target[0]);
		M3508_out[1] = Chassis_Speed[1].Cal_PID(&Chassis_Speed[1],Velocity_Value_3508[1],M3508_target[1]);
		M3508_out[2] = Chassis_Speed[2].Cal_PID(&Chassis_Speed[2],Velocity_Value_3508[2],M3508_target[2]);	
		M3508_out[3] = Chassis_Speed[3].Cal_PID(&Chassis_Speed[3],Velocity_Value_3508[3],M3508_target[3]);
			
//		printf("%d,%d,%d,%d\r\n",Velocity_Value_3508[0],-Velocity_Value_3508[1],-Velocity_Value_3508[2],Velocity_Value_3508[3]);	
		CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
		chassis_sideway=0;
		chassis_following_Gimbal=0;
		chassis_gryo360_move=1;			
}
/**********************************************************************************、
函数名：Chassis_following(int Middle,short way_limit) 
形式参数：
1.1/0是否支持前后车头转换;
2.车头转换阈值，范围0~4095;
作者：
时间：2022年2月3日
功能：底盘跟随云台	
功能实现：
1.增量式PID控制机器人底盘速度，同时实现低功耗控制
2.底盘转速坡度化
**********************************************************************************/
void Chassis_following2(int Middle) //底盘跟随云台
{
	  float gains=V_Gains;
		int Middle_err;
		int Middle_location;
		float Middle_err_D_1,Middle_err_D_2,Middle_err_D_3;
		float M3508_target_set[4];
		float Turn_Middle_Kp=3,Turn_Middle_Kd=0.16;   //3  0.1
		double scaled_X = (RC_Ctl.rc.ch3 - 1024) * gains;
		double scaled_Y = (RC_Ctl.rc.ch2 - 1024) * gains;
		/************************************遥控器控制*********************************************/
			Chassis_Remote_Control[0]=(double)((scaled_X)*10-(scaled_Y)*10);
			Chassis_Remote_Control[1]=(double)(-(scaled_X) *10-(scaled_Y)*10);
			Chassis_Remote_Control[2]=(double)(-(scaled_X) *10+(scaled_Y)*10);
			Chassis_Remote_Control[3]=(double)((scaled_X)*10+(scaled_Y)*10);

		/************************************回中PID*********************************************/	
		/************************************计算偏差角*********************************************/			
			Middle_location = Middle;			
			
			if(Middle_location > 8191)
				{Middle_location = Middle_location - 8191;}
			else if(Middle_location <0)
				{Middle_location = Middle_location + 8191;}
				
			//Position_Value_6020[1] = 8191 - Position_Value_6020[1] ;
			
		if(Middle_location >= Position_Value_6020[1])
		{
			
			if((Middle_location-Position_Value_6020[1]) >= 4095)
			{
				Middle_err = -Position_Value_6020[1]  + (Middle_location - 8191);
			}	
			else if((Middle_location-Position_Value_6020[1]) < 4095)
			{
				Middle_err = (Middle_location - Position_Value_6020[1]);
			}
			
		}	
		else if(Middle_location < Position_Value_6020[1])
		{
			if((Middle_location-Position_Value_6020[1]) >= -4095)
			{
				Middle_err =  -(Position_Value_6020[1] - Middle_location);
			}	
			else if((Middle_location-Position_Value_6020[1]) < -4095)
			{
				Middle_err =  (8191 - Position_Value_6020[1] + Middle_location);
			}	
			
		}		
			
			//printf("%d\r\n",Middle_err);	
		/************************************回中速度限幅*********************************************/
			if(Middle_err > 1048)
				{
					Middle_err = 1048;
				}
			if(Middle_err < -1048)
				{
					Middle_err = -1048;
				}
		/************************************回中PID计算*********************************************/				
			Middle_err_D_1 = Middle_err;
			Middle_err_D_3 = Middle_err_D_1 - Middle_err_D_2;
			Middle_err_D_3 = low_pass_filter(Middle_err_D_3);
			
			M3508_target_set[0] = Chassis_Remote_Control[0]+Middle_err*Turn_Middle_Kp-Middle_err_D_3*Turn_Middle_Kd;
			M3508_target_set[1] = Chassis_Remote_Control[1]+Middle_err*Turn_Middle_Kp-Middle_err_D_3*Turn_Middle_Kd;
			M3508_target_set[2] = Chassis_Remote_Control[2]+Middle_err*Turn_Middle_Kp-Middle_err_D_3*Turn_Middle_Kd;
			M3508_target_set[3] = Chassis_Remote_Control[3]+Middle_err*Turn_Middle_Kp-Middle_err_D_3*Turn_Middle_Kd;
						
			Middle_err_D_2 = Middle_err;
			//printf("%d\r\n",Position_Value_6020[1]);	
		/************************************底盘3508电机PID计算*********************************************/	
			M3508_out[0] = Chassis_Speed[0].Cal_PID(&Chassis_Speed[0],Velocity_Value_3508[0],Chassis_Slope2(M3508_len2,M3508_target_set[0],Velocity_Value_3508[0]));
			M3508_out[1] = Chassis_Speed[1].Cal_PID(&Chassis_Speed[1],Velocity_Value_3508[1],Chassis_Slope2(M3508_len2,M3508_target_set[1],Velocity_Value_3508[1]));
			M3508_out[2] = Chassis_Speed[2].Cal_PID(&Chassis_Speed[2],Velocity_Value_3508[2],Chassis_Slope2(M3508_len2,M3508_target_set[2],Velocity_Value_3508[2]));	
			M3508_out[3] = Chassis_Speed[3].Cal_PID(&Chassis_Speed[3],Velocity_Value_3508[3],Chassis_Slope2(M3508_len2,M3508_target_set[3],Velocity_Value_3508[3]));
				
//			printf("%f,%f\r\n",M3508_target[0],(float)Velocity_Value_3508[0]);	
			CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
			chassis_sideway=0;
			chassis_following_Gimbal=1;
			chassis_gryo360_move=0;
}

/**********************************************************************************、
函数名：Chassis_Regular_exercise(void) 
形式参数：
作者：
时间：
功能：测试
**********************************************************************************/
void Chassis_Regular_exercise(void)
{
			float M3508_target_set[4];
	
			M3508_target_set[0] = (double)(-(RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10);
			M3508_target_set[1] = (double)((RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10);
			M3508_target_set[2] = (double)((RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10);
			M3508_target_set[3] = (double)(-(RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10);
			
			M3508_out[0] = Chassis_Speed[0].Cal_PID(&Chassis_Speed[0],Velocity_Value_3508[0],Chassis_Slope2(M3508_len2,M3508_target_set[0],Velocity_Value_3508[0]));
			M3508_out[1] = Chassis_Speed[1].Cal_PID(&Chassis_Speed[1],Velocity_Value_3508[1],Chassis_Slope2(M3508_len2,M3508_target_set[1],Velocity_Value_3508[1]));
			M3508_out[2] = Chassis_Speed[2].Cal_PID(&Chassis_Speed[2],Velocity_Value_3508[2],Chassis_Slope2(M3508_len2,M3508_target_set[2],Velocity_Value_3508[2]));	
			M3508_out[3] = Chassis_Speed[3].Cal_PID(&Chassis_Speed[3],Velocity_Value_3508[3],Chassis_Slope2(M3508_len2,M3508_target_set[3],Velocity_Value_3508[3]));
				
		CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
	//				printf("%f,%f\r\n",M3508_out[0],(float)Velocity_Value_3508[0]);
}
/**********************************************************************************、
函数名：Chassis_Slope(int len,float erro,float value) 
形式参数：
作者：
时间：
功能：底盘坡度化计算
**********************************************************************************/
float Chassis_Slope2(int len,float target_set,float value)
{
	int M3508_len;
	float M3508_erro,M3508_value,M3508_target,M3508_target_set;
	
	M3508_len = len;
	M3508_target_set = target_set;
	M3508_value = value;

	M3508_erro = M3508_target_set - M3508_value;
	
	if(M3508_erro > 0)
		{
		if(M3508_erro > M3508_len)
			{
				M3508_target = M3508_value + M3508_len;
			}
		else
			{
				M3508_target = M3508_value + M3508_erro;
			}		
		}
	else
		{
			if(M3508_erro < -M3508_len)
			 {
				 M3508_target = M3508_value - M3508_len;
			 }
			else
			 {
				M3508_target = M3508_value + M3508_erro;
			 }
		}
	
	return M3508_target;
}
/**********************************************************************************、
函数名：Chassis_following(int Middle,short way_limit) 
形式参数：
1.1/0是否支持前后车头转换;
2.车头转换阈值，范围0~4095;
作者：
时间：2022年2月3日
功能：底盘跟随云台	
功能实现：
1.增量式PID控制机器人底盘速度，同时实现低功耗控制
2.底盘转速坡度化
**********************************************************************************/

short way_selet = 0,way_chart_a;

void Chassis_following(int Middle,short way_limit) //底盘跟随云台
{
	
	if(Auto_state==0)		//手动控制模式
	{
	way_chart_a = way_limit;
	
	
		if(way_selet == 0)
			{
			Middle_location = Middle;
			}
		else	
			{
			Middle_location =  Middle - 4095;
			}
			
			if(Middle_location > 8191){Middle_location = Middle_location - 8191;}
			else if(Middle_location <0){Middle_location = Middle_location + 8191;}
				
			//Position_Value_6020[1] = 8191 - Position_Value_6020[1] ;
			
			if(Middle_location >= Position_Value_6020[1])
		{
			
			if((Middle_location-Position_Value_6020[1]) >= 4095)
			{
				Middle_err = -Position_Value_6020[1]  + (Middle_location - 8191);
			}	
			else if((Middle_location-Position_Value_6020[1]) < 4095)
			{
				Middle_err = (Middle_location - Position_Value_6020[1]);
			}
			
		}	
		else if(Middle_location < Position_Value_6020[1])
		{
			if((Middle_location-Position_Value_6020[1]) >= -4095)
			{
				Middle_err =  -(Position_Value_6020[1] - Middle_location);
			}	
				if((Middle_location-Position_Value_6020[1]) < -4095)
			{
				Middle_err =  (8191 - Position_Value_6020[1] + Middle_location);
			}	
			
		}
		
		if(way_chart_a == 1)
		{	
		if(Middle_err > 2048 ||Middle_err < -2048){way_selet = !way_selet;}
		}else {way_selet =0;}
		
			Middle_err = Middle_err*2.5;

	
			if(Middle_err > 4811)
				{
					Middle_err = 4811;
				}
			if(Middle_err < -4811)
				{
					Middle_err = -4811;
				}	
				
				
				
			Middle_err_D_1 = Middle_err;
			
		
			
		if(RC_Ctl.rc.s2 != 1)		
		{
			if(way_selet == 0)
					{
					M3508_target_set[0] = (double)(-(RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10+ Middle_err - (Middle_err_D_1 - Middle_err_D_2)*2);
					M3508_target_set[1] = (double)((RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10+ Middle_err - (Middle_err_D_1 - Middle_err_D_2)*2);
					M3508_target_set[2] = (double)((RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10+ Middle_err - (Middle_err_D_1 - Middle_err_D_2)*2);
					M3508_target_set[3] = (double)(-(RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10+  Middle_err - (Middle_err_D_1 - Middle_err_D_2)*2);
					}
					else	
					{
					M3508_target_set[0] = (double)((RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10+ Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8);
					M3508_target_set[1] = (double)(-(RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10+ Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8);
					M3508_target_set[2] = (double)(-(RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10+ Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8);
                    M3508_target_set[3] = (double)((RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10+ Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8);
					}
		}

			Middle_err_D_2 = Middle_err;
				
			M3508_erro[0] = M3508_target_set[0] - Velocity_Value_3508[0];
			M3508_erro[1] = M3508_target_set[1] - Velocity_Value_3508[1];
			M3508_erro[2] = M3508_target_set[2] - Velocity_Value_3508[2];
			M3508_erro[3] = M3508_target_set[3] - Velocity_Value_3508[3];
			
			M3508_len = 461;
			
				if(M3508_erro[0] > 0)
				{
					if(M3508_erro[0] > M3508_len){M3508_target[0] = Velocity_Value_3508[0] + M3508_len;}
					else{M3508_target[0] = Velocity_Value_3508[0] + M3508_erro[0];}			
				}else
				{
					if(M3508_erro[0] < -M3508_len){M3508_target[0] = Velocity_Value_3508[0] - M3508_len;}
					else{M3508_target[0] = Velocity_Value_3508[0] + M3508_erro[0];}	
				}
				if(M3508_erro[1] > 0)
				{
					if(M3508_erro[1] > M3508_len){M3508_target[1] = Velocity_Value_3508[1] + M3508_len;}
					else{M3508_target[1] = Velocity_Value_3508[1] + M3508_erro[1];}			
				}else
				{
					if(M3508_erro[1] < -M3508_len){M3508_target[1] = Velocity_Value_3508[1] - M3508_len;}
					else{M3508_target[1] = Velocity_Value_3508[1] + M3508_erro[1];}	
				}
				
				if(M3508_erro[2] > 0)
				{
					if(M3508_erro[2] > M3508_len){M3508_target[2] = Velocity_Value_3508[2] + M3508_len;}
					else{M3508_target[2] = Velocity_Value_3508[2] + M3508_erro[2];}			
				}else
				{
					if(M3508_erro[2] < -M3508_len){M3508_target[2] = Velocity_Value_3508[2] - M3508_len;}
					else{M3508_target[2] = Velocity_Value_3508[2] + M3508_erro[2];}	
				}
				
				if(M3508_erro[3] > 0)
				{
					if(M3508_erro[3] > M3508_len){M3508_target[3] = Velocity_Value_3508[3] + M3508_len;}
					else{M3508_target[3] = Velocity_Value_3508[3] + M3508_erro[3];}			
				}else
				{
					if(M3508_erro[3] < -M3508_len){M3508_target[3] = Velocity_Value_3508[3] - M3508_len;}
					else{M3508_target[3] = Velocity_Value_3508[3] + M3508_erro[3];}	
				}

			
			PID_M3508_D_1[0] = Velocity_Value_3508[0];
			PID_M3508_D_1[1] = Velocity_Value_3508[1];
			PID_M3508_D_1[2] = Velocity_Value_3508[2];
			PID_M3508_D_1[3] = Velocity_Value_3508[3];
			
			PID_M3508_D_3[0] = PID_M3508_D_1[0] - PID_M3508_D_2[0];
			PID_M3508_D_3[1] = PID_M3508_D_1[1] - PID_M3508_D_2[1];
			PID_M3508_D_3[2] = PID_M3508_D_1[2] - PID_M3508_D_2[2];
			PID_M3508_D_3[3] = PID_M3508_D_1[3] - PID_M3508_D_2[3];
			
			M3508_erro[0] = M3508_target[0] - Velocity_Value_3508[0];
			M3508_erro[1] = M3508_target[1] - Velocity_Value_3508[1];
			M3508_erro[2] = M3508_target[2] - Velocity_Value_3508[2];
			M3508_erro[3] = M3508_target[3] - Velocity_Value_3508[3];
			
			PID_M3508_I[0] = PID_M3508_I[0] + M3508_erro[0]*3 - PID_M3508_D_3[0]*14;// 3 14// 4 14
			PID_M3508_I[1] = PID_M3508_I[1] + M3508_erro[1]*3 - PID_M3508_D_3[1]*14;
			PID_M3508_I[2] = PID_M3508_I[2] + M3508_erro[2]*3 - PID_M3508_D_3[2]*14;
			PID_M3508_I[3] = PID_M3508_I[3] + M3508_erro[3]*3 - PID_M3508_D_3[3]*14;
						
			M3508_out[0] =  PID_M3508_I[0] ;
			M3508_out[1] =  PID_M3508_I[1] ;
			M3508_out[2] =  PID_M3508_I[2] ;
			M3508_out[3] =  PID_M3508_I[3] ;
						
			PID_M3508_D_2[0] = Velocity_Value_3508[0];
			PID_M3508_D_2[1] = Velocity_Value_3508[1];
			PID_M3508_D_2[2] = Velocity_Value_3508[2];
			PID_M3508_D_2[3] = Velocity_Value_3508[3];
			
		CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
	//		if(RC_Ctl.rc.s2 == 1)		//右杆上拨：手动正反小陀螺（用于对抗赛检录时的通讯质量检测）
//		{
//			M3508_target_set[0]=((RC_Ctl.rc.ch2-1024)*10);
//			M3508_target_set[1]=((RC_Ctl.rc.ch2-1024)*10);
//			M3508_target_set[2]=((RC_Ctl.rc.ch2-1024)*10);
//			M3508_target_set[3]=((RC_Ctl.rc.ch2-1024)*10);
//		}		
	}
	
	
	if(Auto_state==1&&Capture==0)		//如果为自动模式，底盘速度运动将自动设置
	{
	
		way_chart_a = way_limit;
	
//	  X_Speed=100;
//	  Y_Speed=100;
//		if(X_Speed!=0)
//		{ch1_in=(X_Speed-(-10206))/9.967;}
//		if(Y_Speed!=0)
//		{ch2_in=(Y_Speed-(-10206))/9.967;}
//		if(X_Speed==0){ch1_in=1024;}
//		if(Y_Speed==0){ch2_in=1024;}
	
		if(way_selet == 0)
			{
			Middle_location = Middle;
			}
		else	
			{
			Middle_location =  Middle - 4095;
			}
			
			if(Middle_location > 8191){Middle_location = Middle_location - 8191;}
			else if(Middle_location <0){Middle_location = Middle_location + 8191;}
				
			//Position_Value_6020[1] = 8191 - Position_Value_6020[1] ;
			
			if(Middle_location >= Position_Value_6020[1])
		{
			
			if((Middle_location-Position_Value_6020[1]) >= 4095)
			{
				Middle_err = -Position_Value_6020[1]  + (Middle_location - 8191);
			}	
			else if((Middle_location-Position_Value_6020[1]) < 4095)
			{
				Middle_err = (Middle_location - Position_Value_6020[1]);
			}
			
		}	
		else if(Middle_location < Position_Value_6020[1])
		{
			if((Middle_location-Position_Value_6020[1]) >= -4095)
			{
				Middle_err =  -(Position_Value_6020[1] - Middle_location);
			}	
				if((Middle_location-Position_Value_6020[1]) < -4095)
			{
				Middle_err =  (8191 - Position_Value_6020[1] + Middle_location);
			}	
			
		}
		
		if(way_chart_a == 1)
		{	
		if(Middle_err > 2048 ||Middle_err < -2048){way_selet = !way_selet;}
		}else {way_selet =0;}
		
			Middle_err = Middle_err*2.5;

	
			if(Middle_err > 4811)
				{
					Middle_err = 4811;
				}
			if(Middle_err < -4811)
				{
					Middle_err = -4811;
				}	
				
				
				
			Middle_err_D_1 = Middle_err;
			


				if(way_selet == 0)
				{
				M3508_target_set[0] = v1+(double)(-(RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				M3508_target_set[1] = v2+(double)((RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				M3508_target_set[2] = v3+(double)((RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				M3508_target_set[3] = v4+(double)(-(RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				}
				else	
				{
				M3508_target_set[0] = v1+(double)((RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				M3508_target_set[1] = v2+(double)(-(RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				M3508_target_set[2] = v3+(double)(-(RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				M3508_target_set[3] = v4+(double)((RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*8;
				}
			


/*复合
		  M3508_target_set[0]=-X_Speed+Y_Speed-Z_Speed;
			M3508_target_set[1]=X_Speed+Y_Speed-Z_Speed;
			M3508_target_set[2]=X_Speed-Y_Speed-Z_Speed;
			M3508_target_set[3]=-X_Speed-Y_Speed-Z_Speed;
*/		
	
/*X前进后退测试
			M3508_target_set[0]=-X_Speed;
			M3508_target_set[1]=X_Speed;
			M3508_target_set[2]=X_Speed;
			M3508_target_set[3]=-X_Speed;
*/
			
/*Y左右平移测试
			M3508_target_set[0]=Y_Speed;
			M3508_target_set[1]=Y_Speed;
			M3508_target_set[2]=-Y_Speed;
			M3508_target_set[3]=-Y_Speed;
*/

/*旋转运动测试
			M3508_target_set[0]=-Z_Speed;
			M3508_target_set[1]=-Z_Speed;
			M3508_target_set[2]=-Z_Speed;
			M3508_target_set[3]=-Z_Speed;
*/
			
			
//			M3508_target[0] = (double)(-(RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*6;
//			M3508_target[1] = (double)((RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*6;
//			M3508_target[2] = (double)((RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*6;
//			M3508_target[3] = (double)(-(RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10) + Middle_err - (Middle_err_D_1 - Middle_err_D_2)*6;	
			Middle_err_D_2 = Middle_err;
				
			M3508_erro[0] = M3508_target_set[0] - Velocity_Value_3508[0];
			M3508_erro[1] = M3508_target_set[1] - Velocity_Value_3508[1];
			M3508_erro[2] = M3508_target_set[2] - Velocity_Value_3508[2];
			M3508_erro[3] = M3508_target_set[3] - Velocity_Value_3508[3];
			
			M3508_len = 461;
			
				if(M3508_erro[0] > 0)
				{
					if(M3508_erro[0] > M3508_len){M3508_target[0] = Velocity_Value_3508[0] + M3508_len;}
					else{M3508_target[0] = Velocity_Value_3508[0] + M3508_erro[0];}			
				}else
				{
					if(M3508_erro[0] < -M3508_len){M3508_target[0] = Velocity_Value_3508[0] - M3508_len;}
					else{M3508_target[0] = Velocity_Value_3508[0] + M3508_erro[0];}	
				}
				if(M3508_erro[1] > 0)
				{
					if(M3508_erro[1] > M3508_len){M3508_target[1] = Velocity_Value_3508[1] + M3508_len;}
					else{M3508_target[1] = Velocity_Value_3508[1] + M3508_erro[1];}			
				}else
				{
					if(M3508_erro[1] < -M3508_len){M3508_target[1] = Velocity_Value_3508[1] - M3508_len;}
					else{M3508_target[1] = Velocity_Value_3508[1] + M3508_erro[1];}	
				}
				
				if(M3508_erro[2] > 0)
				{
					if(M3508_erro[2] > M3508_len){M3508_target[2] = Velocity_Value_3508[2] + M3508_len;}
					else{M3508_target[2] = Velocity_Value_3508[2] + M3508_erro[2];}			
				}else
				{
					if(M3508_erro[2] < -M3508_len){M3508_target[2] = Velocity_Value_3508[2] - M3508_len;}
					else{M3508_target[2] = Velocity_Value_3508[2] + M3508_erro[2];}	
				}
				
				if(M3508_erro[3] > 0)
				{
					if(M3508_erro[3] > M3508_len){M3508_target[3] = Velocity_Value_3508[3] + M3508_len;}
					else{M3508_target[3] = Velocity_Value_3508[3] + M3508_erro[3];}			
				}else
				{
					if(M3508_erro[3] < -M3508_len){M3508_target[3] = Velocity_Value_3508[3] - M3508_len;}
					else{M3508_target[3] = Velocity_Value_3508[3] + M3508_erro[3];}	
				}

			
			PID_M3508_D_1[0] = Velocity_Value_3508[0];
			PID_M3508_D_1[1] = Velocity_Value_3508[1];
			PID_M3508_D_1[2] = Velocity_Value_3508[2];
			PID_M3508_D_1[3] = Velocity_Value_3508[3];
			
			PID_M3508_D_3[0] = PID_M3508_D_1[0] - PID_M3508_D_2[0];
			PID_M3508_D_3[1] = PID_M3508_D_1[1] - PID_M3508_D_2[1];
			PID_M3508_D_3[2] = PID_M3508_D_1[2] - PID_M3508_D_2[2];
			PID_M3508_D_3[3] = PID_M3508_D_1[3] - PID_M3508_D_2[3];
			
			M3508_erro[0] = M3508_target[0] - Velocity_Value_3508[0];
			M3508_erro[1] = M3508_target[1] - Velocity_Value_3508[1];
			M3508_erro[2] = M3508_target[2] - Velocity_Value_3508[2];
			M3508_erro[3] = M3508_target[3] - Velocity_Value_3508[3];
			
			PID_M3508_I[0] = PID_M3508_I[0] + M3508_erro[0]*3 - PID_M3508_D_3[0]*14;// 3 14// 4 14
			PID_M3508_I[1] = PID_M3508_I[1] + M3508_erro[1]*3 - PID_M3508_D_3[1]*14;
			PID_M3508_I[2] = PID_M3508_I[2] + M3508_erro[2]*3 - PID_M3508_D_3[2]*14;
			PID_M3508_I[3] = PID_M3508_I[3] + M3508_erro[3]*3 - PID_M3508_D_3[3]*14;
						
			M3508_out[0] =  PID_M3508_I[0] ;
			M3508_out[1] =  PID_M3508_I[1] ;
			M3508_out[2] =  PID_M3508_I[2] ;
			M3508_out[3] =  PID_M3508_I[3] ;
						
			PID_M3508_D_2[0] = Velocity_Value_3508[0];
			PID_M3508_D_2[1] = Velocity_Value_3508[1];
			PID_M3508_D_2[2] = Velocity_Value_3508[2];
			PID_M3508_D_2[3] = Velocity_Value_3508[3];
			
		CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
	}
}




/**********************************************************************************、
任务名：Chassis_task(void *p_arg)
形式参数：无
作者：
时间：2022年2月3日
功能：逻辑控制机器人底盘动作
模式一：小陀螺运动模式
模式二：云台跟随底盘
模式三：扭腰运动模式
丢控保护：底盘电机无力
任务频率：100FPS
**********************************************************************************/
int Chassis_Mode;
void Chassis_task(void *p_arg)
{
	OS_ERR err2;
	Chassis_PID_Init();
	Chassis_PowerControlParam_Init();
	float Vx=0,Vy=0,Vz=0;
  float simulated_energy=18000.0f; // 初始能量值，实际应从裁判系统获取
//	float Vx=0,Vy=0,Vz=0;
//  float simulated_energy=18000.0f; // 初始能量值，实际应从裁判系统获取
	//CPU_SR_ALLOC();//防止编译警告
//	printf("底盘任务 开始\r\n");
	
	while(1)
	{ 
//		if(Chassis_Mode==0) CAN1_Send_queue_add(0x200,0x08,0,0,0,0);
//		else if (Chassis_Mode==1) RC_Ctl.rc.s1 = 1;
//		else if(Chassis_Mode==2) RC_Ctl.rc.s1 = 2;
//		else if (Chassis_Mode==3) RC_Ctl.rc.s1 = 3;
		
//		Chassis_Remote_t.Vx=(RC_Ctl.rc.ch3-1024)*5;
//		Chassis_Remote_t.Vy=(RC_Ctl.rc.ch2-1024)*5;
//		Chassis_Remote_t.Vz=(Chassis_Remote_t.Z-1024)*10;
//		Chassis_PowerControlTest(Chassis_Remote_t.Vx,Chassis_Remote_t.Vy,0,18000);
//		float speed_factor = 5.0f; // 与Chassis_following中的因子一致
//		float rotation_speed_factor = 10.0f; // 与Chassis_following中的因子一致
//		Vx = (Chassis_Remote_X - 1024) * speed_factor;
//		Vy = (Chassis_Remote_Y - 1024) * speed_factor;
//		Vz = (RC_Ctl.keep.che - 1024) * rotation_speed_factor;
//		Chassis_PowerControlTest(Vx, Vy, Vz, simulated_energy);//调用功率控制，更新V_Gains(速度增益)
		float speed_factor = 5.0f; // 与Chassis_following中的因子一致
		float rotation_speed_factor = 10.0f; // 与Chassis_following中的因子一致
		Vx = (Chassis_Remote_X - 1024) * speed_factor;
		Vy = (Chassis_Remote_Y - 1024) * speed_factor;
		Vz = (RC_Ctl.keep.che - 1024) * rotation_speed_factor;
		Chassis_PowerControlTest(Vx, Vy, Vz, simulated_energy);//调用功率控制，更新V_Gains(速度增益)
		
		if(RC_Ctl.rc.s1 == 1)	//左拨杆上拨(自瞄测试)
		{
//			Auto_state=1;
//			aim_test=1;
//			 Gyro_360_motion(5000,1200);		//开启小陀螺
			Gyro_360_motion2(1742,400);
		}
		
		else if(RC_Ctl.rc.s1 == 2)			//左拨杆杆下拨，切换为自动 
		{
				//Auto_state=1;	
				aim_test=1;
			
		}

		else if(RC_Ctl.rc.s1 == 3)	//左杆回中，手动，模式二：底盘跟随云台
		{
			
			Auto_state=0;
			Chassis_following2(3550);
//            way_selet=0;
//			Chassis_following(2400,0); //底盘跟随云台	7494   4095   5260云台前方6340
			speed_trans();
			trans_speed();
				//aim_test=0;
		}
		
		
		else 	//模式四：丢控保护，底盘无力
		{
			chassis_sideway=0;
			chassis_following_Gimbal=0;
			chassis_gryo360_move=0;		
			CAN1_Send_queue_add(0x200,0x08,0,0,0,0);//发送无力指令
			Auto_state=0;
			
		}
		
		
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err2); //延时500ms		
//		
//		if(Auto_state==1&&RC_Ctl.rc.s1 != 1)		//自动模式下，云台c板对底盘的控制
//		{
//			if(arrive==0)	//雷达控制下
//			{
//				speed_trans();
//				trans_speed();
//				Chassis_following(3450,0); //底盘跟随云台	7494   4095   5260云台前方6340
//				aim_test=0;
//			}
//				
//			
//			else if(arrive==2)		//已经返回巡逻区：自瞄+小陀螺运动
//			{
//				Gyro_360_motion(3450,1200);			//小陀螺运动 1215 121  6316 250  
//				aim_test=1;
//			}
//			else									//到达第一个地点：无敌状态下，直进行自瞄
//			{
////				aim_test=1;
//			}
//		}
//		
		
//		if(Game_progress!=4)		//比赛未开始
//		{
//			game_state = 0;
//		}
//		if((Game_progress==4)&&(game_state==0))	//比赛开始
//		{
//			game_state = 1;
//			
//		}
//		else if(((blue_outpost_HP<500))&&(game_state==1))	//回家条件：自家前哨站残血
//		{
//			game_state = 2;
//		}
//		
	
		//printf("data1=%d,data2=%d,data3=%d,data4=%d",(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);
		//printf("%d\r",RC_Ctl.rc.ch3);

	}
	
	
}

/**********************************************************************************、
函数名：Gyro_360_motion(int Center_adjustment_value ,float target)
形式参数：
1.底盘正前方向校准值，范围0~8191;
2.陀螺转速，范围0~500
作者：
时间：2022年2月3日
功能：
1.实现陀螺运动

功能实现：
1.运动向量合
2.旋转向量坡度化，减少卡位时底盘打滑摩擦摩擦产生能量热消耗
**********************************************************************************/

//void motion(int Center_adjustment_value ,float target,float X,float Y)		
//{
//			Position_Value_6020[1]	= Position_Value_6020[1] + Center_adjustment_value;
//		if(Position_Value_6020[1] > 8191)
//			{	
//			Position_Value_6020[1] = Position_Value_6020[1] - 8191;
//			}	
//		Chassis_360_speed_target =	-(target -(RC_Ctl.keep.che-1024));
//		M3508_len = 40;
//		Chassis_angle_speed = (float)(Velocity_Value_6020[1]*7.8) - gyro_souce[2]/10 ;
//		Chassis_angle_speed_Low_pass_filtering_1 = Chassis_angle_speed;		
//		Chassis_angle_speed = Chassis_angle_speed_Low_pass_filtering_1 * 0.05f + Chassis_angle_speed_Low_pass_filtering_2 * (1-0.05f);
//		Chassis_angle_speed_Low_pass_filtering_2	= Chassis_angle_speed;
//		//printf("data1=%f,data2=%d,\r\n",Chassis_angle_speed,-Velocity_Value_6020[1]);
//		Chassis_360_speed_erro 	= Chassis_360_speed_target - Chassis_angle_speed;
//		if(Chassis_360_speed_erro > 0)
//				{
//					if(Chassis_360_speed_erro > M3508_len){Chassis_360_speed_target = Chassis_angle_speed + M3508_len;}
//					else{Chassis_360_speed_target = Chassis_angle_speed + Chassis_360_speed_erro;}			
//				}else
//				{
//					if(Chassis_360_speed_erro < -M3508_len){Chassis_360_speed_target = Chassis_angle_speed - M3508_len;}
//					else{Chassis_360_speed_target = Chassis_angle_speed + Chassis_360_speed_erro;}	
//				}	
//		Big_top_angle = 3.14159*((8191-Position_Value_6020[1])/22.755)/180;
//		Bit_top_X = ((X) * sin(Big_top_angle)) + ((Y) * cos(Big_top_angle));
//		Bit_top_Y = (X * cos(Big_top_angle)) - (Y * sin(Big_top_angle));
//		M3508_target[0] = (float)(-Bit_top_Y)*10+(Bit_top_X)*10 +3000 ;//Chassis_360_speed_target*13;
//		M3508_target[1] = (float)(Bit_top_Y)*10+(Bit_top_X)*10 + 3000;//Chassis_360_speed_target*13;
//		M3508_target[2] = (float)(Bit_top_Y)*10-(Bit_top_X)*10 + 3000;//Chassis_360_speed_target*13;
//		M3508_target[3] = (float)(-Bit_top_Y)*10-(Bit_top_X)*10 + 3000;//Chassis_360_speed_target*13;
//			PID_M3508_D_1[0] = Velocity_Value_3508[0];
//			PID_M3508_D_1[1] = Velocity_Value_3508[1];
//			PID_M3508_D_1[2] = Velocity_Value_3508[2];
//			PID_M3508_D_1[3] = Velocity_Value_3508[3];
//			PID_M3508_D_3[0] = PID_M3508_D_1[0] - PID_M3508_D_2[0];
//			PID_M3508_D_3[1] = PID_M3508_D_1[1] - PID_M3508_D_2[1];
//			PID_M3508_D_3[2] = PID_M3508_D_1[2] - PID_M3508_D_2[2];
//			PID_M3508_D_3[3] = PID_M3508_D_1[3] - PID_M3508_D_2[3];
//			M3508_erro[0] = M3508_target[0] - Velocity_Value_3508[0];
//			M3508_erro[1] = M3508_target[1] - Velocity_Value_3508[1];
//			M3508_erro[2] = M3508_target[2] - Velocity_Value_3508[2];
//			M3508_erro[3] = M3508_target[3] - Velocity_Value_3508[3];
//			PID_M3508_I[0] = PID_M3508_I[0] + M3508_erro[0]*2 - PID_M3508_D_3[0]*14;
//			PID_M3508_I[1] = PID_M3508_I[1] + M3508_erro[1]*2 - PID_M3508_D_3[1]*14;
//			PID_M3508_I[2] = PID_M3508_I[2] + M3508_erro[2]*2 - PID_M3508_D_3[2]*14;
//			PID_M3508_I[3] = PID_M3508_I[3] + M3508_erro[3]*2 - PID_M3508_D_3[3]*14;
//			M3508_out[0] =  PID_M3508_I[0] ;
//			M3508_out[1] =  PID_M3508_I[1] ;
//			M3508_out[2] =  PID_M3508_I[2] ;
//			M3508_out[3] =  PID_M3508_I[3] ;
//			PID_M3508_D_2[0] = Velocity_Value_3508[0];
//			PID_M3508_D_2[1] = Velocity_Value_3508[1];
//			PID_M3508_D_2[2] = Velocity_Value_3508[2];
//			PID_M3508_D_2[3] = Velocity_Value_3508[3];
//			CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
//			//printf("data1 = %d,data2 = %d,data3 = %d,data4 = %d\r\n",Velocity_Value_3508[0],-Velocity_Value_3508[1],-Velocity_Value_3508[2],Velocity_Value_3508[3]);	
//}
/**********************************************************************************、
函数名：Self_reference_motion(void)
形式参数：无
作者：
时间：2022年2月3日
功能：
1.配合云台任务实现云台跟随底盘
2.当前弃用，做技术保留
功能实现：
1.使用YAW轴电机编码器运行
**********************************************************************************/
//void Self_reference_motion(void) //自身参考运动
//{
//			M3508_target[0] = (double)(-(RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10);
//			M3508_target[1] = (double)((RC_Ctl.rc.ch3-1024)*10+(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10);
//			M3508_target[2] = (double)((RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10);
//			M3508_target[3] = (double)(-(RC_Ctl.rc.ch3-1024)*10-(RC_Ctl.rc.ch2-1024)*10-(RC_Ctl.keep.che-1024)*10);
//			PID_M3508_D_1[0] = Velocity_Value_3508[0];
//			PID_M3508_D_1[1] = Velocity_Value_3508[1];
//			PID_M3508_D_1[2] = Velocity_Value_3508[2];
//			PID_M3508_D_1[3] = Velocity_Value_3508[3];
//			PID_M3508_D_3[0] = PID_M3508_D_1[0] - PID_M3508_D_2[0];
//			PID_M3508_D_3[1] = PID_M3508_D_1[1] - PID_M3508_D_2[1];
//			PID_M3508_D_3[2] = PID_M3508_D_1[2] - PID_M3508_D_2[2];
//			PID_M3508_D_3[3] = PID_M3508_D_1[3] - PID_M3508_D_2[3];
//			M3508_erro[0] = M3508_target[0] - Velocity_Value_3508[0];
//			M3508_erro[1] = M3508_target[1] - Velocity_Value_3508[1];
//			M3508_erro[2] = M3508_target[2] - Velocity_Value_3508[2];
//			M3508_erro[3] = M3508_target[3] - Velocity_Value_3508[3];
//			PID_M3508_I[0] = PID_M3508_I[0] + M3508_erro[0] - PID_M3508_D_3[0]*14;
//			PID_M3508_I[1] = PID_M3508_I[1] + M3508_erro[1] - PID_M3508_D_3[1]*14;
//			PID_M3508_I[2] = PID_M3508_I[2] + M3508_erro[2] - PID_M3508_D_3[2]*14;
//			PID_M3508_I[3] = PID_M3508_I[3] + M3508_erro[3] - PID_M3508_D_3[3]*14;
//			M3508_out[0] =  PID_M3508_I[0] ;
//			M3508_out[1] =  PID_M3508_I[1] ;
//			M3508_out[2] =  PID_M3508_I[2] ;
//			M3508_out[3] =  PID_M3508_I[3] ;			
//			PID_M3508_D_2[0] = Velocity_Value_3508[0];
//			PID_M3508_D_2[1] = Velocity_Value_3508[1];
//			PID_M3508_D_2[2] = Velocity_Value_3508[2];
//			PID_M3508_D_2[3] = Velocity_Value_3508[3];
//			PID_M3508_D_2[0] = Velocity_Value_3508[0];
//			PID_M3508_D_2[1] = Velocity_Value_3508[1];
//			PID_M3508_D_2[2] = Velocity_Value_3508[2];
//			PID_M3508_D_2[3] = Velocity_Value_3508[3];
//			CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
//			//printf("data1 = %d,data2 = %d,data3 = %d,data4 = %d\r\n",Velocity_Value_3508[0],-Velocity_Value_3508[1],-Velocity_Value_3508[2],Velocity_Value_3508[3]);	
//}
/**********************************************************************************、
函数名：chassis_sprain(int Middle,int left_limit,int right_limit)
形式参数：
1.1/0是否支持前后车头转换;
2.左扭腰角度，范围0~8191;
3.右扭腰角度，0~8191
作者：
时间：2022年2月3日
功能：
1.机器人正弦扭腰
2.机器人前后车头转换	

功能实现：
1.增量式PID控制机器人底盘速度，同时实现低功耗控制
2.底盘转速坡度化
3.角速度转换正弦值
4.运动矢量合控制机器人全向移动
**********************************************************************************/
//	int Middle_location ;
//	int left ;
//	int right ;
//	int the_way = 0;
//	int target_sprain = 0;
//	int errer = 0;
//void chassis_sprain(int Middle,int left_limit,int right_limit)
//{
//	
//	 Middle_location = Middle;
//	 left = left_limit;
//	 right = right_limit;
//	 
//	 target_sprain = 0;
//	
//	if(the_way == 0)
//	{
//		Middle_location = Middle + left_limit;
//		
//		if(Middle_location > 8191)
//		{
//			Middle_location = Middle_location - 8191;
//		}
//		else if(Middle_location < 0)
//		{
//			Middle_location = Middle_location + 8191;
//		}
//	}
//	else if(the_way == 1)
//	{
//		Middle_location = Middle + right_limit;
//		
//				if(Middle_location > 8191)
//		{
//			Middle_location = Middle_location - 8191;
//		}
//		else if(Middle_location < 0)
//		{
//			Middle_location = Middle_location + 8191;
//		}
//	}
//	
//	


//	
//if(Middle_location >= Position_Value_6020[1])
//		{
//			
//			if((Middle_location-Position_Value_6020[1]) >= 4095)
//			{
//				Middle_err = -Position_Value_6020[1]  + (Middle_location - 8191);
//			}	
//			else if((Middle_location-Position_Value_6020[1]) < 4095)
//			{
//				Middle_err = (Middle_location - Position_Value_6020[1]);
//			}
//			
//		}	
//		else if(Middle_location < Position_Value_6020[1])
//		{
//			if((Middle_location-Position_Value_6020[1]) >= -4095)
//			{
//				Middle_err =  -(Position_Value_6020[1] - Middle_location);
//			}	
//				if((Middle_location-Position_Value_6020[1]) < -4095)
//			{
//				Middle_err =  (8191 - Position_Value_6020[1] + Middle_location);
//			}	
//			
//		}
//		
//			Middle_err = Middle_err*2;
//		
//	//printf("p_yaw=%d , Middle_err=%d, target_sprain=%d, the_way=%d\r\n",Position_Value_6020[1],Middle_err,Middle_location,the_way);
//		
//		
//		if(abs(Middle_err) < 32)
//		{
//			if(the_way == 1)
//			{
//				the_way = 0;
//			}
//			
//			else if(the_way == 0)
//			{
//				the_way = 1;
//			}
//		}
//		
//			if(Middle_err > 4811)
//				{
//					Middle_err = 4811;
//				}
//			if(Middle_err < -4811)
//				{
//					Middle_err = -4811;
//				}	
//				
//				
//				
//		Position_Value_6020[1]	= Position_Value_6020[1] + 697;
//			
//		if(Position_Value_6020[1] > 8191)
//			{	
//			Position_Value_6020[1] = Position_Value_6020[1] - 8191;
//			}	
//			
//			
//		Big_top_angle = 3.14159*((8191-Position_Value_6020[1])/22.7555)/180;
//			
//		Bit_top_X = ((RC_Ctl.rc.ch3-1024) * sin(Big_top_angle)) + ((RC_Ctl.rc.ch2-1024) * cos(Big_top_angle));
//		Bit_top_Y = ((RC_Ctl.rc.ch3-1024) * cos(Big_top_angle)) - ((RC_Ctl.rc.ch2-1024) * sin(Big_top_angle));
//				

//			
//			
//			if(Middle_err > 0)
//			{	
//					errer = 3122*fabs(cos(Big_top_angle));
//					if(errer < 1222)
//					{
//						errer=1222;
//					}
//			}

//			if(Middle_err < 0)
//			{	
//						errer = 3122*fabs(cos(Big_top_angle));
//					if(errer < 1222)
//					{
//						errer=1222;
//					}
//					
//					errer=-errer;
//			}


//			Middle_err_D_1 = errer;
//			M3508_target_set[0] = (double)(-(Bit_top_Y)*10+(Bit_top_X)*10-(RC_Ctl.keep.che-1024)*10) + errer ;
//			M3508_target_set[1] = (double)((Bit_top_Y)*10+(Bit_top_X)*10-(RC_Ctl.keep.che-1024)*10) + errer ;
//			M3508_target_set[2] = (double)((Bit_top_Y)*10-(Bit_top_X)*10-(RC_Ctl.keep.che-1024)*10) + errer ;
//			M3508_target_set[3] = (double)(-(Bit_top_Y)*10-(Bit_top_X)*10-(RC_Ctl.keep.che-1024)*10) + errer ;
//			Middle_err_D_2 = errer;


//			
//			M3508_erro[0] = M3508_target_set[0] - Velocity_Value_3508[0];
//			M3508_erro[1] = M3508_target_set[1] - Velocity_Value_3508[1];
//			M3508_erro[2] = M3508_target_set[2] - Velocity_Value_3508[2];
//			M3508_erro[3] = M3508_target_set[3] - Velocity_Value_3508[3];
//			
//				M3508_len = 421;
//			
//				if(M3508_erro[0] > 0)
//				{
//					if(M3508_erro[0] > M3508_len){M3508_target[0] = Velocity_Value_3508[0] + M3508_len;}
//					else{M3508_target[0] = Velocity_Value_3508[0] + M3508_erro[0];}			
//				}else
//				{
//					if(M3508_erro[0] < -M3508_len){M3508_target[0] = Velocity_Value_3508[0] - M3508_len;}
//					else{M3508_target[0] = Velocity_Value_3508[0] + M3508_erro[0];}	
//				}
//				if(M3508_erro[1] > 0)
//				{
//					if(M3508_erro[1] > M3508_len){M3508_target[1] = Velocity_Value_3508[1] + M3508_len;}
//					else{M3508_target[1] = Velocity_Value_3508[1] + M3508_erro[1];}			
//				}else
//				{
//					if(M3508_erro[1] < -M3508_len){M3508_target[1] = Velocity_Value_3508[1] - M3508_len;}
//					else{M3508_target[1] = Velocity_Value_3508[1] + M3508_erro[1];}	
//				}
//				
//				if(M3508_erro[2] > 0)
//				{
//					if(M3508_erro[2] > M3508_len){M3508_target[2] = Velocity_Value_3508[2] + M3508_len;}
//					else{M3508_target[2] = Velocity_Value_3508[2] + M3508_erro[2];}			
//				}else
//				{
//					if(M3508_erro[2] < -M3508_len){M3508_target[2] = Velocity_Value_3508[2] - M3508_len;}
//					else{M3508_target[2] = Velocity_Value_3508[2] + M3508_erro[2];}	
//				}
//				
//				if(M3508_erro[3] > 0)
//				{
//					if(M3508_erro[3] > M3508_len){M3508_target[3] = Velocity_Value_3508[3] + M3508_len;}
//					else{M3508_target[3] = Velocity_Value_3508[3] + M3508_erro[3];}			
//				}else
//				{
//					if(M3508_erro[3] < -M3508_len){M3508_target[3] = Velocity_Value_3508[3] - M3508_len;}
//					else{M3508_target[3] = Velocity_Value_3508[3] + M3508_erro[3];}	
//				}

//			
//			PID_M3508_D_1[0] = Velocity_Value_3508[0];
//			PID_M3508_D_1[1] = Velocity_Value_3508[1];
//			PID_M3508_D_1[2] = Velocity_Value_3508[2];
//			PID_M3508_D_1[3] = Velocity_Value_3508[3];
//			
//			PID_M3508_D_3[0] = PID_M3508_D_1[0] - PID_M3508_D_2[0];
//			PID_M3508_D_3[1] = PID_M3508_D_1[1] - PID_M3508_D_2[1];
//			PID_M3508_D_3[2] = PID_M3508_D_1[2] - PID_M3508_D_2[2];
//			PID_M3508_D_3[3] = PID_M3508_D_1[3] - PID_M3508_D_2[3];
//			
//			M3508_erro[0] = M3508_target[0] - Velocity_Value_3508[0];
//			M3508_erro[1] = M3508_target[1] - Velocity_Value_3508[1];
//			M3508_erro[2] = M3508_target[2] - Velocity_Value_3508[2];
//			M3508_erro[3] = M3508_target[3] - Velocity_Value_3508[3];
//			
//			PID_M3508_I[0] = PID_M3508_I[0] + M3508_erro[0]*4 - PID_M3508_D_3[0]*14;//14
//			PID_M3508_I[1] = PID_M3508_I[1] + M3508_erro[1]*4 - PID_M3508_D_3[1]*14;
//			PID_M3508_I[2] = PID_M3508_I[2] + M3508_erro[2]*4 - PID_M3508_D_3[2]*14;
//			PID_M3508_I[3] = PID_M3508_I[3] + M3508_erro[3]*4 - PID_M3508_D_3[3]*14;
//						
//			M3508_out[0] =  PID_M3508_I[0] ;
//			M3508_out[1] =  PID_M3508_I[1] ;
//			M3508_out[2] =  PID_M3508_I[2] ;
//			M3508_out[3] =  PID_M3508_I[3] ;
//						
//			PID_M3508_D_2[0] = Velocity_Value_3508[0];
//			PID_M3508_D_2[1] = Velocity_Value_3508[1];
//			PID_M3508_D_2[2] = Velocity_Value_3508[2];
//			PID_M3508_D_2[3] = Velocity_Value_3508[3];
//			
//		CAN1_Send_queue_add(0x200,0x08,(int)M3508_out[0],(int)M3508_out[1],(int)M3508_out[2],(int)M3508_out[3]);//控制四个底盘电机
//}

void Chassis_PID_Init(void)
{
	pid_init(&Chassis_Speed[0]);
	pid_init(&Chassis_Speed[1]);
	pid_init(&Chassis_Speed[2]);
	pid_init(&Chassis_Speed[3]);
	
	Chassis_Speed[0].Param_Init(&Chassis_Speed[0],PID_DELTA,Chassis_Speed_PID,0,0,0);
	Chassis_Speed[1].Param_Init(&Chassis_Speed[1],PID_DELTA,Chassis_Speed_PID,0,0,0);
	Chassis_Speed[2].Param_Init(&Chassis_Speed[2],PID_DELTA,Chassis_Speed_PID,0,0,0);
	Chassis_Speed[3].Param_Init(&Chassis_Speed[3],PID_DELTA,Chassis_Speed_PID,0,0,0);
}







