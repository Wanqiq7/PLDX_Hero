#include "Base_Control.h"
#include "ChassisTask.h"
#include "AppTask.h"
#include "usart1.h"
#include "timer.h"
#include "can1.h"
#include "can2.h"
#include "GimbalTask.h"
#include "usart.h"
#include "Ros_Control_Task.h"

//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;  

//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

//速度控制PID参数
float Velocity_KP=3,Velocity_KI=14; //300 300

extern float speed_x,speed_y,speed_w;		//xyz轴目标速度 单位：m/s

extern unsigned char Auto_state;					//自动模式状态标志	1为自动 0为手动

extern short Velocity_Value_3508[4];
extern float M3508_out[4];
extern float PID_M3508_I[4],PID_M3508_D_1[4],PID_M3508_D_2[4],PID_M3508_D_3[4];
extern float M3508_erro[4];
extern float M3508_target[4];
extern float M3508_out[4];
extern float M3508_target_set[4];
extern float M3508_erro[4];
extern float M3508_len;
//extern unsigned char Auto_state=0;					//自动模式状态标志 1:自动  0：手动   
extern char aim_test;		//自瞄测试
extern int game_state;
extern unsigned char aim_auto;

float Big_top_yaw_angle;
float vx_chassis;
float vy_chassis;
float Ros_Chassis_angle_speed_Low_pass_filtering_1;
float Ros_Chassis_angle_speed_Low_pass_filtering_2;
float Ros_Chassis_360_speed_erro;
float Ros_Chassis_360_speed_line;
double Ros_Chassis_angle_speed;
int Ros_Chassis_360_speed_target;


/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}



/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (short Now_Speed,float Target)
{ 	
	 static float Bias,Speed,Last_bias;
	 Bias=Target-Now_Speed; //Calculate the deviation //计算偏差
	 Speed+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Speed>16384)Speed=16384;
	 if(Speed<-16384)Speed=-16384;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Speed;    
}
int Incremental_PI_B (short Now_Speed,float Target)
{ 	
	 static float Bias,Speed,Last_bias;
	 Bias=Target-Now_Speed; //Calculate the deviation //计算偏差
	 Speed+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Speed>16384)Speed=16384;
	 if(Speed<-16384)Speed=-16384;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Speed;    
}
int Incremental_PI_C (short Now_Speed,float Target)
{ 	
	 static float Bias,Speed,Last_bias;
	 Bias=Target-Now_Speed; //Calculate the deviation //计算偏差
	 Speed+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Speed>16384)Speed=16384;
	 if(Speed<-16384)Speed=-16384;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Speed;    
}
int Incremental_PI_D (short Now_Speed,float Target)
{ 	
	 static float Bias,Speed,Last_bias;
	 Bias=Target-Now_Speed; //Calculate the deviation //计算偏差
	 Speed+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Speed>16384)Speed=16384;
	 if(Speed<-16384)Speed=-16384;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Speed;    
}



/**********************************************************************************、
Ros_Gyro_360_motion(int Center_adjustment_value,float Vx,float Vy,float Vz)
形式参数：
1.底盘正前方向校准值，范围0~8191;
作者：
时间：2024年9月30日
功能：
1.实现陀螺运动

功能实现：
1.运动向量合
2.旋转向量坡度化，减少卡位时底盘打滑摩擦摩擦产生能量热消耗
**********************************************************************************/

void Ros_Gyro_360_motion(int Center_adjustment_value,float Vx,float Vy,float Vz)
{
		Position_Value_6020[1]	= Position_Value_6020[1] + Center_adjustment_value;//
		if(Position_Value_6020[1] > 8191)
			{	
			Position_Value_6020[1] = Position_Value_6020[1] - 8191;
			}	
		Ros_Chassis_360_speed_target =310;//310
		
		M3508_len = 40;
		Ros_Chassis_angle_speed = (float)(Velocity_Value_6020[1]*7.8f) - gyro_souce[2]/10 ;
			
		Ros_Chassis_angle_speed_Low_pass_filtering_1 = Ros_Chassis_angle_speed;		
		Ros_Chassis_angle_speed = Ros_Chassis_angle_speed_Low_pass_filtering_1 * 0.05f + Ros_Chassis_angle_speed_Low_pass_filtering_2 * (1-0.05f);
		Ros_Chassis_angle_speed_Low_pass_filtering_2	= Ros_Chassis_angle_speed;
		//printf("data1=%f,data2=%d,\r\n",Chassis_angle_speed,-Velocity_Value_6020[1]);
		Ros_Chassis_360_speed_erro 	= Ros_Chassis_360_speed_target - Ros_Chassis_angle_speed;

		Big_top_yaw_angle = 3.14159*((8191-Position_Value_6020[1])/22.755)/180;//底盘和云台夹角
		//CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);	
		vx_chassis = (cos(Big_top_yaw_angle)*Vx) + (sin(Big_top_yaw_angle)*Vy);
		vy_chassis = (cos(Big_top_yaw_angle)*Vy) - (sin(Big_top_yaw_angle)*Vx);//解算到底盘x,y方向的速度
			
}
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度m/s
返回  值：无
**************************************************************************/

float MECWHEEL_R=0.076;	//麦克纳姆轮半径	（单位：m）
float MECWHEEL_PI=3.14;	//PI常数
float MECWHEEL_RATIO=19;	//电机减速比
float MECWHEEL_K=0.345;	//以车为中心点，电机坐标xy绝对值之和（单位：m）
//float p;

void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	  
		Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
					
		Vx=smooth_control.VX;     
		Vy=smooth_control.VY;
		Vz=smooth_control.VZ;//获取平滑处理后的数据	云台坐标系速度
	
		//Vx = 0.4;
	  
		Ros_Gyro_360_motion(5750,Vx,Vy,Vz);//转换到底盘坐标系
	
	
		Vx = vx_chassis;
		Vy = vy_chassis;
		Vz = Vz;              //底盘坐标系解算
	
	  
		//Inverse kinematics //运动学逆解//麦克纳姆轮小车
		MOTOR_A.Target  = +Vy+Vx-Vz*MECWHEEL_K;
		MOTOR_B.Target  = -Vy+Vx-Vz*MECWHEEL_K;
		MOTOR_C.Target  = +Vy+Vx+Vz*MECWHEEL_K;
		MOTOR_D.Target  = -Vy+Vx+Vz*MECWHEEL_K;
		
		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅  目标线速度m/s
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude);
	
	
		MOTOR_A.Target  = MOTOR_A.Target/(2.0f*MECWHEEL_PI*MECWHEEL_R);
		MOTOR_B.Target  = MOTOR_B.Target/(2.0f*MECWHEEL_PI*MECWHEEL_R);
		MOTOR_C.Target  = MOTOR_C.Target/(2.0f*MECWHEEL_PI*MECWHEEL_R);
		MOTOR_D.Target  = MOTOR_D.Target/(2.0f*MECWHEEL_PI*MECWHEEL_R);//转换为轮子的速度(弧度/s)
		
		MOTOR_A.Target =MOTOR_A.Target*MECWHEEL_RATIO*60;    //转换为电机速度　单位　RPM  期望值
		MOTOR_B.Target =MOTOR_B.Target*MECWHEEL_RATIO*60;
		MOTOR_C.Target =MOTOR_C.Target*MECWHEEL_RATIO*60;
		MOTOR_D.Target =MOTOR_D.Target*MECWHEEL_RATIO*60;

//		MOTOR_A.Motor_Out = Incremental_PI_A(Velocity_Value_3508[0],MOTOR_A.Target);
//		MOTOR_B.Motor_Out = Incremental_PI_B(Velocity_Value_3508[1],MOTOR_B.Target);
//		MOTOR_C.Motor_Out = Incremental_PI_C(Velocity_Value_3508[2],MOTOR_C.Target);
//		MOTOR_D.Motor_Out = Incremental_PI_D(Velocity_Value_3508[3],MOTOR_D.Target);
		
		M3508_target_set[0]=-MOTOR_C.Target;
		M3508_target_set[1]=MOTOR_B.Target;
		M3508_target_set[2]=MOTOR_A.Target;
		M3508_target_set[3]=-MOTOR_D.Target;
		
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
		//CAN1_Send_queue_add(0x200,0x08,-1000,1000,1000,-1000);
}



/**********************************************************************************、
任务名：Chassis_task(void *p_arg)
形式参数：无
作者：cwq
时间：2024年9月30日
功能：ROS端控制机器人底盘动作
任务频率：100FPS
**********************************************************************************/
void Ros_Control_Task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{		
		if(RC_Ctl.rc.s1 == 1)	//左拨杆上拨(自瞄测试)
		{
			
		}
		
		else if(RC_Ctl.rc.s1 == 2)			//左拨杆杆下拨，切换为自动 
		{
				Drive_Motor(speed_x,speed_y,0  );//ROS接管底盘控制
				//Auto_state= 1;
		}

		else if(RC_Ctl.rc.s1 == 3)	//左杆回中，手动，模式二：云台跟随底盘
		{
			
		}
		else 	//模式四：丢控保护，底盘无力
		{
			CAN1_Send_queue_add(0x200,0x08,0,0,0,0);//发送无力指令
			speed_x = 0;
			speed_y = 0;
			speed_w = 0;

		}
	
		game_state = 3;
		data_transition();
		USART1_SEND();
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //延时2、10ms
	}
}
