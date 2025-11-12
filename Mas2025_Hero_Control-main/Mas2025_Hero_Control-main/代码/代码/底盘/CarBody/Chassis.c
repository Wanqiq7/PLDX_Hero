#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "PID.h"
#include "Parameter.h"
#include "RefereeSystem.h"
#include "Delay.h"
#include "arm_math.h"
#include "M3508.h"
#include "Chassis.h"
#include "Remote.h"
#include "GM6020.h"
#include "Ultra_CAP.h"
#include "AttitudeAlgorithms.h"

/*=============================================舵向=============================================*/
#define Chassis_SteerPowerProportion		0.75f

float SteerPower;
float Chassis_SteerCurrent;
float Steer_EstimatedPower;

void Chassis_SteerInit(void)
{
	//DM_G6220_Init();
}

void Chassis_SteerLQRControl(float LeftFrontTheta,float RightFrontTheta,float LeftRearTheta,float RightRearTheta)
{
	#define LQR_K1		3.1623f
	#define LQR_K2		0.0753f//Q=diag(0.5 0.0003) R=0.5//3.1623    0.0753
	
	#define LQR_K3		3.1623f
	#define LQR_K4		0.0753f//Q=diag(0.5 0.001) R=0.5
	
	#define LQR_K5		3.1623f
	#define LQR_K6		0.0753f//Q=diag(0.5 0.0005) R=0.5
	
	#define LQR_K7		3.1623f
	#define LQR_K8		0.0753f//Q=diag(0.4 0.0002) R=0.5//0.8944    0.0441
	
	#define TauLimit	2.0f
	#define Power_K		2.5f
	
	float tau[4]={0};
//	float AllSteerPowerLimit=RefereeSystem_Ref*Chassis_SteerPowerProportion;
	float SingleSteerPowerLimit=RefereeSystem_Ref*Chassis_SteerPowerProportion*0.25f;
	
	tau[Chassis_LeftFrontSteer-0x205]=LQR_K1*(LeftFrontTheta-GM6020_MotorStatus1[Chassis_LeftFrontSteer-0x205].Position)-LQR_K2*GM6020_MotorStatus1[Chassis_LeftFrontSteer-0x205].Speed;
	if(tau[Chassis_LeftFrontSteer-0x205]>TauLimit)tau[Chassis_LeftFrontSteer-0x205]=TauLimit;
	if(tau[Chassis_LeftFrontSteer-0x205]<-TauLimit)tau[Chassis_LeftFrontSteer-0x205]=-TauLimit;
	float LeftFrontSteerPower=Power_K*fabs(1.732050807568877f*GM6020_MotorStatus1[Chassis_LeftFrontSteer-0x205].Speed*tau[Chassis_LeftFrontSteer-0x205]);
	if(LeftFrontSteerPower>SingleSteerPowerLimit)tau[Chassis_LeftFrontSteer-0x205]*=SingleSteerPowerLimit/LeftFrontSteerPower;
	
	tau[Chassis_RightFrontSteer-0x205]=LQR_K3*(RightFrontTheta-GM6020_MotorStatus1[Chassis_RightFrontSteer-0x205].Position)-LQR_K4*GM6020_MotorStatus1[Chassis_RightFrontSteer-0x205].Speed;
	if(tau[Chassis_RightFrontSteer-0x205]>TauLimit)tau[Chassis_RightFrontSteer-0x205]=TauLimit;
	if(tau[Chassis_RightFrontSteer-0x205]<-TauLimit)tau[Chassis_RightFrontSteer-0x205]=-TauLimit;
	float RightFrontSteerPower=Power_K*fabs(1.732050807568877f*GM6020_MotorStatus1[Chassis_RightFrontSteer-0x205].Speed*tau[Chassis_RightFrontSteer-0x205]);
	if(RightFrontSteerPower>SingleSteerPowerLimit)tau[Chassis_RightFrontSteer-0x205]*=SingleSteerPowerLimit/RightFrontSteerPower;

	tau[Chassis_LeftRearSteer-0x205]=LQR_K5*(LeftRearTheta-GM6020_MotorStatus1[Chassis_LeftRearSteer-0x205].Position)-LQR_K6*GM6020_MotorStatus1[Chassis_LeftRearSteer-0x205].Speed;
	if(tau[Chassis_LeftRearSteer-0x205]>TauLimit)tau[Chassis_LeftRearSteer-0x205]=TauLimit;
	if(tau[Chassis_LeftRearSteer-0x205]<-TauLimit)tau[Chassis_LeftRearSteer-0x205]=-TauLimit;
	float LeftRearSteerPower=Power_K*fabs(1.732050807568877f*GM6020_MotorStatus1[Chassis_LeftRearSteer-0x205].Speed*tau[Chassis_LeftRearSteer-0x205]);
	if(LeftRearSteerPower>SingleSteerPowerLimit)tau[Chassis_LeftRearSteer-0x205]*=SingleSteerPowerLimit/LeftRearSteerPower;

	tau[Chassis_RightRearSteer-0x205]=LQR_K7*(RightRearTheta-GM6020_MotorStatus1[Chassis_RightRearSteer-0x205].Position)-LQR_K8*GM6020_MotorStatus1[Chassis_RightRearSteer-0x205].Speed;
	if(tau[Chassis_RightRearSteer-0x205]>TauLimit)tau[Chassis_RightRearSteer-0x205]=TauLimit;
	if(tau[Chassis_RightRearSteer-0x205]<-TauLimit)tau[Chassis_RightRearSteer-0x205]=-TauLimit;
	float RightRearSteerPower=Power_K*fabs(1.732050807568877f*GM6020_MotorStatus1[Chassis_RightRearSteer-0x205].Speed*tau[Chassis_RightRearSteer-0x205]);
	if(RightRearSteerPower>SingleSteerPowerLimit)tau[Chassis_RightRearSteer-0x205]*=SingleSteerPowerLimit/RightRearSteerPower;

////////////////	Steer_EstimatedPower=LeftFrontSteerPower+RightFrontSteerPower+LeftRearSteerPower+RightRearSteerPower;
//	if(Steer_EstimatedPower>AllSteerPowerLimit)
//		for(uint8_t i=0;i<4;i++)
//			tau[i]*=AllSteerPowerLimit/Steer_EstimatedPower;
	
	float Current[4];
	Current[Chassis_LeftFrontSteer-0x205]=tau[Chassis_LeftFrontSteer-0x205]/0.741f/3.0f*16384.0f;
	if(Current[Chassis_LeftFrontSteer-0x205]>16384)Current[Chassis_LeftFrontSteer-0x205]=16384;
	if(Current[Chassis_LeftFrontSteer-0x205]<-16384)Current[Chassis_LeftFrontSteer-0x205]=-16384;

	Current[Chassis_RightFrontSteer-0x205]=tau[Chassis_RightFrontSteer-0x205]/0.741f/3.0f*16384.0f;
	if(Current[Chassis_RightFrontSteer-0x205]>16384)Current[Chassis_RightFrontSteer-0x205]=16384;
	if(Current[Chassis_RightFrontSteer-0x205]<-16384)Current[Chassis_RightFrontSteer-0x205]=-16384;

	Current[Chassis_LeftRearSteer-0x205]=tau[Chassis_LeftRearSteer-0x205]/0.741f/3.0f*16384.0f;
	if(Current[Chassis_LeftRearSteer-0x205]>16384)Current[Chassis_LeftRearSteer-0x205]=16384;
	if(Current[Chassis_LeftRearSteer-0x205]<-16384)Current[Chassis_LeftRearSteer-0x205]=-16384;

	Current[Chassis_RightRearSteer-0x205]=tau[Chassis_RightRearSteer-0x205]/0.741f/3.0f*16384.0f;
	if(Current[Chassis_RightRearSteer-0x205]>16384)Current[Chassis_RightRearSteer-0x205]=16384;
	if(Current[Chassis_RightRearSteer-0x205]<-16384)Current[Chassis_RightRearSteer-0x205]=-16384;

	GM6020_CAN1SetLIDCurrent(Current[Chassis_RightFrontSteer-0x205],Current[Chassis_LeftFrontSteer-0x205],Current[Chassis_LeftRearSteer-0x205],Current[Chassis_RightRearSteer-0x205]);
	//GM6020_CAN1SetLIDCurrent(0,0,0,0);
}

float Chassis_SteerGetAngle(float vx,float vy,float Last_SteerAngle,float *r)
{
	float SteerAngle;
	
	if(vy>0)SteerAngle=atanf(vx/vy);
	else if(vy==0)
	{
		if(vx>0)SteerAngle=PI/2.0f;
		else if(vx==0)SteerAngle=0;
		else if(vx<0)SteerAngle=-PI/2.0f;
	}
	else if(vy<0)
	{
		if(vx>=0)SteerAngle=atanf(vx/vy)+PI;
		else if(vx<0)SteerAngle=atanf(vx/vy)-PI;
	}
	SteerAngle+=2.0f*PI*(*r);
	
	float Delta_NowAndLast=fabs(SteerAngle-Last_SteerAngle);
	float Delta_NowPlusOneRoundAndLast=fabs(SteerAngle+2.0f*PI-Last_SteerAngle);
	float Delta_NowMinusOneRoundAndLast=fabs(SteerAngle-2.0f*PI-Last_SteerAngle);
	
	float Delta_MIN=Delta_NowAndLast,dr=0;
	if(Delta_MIN>Delta_NowPlusOneRoundAndLast){Delta_MIN=Delta_NowPlusOneRoundAndLast;dr=1;}
	if(Delta_MIN>Delta_NowMinusOneRoundAndLast){Delta_MIN=Delta_NowMinusOneRoundAndLast;dr=-1;}
	(*r)=(*r)+dr;
	SteerAngle+=2.0f*PI*dr;
	
	return SteerAngle;
}

void Chassis_SteerGetAnglePlus(float vx,float vy,float ThetaNow,float *ThetaTarget,float *MotorSign)
{
	if(vx==0 && vy==0)return;
	
	float theta_Target=-atan2(vy,vx)+PI/2.0f;
	if(theta_Target<0)theta_Target+=2.0f*PI;
	
	float theta_e=theta_Target-ThetaNow;
	theta_e=fmodf(theta_e,2.0f*PI);
	if(theta_e<-PI)theta_e+=2.0f*PI;
	if(theta_e>PI)theta_e-=2.0f*PI;
	
	if(theta_e>PI/2.0f)
	{
		(*ThetaTarget)=ThetaNow+theta_e-PI;
		(*MotorSign)=-1;
	}
	else if(theta_e<-PI/2.0f)
	{
		(*ThetaTarget)=ThetaNow+theta_e+PI;
		(*MotorSign)=-1;
	}
	else
	{
		(*ThetaTarget)=ThetaNow+theta_e;
		(*MotorSign)=1;
	}
}

void Chassis_SteerInverseMotionControl(float vx,float vy,float w,float PowerLimit)
{
	static float Last_thetaTarget[4]={0},theta_Target[4]={0},r[4]={0};

	Last_thetaTarget[Chassis_LeftFrontSteer-0x205]=theta_Target[Chassis_LeftFrontSteer-0x205];
	Last_thetaTarget[Chassis_RightFrontSteer-0x205]=theta_Target[Chassis_RightFrontSteer-0x205];
	Last_thetaTarget[Chassis_LeftRearSteer-0x205]=theta_Target[Chassis_LeftRearSteer-0x205];
	Last_thetaTarget[Chassis_RightRearSteer-0x205]=theta_Target[Chassis_RightRearSteer-0x205];
	
	float LeftFront_vx=vx-w*Chassis_ry,LeftFront_vy=vy-w*Chassis_rx;
	float RightFront_vx=vx-w*Chassis_ry,RightFront_vy=vy+w*Chassis_rx;
	float LeftRear_vx=vx+w*Chassis_ry,LeftRear_vy=vy-w*Chassis_rx;
	float RightRear_vx=vx+w*Chassis_ry,RightRear_vy=vy+w*Chassis_rx;
	theta_Target[Chassis_LeftFrontSteer-0x205]=Chassis_SteerGetAngle(LeftFront_vx,LeftFront_vy,Last_thetaTarget[Chassis_LeftFrontSteer-0x205],&r[Chassis_LeftFrontSteer-0x205]);
	theta_Target[Chassis_RightFrontSteer-0x205]=Chassis_SteerGetAngle(RightFront_vx,RightFront_vy,Last_thetaTarget[Chassis_RightFrontSteer-0x205],&r[Chassis_RightFrontSteer-0x205]);
	theta_Target[Chassis_LeftRearSteer-0x205]=Chassis_SteerGetAngle(LeftRear_vx,LeftRear_vy,Last_thetaTarget[Chassis_LeftRearSteer-0x205],&r[Chassis_LeftRearSteer-0x205]);
	theta_Target[Chassis_RightRearSteer-0x205]=Chassis_SteerGetAngle(RightRear_vx,RightRear_vy,Last_thetaTarget[Chassis_RightRearSteer-0x205],&r[Chassis_RightRearSteer-0x205]);
	
	Chassis_SteerLQRControl(theta_Target[Chassis_LeftFrontSteer-0x205],theta_Target[Chassis_RightFrontSteer-0x205],theta_Target[Chassis_LeftRearSteer-0x205],theta_Target[Chassis_RightRearSteer-0x205]);
}

/*=============================================轮向=============================================*/
PID_PositionInitTypedef Chassis_WheelSpeedPID[4];//底盘四个电机的转速PID
PID_PositionInitTypedef Chassis_WheelTrackPID;//底盘跟随PID
float Chassis_WheelCurrent,Chassis_WheelCurrentLimit=16384*4.0f;//底盘电流总和,底盘电流限幅
float Chassis_WheelPowerLimit;//功率控制功率上限
float Mecanum_EstimatedPower;

/*
 *函数简介:麦轮初始化
 *参数说明:无
 *返回类型:无
 *备注:即四个转速PID的初始化
 *备注:四个电机的ID参照上方的宏定义,最好使用M3508电机的ID1~4,否则下方电机控制的代码需要修改
 */
void Chassis_WheelInit(void)
{
//	PID_PositionStructureInit(&Chassis_WheelSpeedPID[0],0);//左前轮
//	PID_PositionSetParameter(&Chassis_WheelSpeedPID[0],0.8,0,0);
//	PID_PositionSetEkRange(&Chassis_WheelSpeedPID[0],-5,5);
//	PID_PositionSetOUTRange(&Chassis_WheelSpeedPID[0],-15000,15000);
//	
//	PID_PositionStructureInit(&Chassis_WheelSpeedPID[1],0);//右前轮
//	PID_PositionSetParameter(&Chassis_WheelSpeedPID[1],0.8,0,0);
//	PID_PositionSetEkRange(&Chassis_WheelSpeedPID[1],-5,5);
//	PID_PositionSetOUTRange(&Chassis_WheelSpeedPID[1],-15000,15000);
//	
//	PID_PositionStructureInit(&Chassis_WheelSpeedPID[2],0);//左后轮
//	PID_PositionSetParameter(&Chassis_WheelSpeedPID[2],0.8,0,0);
//	PID_PositionSetEkRange(&Chassis_WheelSpeedPID[2],-5,5);
//	PID_PositionSetOUTRange(&Chassis_WheelSpeedPID[2],-15000,15000);
//	
//	PID_PositionStructureInit(&Chassis_WheelSpeedPID[3],0);//右后轮
//	PID_PositionSetParameter(&Chassis_WheelSpeedPID[3],0.8,0,0);
//	PID_PositionSetEkRange(&Chassis_WheelSpeedPID[3],-5,5);
//	PID_PositionSetOUTRange(&Chassis_WheelSpeedPID[3],-15000,15000);
	
	PID_PositionStructureInit(&Chassis_WheelTrackPID,Yaw_GM6020PositionValue);//底盘跟随
	PID_PositionSetParameter(&Chassis_WheelTrackPID,0.004,0,0);
	PID_PositionSetEkRange(&Chassis_WheelTrackPID,-5,5);
	PID_PositionSetOUTRange(&Chassis_WheelTrackPID,-4,4);
}

/*
 *函数简介:麦轮PID清理
 *参数说明:无
 *返回类型:无
 *备注:清理四个转速位置式PID
 */
void Chassis_WheelCleanPID(void)
{
//	PID_PositionClean(&Chassis_WheelSpeedPID[0]);//左前轮
//	PID_PositionClean(&Chassis_WheelSpeedPID[1]);//右前轮
//	PID_PositionClean(&Chassis_WheelSpeedPID[2]);//左后轮
//	PID_PositionClean(&Chassis_WheelSpeedPID[3]);//右后轮
	PID_PositionClean(&Chassis_WheelTrackPID);//底盘跟随
}

float Mecanum_Pitch=0;
/*
 *函数简介:麦轮速度控制
 *参数说明:左前轮速度
 *参数说明:右前轮速度
 *参数说明:左后轮速度
 *参数说明:右后轮速度
 *返回类型:无
 *备注:单独控制四个轮子的速度
 */
void Chassis_WheelSpeedControl(int16_t LeftFrontSpeed,int16_t RightFrontSpeed,int16_t LeftRearSpeed,int16_t RightRearSpeed)
{
	#define LQR_K		0.0224f
	
	float LeftFrontTau=LQR_K*(LeftFrontSpeed/60.0f*2.0f*PI-M3508_MotorStatus[Chassis_LeftFrontWheel-0x201].RotorSpeed/60.0f*2.0f*PI);
	float RightFrontTau=LQR_K*(RightFrontSpeed/60.0f*2.0f*PI-M3508_MotorStatus[Chassis_RightFrontWheel-0x201].RotorSpeed/60.0f*2.0f*PI);
	float LeftRearTau=LQR_K*(LeftRearSpeed/60.0f*2.0f*PI-M3508_MotorStatus[Chassis_LeftRearWheel-0x201].RotorSpeed/60.0f*2.0f*PI);
	float RightRearTau=LQR_K*(RightRearSpeed/60.0f*2.0f*PI-M3508_MotorStatus[Chassis_RightRearWheel-0x201].RotorSpeed/60.0f*2.0f*PI);

	float LeftFrontCurrent=(LeftFrontTau)/0.3f;
	if(LeftFrontCurrent>20)LeftFrontCurrent=20;if(LeftFrontCurrent<-20)LeftFrontCurrent=-20;
	float RightFrontCurrent=(RightFrontTau)/0.3f;
	if(RightFrontCurrent>20)RightFrontCurrent=20;if(RightFrontCurrent<-20)RightFrontCurrent=-20;
	float LeftRearCurrent=(LeftRearTau)/0.3f;
	if(LeftRearCurrent>20)LeftRearCurrent=20;if(LeftRearCurrent<-20)LeftRearCurrent=-20;
	float RightRearCurrent=(RightRearTau)/0.3f;
	if(RightRearCurrent>20)RightRearCurrent=20;if(RightRearCurrent<-20)RightRearCurrent=-20;
	
	Mecanum_Pitch=AttitudeAlgorithms_DegPitch;
	
	float LeftFrontPower=M3508_MotorStatus[Chassis_LeftFrontWheel-0x201].ShaftSpeed*LeftFrontCurrent*0.031413612565445f;
	float RightFrontPower=M3508_MotorStatus[Chassis_RightFrontWheel-0x201].ShaftSpeed*RightFrontCurrent*0.031413612565445f;
	float LeftRearPower=M3508_MotorStatus[Chassis_LeftRearWheel-0x201].ShaftSpeed*LeftRearCurrent*0.031413612565445f;
	float RightRearPower=M3508_MotorStatus[Chassis_RightRearWheel-0x201].ShaftSpeed*RightRearCurrent*0.031413612565445f;
	float Mecanum_FrontEstimatedPower=1.0f*1.732050807568877f*(fabs(LeftFrontPower)+fabs(RightFrontPower));
	float Mecanum_RearEstimatedPower=1.0f*1.732050807568877f*(fabs(LeftRearPower)+fabs(RightRearPower));
	Mecanum_EstimatedPower=Mecanum_FrontEstimatedPower+Mecanum_RearEstimatedPower;
	
//	if(Mecanum_EstimatedPower>(Chassis_WheelPowerLimit*1.0f))
//	{
//		LeftFrontCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
//		RightFrontCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
//		LeftRearCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
//		RightRearCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
//	}
	if(Mecanum_Pitch>10.0f)
	{
		if(Mecanum_FrontEstimatedPower>(Chassis_WheelPowerLimit*0.2f))
		{
			LeftFrontCurrent*=((Chassis_WheelPowerLimit*0.2f)/Mecanum_FrontEstimatedPower);
			RightFrontCurrent*=((Chassis_WheelPowerLimit*0.2f)/Mecanum_FrontEstimatedPower);
		}
		if(Mecanum_RearEstimatedPower>(Chassis_WheelPowerLimit*0.8f))
		{
			LeftRearCurrent*=((Chassis_WheelPowerLimit*0.8f)/Mecanum_RearEstimatedPower);
			RightRearCurrent*=((Chassis_WheelPowerLimit*0.8f)/Mecanum_RearEstimatedPower);
		}
	}
	else if(Mecanum_Pitch<-10.0f)
	{
		if(Mecanum_FrontEstimatedPower>(Chassis_WheelPowerLimit*0.8f))
		{
			LeftFrontCurrent*=((Chassis_WheelPowerLimit*0.8f)/Mecanum_FrontEstimatedPower);
			RightFrontCurrent*=((Chassis_WheelPowerLimit*0.8f)/Mecanum_FrontEstimatedPower);
		}
		if(Mecanum_RearEstimatedPower>(Chassis_WheelPowerLimit*0.2f))
		{
			LeftRearCurrent*=((Chassis_WheelPowerLimit*0.2f)/Mecanum_RearEstimatedPower);
			RightRearCurrent*=((Chassis_WheelPowerLimit*0.2f)/Mecanum_RearEstimatedPower);
		}
	}
	else
	{
		if(Mecanum_EstimatedPower>(Chassis_WheelPowerLimit*1.0f))
		{
			LeftFrontCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
			RightFrontCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
			LeftRearCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
			RightRearCurrent*=((Chassis_WheelPowerLimit*1.0f)/Mecanum_EstimatedPower);
		}
	}
	
	LeftFrontCurrent=LeftFrontCurrent/20.0f*16384.0f;
	RightFrontCurrent=RightFrontCurrent/20.0f*16384.0f;
	LeftRearCurrent=LeftRearCurrent/20.0f*16384.0f;
	RightRearCurrent=RightRearCurrent/20.0f*16384.0f;
	
	M3508_CANSetLIDCurrent(RightFrontCurrent,LeftFrontCurrent,LeftRearCurrent,RightRearCurrent);//M3508控制
}

/*
 *函数简介:麦轮逆运动解算
 *参数说明:x轴速度,单位m/s(以前为正)
 *参数说明:y轴速度,单位m/s(以左为正)
 *参数说明:z轴转速,单位rad/s(以逆时针为正)
 *返回类型:无
 *备注:速度转速转换系数:
 *	   w'=v/R (rad/s)=v/(2Π×R) (r/s)=60×v/(2Π×R) (r/min)=1/19 w
 *	   ⇒ w=19×60×v/(2Π×R/100)=19×60×100×v/(2Π×R)=18143.663512×v/R,R单位cm
 *备注:麦轮半径参数在上方宏定义Mecanum_WheelRadius修改,默认7cm,转换参数2591.95
 *备注:底盘中心到轮子中心的距离由上方宏定义x轴分量Mecanum_rx和y轴分量Mecanum_ry决定
 */
void Chassis_WheelInverseMotionControl(float vx,float vy,float w)
{
	//逆运动解算
	float LeftFront_vx=vx-w*Chassis_ry,LeftFront_vy=vy-w*Chassis_rx,LeftFront_v=LeftFront_vx*LeftFront_vx+LeftFront_vy*LeftFront_vy;
	float RightFront_vx=vx-w*Chassis_ry,RightFront_vy=vy+w*Chassis_rx,RightFront_v=RightFront_vx*RightFront_vx+RightFront_vy*RightFront_vy;
	float LeftRear_vx=vx+w*Chassis_ry,LeftRear_vy=vy-w*Chassis_rx,LeftRear_v=LeftRear_vx*LeftRear_vx+LeftRear_vy*LeftRear_vy;
	float RightRear_vx=vx+w*Chassis_ry,RightRear_vy=vy+w*Chassis_rx,RightRear_v=RightRear_vx*RightRear_vx+RightRear_vy*RightRear_vy;
	arm_sqrt_f32(LeftFront_v,&LeftFront_v);
	arm_sqrt_f32(RightFront_v,&RightFront_v);
	arm_sqrt_f32(LeftRear_v,&LeftRear_v);
	arm_sqrt_f32(RightRear_v,&RightRear_v);
	int16_t LeftFrontSpeed=(int16_t)(LeftFront_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//左前轮
	int16_t RightFrontSpeed=(int16_t)(RightFront_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//右前轮
	int16_t LeftRearSpeed=(int16_t)(LeftRear_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//左后轮
	int16_t RightRearSpeed=(int16_t)(RightRear_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//右后轮
	
	Chassis_WheelSpeedControl(-LeftFrontSpeed,RightFrontSpeed,-LeftRearSpeed,RightRearSpeed);//M3508速度控制
}

/*=============================================底盘=============================================*/
#define Chassis_PowerControlSpeedNormalizationValue			3.0f//功率控制归一化速度标准值,范围[0,3]
#define Chassis_PowerControlGainCoefficientInitialValue		0.5f//功率控制增益系数初始值
#define Chassis_PowerControl_T								5.0f//功率控制周期(T=Mecanum_PowerControl_T*2ms)
#define Chassis_PowerControl_UseBuffer						30.0f//功率控制消耗的缓冲能量
#define Chassis_PowerControl_PowerMax						4.0f//功率控制功率上限与裁判系统功率上限比值上限
#define Chassis_PowerControl_UltraCAPPower					100.0f//使用超电增加功率

#define Chassis_TrackLimit									200//200 <-> 10deg左右

PID_PositionInitTypedef Chassis_TrackPID;//底盘跟随PID
float Chassis_YawTheta;//底盘云台相对偏航角度
uint8_t Chassis_GyroScopeFlag;//底盘小陀螺标志位
float Chassis_WheelPower;//底盘功率(软件计算值)
float Chassis_PowerLimit;//功率控制功率上限

void Chassis_Init(void)
{
	PID_PositionStructureInit(&Chassis_TrackPID,Yaw_GM6020PositionValue);//底盘跟随
	PID_PositionSetParameter(&Chassis_TrackPID,0.004,0.0000005,0);
	PID_PositionSetEkRange(&Chassis_TrackPID,-5,5);
	PID_PositionSetEkSumRange(&Chassis_TrackPID,-500,500);
	PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
	
	Chassis_SteerInit();
	Chassis_WheelInit();
}

/*
 *函数简介:麦轮功率控制
 *参数说明:无
 *返回类型:无
 *备注:麦轮是否开启可功率控制模式决定于上方宏定义Mecanum_PowerControl
 *备注:以Mecanum_PowerControl_T*2ms为控制周期,取底盘平均功率(通过M3508反馈数据计算),通过速度增益系数和速度归一化控制速度大小,速度增益系数在[0,2]范围内
 *备注:速度归一化的标准值在上方宏定义Mecanum_PowerControlSpeedNormalizationValue修改,范围[0,3]
 *备注:增益系数的初始值在上方宏定义Mecanum_PowerControlGainCoefficientInitialValue修改
 *备注:设定功率上限最高为150W限幅
 *备注:速度转速转换系数:
 *	   w'=v/R (rad/s)=v/(2Π×R) (r/s)=60×v/(2Π×R) (r/min)=1/19 w
 *	   ⇒ w=19×60×v/(2Π×R/100)=19×60×100×v/(2Π×R)=18143.663512×v/R,R单位cm
 *备注:麦轮半径参数在上方宏定义Mecanum_WheelRadius修改,默认7cm,转换参数2591.95
 *备注:底盘中心到轮子中心的距离由上方宏定义x轴分量Mecanum_rx和y轴分量Mecanum_ry决定
 */
void Chassis_Control(void)
{
	static uint8_t Chassis_GyroScopeCloseFlag=0;//小陀螺待关闭标志位
	static float GainCoefficient=1.0f;//速度增益系数
	static uint8_t Count=0;//计数器,以Mecanum_PowerControl_T*2ms为控制周期

	static float Last_thetaTarget[4]={0},theta_Target[4]={0},r[4]={0};
	
	static uint8_t BushuModel_Flag=0;
	static uint8_t Last_Remote_LS=0;
	if(Last_Remote_LS==3 && Remote_RxData.Remote_LS==2)BushuModel_Flag=!BushuModel_Flag;
	Last_Remote_LS=Remote_RxData.Remote_LS;
	
	static uint8_t F_LastStatus=0;
	if(F_LastStatus==1 && Remote_RxData.Remote_Key_F==0)BushuModel_Flag=0;
	F_LastStatus=Remote_RxData.Remote_Key_F;

	if(BushuModel_Flag==1)
	{
		Last_thetaTarget[Chassis_LeftFrontSteer-0x205]=theta_Target[Chassis_LeftFrontSteer-0x205];
		Last_thetaTarget[Chassis_RightFrontSteer-0x205]=theta_Target[Chassis_RightFrontSteer-0x205];
		Last_thetaTarget[Chassis_LeftRearSteer-0x205]=theta_Target[Chassis_LeftRearSteer-0x205];
		Last_thetaTarget[Chassis_RightRearSteer-0x205]=theta_Target[Chassis_RightRearSteer-0x205];
	
		float LeftFront_vx=-1,LeftFront_vy=1;
		float RightFront_vx=1,RightFront_vy=1;
		float LeftRear_vx=1,LeftRear_vy=1;
		float RightRear_vx=-1,RightRear_vy=1;
		theta_Target[Chassis_LeftFrontSteer-0x205]=Chassis_SteerGetAngle(LeftFront_vx,LeftFront_vy,Last_thetaTarget[Chassis_LeftFrontSteer-0x205],&r[Chassis_LeftFrontSteer-0x205]);
		theta_Target[Chassis_RightFrontSteer-0x205]=Chassis_SteerGetAngle(RightFront_vx,RightFront_vy,Last_thetaTarget[Chassis_RightFrontSteer-0x205],&r[Chassis_RightFrontSteer-0x205]);
		theta_Target[Chassis_LeftRearSteer-0x205]=Chassis_SteerGetAngle(LeftRear_vx,LeftRear_vy,Last_thetaTarget[Chassis_LeftRearSteer-0x205],&r[Chassis_LeftRearSteer-0x205]);
		theta_Target[Chassis_RightRearSteer-0x205]=Chassis_SteerGetAngle(RightRear_vx,RightRear_vy,Last_thetaTarget[Chassis_RightRearSteer-0x205],&r[Chassis_RightRearSteer-0x205]);
	
		Chassis_SteerLQRControl(theta_Target[Chassis_LeftFrontSteer-0x205],theta_Target[Chassis_RightFrontSteer-0x205],theta_Target[Chassis_LeftRearSteer-0x205],theta_Target[Chassis_RightRearSteer-0x205]);
		Chassis_WheelSpeedControl(0,0,0,0);//M3508速度控制
		return;
	}
	
	
	/*==========三轴速度获取==========*/
	float vx=Remote_RxData.Remote_R_RL-1024;
	float vy=Remote_RxData.Remote_R_UD-1024;
	float w=1024-Remote_RxData.Remote_L_RL;//获取三个轴的速度参量
	float w_track;
	
	if(Remote_Status==0)vx=vy=0;
	
	float sigma=sqrtf(vx*vx+vy*vy);//获取xy轴速度归一化系数
	if(sigma!=0)//x,y轴速度归一化(正交合成速度不变为标准值)
	{
		vx=vx/sigma*Chassis_PowerControlSpeedNormalizationValue;
		vy=vy/sigma*Chassis_PowerControlSpeedNormalizationValue;
	}
	
	int16_t Raw_Theta=Yaw_GM6020PositionValue-GM6020_MotorStatus[0].Angle;//获取底盘云台相对角度原始数据
	if(Raw_Theta<0)Raw_Theta+=8192;
	//Chassis_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Chassis_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对角度
	
	float vx_=vx,vy_=vy;
	vx=vx_*cosf(Chassis_YawTheta)-vy_*sinf(Chassis_YawTheta);
	vy=vx_*sinf(Chassis_YawTheta)+vy_*cosf(Chassis_YawTheta);//根据底盘云台相对角度修正xy轴速度
	
	/*==========小陀螺处理==========*/
	if(Yaw_GM6020PositionValue<4096 && Remote_RxData.Remote_LS!=0x02)//正向小陀螺为逆时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Chassis_GyroScopeFlag==1 && Chassis_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+500))//小陀螺模式
		{
			w=Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=1;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_RS==1 || (Chassis_GyroScopeFlag==2 && Chassis_GyroScopeCloseFlag==2 && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue+8092-500))//反向小陀螺模式(仅检录使用)
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=2;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=2;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==2 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			if(Chassis_GyroScopeCloseFlag!=0)
			{
				GM6020_MotorStatus[0].r=0;
				Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
			}
			else
			{
				PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
				PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
				w=0;
				w_track=Chassis_TrackPID.OUT;
			}
		}
		else
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-5,5);
			Chassis_GyroScopeFlag=0;
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			if(GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+Chassis_TrackLimit && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue-Chassis_TrackLimit && vx==0 && vy==0)
				w_track=0;
		}
	}
	else if(Remote_RxData.Remote_LS!=0x02)//正向小陀螺为顺时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Chassis_GyroScopeFlag==1 && Chassis_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue+8192-1000))//小陀螺模式
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=1;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			if(Chassis_GyroScopeCloseFlag!=0)
			{
				GM6020_MotorStatus[0].r=0;
				Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态				
			}
			else
			{
				PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
				PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
				w=0;
				w_track=Chassis_TrackPID.OUT;
			}
		}
		else if(Remote_RxData.Remote_RS==1 || (Chassis_GyroScopeFlag==2 && Chassis_GyroScopeCloseFlag==2 && GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+1000))//反向小陀螺模式(仅检录使用)
		{
			w=Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=2;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=2;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==2 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-5,5);
			Chassis_GyroScopeFlag=0;
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			if(GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+Chassis_TrackLimit && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue-Chassis_TrackLimit && vx==0 && vy==0)
				w_track=0;
		}
	}		
	if(Remote_RxData.Remote_Mouse_KeyR==1)w_track=0;
	
	/*==========功率上限处理==========*/
	/*由缓冲能量得到功率控制功率上限*/
	float Mecanum_PowerRef=1.0f/(60.0f-Chassis_PowerControl_UseBuffer)*RefereeSystem_Buffer;//功率增益
	if(Mecanum_PowerRef>Chassis_PowerControl_PowerMax)Mecanum_PowerRef=Chassis_PowerControl_PowerMax;
	Chassis_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;
	
	/*由运动状态约束功率控制功率上限*/
	float Scale=sigma/660.0f;
	if(Scale<1 && Scale>0)Chassis_PowerLimit*=Scale;//平移约束
	if(Chassis_GyroScopeFlag==1)Chassis_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;//小陀螺约束
	//if(Chassis_PowerLimit>150.0f)Chassis_PowerLimit=150.0f;//限幅约束
	
	/*由超电约束功率控制功率上限*/
//	if(Remote_RxData.Remote_KeyPush_Shift==1)//开启超电
//	{
//		if(Chassis_PowerLimit>0.0f)
//		{
//			if(Chassis_PowerLimit<RefereeSystem_Ref)Chassis_PowerLimit+=Chassis_PowerControl_UltraCAPPower;
//			else Chassis_PowerLimit=RefereeSystem_Ref+Chassis_PowerControl_UltraCAPPower;
//		}
//		Ultra_CAP_SetPower(RefereeSystem_Ref);
//	}
//	else
//		Ultra_CAP_SetPower(Chassis_PowerLimit);
	
	if(Chassis_PowerLimit<RefereeSystem_Ref*Chassis_SteerPowerProportion)Chassis_PowerLimit=RefereeSystem_Ref*Chassis_SteerPowerProportion;
	SteerPower=1.732050807568877f*(GM6020_MotorStatus1[Chassis_LeftFrontSteer-0x205].Power \
			   +GM6020_MotorStatus1[Chassis_RightFrontSteer-0x205].Power \
			   +GM6020_MotorStatus1[Chassis_LeftRearSteer-0x205].Power \
			   +GM6020_MotorStatus1[Chassis_RightRearSteer-0x205].Power);
	Chassis_WheelPowerLimit=Chassis_PowerLimit-SteerPower;
	if(Chassis_WheelPowerLimit<0)Chassis_WheelPowerLimit=0;
	
	/*==========功率控制==========*/
	static float PowerSum=0;//控制周期内的功率积分
	Chassis_WheelPower=1.0f*1.732050807568877f*(M3508_MotorStatus[Chassis_LeftFrontWheel-0x201].Power \
					   +M3508_MotorStatus[Chassis_RightFrontWheel-0x201].Power \
					   +M3508_MotorStatus[Chassis_LeftRearWheel-0x201].Power \
					   +M3508_MotorStatus[Chassis_RightRearWheel-0x201].Power);//获取底盘功率
	PowerSum+=Chassis_WheelPower;
	
	/*速度增益*/
	if(Count==Chassis_PowerControl_T)//一个控制周期
	{
		Count=0;//计数器清零
		float Power_avg=PowerSum/Chassis_PowerControl_T;
		PowerSum=0;
		
		if(Power_avg<Chassis_WheelPowerLimit/2.0f)
		{
			GainCoefficient+=0.05f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg<Chassis_WheelPowerLimit-5)
		{
			GainCoefficient+=0.02f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg>Chassis_WheelPowerLimit+5)
		{
			if(RefereeSystem_Buffer<60.0f-Chassis_PowerControl_UseBuffer)GainCoefficient-=0.05f;//增益系数递增
			else GainCoefficient-=0.02f;//增益系数递增
			if(GainCoefficient<Chassis_PowerControlGainCoefficientInitialValue/2.0f)GainCoefficient=Chassis_PowerControlGainCoefficientInitialValue/2.0f;//增益系数限下幅
		}
	}
	else if(vx!=0 || vy!=0 || Chassis_GyroScopeFlag==1)//正常运动
		Count++;
	else//停止运动
		GainCoefficient=Chassis_PowerControlGainCoefficientInitialValue;//增益系数回归初始值
	if(Chassis_GyroScopeFlag==1 && (vx!=0 || vy!=0)){w*=1.0f;vx*=1.0f;vy*=1.0f;}//小陀螺移动直接降速,保证移动瞬间为缓冲能量增加
	
	/*==========运动控制==========*/
	vx*=GainCoefficient;vy*=GainCoefficient;
	if(Chassis_GyroScopeFlag==1)w*=GainCoefficient;
	
	float LeftFront_vx=vx-(w+w_track)*Chassis_ry,LeftFront_vy=vy-(w+w_track)*Chassis_rx,LeftFront_v=LeftFront_vx*LeftFront_vx+LeftFront_vy*LeftFront_vy;
	float RightFront_vx=vx-(w+w_track)*Chassis_ry,RightFront_vy=vy+(w+w_track)*Chassis_rx,RightFront_v=RightFront_vx*RightFront_vx+RightFront_vy*RightFront_vy;
	float LeftRear_vx=vx+(w+w_track)*Chassis_ry,LeftRear_vy=vy-(w+w_track)*Chassis_rx,LeftRear_v=LeftRear_vx*LeftRear_vx+LeftRear_vy*LeftRear_vy;
	float RightRear_vx=vx+(w+w_track)*Chassis_ry,RightRear_vy=vy+(w+w_track)*Chassis_rx,RightRear_v=RightRear_vx*RightRear_vx+RightRear_vy*RightRear_vy;
	arm_sqrt_f32(LeftFront_v,&LeftFront_v);
	arm_sqrt_f32(RightFront_v,&RightFront_v);
	arm_sqrt_f32(LeftRear_v,&LeftRear_v);
	arm_sqrt_f32(RightRear_v,&RightRear_v);
	
	//舵向
	Last_thetaTarget[Chassis_LeftFrontSteer-0x205]=theta_Target[Chassis_LeftFrontSteer-0x205];
	Last_thetaTarget[Chassis_RightFrontSteer-0x205]=theta_Target[Chassis_RightFrontSteer-0x205];
	Last_thetaTarget[Chassis_LeftRearSteer-0x205]=theta_Target[Chassis_LeftRearSteer-0x205];
	Last_thetaTarget[Chassis_RightRearSteer-0x205]=theta_Target[Chassis_RightRearSteer-0x205];
	
	theta_Target[Chassis_LeftFrontSteer-0x205]=Chassis_SteerGetAngle(LeftFront_vx,LeftFront_vy,Last_thetaTarget[Chassis_LeftFrontSteer-0x205],&r[Chassis_LeftFrontSteer-0x205]);
	theta_Target[Chassis_RightFrontSteer-0x205]=Chassis_SteerGetAngle(RightFront_vx,RightFront_vy,Last_thetaTarget[Chassis_RightFrontSteer-0x205],&r[Chassis_RightFrontSteer-0x205]);
	theta_Target[Chassis_LeftRearSteer-0x205]=Chassis_SteerGetAngle(LeftRear_vx,LeftRear_vy,Last_thetaTarget[Chassis_LeftRearSteer-0x205],&r[Chassis_LeftRearSteer-0x205]);
	theta_Target[Chassis_RightRearSteer-0x205]=Chassis_SteerGetAngle(RightRear_vx,RightRear_vy,Last_thetaTarget[Chassis_RightRearSteer-0x205],&r[Chassis_RightRearSteer-0x205]);
	
	Chassis_SteerLQRControl(theta_Target[Chassis_LeftFrontSteer-0x205],theta_Target[Chassis_RightFrontSteer-0x205],theta_Target[Chassis_LeftRearSteer-0x205],theta_Target[Chassis_RightRearSteer-0x205]);
	//Chassis_SteerLQRControl(0,0,0,0);
	
	//轮向
	int16_t LeftFrontSpeed=(int16_t)(LeftFront_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//左前轮
	int16_t RightFrontSpeed=(int16_t)(RightFront_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//右前轮
	int16_t LeftRearSpeed=(int16_t)(LeftRear_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//左后轮
	int16_t RightRearSpeed=(int16_t)(RightRear_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//右后轮
	
	Chassis_WheelSpeedControl(-LeftFrontSpeed,RightFrontSpeed,-LeftRearSpeed,RightRearSpeed);//M3508速度控制
}

float theta_Target[4]={0},Motor_Sign[4]={1,1,1,1};

void Chassis_Move_Control(float vx,float vy,float w)
{
	float LeftFront_vx=vx-w*Chassis_ry,  LeftFront_vy=vy-w*Chassis_rx,  LeftFront_v=LeftFront_vx*LeftFront_vx+LeftFront_vy*LeftFront_vy;
	float RightFront_vx=vx-w*Chassis_ry, RightFront_vy=vy+w*Chassis_rx, RightFront_v=RightFront_vx*RightFront_vx+RightFront_vy*RightFront_vy;
	float LeftRear_vx=vx+w*Chassis_ry,   LeftRear_vy=vy-w*Chassis_rx,   LeftRear_v=LeftRear_vx*LeftRear_vx+LeftRear_vy*LeftRear_vy;
	float RightRear_vx=vx+w*Chassis_ry,  RightRear_vy=vy+w*Chassis_rx,  RightRear_v=RightRear_vx*RightRear_vx+RightRear_vy*RightRear_vy;
	arm_sqrt_f32(LeftFront_v,&LeftFront_v);
	arm_sqrt_f32(RightFront_v,&RightFront_v);
	arm_sqrt_f32(LeftRear_v,&LeftRear_v);
	arm_sqrt_f32(RightRear_v,&RightRear_v);
	
	//舵向
	Chassis_SteerGetAnglePlus(LeftFront_vx,LeftFront_vy,GM6020_MotorStatus1[Chassis_LeftFrontSteer-0x205].Position,&(theta_Target[Chassis_LeftFrontSteer-0x205]),&(Motor_Sign[Chassis_LeftFrontSteer-0x205]));
	Chassis_SteerGetAnglePlus(RightFront_vx,RightFront_vy,GM6020_MotorStatus1[Chassis_RightFrontSteer-0x205].Position,&(theta_Target[Chassis_RightFrontSteer-0x205]),&(Motor_Sign[Chassis_RightFrontSteer-0x205]));
	Chassis_SteerGetAnglePlus(LeftRear_vx,LeftRear_vy,GM6020_MotorStatus1[Chassis_LeftRearSteer-0x205].Position,&(theta_Target[Chassis_LeftRearSteer-0x205]),&(Motor_Sign[Chassis_LeftRearSteer-0x205]));
	Chassis_SteerGetAnglePlus(RightRear_vx,RightRear_vy,GM6020_MotorStatus1[Chassis_RightRearSteer-0x205].Position,&(theta_Target[Chassis_RightRearSteer-0x205]),&(Motor_Sign[Chassis_RightRearSteer-0x205]));

	Chassis_SteerLQRControl(theta_Target[Chassis_LeftFrontSteer-0x205],theta_Target[Chassis_RightFrontSteer-0x205],theta_Target[Chassis_LeftRearSteer-0x205],theta_Target[Chassis_RightRearSteer-0x205]);
	//Chassis_SteerLQRControl(0,0,0,0);
	
	//轮向
	int16_t LeftFrontSpeed=(int16_t)(LeftFront_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//左前轮
	int16_t RightFrontSpeed=(int16_t)(RightFront_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//右前轮
	int16_t LeftRearSpeed=(int16_t)(LeftRear_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//左后轮
	int16_t RightRearSpeed=(int16_t)(RightRear_v/Chassis_WheelRadius*M3508_ReductionRatio*9.549296f);//右后轮
	
	Chassis_WheelSpeedControl(-Motor_Sign[Chassis_LeftFrontSteer-0x205]*LeftFrontSpeed,Motor_Sign[Chassis_RightFrontSteer-0x205]*RightFrontSpeed,-Motor_Sign[Chassis_LeftRearSteer-0x205]*LeftRearSpeed,Motor_Sign[Chassis_RightRearSteer-0x205]*RightRearSpeed);//M3508速度控制
}

uint8_t BushuModel_Flag=0;
void Chassis_Control_Plus(void)
{
	static uint8_t Chassis_GyroScopeCloseFlag=0;//小陀螺待关闭标志位
	static float GainCoefficient=1.0f;//速度增益系数
	static uint8_t Count=0;//计数器,以Mecanum_PowerControl_T*2ms为控制周期
	
	static uint8_t Last_Remote_LS=0;
	if(Last_Remote_LS==2 && Remote_RxData.Remote_LS==3)BushuModel_Flag=!BushuModel_Flag;
	Last_Remote_LS=Remote_RxData.Remote_LS;
	
	static uint8_t F_LastStatus=0;
	if(F_LastStatus==1 && Remote_RxData.Remote_Key_F==0)BushuModel_Flag=0;
	F_LastStatus=Remote_RxData.Remote_Key_F;

	if(BushuModel_Flag==1)
	{
		int16_t Raw_Theta=Yaw_GM6020PositionValue-GM6020_MotorStatus[0].Angle;//获取底盘云台相对角度原始数据
		if(Raw_Theta<0)Raw_Theta+=8192;
		//Chassis_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
		Chassis_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对角度
		
		float LeftFront_vx=-1,  LeftFront_vy=1;
		float RightFront_vx=1,  RightFront_vy=1;
		float LeftRear_vx=1,    LeftRear_vy=1;
		float RightRear_vx=-1,  RightRear_vy=1;
		
		//舵向
		Chassis_SteerGetAnglePlus(LeftFront_vx,LeftFront_vy,GM6020_MotorStatus1[Chassis_LeftFrontSteer-0x205].Position,&(theta_Target[Chassis_LeftFrontSteer-0x205]),&(Motor_Sign[Chassis_LeftFrontSteer-0x205]));
		Chassis_SteerGetAnglePlus(RightFront_vx,RightFront_vy,GM6020_MotorStatus1[Chassis_RightFrontSteer-0x205].Position,&(theta_Target[Chassis_RightFrontSteer-0x205]),&(Motor_Sign[Chassis_RightFrontSteer-0x205]));
		Chassis_SteerGetAnglePlus(LeftRear_vx,LeftRear_vy,GM6020_MotorStatus1[Chassis_LeftRearSteer-0x205].Position,&(theta_Target[Chassis_LeftRearSteer-0x205]),&(Motor_Sign[Chassis_LeftRearSteer-0x205]));
		Chassis_SteerGetAnglePlus(RightRear_vx,RightRear_vy,GM6020_MotorStatus1[Chassis_RightRearSteer-0x205].Position,&(theta_Target[Chassis_RightRearSteer-0x205]),&(Motor_Sign[Chassis_RightRearSteer-0x205]));

		Chassis_SteerLQRControl(theta_Target[Chassis_LeftFrontSteer-0x205],theta_Target[Chassis_RightFrontSteer-0x205],theta_Target[Chassis_LeftRearSteer-0x205],theta_Target[Chassis_RightRearSteer-0x205]);
		//Chassis_SteerLQRControl(0,0,0,0);
		
		//轮向
		Chassis_WheelSpeedControl(0,0,0,0);//M3508速度控制
		
		return;
	}
	
	
	/*==========三轴速度获取==========*/
	float vx=Remote_RxData.Remote_R_RL-1024;
	float vy=Remote_RxData.Remote_R_UD-1024;
	float w=1024-Remote_RxData.Remote_L_RL;//获取三个轴的速度参量
	float w_track;
	
	if(Remote_Status==0)vx=vy=0;
	
	float sigma=sqrtf(vx*vx+vy*vy);//获取xy轴速度归一化系数
	if(sigma!=0)//x,y轴速度归一化(正交合成速度不变为标准值)
	{
		vx=vx/sigma*Chassis_PowerControlSpeedNormalizationValue;
		vy=vy/sigma*Chassis_PowerControlSpeedNormalizationValue;
	}
	
	int16_t Raw_Theta=Yaw_GM6020PositionValue-GM6020_MotorStatus[0].Angle;//获取底盘云台相对角度原始数据
	if(Raw_Theta<0)Raw_Theta+=8192;
	//Chassis_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Chassis_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对角度
	
	float vx_=vx,vy_=vy;
	vx=vx_*cosf(Chassis_YawTheta)-vy_*sinf(Chassis_YawTheta);
	vy=vx_*sinf(Chassis_YawTheta)+vy_*cosf(Chassis_YawTheta);//根据底盘云台相对角度修正xy轴速度
	
	/*==========小陀螺处理==========*/
	if(Yaw_GM6020PositionValue<4096 && Remote_RxData.Remote_LS!=0x02)//正向小陀螺为逆时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Chassis_GyroScopeFlag==1 && Chassis_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+500))//小陀螺模式
		{
			w=Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=1;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_RS==1 || (Chassis_GyroScopeFlag==2 && Chassis_GyroScopeCloseFlag==2 && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue+8092-500))//反向小陀螺模式(仅检录使用)
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=2;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=2;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==2 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			if(Chassis_GyroScopeCloseFlag!=0)
			{
				GM6020_MotorStatus[0].r=0;
				Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
			}
			else
			{
				PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
				PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
				w=0;
				w_track=Chassis_TrackPID.OUT;
			}
		}
		else
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-5,5);
			Chassis_GyroScopeFlag=0;
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			if(GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+Chassis_TrackLimit && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue-Chassis_TrackLimit && vx==0 && vy==0)
				w_track=0;
		}
	}
	else if(Remote_RxData.Remote_LS!=0x02)//正向小陀螺为顺时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Chassis_GyroScopeFlag==1 && Chassis_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue+8192-1000))//小陀螺模式
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=1;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			if(Chassis_GyroScopeCloseFlag!=0)
			{
				GM6020_MotorStatus[0].r=0;
				Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态				
			}
			else
			{
				PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
				PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
				w=0;
				w_track=Chassis_TrackPID.OUT;
			}
		}
		else if(Remote_RxData.Remote_RS==1 || (Chassis_GyroScopeFlag==2 && Chassis_GyroScopeCloseFlag==2 && GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+1000))//反向小陀螺模式(仅检录使用)
		{
			w=Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Chassis_GyroScopeFlag=2;//处于小陀螺状态
			Chassis_GyroScopeCloseFlag=2;//小陀螺处于待关闭状态
		}
		else if(Chassis_GyroScopeFlag==2 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-4,4);
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			Chassis_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else
		{
			PID_PositionSetOUTRange(&Chassis_TrackPID,-5,5);
			Chassis_GyroScopeFlag=0;
			PID_PositionCalc(&Chassis_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=Chassis_TrackPID.OUT;
			if(GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+Chassis_TrackLimit && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue-Chassis_TrackLimit && vx==0 && vy==0)
				w_track=0;
		}
	}		
	if(Remote_RxData.Remote_Mouse_KeyR==1)w_track=0;
	
	/*==========功率上限处理==========*/
	/*由缓冲能量得到功率控制功率上限*/
	float Mecanum_PowerRef=1.0f/(60.0f-Chassis_PowerControl_UseBuffer)*RefereeSystem_Buffer;//功率增益
	if(Mecanum_PowerRef>Chassis_PowerControl_PowerMax)Mecanum_PowerRef=Chassis_PowerControl_PowerMax;
	Chassis_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;
	
	/*由运动状态约束功率控制功率上限*/
	float Scale=sigma/660.0f;
	if(Scale<1 && Scale>0)Chassis_PowerLimit*=Scale;//平移约束
	if(Chassis_GyroScopeFlag==1)Chassis_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;//小陀螺约束
	//if(Chassis_PowerLimit>150.0f)Chassis_PowerLimit=150.0f;//限幅约束
	
	/*由超电约束功率控制功率上限*/
	if(Ultra_CAP_Energy/255.0f>0.1f)
	{
		if(Remote_RxData.Remote_KeyPush_Shift==1)//开启超电
		{
			if(Chassis_PowerLimit>0.0f)
			{
				if(Chassis_PowerLimit<RefereeSystem_Ref)Chassis_PowerLimit+=Chassis_PowerControl_UltraCAPPower;
				else Chassis_PowerLimit=RefereeSystem_Ref+Chassis_PowerControl_UltraCAPPower;
			}
			Ultra_CAP_SetPower(RefereeSystem_Ref,RefereeSystem_Buffer,ENABLE);
		}
		else
		{
//			if(RefereeSystem_GameStatus==0x04 && RefereeSystem_RemainTime>0 && Chassis_Energy>0)
//			{
//				Energy_PowerLimit=Chassis_Energy/1.0f/RefereeSystem_RemainTime;
//				if(Mecanum_PowerLimit>Energy_PowerLimit)Mecanum_PowerLimit=Energy_PowerLimit;
//			}
			
			Ultra_CAP_SetPower(Chassis_PowerLimit,50,ENABLE);
		}
	}
	else
	{
		if(vx!=0 || vy!=0 || w!=0)Ultra_CAP_SetPower(Chassis_PowerLimit,50,ENABLE);
		else Ultra_CAP_SetPower(RefereeSystem_Ref,RefereeSystem_Buffer,ENABLE);
		
		Remote_RxData.Remote_KeyPush_Shift=0;
//		if(RefereeSystem_GameStatus==0x04 && RefereeSystem_RemainTime>0 && Chassis_Energy>0)
//		{
//			Energy_PowerLimit=Chassis_Energy/1.0f/RefereeSystem_RemainTime;
//			if(Mecanum_PowerLimit>Energy_PowerLimit)Mecanum_PowerLimit=Energy_PowerLimit;
//		}
//			if(RefereeSystem_GameStatus==0x04 && RefereeSystem_RemainTime>0 && Chassis_Energy>0)
//			{
//				Energy_PowerLimit=Chassis_Energy/1.0f/RefereeSystem_RemainTime;
//				if(Mecanum_PowerLimit>Energy_PowerLimit)Mecanum_PowerLimit=Energy_PowerLimit;
//			}
	}
	
	//if(Chassis_PowerLimit<RefereeSystem_Ref*Chassis_SteerPowerProportion)Chassis_PowerLimit=RefereeSystem_Ref*Chassis_SteerPowerProportion;
	SteerPower=2.5f*1.732050807568877f*(GM6020_MotorStatus1[Chassis_LeftFrontSteer-0x205].Power \
			   +GM6020_MotorStatus1[Chassis_RightFrontSteer-0x205].Power \
			   +GM6020_MotorStatus1[Chassis_LeftRearSteer-0x205].Power \
			   +GM6020_MotorStatus1[Chassis_RightRearSteer-0x205].Power);
	Chassis_WheelPowerLimit=Chassis_PowerLimit-SteerPower;
	if(Chassis_WheelPowerLimit<0)Chassis_WheelPowerLimit=0;
	
	/*==========功率控制==========*/
	static float PowerSum=0;//控制周期内的功率积分
	Chassis_WheelPower=1.0f*1.732050807568877f*(M3508_MotorStatus[Chassis_LeftFrontWheel-0x201].Power \
					   +M3508_MotorStatus[Chassis_RightFrontWheel-0x201].Power \
					   +M3508_MotorStatus[Chassis_LeftRearWheel-0x201].Power \
					   +M3508_MotorStatus[Chassis_RightRearWheel-0x201].Power);//获取底盘功率
	PowerSum+=Chassis_WheelPower;
	
	/*速度增益*/
	if(Count==Chassis_PowerControl_T)//一个控制周期
	{
		Count=0;//计数器清零
		float Power_avg=PowerSum/Chassis_PowerControl_T;
		PowerSum=0;
		
		if(Power_avg<Chassis_WheelPowerLimit/2.0f)
		{
			GainCoefficient+=0.05f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg<Chassis_WheelPowerLimit-5)
		{
			GainCoefficient+=0.02f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg>Chassis_WheelPowerLimit+5)
		{
			if(RefereeSystem_Buffer<60.0f-Chassis_PowerControl_UseBuffer)GainCoefficient-=0.05f;//增益系数递增
			else GainCoefficient-=0.02f;//增益系数递增
			if(GainCoefficient<Chassis_PowerControlGainCoefficientInitialValue/2.0f)GainCoefficient=Chassis_PowerControlGainCoefficientInitialValue/2.0f;//增益系数限下幅
		}
	}
	else if(vx!=0 || vy!=0 || Chassis_GyroScopeFlag==1)//正常运动
		Count++;
	else//停止运动
		GainCoefficient=Chassis_PowerControlGainCoefficientInitialValue;//增益系数回归初始值
	if(Chassis_GyroScopeFlag==1 && (vx!=0 || vy!=0)){w*=1.0f;vx*=1.0f;vy*=1.0f;}//小陀螺移动直接降速,保证移动瞬间为缓冲能量增加
	
	/*==========运动控制==========*/
	vx*=GainCoefficient;vy*=GainCoefficient;
	if(Chassis_GyroScopeFlag==1)w*=GainCoefficient;
	
	Chassis_Move_Control(vx,vy,w+w_track);
}
