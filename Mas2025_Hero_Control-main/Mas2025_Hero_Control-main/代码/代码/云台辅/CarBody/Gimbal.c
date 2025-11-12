#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Parameter.h"
#include "PID.h"
#include "Remote.h"
#include "AttitudeAlgorithms.h"
#include "M3508.h"
#include "GM6020.h"
#include "Laser.h"
#include "RefereeSystem.h"
#include "Visual.h"
#include "Delay.h"
#include "arm_math.h"
#include "DM_J4310.h"
#include "M2006.h"
#include "DM_J4310.h"
#include "Switch.h"

#define Gimbal_YawMotor										GM6020_1//Yaw轴电机
#define Gimbal_PitchMotor									DM_J4310_2//Pitch轴电机
#define Gimbal_L_FrictionWheel1								M3508_1//左摩擦轮
#define Gimbal_L_FrictionWheel2								M3508_3//左摩擦轮
#define Gimbal_R_FrictionWheel1								M3508_2//右摩擦轮
#define Gimbal_R_FrictionWheel2								M3508_4//右摩擦轮

uint8_t Gimbal_FrictionWheelFlag;//云台小陀螺标志位,云台开摩擦轮标志位

PID_PositionInitTypedef Gimbal_RammerSpinSpeedPID;//拨弹盘旋转PID

float Pitch_TargetTheta=0;
float Yaw_TargetTheta=0;
float FiringMechanismL_TargetSpeed=0,FiringMechanismR_TargetSpeed=0;
float Rammer_TargetTheta=0,Dingxin_TargetTheta=0;

/*
 *函数简介:云台初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_Init(void)
{	
	DM_J4310_Init();
	Laser_Init();
}

/*
 *函数简介:云台PID清理
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_CleanPID(void)
{
	Rammer_TargetTheta=DM_J4310_MotorStatus[6].Position;
	Dingxin_TargetTheta=M2006_MotorStatus[6].ShaftPosition/180.0f*3.1415926f;
}
float tau;
uint8_t Fire_Flag=0;

/*
 *函数简介:摩擦轮控制
 *参数说明:无
 *返回类型:无
 *备注:遥控左拨动开关向上拨(Remote_LS=1)开摩擦轮,摩擦轮打开的同时会打开激光
 */
void Gimbal_FiringMechanismControl(void)
{
	#define LQR_K5		0.0707f
	
	if(Remote_Status==0){}
	else
	{
		if(((Remote_RxData.Remote_LS==1 && RefereeSystem_Status==0) || PC_FrictionWheel==1) && RefereeSystem_ShooterStatus==1)//摩擦轮开
		{
			FiringMechanismL_TargetSpeed=-Gimbal_FrictionWheelSpeed;FiringMechanismR_TargetSpeed=Gimbal_FrictionWheelSpeed;
			Laser_ON();//开激光
			Gimbal_FrictionWheelFlag=1;
		}
		else//摩擦轮关
		{
			FiringMechanismL_TargetSpeed=FiringMechanismR_TargetSpeed=0;
			Laser_OFF();//关激光
			Gimbal_FrictionWheelFlag=0;
		}
	}
	
	float tauL1=LQR_K5*(FiringMechanismL_TargetSpeed-M3508_MotorStatus[Gimbal_L_FrictionWheel1-0x201].RotorSpeed/60.0f*2.0f*3.1415926f);
	float CurrentL1=tauL1/0.3f/20.0f*16384.0f;
	if(CurrentL1>16384)CurrentL1=16384;if(CurrentL1<-16384)CurrentL1=-16384;
	
	float tauL2=LQR_K5*(FiringMechanismL_TargetSpeed-M3508_MotorStatus[Gimbal_L_FrictionWheel2-0x201].RotorSpeed/60.0f*2.0f*3.1415926f);
	float CurrentL2=tauL2/0.3f/20.0f*16384.0f;
	if(CurrentL2>16384)CurrentL2=16384;if(CurrentL2<-16384)CurrentL2=-16384;
	
	float tauR1=LQR_K5*(FiringMechanismR_TargetSpeed-M3508_MotorStatus[Gimbal_R_FrictionWheel1-0x201].RotorSpeed/60.0f*2.0f*3.1415926f);
	float CurrentR1=tauR1/0.3f/20.0f*16384.0f;
	if(CurrentR1>16384)CurrentR1=16384;if(CurrentR1<-16384)CurrentR1=-16384;
	
	float tauR2=LQR_K5*(FiringMechanismR_TargetSpeed-M3508_MotorStatus[Gimbal_R_FrictionWheel2-0x201].RotorSpeed/60.0f*2.0f*3.1415926f);
	float CurrentR2=tauR2/0.3f/20.0f*16384.0f;
	if(CurrentR2>16384)CurrentR2=16384;if(CurrentR2<-16384)CurrentR2=-16384;
	
	M3508_CAN2SetLIDCurrent(CurrentL1,CurrentR1,CurrentL2,CurrentR2);
	//M3508_CANSetLIDCurrent(0,0,0,0);
}

/*
 *函数简介:拨弹盘控制
 *参数说明:无
 *返回类型:无
 *备注:俯仰轴GM6020报文标识符和M2006高位ID一致,故均在拨弹盘控制函数中统一发送控制报文
 */
void Gimbal_Rammer(void)
{
	#define LQR_K7		10.0000f
	#define LQR_K8		10.0000f
	
	#define Dingxin_Angle	60

	static uint16_t timeCount=0;
	if(timeCount<8000)timeCount++;
	else timeCount=0;

	if(Remote_Status==0){}
	else
	{
		if(Gimbal_FrictionWheelFlag==1 && timeCount>500.0f/Gimbal_RammerSpeed && \
			(Rammer_TargetTheta-DM_J4310_MotorStatus[6].Position<1.5f*3.1415926f/Gimbal_RammerCount*4.0f && Rammer_TargetTheta-DM_J4310_MotorStatus[6].Position>-1.5f*3.1415926f/Gimbal_RammerCount*4.0f))
		{
			{
				if((Remote_RxData.Remote_ThumbWheel<1000 && RefereeSystem_Status==0) || PC_Fire==1)
				{
					if(DM_J4310_MotorStatus[6].Speed<5.0f*2.0f*PI/60.0f && DM_J4310_MotorStatus[6].Speed>-5.0f*2.0f*PI/60.0f)
					{
						Rammer_TargetTheta+=2*3.1415926f/Gimbal_RammerCount*4.0f;//乘减速比
						timeCount=0;
					}
					else if(timeCount>500.0f/Gimbal_RammerSpeed)
					{
						Rammer_TargetTheta+=2*3.1415926f/Gimbal_RammerCount*4.0f;//乘减速比
						timeCount=0;
					}
				}
			}
		}
	}
	
	if(Switch_Flag==1 && Gimbal_FrictionWheelFlag==1)
		Dingxin_TargetTheta+=Dingxin_Angle/180.0f*3.1415926f;
	Switch_Flag=0;
	
	float v=LQR_K7*(Rammer_TargetTheta-DM_J4310_MotorStatus[6].Position);
	if(v>20)v=20;
	if(v<-20)v=-20;
	
	float tau=LQR_K8*(v-DM_J4310_MotorStatus[6].Speed);
	tau+=0.8f;
	
	float v2=LQR_K7*(Dingxin_TargetTheta-M2006_MotorStatus[7].ShaftPosition/180.0f*3.1415926f);
	if(v2>10)v2=10;
	if(v2<-10)v2=-10;
	
	float tau2=LQR_K8*(v2-M2006_MotorStatus[7].ShaftSpeed/60.0f*2.0f*3.1415926f);
	float Current2=(tau2/36.0f)/0.18f/10.0f*10000.0f;
	if(Current2>10000)Current2=10000;
	if(Current2<-10000)Current2=-10000;
	//Current2=0;
	
	if(Gimbal_FrictionWheelFlag==0){tau=0;Current2=0;}
	
	M2006_CAN2SetHIDCurrent(0,0,0,Current2);
	
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	static uint16_t EnableCount=0;
	EnableCount++;
	if(EnableCount==100 && DM_J4310_MotorStatus[6].Status==0x00)
		DM_J4310_CANSend(DM_J4310_7,Start);
	else
	{
		if(EnableCount==200)EnableCount=0;
		DM_J4310_Set(DM_J4310_7,tau);
	}
}

/*
 *函数简介:云台运动控制
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_MoveControl(void)
{	
	Gimbal_FiringMechanismControl();//摩擦轮控制
	
	Gimbal_Rammer();//拨弹盘控制
}
