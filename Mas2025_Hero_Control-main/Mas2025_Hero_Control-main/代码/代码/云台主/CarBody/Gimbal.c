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
#include "PWM.h"

#define Gimbal_YawMotor										GM6020_1//Yaw轴电机
#define Gimbal_PitchMotor									DM_J4310_2//Pitch轴电机
#define Gimbal_L_FrictionWheel1								M3508_1//左摩擦轮
#define Gimbal_L_FrictionWheel2								M3508_3//左摩擦轮
#define Gimbal_R_FrictionWheel1								M3508_2//右摩擦轮
#define Gimbal_R_FrictionWheel2								M3508_4//右摩擦轮
#define Gimabl_VT											PWM1
#define Gimbal_Scope										PWM2

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
	
	PWM_Init(Gimabl_VT,50,8.7f);//8.7 9.8
	PWM_Init(Gimbal_Scope,50,5.5f);//7.9 10.3
}

/*
 *函数简介:云台PID清理
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_CleanPID(void)
{
}
float tau;
uint8_t Fire_Flag=0;

/*
 *函数简介:云台Pitch轴控制
 *参数说明:无
 *返回类型:无
 *备注:根据拨杆或鼠标获得俯仰角度,映射比例在上方宏定义Gimbal_LeverSpeedMapRate更改
 *备注:俯仰限幅由结构决定,参数由Parameter.h文件中的Pitch_GM6020PositionLowerLinit和Pitch_GM6020PositionUpperLinit决定
 *备注:俯仰轴GM6020报文标识符和M2006高位ID一致,故均在拨弹盘控制函数中统一发送控制报文
 *备注:在此函数中进行了视觉自瞄处理,由于视觉组摆烂,并没有开发出自瞄,也没用进行过联调,故自瞄部分没有拆出去独立函数
 */
void Gimbal_PitchControl(void)
{
	#define LQR_K1			40.0000f
	#define LQR_K2			2.3802f//40.0000    2.3802
	
	#define LQR_Ki			0.15f//0.15f
	#define LQR_Ki_ScalarA	(15.0f*0.01745329251994329576923690768489f)
	#define LQR_Ki_ScalarB	(5.0f*0.01745329251994329576923690768489f)
	
	#define LQR_tff			1.0f//1.0f
	
	static float Pitch_LQR_IOUT=0;
	
	
	static uint16_t Count=0;
	static uint8_t PitchFlag=0;
	
	if(PitchFlag==1)PitchFlag=0;
	else if(Remote_StartFlag==2)//遥控器刚建立连接时,复位Pitch轴角度并清理Pitch轴位置环积分项
	{
		Count=0;
		Pitch_TargetTheta=0.0f;//AttitudeAlgorithms_RadRoll;
	}
	
	if(Remote_Status==0){PitchFlag=1;}
	else
	{
		if(((Remote_RxData.Remote_L_UD>1050 && RefereeSystem_Status==0) || (1024+Remote_RxData.Remote_Mouse_DU*3)<1024) && Pitch_TargetTheta>Pitch_AngleUpperLinit)
		{
			if(PC_Pitch==0)Pitch_TargetTheta-=Gimbal_LeverSpeedMapRate*0.0439453125f*((Remote_RxData.Remote_L_UD-1024)/660.0f)/180.0f*3.1415926f;
			else Pitch_TargetTheta+=Gimbal_LeverSpeedMapRate*0.0439453125f*(PC_Pitch*PC_Mouse_DUSensitivity/660.0f*2)/180.0f*3.1415926f;
		}
		else if(((Remote_RxData.Remote_L_UD<1000 && RefereeSystem_Status==0) || (1024+Remote_RxData.Remote_Mouse_DU*3)>1024) && Pitch_TargetTheta<Pitch_AngleLowerLinit)
		{
			if(PC_Pitch==0)Pitch_TargetTheta+=Gimbal_LeverSpeedMapRate*0.0439453125f*((1024-Remote_RxData.Remote_L_UD)/660.0f)/180.0f*3.1415926f;
			else Pitch_TargetTheta+=Gimbal_LeverSpeedMapRate*0.0439453125f*(PC_Pitch*PC_Mouse_DUSensitivity/660.0f*2)/180.0f*3.1415926f;
		}
	
		if(Remote_RxData.Remote_Mouse_KeyR==1 && Visual_ReceiveFlag==1 && (Visual_Pitch!=0 || Visual_Yaw!=0))//自瞄,补偿角度
		{
			Visual_ReceiveFlag=0;
			Fire_Flag=1;
			Pitch_TargetTheta=Visual_Pitch;
			if(Pitch_TargetTheta<Pitch_AngleUpperLinit)Pitch_TargetTheta=Pitch_AngleUpperLinit;
			if(Pitch_TargetTheta>Pitch_AngleLowerLinit)Pitch_TargetTheta=Pitch_AngleLowerLinit;
			//Gimbal_YawAnglePositionPID.Need_Value=Visual_Yaw;
			Yaw_TargetTheta=Visual_Yaw/180.0f*3.1415926f;
		}
	}
	
	if(Pitch_TargetTheta<Pitch_AngleUpperLinit)Pitch_TargetTheta=Pitch_AngleUpperLinit;
	if(Pitch_TargetTheta>Pitch_AngleLowerLinit)Pitch_TargetTheta=Pitch_AngleLowerLinit;
	
	
	//float Ek=Pitch_TargetTheta-AttitudeAlgorithms_RadPitch;
	float Ek=Pitch_TargetTheta-DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Position;
	//if(Ek<0.004f && Ek>-0.004f)Ek=0;
	float ITerm=LQR_Ki*Ek;
	if(Ek*Pitch_LQR_IOUT>0)//增长趋势
    {
        if(fabs(Ek)<=LQR_Ki_ScalarB)ITerm*=1.0f;
        else if(fabs(Ek)<=(LQR_Ki_ScalarA+LQR_Ki_ScalarB))ITerm*=(LQR_Ki_ScalarA-fabs(Ek)+LQR_Ki_ScalarB)/LQR_Ki_ScalarA;
        else ITerm=0;
    }

	Pitch_LQR_IOUT+=ITerm;
	tau=-LQR_tff+LQR_K1*Ek+Pitch_LQR_IOUT-LQR_K2*DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Speed;
	//tau=-LQR_tff;
	
	if(tau>5.0f)tau=5.0f;
	if(tau<-5.0f)tau=-5.0f;
	
	//if(Count>5)DM_J4310_Set(Gimbal_PitchMotor,0);
	if(Count>5)DM_J4310_Set(Gimbal_PitchMotor,tau);
	else
	{
		Count++;
		DM_J4310_Set(Gimbal_PitchMotor,0);
	}
	
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	static uint16_t EnableCount=0;
	EnableCount++;
	if(EnableCount==100 && DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Status==0)DM_J4310_CANSend(Gimbal_PitchMotor,Start);
	else if(EnableCount==200)EnableCount=0;
}

/*
 *函数简介:云台Yaw轴控制
 *参数说明:无
 *返回类型:无
 *备注:根据拨杆或鼠标获得偏航角度,映射比例在上方宏定义Gimbal_LeverSpeedMapRate和Gimbal_YawPitchSpeedRate更改
 *备注:由于云台一直根据陀螺仪角度闭环,不需要考虑小陀螺问题
 */
float tau;
uint8_t SteerFlag=0;
void Gimbal_YawControl(void)
{
	#define LQR_K3		31.6228f
	#define LQR_K4		3.3113f//31.6228    3.3113
	
	static uint8_t YawFlag=0;
	
	if(YawFlag==1)YawFlag=0;
	else if(Remote_StartFlag==2)//遥控器刚建立连接时,复位Pitch轴角度并清理Pitch轴位置环积分项
	{
		Yaw_TargetTheta=AttitudeAlgorithms_RadYaw;
		Rammer_TargetTheta=M3508_MotorStatus[6].ShaftPosition/180.0f*3.1415926f;
		Dingxin_TargetTheta=M2006_MotorStatus[6].ShaftPosition/180.0f*3.1415926f;
	}
	
	if(Remote_Status==0){YawFlag=1;}
	else
	{
		if((Remote_RxData.Remote_L_RL>1024 && RefereeSystem_Status==0) || 1024+PC_Spin*PC_Mouse_RLSensitivity>1024)//根据摇杆改变偏航
		{
			if(PC_Spin==0)Yaw_TargetTheta-=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*Gimbal_YawPitchSpeedRate*0.0439453125f*((Remote_RxData.Remote_L_RL-1024)/660.0f)/180.0f*3.1415926f;
			else Yaw_TargetTheta-=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*Gimbal_YawPitchSpeedRate*0.0439453125f*(PC_Spin*PC_Mouse_RLSensitivity/660.0f*2)/180.0f*3.1415926f;
		}
		else if((Remote_RxData.Remote_L_RL<1024 && RefereeSystem_Status==0) || 1024+PC_Spin*PC_Mouse_RLSensitivity<1024)
		{
			if(PC_Spin==0)Yaw_TargetTheta+=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*Gimbal_YawPitchSpeedRate*0.0439453125f*((1024-Remote_RxData.Remote_L_RL)/660.0f)/180.0f*3.1415926f;
			else Yaw_TargetTheta-=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*Gimbal_YawPitchSpeedRate*0.0439453125f*(PC_Spin*PC_Mouse_RLSensitivity/660.0f*2)/180.0f*3.1415926f;
		}
		
		static uint8_t F_LastStatus=0;
		if(F_LastStatus==1 && Remote_RxData.Remote_Key_F==0)
		{
			Yaw_TargetTheta-=PI;
			
			PC_VT=0;
			PC_Scope=0;
			
		}
		F_LastStatus=Remote_RxData.Remote_Key_F;
	}
	
	tau=LQR_K3*(Yaw_TargetTheta-AttitudeAlgorithms_RadYaw)-LQR_K4*AttitudeAlgorithms_RaddYaw_E;
	float Current=tau/0.741f/3.0f*16384.0f;
	if(Current>16384)Current=16384;
	if(Current<-16384)Current=-16384;
	GM6020_CAN2SetLIDCurrent(-Current,0,0,0);
}

void Gimbal_SteerControl(void)
{
	static uint8_t Last_Remote_RS=3;
	static uint8_t VT_Flag=0,Scope_Flag=0;
	
	if(RefereeSystem_Status==1)
	{
		if(PC_VT==1)PWM_SetDuty(Gimabl_VT,10.3f);
		else PWM_SetDuty(Gimabl_VT,8.7f);
		
		if(PC_Scope==1)PWM_SetDuty(Gimbal_Scope,10.1f);
		else PWM_SetDuty(Gimbal_Scope,5.5f);
	}
	else if(Remote_RxData.Remote_LS==0x02 && RefereeSystem_Status==0)
	{
		if(Last_Remote_RS==1 && Remote_RxData.Remote_RS==3)
		{
			if(VT_Flag==0){PWM_SetDuty(Gimabl_VT,10.3f);VT_Flag=1;}
			else if(VT_Flag==1){PWM_SetDuty(Gimabl_VT,8.7f);VT_Flag=0;}
		}
		else if(Last_Remote_RS==2 && Remote_RxData.Remote_RS==3)
		{
			if(Scope_Flag==0){PWM_SetDuty(Gimbal_Scope,10.1f);Scope_Flag=1;}
			else if(Scope_Flag==1){PWM_SetDuty(Gimbal_Scope,5.5f);Scope_Flag=0;}
		}
	}
	
	Last_Remote_RS=Remote_RxData.Remote_RS;
}

/*
 *函数简介:云台运动控制
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_MoveControl(void)
{
	Gimbal_PitchControl();//云台Pitch轴控制
	Gimbal_YawControl();//云台Yaw轴控制
	
	Delay_us(200);
	
	Gimbal_SteerControl();
//	
//	Gimbal_FiringMechanismControl();//摩擦轮控制
//	
//	Gimbal_Rammer();//拨弹盘控制
}
