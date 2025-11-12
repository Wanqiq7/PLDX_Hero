#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <math.h>
#include "M3508.h"
#include "GM6020.h"
#include "Remote.h"
#include "PID.h"
#include "Parameter.h"
#include "RefereeSystem.h"
#include "Ultra_CAP.h"
#include "Mecanum.h"

#define Mecanum_PowerControlSpeedNormalizationValue			3.0f//功率控制归一化速度标准值,范围[0,3]
#define Mecanum_PowerControlGainCoefficientInitialValue		0.5f//功率控制增益系数初始值
#define Mecanum_PowerControl_T								5.0f//功率控制周期(T=Mecanum_PowerControl_T*2ms)
#define Mecanum_PowerControl_UseBuffer						30.0f//功率控制消耗的缓冲能量
#define Mecanum_PowerControl_PowerMax						4.0f//功率控制功率上限与裁判系统功率上限比值上限
#define Mecanum_PowerControl_UltraCAPPower					100.0f//使用超电增加功率

PID_PositionInitTypedef Mecanum_SpeedPID[4];//底盘四个电机的转速PID
PID_PositionInitTypedef Mecanum_TrackPID;//底盘跟随PID
float Mecanum_YawTheta;//底盘云台相对偏航角度
uint8_t Mecanum_GyroScopeFlag;//底盘小陀螺标志位
float Mecanum_Power;//底盘功率(软件计算值)
float Mecanum_Current,Mecanum_PowerLimit,Mecanum_CurrentLimit=16384*4.0f;//底盘电流总和,功率控制功率上限,底盘电流限幅
	
/*
 *函数简介:麦轮初始化
 *参数说明:无
 *返回类型:无
 *备注:即四个转速PID的初始化
 *备注:四个电机的ID参照上方的宏定义,最好使用M3508电机的ID1~4,否则下方电机控制的代码需要修改
 */
void Mecanum_Init(void)
{
	PID_PositionStructureInit(&Mecanum_SpeedPID[0],0);//左前轮
	PID_PositionSetParameter(&Mecanum_SpeedPID[0],0.8,0,0);
	PID_PositionSetEkRange(&Mecanum_SpeedPID[0],-5,5);
	PID_PositionSetOUTRange(&Mecanum_SpeedPID[0],-15000,15000);
	
	PID_PositionStructureInit(&Mecanum_SpeedPID[1],0);//右前轮
	PID_PositionSetParameter(&Mecanum_SpeedPID[1],0.8,0,0);
	PID_PositionSetEkRange(&Mecanum_SpeedPID[1],-5,5);
	PID_PositionSetOUTRange(&Mecanum_SpeedPID[1],-15000,15000);
	
	PID_PositionStructureInit(&Mecanum_SpeedPID[2],0);//左后轮
	PID_PositionSetParameter(&Mecanum_SpeedPID[2],0.8,0,0);
	PID_PositionSetEkRange(&Mecanum_SpeedPID[2],-5,5);
	PID_PositionSetOUTRange(&Mecanum_SpeedPID[2],-15000,15000);
	
	PID_PositionStructureInit(&Mecanum_SpeedPID[3],0);//右后轮
	PID_PositionSetParameter(&Mecanum_SpeedPID[3],0.8,0,0);
	PID_PositionSetEkRange(&Mecanum_SpeedPID[3],-5,5);
	PID_PositionSetOUTRange(&Mecanum_SpeedPID[3],-15000,15000);
	
	PID_PositionStructureInit(&Mecanum_TrackPID,Yaw_GM6020PositionValue);//底盘跟随
	PID_PositionSetParameter(&Mecanum_TrackPID,0.004,0,0);
	PID_PositionSetEkRange(&Mecanum_TrackPID,-5,5);
	PID_PositionSetOUTRange(&Mecanum_TrackPID,-4,4);
}

/*
 *函数简介:麦轮PID清理
 *参数说明:无
 *返回类型:无
 *备注:清理四个转速位置式PID
 */
void Mecanum_CleanPID(void)
{
	PID_PositionClean(&Mecanum_SpeedPID[0]);//左前轮
	PID_PositionClean(&Mecanum_SpeedPID[1]);//右前轮
	PID_PositionClean(&Mecanum_SpeedPID[2]);//左后轮
	PID_PositionClean(&Mecanum_SpeedPID[3]);//右后轮
	PID_PositionClean(&Mecanum_TrackPID);//底盘跟随
}

/*
 *函数简介:麦轮速度控制
 *参数说明:左前轮速度
 *参数说明:右前轮速度
 *参数说明:左后轮速度
 *参数说明:右后轮速度
 *返回类型:无
 *备注:单独控制四个轮子的速度
 */
void Mecanum_ControlSpeed(int16_t LeftFrontSpeed,int16_t RightFrontSpeed,int16_t LeftRearSpeed,int16_t RightRearSpeed)
{
	//更改期望
	Mecanum_SpeedPID[0].Need_Value=LeftFrontSpeed;//左前轮
	Mecanum_SpeedPID[1].Need_Value=RightFrontSpeed;//右前轮;
	Mecanum_SpeedPID[2].Need_Value=LeftRearSpeed;//左后轮
	Mecanum_SpeedPID[3].Need_Value=RightRearSpeed;//右后轮
	
	//PID计算
	PID_PositionCalc(&Mecanum_SpeedPID[0],M3508_MotorStatus[Mecanum_LeftFrontWheel-0x201].RotorSpeed);//左前轮
	PID_PositionCalc(&Mecanum_SpeedPID[1],M3508_MotorStatus[Mecanum_RightFrontWheel-0x201].RotorSpeed);//右前轮
	PID_PositionCalc(&Mecanum_SpeedPID[2],M3508_MotorStatus[Mecanum_LeftRearWheel-0x201].RotorSpeed);//左后轮
	PID_PositionCalc(&Mecanum_SpeedPID[3],M3508_MotorStatus[Mecanum_RightRearWheel-0x201].RotorSpeed);//右后轮

	Mecanum_Current=0;
	for(uint8_t i=0;i<4;i++)
		Mecanum_Current+=fabs(Mecanum_SpeedPID[i].OUT);
	if(Mecanum_Current>Mecanum_CurrentLimit)
	{
		Mecanum_SpeedPID[0].OUT*=(Mecanum_CurrentLimit/Mecanum_Current);
		Mecanum_SpeedPID[1].OUT*=(Mecanum_CurrentLimit/Mecanum_Current);
		Mecanum_SpeedPID[2].OUT*=(Mecanum_CurrentLimit/Mecanum_Current);
		Mecanum_SpeedPID[3].OUT*=(Mecanum_CurrentLimit/Mecanum_Current);
	}
	
	M3508_CANSetLIDCurrent(Mecanum_SpeedPID[1].OUT,Mecanum_SpeedPID[0].OUT,Mecanum_SpeedPID[2].OUT,Mecanum_SpeedPID[3].OUT);//M3508控制
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
void Mecanum_InverseMotionControl(float v_x,float v_y,float w)
{
	//逆运动解算
	int16_t LeftFrontSpeed=(int16_t)((-v_x-v_y-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//左前轮
	int16_t RightFrontSpeed=(int16_t)((v_x-v_y-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//右前轮
	int16_t LeftRearSpeed=(int16_t)((-v_x+v_y-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//左后轮
	int16_t RightRearSpeed=(int16_t)((v_x+v_y-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//右后轮
	
	Mecanum_ControlSpeed(LeftFrontSpeed,RightFrontSpeed,LeftRearSpeed,RightRearSpeed);//M3508速度控制
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
void Mecanum_PowerMoveControl(void)
{
	static uint8_t Mecanum_GyroScopeCloseFlag=0;//小陀螺待关闭标志位
	static float GainCoefficient=1.0f;//速度增益系数
	static uint8_t Count=0;//计数器,以Mecanum_PowerControl_T*2ms为控制周期

	/*==========三轴速度获取==========*/
	float vx=1024-Remote_RxData.Remote_R_UD;
	float vy=1024-Remote_RxData.Remote_R_RL;
	float w=1024-Remote_RxData.Remote_L_RL;//获取三个轴的速度参量
	float w_track;
	float sigma=sqrtf(vx*vx+vy*vy);//获取xy轴速度归一化系数
	if(sigma!=0)//x,y轴速度归一化(正交合成速度不变为标准值)
	{
		vx=vx/sigma*Mecanum_PowerControlSpeedNormalizationValue;
		vy=vy/sigma*Mecanum_PowerControlSpeedNormalizationValue;
	}
	
	int16_t Raw_Theta=Yaw_GM6020PositionValue-GM6020_MotorStatus[0].Angle;//获取底盘云台相对角度原始数据
	if(Raw_Theta<0)Raw_Theta+=8192;
	//Mecanum_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Mecanum_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对角度
	
	float vx_=vx,vy_=vy;
	vx=vx_*cosf(Mecanum_YawTheta)-vy_*sinf(Mecanum_YawTheta);
	vy=vx_*sinf(Mecanum_YawTheta)+vy_*cosf(Mecanum_YawTheta);//根据底盘云台相对角度修正xy轴速度
	
	/*==========小陀螺处理==========*/
	if(Yaw_GM6020PositionValue<4096)//正向小陀螺为逆时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+500))//小陀螺模式
		{
			w=Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Mecanum_GyroScopeFlag=1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-4,4);
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_RS==1 || (Mecanum_GyroScopeFlag==2 && Mecanum_GyroScopeCloseFlag==2 && GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+8092-100))//反向小陀螺模式(仅检录使用)
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			w_track=0;
			Mecanum_GyroScopeFlag=2;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=2;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==2 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			GM6020_MotorStatus[0].r=0;
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-4,4);
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-5,5);
			Mecanum_GyroScopeFlag=0;
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=0;
			w_track=-Mecanum_TrackPID.OUT;
		}
	}
	else//正向小陀螺为顺时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue-500))//小陀螺模式
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			Mecanum_GyroScopeFlag=1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+5 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-5) || (GM6020_MotorStatus[0].Speed>1||GM6020_MotorStatus[0].Speed<-1)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-2,2);
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_RS==1)//反向小陀螺模式(仅检录使用)
			w=Mecanum_GyroScopeAngularVelocity;
		else
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-5,5);
			Mecanum_GyroScopeFlag=0;
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
		}
	}		
	
	/*==========功率上限处理==========*/
	/*由缓冲能量得到功率控制功率上限*/
	float Mecanum_PowerRef=1.0f/(60.0f-Mecanum_PowerControl_UseBuffer)*RefereeSystem_Buffer;//功率增益
	if(Mecanum_PowerRef>Mecanum_PowerControl_PowerMax)Mecanum_PowerRef=Mecanum_PowerControl_PowerMax;
	Mecanum_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;
	
	/*由运动状态约束功率控制功率上限*/
	float Scale=sigma/660.0f;
	if(Scale<1 && Scale>0)Mecanum_PowerLimit*=Scale;//平移约束
	if(Mecanum_GyroScopeFlag==1)Mecanum_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;//小陀螺约束
	if(Mecanum_PowerLimit>150.0f)Mecanum_PowerLimit=150.0f;//限幅约束
	
	/*由超电约束功率控制功率上限*/
//	if(Remote_RxData.Remote_KeyPush_Shift==1)//开启超电
//	{
//		if(Mecanum_PowerLimit>0.0f)
//		{
//			if(Mecanum_PowerLimit<RefereeSystem_Ref)Mecanum_PowerLimit+=Mecanum_PowerControl_UltraCAPPower;
//			else Mecanum_PowerLimit=RefereeSystem_Ref+Mecanum_PowerControl_UltraCAPPower;
//		}
//		Ultra_CAP_SetPower(RefereeSystem_Ref);
//	}
//	else
//		Ultra_CAP_SetPower(Mecanum_PowerLimit);
	
	/*==========功率控制==========*/
	static float PowerSum=0;//控制周期内的功率积分
	Mecanum_Power=1.732050807568877f*(M3508_MotorStatus[Mecanum_LeftFrontWheel-0x201].Power \
				  +M3508_MotorStatus[Mecanum_RightFrontWheel-0x201].Power \
				  +M3508_MotorStatus[Mecanum_LeftRearWheel-0x201].Power \
				  +M3508_MotorStatus[Mecanum_RightRearWheel-0x201].Power);//获取底盘功率
	PowerSum+=Mecanum_Power;
	
	/*速度增益*/
	if(Count==Mecanum_PowerControl_T)//一个控制周期
	{
		Count=0;//计数器清零
		float Power_avg=PowerSum/Mecanum_PowerControl_T;
		PowerSum=0;
		
		if(Power_avg<Mecanum_PowerLimit/2.0f)
		{
			GainCoefficient+=0.05f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg<Mecanum_PowerLimit-5)
		{
			GainCoefficient+=0.02f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg>Mecanum_PowerLimit+5)
		{
			if(RefereeSystem_Buffer<60.0f-Mecanum_PowerControl_UseBuffer)GainCoefficient-=0.05f;//增益系数递增
			else GainCoefficient-=0.02f;//增益系数递增
			if(GainCoefficient<Mecanum_PowerControlGainCoefficientInitialValue/2.0f)GainCoefficient=Mecanum_PowerControlGainCoefficientInitialValue/2.0f;//增益系数限下幅
		}
	}
	else if(vx!=0 || vy!=0 || Mecanum_GyroScopeFlag==1)//正常运动
		Count++;
	else//停止运动
		GainCoefficient=Mecanum_PowerControlGainCoefficientInitialValue;//增益系数回归初始值
	if(Mecanum_GyroScopeFlag==1 && (vx!=0 || vy!=0)){w*=0.6f;vx*=0.5f;vy*=0.5f;}//小陀螺移动直接降速,保证移动瞬间为缓冲能量增加
	
	/*电流限幅*/
	float Mecanum_BufferCurrent=Mecanum_PowerLimit*512.0f/15.0f;
	float Mecanum_PowerCurrent=Mecanum_PowerLimit*512.0f/15.0f*5.0f;
	float B,C=0.9f;
	if(60.0f-Mecanum_PowerControl_UseBuffer-10.0f>10.0f)B=60.0f-Mecanum_PowerControl_UseBuffer-10.0f;
	else B=10.0f;
	if(RefereeSystem_Buffer>B)
	{
		float PowerScale=0;
		if(Mecanum_Power>C*Mecanum_PowerLimit)
		{
			if(Mecanum_Power>Mecanum_PowerLimit)PowerScale=0;
			else PowerScale=(Mecanum_PowerLimit-Mecanum_Power)/(Mecanum_PowerLimit-C*Mecanum_PowerLimit);
		}
		else PowerScale=1;
		Mecanum_CurrentLimit=Mecanum_BufferCurrent+Mecanum_PowerCurrent*PowerScale;
	}
	else
	{
		if(RefereeSystem_Buffer<5.0f)Mecanum_CurrentLimit=Mecanum_BufferCurrent*0.5f;
		else Mecanum_CurrentLimit=Mecanum_BufferCurrent*(RefereeSystem_Buffer+B-10.0f)/(2*B-10.0f);
	}
	if(Mecanum_CurrentLimit>16384*4)Mecanum_CurrentLimit=16384*4;

	/*==========运动控制==========*/
	vx*=GainCoefficient;vy*=GainCoefficient;
	if(Mecanum_GyroScopeFlag==1)w*=GainCoefficient;
	int16_t LeftFrontSpeed=(int16_t)((-vx-vy-(w+w_track)*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//左前
	int16_t RightFrontSpeed=(int16_t)((vx-vy-(w+w_track)*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//右前
	int16_t LeftRearSpeed=(int16_t)((-vx+vy-(w+w_track)*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//左后
	int16_t RightRearSpeed=(int16_t)((vx+vy-(w+w_track)*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//右后
	
	Mecanum_ControlSpeed(LeftFrontSpeed,RightFrontSpeed,LeftRearSpeed,RightRearSpeed);
}
