#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "M3508.h"
#include "Delay.h"

#define M3508_Speed				8000	//PWM最高转速

#define M3508_PWMMode			0		//PWM模式(0-单向,1-双向)
#define M3508_PWM0Checksum		500		//单向模式电调校验值,对应1000us
#define M3508_PWM1Checksum		727		//双向模式电调校验值,对应1500us
#define M3508_PWMTime			100		//校验完成等待时间

//M3508和M2006标识符完全相同
#define M3508_Control_ID_L		0x200//M3508低位ID发送报文标识符
#define M3508_Control_ID_H		0x1FF//M3508高位ID发送报文标识符(M3508高位标识符和GM6020低位标识符相同)

#define M3508_ReductionRatio	(3591.0f/187.0f)//M3508减速比3591:187(≈19:1)
#define M3508_TorqueConstant	0.3f//M3508转矩常数0.3N·m/A

int16_t M3508_PWMNowDuty=0;//PWM当前占空比
M3508_Motor M3508_MotorStatus[8];//M3508电机状态数组

/*
 *函数简介:PWM控制M3508电机初始化
 *参数说明:无
 *返回类型:无
 *备注:默认使用PWM1(C1为PE9),默认500Hz
 *备注:需要校准"电调校验值"和"校验完成等待时间"
 */
void M3508_PWMInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM1);//选择时基单元TIM1
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;   
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);//配置C1-PE9
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);//开启C1的TIM1复用模式
	
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_InitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_InitStructure.TIM_Period=1000-1;//ARR,PWM为千分位2ms
	TIM_InitStructure.TIM_Prescaler=336-1;//PSC
	TIM_InitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM1,&TIM_InitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//配置输出比较模式
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//配置输出比较的极性
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出使能
	TIM_OCInitStructure.TIM_Pulse=0;//配置输出比较寄存器CCR的值
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);//配置C1输出PWM
	
	TIM_Cmd(TIM1,ENABLE);//启动定时器
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//开启TIM1的PWM输出
	
	if(M3508_PWMMode==0)
	{
		TIM_SetCompare1(TIM1,M3508_PWM0Checksum);//电调校准
		Delay_ms(M3508_PWMTime);//等待校准完成
		M3508_PWMNowDuty=M3508_PWM0Checksum+40;//校验当前占空比
	}
	else
	{
		TIM_SetCompare1(TIM1,M3508_PWM1Checksum);//电调校准
		Delay_ms(M3508_PWMTime);//等待校准完成
		M3508_PWMNowDuty=M3508_PWM1Checksum;//校验当前占空比
	}
}

/*
 *函数简介:PWM设置M3508转速
 *参数说明:速度,单向模式0~vm,双向模式-vm~+vm
 *参数说明:标志位,决定是否使用缓启动
 *返回类型:无
 *备注:无
 */
void M3508_PWMSetSpeed(int16_t Speed,uint8_t Flag)
{
	uint16_t Duty=0;
	if(M3508_PWMMode==0)//单向模式(0-2000,1080->v=0,1920->v=vm)
	{
		Duty=M3508_PWM0Checksum+40+Speed*840/M3508_Speed/2;
		if(Flag==0)//硬启动
			TIM_SetCompare1(TIM1,Duty);
		else//从当前速度缓启动到目标速度,10分位
			for(uint8_t i=0;i<=10;i++)
			{
				TIM_SetCompare1(TIM1,(int)(M3508_PWMNowDuty+(Duty-M3508_PWMNowDuty)*i/10));
				Delay_ms(20);
			}
		M3508_PWMNowDuty=Duty;//获取当前占空比
	}
	else//双向模式
	{
		if(Speed<0)//反转(0-1500,1080->v=-vm,1480->v=0)
		{
			Duty=M3508_PWM1Checksum-10+Speed*400/M3508_Speed/2;
			if(Flag==0)//硬启动
				TIM_SetCompare1(TIM1,Duty);
			else//缓启动,10分位
				for(uint8_t i=0;i<=10;i++)
				{
					TIM_SetCompare1(TIM1,(int)(M3508_PWMNowDuty+(Duty-M3508_PWMNowDuty)*i/10));
					Delay_ms(20);
				}
		}
		else//正转(1500-2000,1520->v=0,1920->v=vm)
		{
			Duty=M3508_PWM1Checksum+10+Speed*400/M3508_Speed/2;
			if(Flag==0)//硬启动
				TIM_SetCompare1(TIM1,Duty);
			else//从当前速度缓启动到目标速度,10分位
				for(uint8_t i=0;i<=10;i++)
				{
					TIM_SetCompare1(TIM1,(int)(M3508_PWMNowDuty+(Duty-M3508_PWMNowDuty)*i/10));
					Delay_ms(20);
				}
		}
		M3508_PWMNowDuty=Duty;//获取当前占空比
	}
}

/*
 *函数简介:CAN总线设置M3508低位ID电流
 *参数说明:Currrent1~4分别对应ID1~4
 *返回类型:1-发送成功,0-发送失败
 *备注:只能配置ID1~4(标识符0x200)
 *备注:默认标准格式数据帧,8字节数据段
 *备注:注意M3508的报文标识符与M2006的报文标识符完全相同
 *备注:给电机一定的电流,会促使电机产生加速度
 */
uint8_t M3508_CANSetLIDCurrent(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=M3508_Control_ID_L;//低位ID标准标识符0x200
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Current1>>8;//ID1电流高八位
	TxMessage.Data[1]=Current1;//ID1电流低八位
	TxMessage.Data[2]=Current2>>8;//ID2电流高八位
	TxMessage.Data[3]=Current2;//ID2电流低八位
	TxMessage.Data[4]=Current3>>8;//ID3电流高八位
	TxMessage.Data[5]=Current3;//ID3电流低八位
	TxMessage.Data[6]=Current4>>8;//ID4电流高八位
	TxMessage.Data[7]=Current4;//ID4电流低八位
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:CAN总线设置M3508高位ID电流
 *参数说明:Currrent5~8分别对应ID5~8
 *返回类型:1-发送成功,0-发送失败
 *备注:只能配置ID5~8(标识符0x1FF)
 *备注:默认标准格式数据帧,8字节数据段
 *备注:注意M3508的报文标识符与M2006的报文标识符完全相同
 *备注:注意M3508的ID5~8报文标识符与GM6020的ID1~4报文标识符相同
 *备注:给电机一定的电流,会促使电机产生加速度
 */
uint8_t M3508_CANSetHIDCurrent(int16_t Current5,int16_t Current6,int16_t Current7,int16_t Current8)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=M3508_Control_ID_H;//高位ID标准标识符0x1FF
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Current5>>8;//ID5电流高八位
	TxMessage.Data[1]=Current5;//ID5电流低八位
	TxMessage.Data[2]=Current6>>8;//ID6电流高八位
	TxMessage.Data[3]=Current6;//ID6电流低八位
	TxMessage.Data[4]=Current7>>8;//ID7电流高八位
	TxMessage.Data[5]=Current7;//ID7电流低八位
	TxMessage.Data[6]=Current8>>8;//ID8电流高八位
	TxMessage.Data[7]=Current8;//ID8电流低八位
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

uint8_t M3508_CAN2SetHIDCurrent(int16_t Current5,int16_t Current6,int16_t Current7,int16_t Current8)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=M3508_Control_ID_H;//高位ID标准标识符0x1FF
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Current5>>8;//ID5电流高八位
	TxMessage.Data[1]=Current5;//ID5电流低八位
	TxMessage.Data[2]=Current6>>8;//ID6电流高八位
	TxMessage.Data[3]=Current6;//ID6电流低八位
	TxMessage.Data[4]=Current7>>8;//ID7电流高八位
	TxMessage.Data[5]=Current7;//ID7电流低八位
	TxMessage.Data[6]=Current8>>8;//ID8电流高八位
	TxMessage.Data[7]=Current8;//ID8电流低八位
	
	uint8_t mbox=CAN_Transmit(CAN2,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN2,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:M3508数据处理
 *参数说明:M3508电机ID号枚举,M3508_1~8对应ID号0x201~0x208
 *参数说明:反馈数据(8字节)
 *返回类型:无
 *备注:保存到M3508_MotorStatus结构体数组
 *备注:M3508减速比3591:187(≈19:1),转矩常数0.3N·m/A
 */
void M3508_CANDataProcess(M3508_ID ID,uint8_t *Data)
{
	uint16_t M3508_RotorNowAngle=(uint16_t)((((uint16_t)Data[0])<<8)|Data[1]);//本次转子机械角度原始数据
	if(M3508_RotorNowAngle-M3508_MotorStatus[ID-0x201].RawRotorAngle>4000 && M3508_MotorStatus[ID-0x201].First_Flag==1)M3508_MotorStatus[ID-0x201].Rotor_r--;//本次转子机械角度原始数据和上次转子机械角度原始数据出现跃变
	else if(M3508_MotorStatus[ID-0x201].RawRotorAngle-M3508_RotorNowAngle>4000 && M3508_MotorStatus[ID-0x201].First_Flag==1)M3508_MotorStatus[ID-0x201].Rotor_r++;
	else if(M3508_MotorStatus[ID-0x201].First_Flag!=1)M3508_MotorStatus[ID-0x201].First_Flag=1;
	
	M3508_MotorStatus[ID-0x201].RawRotorAngle=M3508_RotorNowAngle;//转子机械角度原始数据
	M3508_MotorStatus[ID-0x201].RotorAngle=M3508_RotorNowAngle*0.0439453125f;//=M3508_RotorNowAngle/8192.0f*360.0f;//转子机械角度
	M3508_MotorStatus[ID-0x201].RawRotorPosition=8192*M3508_MotorStatus[ID-0x201].Rotor_r+M3508_RotorNowAngle;//转子角度位置原始数据
	M3508_MotorStatus[ID-0x201].RotorPosition=360.0f*M3508_MotorStatus[ID-0x201].Rotor_r+M3508_MotorStatus[ID-0x201].RotorAngle;//转子角度位置
	M3508_MotorStatus[ID-0x201].RotorSpeed=(int16_t)((((uint16_t)Data[2])<<8)|Data[3]);//转子转速原始数据
	
	M3508_MotorStatus[ID-0x201].ShaftPosition=M3508_MotorStatus[ID-0x201].RotorPosition*0.0520746310219994f;//=M3508_MotorStatus[ID-0x201].RotorPosition/M3508_ReductionRatio;//转轴角度位置
	M3508_MotorStatus[ID-0x201].Shaft_r=(int64_t)(M3508_MotorStatus[ID-0x201].ShaftPosition)/360;
	if(M3508_MotorStatus[ID-0x201].ShaftPosition<0 && M3508_MotorStatus[ID-0x201].ShaftPosition-360.0f*M3508_MotorStatus[ID-0x201].Shaft_r<0)M3508_MotorStatus[ID-0x201].Shaft_r--;//获取转轴圈数
	M3508_MotorStatus[ID-0x201].ShaftAngle=M3508_MotorStatus[ID-0x201].ShaftPosition-360.0f*M3508_MotorStatus[ID-0x201].Shaft_r;//转轴机械角度
	M3508_MotorStatus[ID-0x201].ShaftSpeed=M3508_MotorStatus[ID-0x201].RotorSpeed*0.0520746310219994f;//=M3508_MotorStatus[ID-0x201].RotorSpeed/M3508_ReductionRatio;//转轴转速
	
	M3508_MotorStatus[ID-0x201].RawCurrent=(int16_t)((((uint16_t)Data[4])<<8)|Data[5]);//转矩电流原始数据
	M3508_MotorStatus[ID-0x201].Current=M3508_MotorStatus[ID-0x201].RawCurrent*0.001220703125f;//=M3508_MotorStatus[ID-0x201].RawCurrent/16384.0f*20.0f;//转矩电流
	
	M3508_MotorStatus[ID-0x201].Power=M3508_MotorStatus[ID-0x201].ShaftSpeed*M3508_MotorStatus[ID-0x201].Current*0.031413612565445f;//=M3508_MotorStatus[ID-0x201].ShaftSpeed*M3508_MotorStatus[ID-0x201].Current*M3508_TorqueConstant/9.55f;//电机功率(功率P(kW)=转轴转速v(RPM)*转矩T(N·m)/9550,转矩T=转矩电流*转矩常数)
	if(M3508_MotorStatus[ID-0x201].Power<0)M3508_MotorStatus[ID-0x201].Power*=-1;//功率去负数化
	M3508_MotorStatus[ID-0x201].Temperature=Data[6];//电机温度
}
