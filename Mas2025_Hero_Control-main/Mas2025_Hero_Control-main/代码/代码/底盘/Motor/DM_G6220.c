#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "DM_G6220.h"
#include "Delay.h"

#define DM_G6220_PMAX			12.5f
#define DM_G6220_VMAX			45.0f
#define DM_G6220_TMAX			10.0f

DM_G6220_Motor DM_G6220_MotorStatus[10];//M3508电机状态数组

uint8_t DM_G6220_CANSend(DM_G6220_ID ID,uint8_t *Data)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=ID;//高位ID标准标识符0x1FF
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Data[0];//ID5电流高八位
	TxMessage.Data[1]=Data[1];//ID5电流低八位
	TxMessage.Data[2]=Data[2];//ID6电流高八位
	TxMessage.Data[3]=Data[3];//ID6电流低八位
	TxMessage.Data[4]=Data[4];//ID7电流高八位
	TxMessage.Data[5]=Data[5];//ID7电流低八位
	TxMessage.Data[6]=Data[6];//ID8电流高八位
	TxMessage.Data[7]=Data[7];//ID8电流低八位
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

void DM_G6220_Init(void)
{
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	DM_G6220_CANSend(DM_G6220_1,Start);
	DM_G6220_CANSend(DM_G6220_1,Start);
	Delay_us(200);
	DM_G6220_CANSend(DM_G6220_2,Start);
	DM_G6220_CANSend(DM_G6220_2,Start);
	Delay_us(200);
	DM_G6220_CANSend(DM_G6220_3,Start);
	DM_G6220_CANSend(DM_G6220_3,Start);
	Delay_us(200);
	DM_G6220_CANSend(DM_G6220_4,Start);
	DM_G6220_CANSend(DM_G6220_4,Start);
	Delay_us(200);
	DM_G6220_CANSend(DM_G6220_1,Start);
	DM_G6220_CANSend(DM_G6220_1,Start);
	Delay_us(200);
}

void DM_G6220_SaveZero(void)
{
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
	DM_G6220_CANSend(DM_G6220_1,Start);
	DM_G6220_CANSend(DM_G6220_1,Start);
	Delay_us(200);
	DM_G6220_CANSend(DM_G6220_2,Start);
	DM_G6220_CANSend(DM_G6220_2,Start);
	Delay_us(200);
	DM_G6220_CANSend(DM_G6220_3,Start);
	DM_G6220_CANSend(DM_G6220_3,Start);
	Delay_us(200);
	DM_G6220_CANSend(DM_G6220_4,Start);
	DM_G6220_CANSend(DM_G6220_4,Start);
	Delay_us(200);
}

uint16_t float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset=x_min;
	return (uint16_t)((x-offset)*((float)((1<<bits)-1))/span);
}

void DM_G6220_Set(DM_G6220_ID ID,float tau)
{
	if(tau>1.0f)tau=1.0f;
	else if(tau<-1.0f)tau=-1.0f;
	uint8_t Data[8];
	uint16_t Tau=float_to_uint(tau,-DM_G6220_TMAX,DM_G6220_TMAX,12);
	Data[0]=0;
    Data[1]=0;
    Data[2]=0;
    Data[3]=0;
    Data[4]=0;
    Data[5]=0;
    Data[6]=Tau>>8;
    Data[7]=Tau & 0x00FF;
	DM_G6220_CANSend(ID,Data);
}	

float uint_to_float(int x_int,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1))+offset;
}

void DM_G6220_DataProcess(DM_G6220_ID ID,uint8_t *Data)
{
	DM_G6220_MotorStatus[ID-0xF1].Status=(Data[0]>>4) & 0x0F;
	
	float NowAngle=0;
	uint16_t Position=(int16_t)((uint16_t)Data[1]<<8 | Data[2]);
	uint16_t Speed=(int16_t)((uint16_t)Data[3]<<4 | Data[4]>>4);
	uint16_t T=(int16_t)((uint16_t)(Data[4]&0x0F)<<8 | Data[5]);
	
	NowAngle=uint_to_float(Position,-DM_G6220_PMAX,DM_G6220_PMAX,16);//本次机械角度原始数据
	if(NowAngle-DM_G6220_MotorStatus[ID-0xF1].Angle>DM_G6220_PMAX && DM_G6220_MotorStatus[ID-0xF1].First_Flag==1)DM_G6220_MotorStatus[ID-0xF1].r--;//本次机械角度原始数据和上次机械角度原始数据出现跃变
	else if(DM_G6220_MotorStatus[ID-0xF1].Angle-NowAngle>DM_G6220_PMAX && DM_G6220_MotorStatus[ID-0xF1].First_Flag==1)DM_G6220_MotorStatus[ID-0xF1].r++;
	else if(DM_G6220_MotorStatus[ID-0xF1].First_Flag!=1)DM_G6220_MotorStatus[ID-0xF1].First_Flag++;
	
	DM_G6220_MotorStatus[ID-0xF1].Position=NowAngle+2*DM_G6220_PMAX*DM_G6220_MotorStatus[ID-0xF1].r;
	DM_G6220_MotorStatus[ID-0xF1].Angle=NowAngle;
	
	DM_G6220_MotorStatus[ID-0xF1].Speed_Half=uint_to_float(Speed,-DM_G6220_VMAX/2.0f,DM_G6220_VMAX/2.0f,12);
	DM_G6220_MotorStatus[ID-0xF1].Speed=uint_to_float(Speed,-DM_G6220_VMAX,DM_G6220_VMAX,12);

	DM_G6220_MotorStatus[ID-0xF1].T=uint_to_float(T,-DM_G6220_TMAX,DM_G6220_TMAX,12);
	DM_G6220_MotorStatus[ID-0xF1].Power=DM_G6220_MotorStatus[ID-0xF1].Speed*DM_G6220_MotorStatus[ID-0xF1].T;
	if(DM_G6220_MotorStatus[ID-0xF1].Power<0)DM_G6220_MotorStatus[ID-0xF1].Power*=-1;
}	
