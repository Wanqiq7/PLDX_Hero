#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Ultra_CAP.h"
#include <string.h>

float Ultra_CAP_Energy,Ultra_CAP_Power;

uint8_t Ultra_CAP_SetPower(float PowerLimit,float Buffer,FunctionalState NewStatus)
{
    TxData txData;
    
    // 设置数据
    txData.enableDCDC=NewStatus;
    txData.systemRestart=0;
    txData.resv0=0;
    txData.feedbackRefereePowerLimit=PowerLimit;   // 示例值
    txData.feedbackRefereeEnergyBuffer=Buffer; // 示例值
    txData.resv1[0]=0;
    txData.resv1[1]=0;
    txData.resv1[2]=0;

	CanTxMsg TxMessage;
	TxMessage.StdId=Ultra_CAP_Master;//低位ID标准标识符0x200
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=sizeof(TxData);//8字节数据段
    memcpy(TxMessage.Data,&txData,sizeof(TxData));  // 复制数据到缓冲区
	
	uint8_t mbox=CAN_Transmit(CAN2,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN2,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:超电初始化
 *参数说明:无
 *返回类型:无
 *备注:默认设定电压23V,电流8A,功率150W
 */
void Ultra_CAP_Init(void)
{
	Ultra_CAP_SetPower(45,50,ENABLE);
}

void Ultra_CAP_DataProcess(uint8_t *Data)
{
    // 打包 float 到 uint8_t 数组（使用 memcpy）
    memcpy(&Ultra_CAP_Power,&Data[1],4);       // 后4字节是底盘功率
	Ultra_CAP_Energy=Data[7];
}
