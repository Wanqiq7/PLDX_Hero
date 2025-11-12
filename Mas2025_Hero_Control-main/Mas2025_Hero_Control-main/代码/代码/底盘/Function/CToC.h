#ifndef __CTOC_H
#define __CTOC_H

#define CToC_MasterID1	0x019//主机ID1

#define CToC_SlaveID1	0x149//从机ID1
#define CToC_SlaveID2	0x189//从机ID2
#define CToC_SlaveID4	0x065//从机ID2

extern float Gimbal_Pitch,Gimbal_Yaw;

void CToC_SlaveInit(void);//板间通讯从机初始化
uint8_t CToC_SlaveSendRefereeSystemData(void);//板间通讯从机发送裁判系统数据
void CToC_CANDataProcess(uint32_t ID,uint8_t *Data);//板间通讯数据处理

#endif
