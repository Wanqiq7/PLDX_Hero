#ifndef _CANTASK_H
#define _CANTASK_H
#include "delay.h"

// 超电设备参数范围定义
#define SCAP_MIN_POWER_LIMIT 30.0f
#define SCAP_MAX_POWER_LIMIT 250.0f
#define SCAP_MIN_ENERGY_BUFFER 0.0f
#define SCAP_MAX_ENERGY_BUFFER 300.0f

// 确保使用与超电设备相同的结构体定义
typedef __packed struct {
    uint8_t enableDCDC : 1;
    uint8_t systemRestart : 1;
    uint8_t resv0 : 6;
    uint16_t feedbackRefereePowerLimit;
    uint16_t feedbackRefereeEnergyBuffer;
    uint8_t resv1[3];
}SCAP_Tx;

uint8_t SCAP_Control(float power_limit, float buffer, uint8_t dcdc, uint8_t restart);
void CAN_Send_Task(void);
void CAN1_Send_queue_add(int CAN1_Location_add,int CAN1_DLC_add,int CAN1_Data_1,int CAN1_Data_2,int CAN1_Data_3,int CAN1_Data_4);
void CAN2_Send_queue_add(int CAN2_Location_add,int CAN2_DLC_add,int CAN2_Data_1,int CAN2_Data_2,int CAN2_Data_3,int CAN2_Data_4);

extern int CAN1_Location[100];
extern int CAN1_Task_number;
extern int CAN1_Location[100];
extern int CAN1_DLC[100];
extern int CAN1_Data[100][8];
extern  int test;
#endif


