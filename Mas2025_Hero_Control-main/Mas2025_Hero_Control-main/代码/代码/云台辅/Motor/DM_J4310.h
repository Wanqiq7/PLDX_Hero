#ifndef __DM_J4310_2EC_H
#define __DM_J4310_2EC_H

#include <stdint.h>

typedef enum
{
	DM_J4310_IDReceiveOffset=0xF0,
	DM_J4310_7=0x07,//ID1
}DM_J4310_ID;//GM6020电机ID号枚举

typedef struct
{
	uint8_t Status;
	
	uint8_t First_Flag;
	int64_t r;
	
	float Angle;
	float Position;
	float Speed;
	
	float T;
	float Power;
}DM_J4310_Motor;//M3508电机状态结构体(减速比3591:187(≈19:1),转矩系数0.3N·m/A)

extern DM_J4310_Motor DM_J4310_MotorStatus[];//M3508电机状态数组

void DM_J4310_Init(void);
uint8_t DM_J4310_CANSend(DM_J4310_ID ID,uint8_t *Data);
void DM_J4310_Set(DM_J4310_ID ID,float tau);
void DM_J4310_DataProcess(DM_J4310_ID ID,uint8_t *Data);

#endif
