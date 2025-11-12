#ifndef __DM_G6220_H
#define __DM_G6220_H

#include <stdint.h>

typedef enum
{
	DM_G6220_IDReceiveOffset=0xF0,
	DM_G6220_1=0x01,//ID1
	DM_G6220_2=0x02,//ID1
	DM_G6220_3=0x03,//ID1
	DM_G6220_4=0x04,//ID1
}DM_G6220_ID;//GM6020电机ID号枚举

typedef struct
{
	uint8_t Status;
	
	uint8_t First_Flag;
	int64_t r;
	
	float Angle;
	float Position;
	float Speed;
	float Speed_Half;
	
	float T;
	float Power;
}DM_G6220_Motor;//M3508电机状态结构体(减速比3591:187(≈19:1),转矩系数0.3N·m/A)

extern DM_G6220_Motor DM_G6220_MotorStatus[];//M3508电机状态数组

void DM_G6220_Init(void);
uint8_t DM_G6220_CANSend(DM_G6220_ID ID,uint8_t *Data);
void DM_G6220_SaveZero(void);
void DM_G6220_Set(DM_G6220_ID ID,float tau);
void DM_G6220_DataProcess(DM_G6220_ID ID,uint8_t *Data);

#endif
