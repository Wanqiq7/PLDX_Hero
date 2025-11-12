#ifndef __ULTRA_CAP_H
#define __ULTRA_CAP_H

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "stdint.h"

#define Ultra_CAP_Master	0x061
#define Ultra_CAP_Slave		0x051

typedef struct __attribute__((packed))
{
    uint8_t enableDCDC:1;
    uint8_t systemRestart:1;
    uint8_t resv0:6;
    uint16_t feedbackRefereePowerLimit;
    uint16_t feedbackRefereeEnergyBuffer;
    uint8_t resv1[3];
}TxData;

extern float Ultra_CAP_Energy,Ultra_CAP_Power;

void Ultra_CAP_Init(void);//超电初始化
uint8_t Ultra_CAP_SetPower(float PowerLimit,float Buffer,FunctionalState NewStatus);//超电设置功率上限
void Ultra_CAP_DataProcess(uint8_t *Data);

#endif
