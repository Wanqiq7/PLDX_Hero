#ifndef __CAN_H
#define __CAN_H

#include <stdint.h>

typedef enum
{
	CAN_M3508=0,//M3508
	CAN_GM6020,//GM6020
	CAN_RoboMasterC,//C板
	CAN_DM_G6220,
	CAN_Ultra_CAP,
}CAN_MotorModel;//CAN总线设备分类枚举

extern uint8_t CAN_DeviceNumber;//CAN总线上设备数量
extern uint8_t CAN_IDSelect;//CAN总线上ID列表选择位
extern float PM_Voltage,PM_Current,PM_Power;

void CAN_CANInit(void);//CAN总线初始化

#endif
