#ifndef __REFEREESYSTEM_H
#define __REFEREESYSTEM_H

#include <stdint.h>

extern uint8_t RefereeSystem_ShooterStatus;//发射机构状态,0-发射机构未上电,1-发射机构上电
extern uint16_t RefereeSystem_Ref,RefereeSystem_Buffer;//底盘功率上限,底盘功率缓冲能量
extern float RefereeSystem_Power;//底盘实时功率
extern uint8_t RefereeSystem_RobotID;//机器人ID
extern uint8_t RefereeSystem_GameStatus;
extern float RefereeSystem_Energy;
extern uint16_t RefereeSystem_RemainTime;

uint8_t RefereeSystem_GetCRC8CheckSum(uint8_t *Data,uint16_t Length,uint8_t Initial);//裁判系统CRC8查表计算
uint16_t RefereeSystem_GetCRC16CheckSum(uint8_t *Data,uint32_t Length,uint16_t Initial);//裁判系统CRC16查表计算
void RefereeSystem_Init(void);//裁判系统接收初始化

#endif
