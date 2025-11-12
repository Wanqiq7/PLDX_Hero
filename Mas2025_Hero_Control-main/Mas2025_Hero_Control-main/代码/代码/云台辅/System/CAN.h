#ifndef __CAN_H
#define __CAN_H

typedef enum
{
	CAN_M3508=0,//M3508
	CAN_M2006,//M2006
	CAN_GM6020,//GM6020
	CAN_RoboMasterC,//C板
	CAN_DM_J4310,
}CAN_MotorModel;//CAN总线设备分类枚举

extern uint8_t CAN_CAN1DeviceNumber;//CAN1总线上设备数量
extern uint8_t CAN_CAN2DeviceNumber;//CAN2总线上设备数量
extern uint8_t CAN_DeviceNumber;//CAN总线上设备数量
extern int8_t CAN_IDSelect;//CAN总线上ID列表选择位
extern uint16_t LinkError_Count;

void CAN_CANInit(void);//CAN总线初始化
void CAN_CANIDReset(void);//CAN接收ID列表复位
void CAN_CAN_GetRefereeSystemData(void);//CAN接收获取裁判系统状态
void CAN_CAN_GetRefereeSystemData2(void);

#endif
