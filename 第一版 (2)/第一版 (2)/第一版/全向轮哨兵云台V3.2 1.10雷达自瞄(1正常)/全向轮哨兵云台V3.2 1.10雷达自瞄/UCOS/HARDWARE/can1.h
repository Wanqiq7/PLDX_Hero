#ifndef __CAN1_H__
#define __CAN1_H__

#include "sys.h"

extern short	Velocity_Value_6020[2];
extern short	Position_Value_6020[2];
extern short	Torque_Value_6020[2];
extern unsigned char Temp_Value_6020[2];

extern short	Velocity_Value_2006[2];
extern short	Position_Value_2006[2];
extern short	Torque_Value_2006[2];
extern unsigned char Temp_Value_2006[2];

extern short morot1_test;
extern short morot2_test;
extern short morot3_test;
extern short morot4_test;

extern short	Velocity_Value_3508_FW[4];
extern short	Position_Value_3508_FW[4];
extern short	Torque_Value_3508_FW[4];
extern unsigned char Temp_Value_3508_FW[4];
extern float Motor_Power[4];
extern uint8_t capEnergy;


extern short can_test[4];


extern unsigned char CAN1_mode_init(unsigned char tsjw, unsigned char tbs2, unsigned char tbs1, unsigned short int brp, unsigned char mode);

void CAN_CMD_6020_yaw(unsigned short int yaw);

void CAN_CMD_2006(int pitch);

#endif

