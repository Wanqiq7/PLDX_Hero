#ifndef __CAN2_H__
#define __CAN2_H__

extern short	Velocity_Value_3508[4];
extern short	Position_Value_3508[4];
extern short	Torque_Value_3508[4];
extern unsigned char Temp_Value_3508[4];
#include <stdio.h>
#include <math.h>
//#include "arm_math.h"
#include <string.h>
#include <stdarg.h>
extern short	Velocity_Value_6020[2];
extern short	Position_Value_6020[2];
extern short	Torque_Value_6020[2];
extern unsigned char Temp_Value_6020[2];

extern short	Velocity_Value_2006[2];
extern short	Position_Value_2006[2];
extern short	Torque_Value_2006[2];
extern unsigned char Temp_Value_2006[2];
unsigned char CAN2_mode_init(unsigned char tsjw, unsigned char tbs2, unsigned char tbs1, unsigned short int brp, unsigned char mode);

extern short	Velocity_Value_3508_FW[4];
extern short	Position_Value_3508_FW[4];
extern short	Torque_Value_3508_FW[4];
extern unsigned char Temp_Value_3508_FW[4];

extern short	SuperCap_power[4];
void CAN_CMD_3508(short int motor1, short int motor2, short int motor3, short int motor4);


#endif
