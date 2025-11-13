#ifndef _FLOATTASK_H
#define _FLOATTASK_H
#include "delay.h"
#include "includes.h"
#include "includes.h"

extern uint8_t chassis_gryo360_move;
extern uint8_t chassis_sideway;
extern uint8_t chassis_following_Gimbal;

typedef struct Chassis_Remote
{
	float X;
	float Y;
	float Z;
	float Vx;
	float Vy;
	float Vz;
	double scaled_X;
	double scaled_Y;
	double scaled_Z;
	float gains;
	uint32_t SlopeLen;
}Chassis_Remote;

void Chassis_task(void *p_arg);

extern float speed_x,speed_y,speed_w;		//xyz轴目标速度 单位：m/s
void Gyro_360_motion(int Center_adjustment_value ,float target);
void Chassis_PID_Init(void);
void Gyro_360_motion2(int Center_adjustment_value ,float target);
void Chassis_following2(int Middle);
void Chassis_Regular_exercise(void);
float Chassis_Slope2(int len,float target_set,float value);

#endif

