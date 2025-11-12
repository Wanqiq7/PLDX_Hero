#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>

/*==========轮向==========*/
#define Chassis_LeftFrontWheel								M3508_2//左前轮
#define Chassis_RightFrontWheel								M3508_1//右前轮
#define Chassis_LeftRearWheel								M3508_3//左后轮
#define Chassis_RightRearWheel								M3508_4//右后轮

/*==========舵向==========*/
#define Chassis_LeftFrontSteer								GM6020_2//左前轮
#define Chassis_RightFrontSteer								GM6020_1//右前轮
#define Chassis_LeftRearSteer								GM6020_3//左后轮
#define Chassis_RightRearSteer								GM6020_4//右后轮

extern float SteerPower;
extern float Chassis_WheelPower,Mecanum_EstimatedPower;
extern float Chassis_YawTheta;//底盘云台相对偏航角度
extern uint8_t BushuModel_Flag;

void Chassis_Init(void);
void Chassis_Control(void);
void Chassis_Control_Plus(void);

#endif
