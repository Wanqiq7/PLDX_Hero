#ifndef __MECANUM_H
#define __MECANUM_H

#define Mecanum_LeftFrontWheel								M3508_2//左前轮
#define Mecanum_RightFrontWheel								M3508_1//右前轮
#define Mecanum_LeftRearWheel								M3508_3//左后轮
#define Mecanum_RightRearWheel								M3508_4//右后轮

extern float Mecanum_Power;//底盘功率(软件计算值)
extern float Mecanum_YawTheta;//底盘云台相对偏航角度

void Mecanum_Init(void);//麦轮初始化
void Mecanum_CleanPID(void);//麦轮PID清理
void Mecanum_ControlSpeed(int16_t LeftFrontSpeed,int16_t RightFrontSpeed,int16_t LeftRearSpeed,int16_t RightRearSpeed);//麦轮速度控制
void Mecanum_InverseMotionControl(float v_x,float v_y,float w);//麦轮逆运动解算
void Mecanum_PowerMoveControl(void);//麦轮功率控制

#endif
