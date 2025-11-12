#ifndef __GIMBAL_H
#define __GIMBAL_H

extern uint8_t Fire_Flag;

void Gimbal_Init(void);//云台初始化
void Gimbal_CleanPID(void);//云台PID清理
void Gimbal_MoveControl(void);//云台运动控制

#endif
