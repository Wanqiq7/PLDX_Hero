#ifndef __VISUAL_H
#define __VISUAL_H

extern float Visual_Yaw,Visual_Pitch,Visual_Fire;//视觉数据偏航角,视觉数据俯仰角
extern uint8_t Visual_ReceiveFlag;//视觉数据接收完成标志位

void Visual_Init(void);//视觉初始化
void Visual_SendData(void);

#endif
