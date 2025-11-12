#ifndef __REMOTE_H
#define __REMOTE_H

typedef struct
{
	uint16_t Remote_R_RL;//通道0-右摇杆左右(右为大),范围364(最左端)~1684(最右端),默认值1024(中间)
	uint16_t Remote_R_UD;//通道1-右摇杆上下(上为大),范围364(最下端)~1684(最上端),默认值1024(中间)
	uint16_t Remote_L_RL;//通道2-左摇杆左右(右为大),范围364(最左端)~1684(最右端),默认值1024(中间)
	uint16_t Remote_L_UD;//通道3-左摇杆上下(上为大),范围364(最下端)~1684(最上端),默认值1024(中间)
	
	uint8_t Remote_LS;//S1-左侧拨动开关,范围1~3,上为1,下为2,中间为3
	uint8_t Remote_RS;//S2-右侧拨动开关,范围1~3,上为1,下为2,中间为3
	
	int16_t Remote_Mouse_RL;//鼠标X轴-鼠标左右速度,范围-32768~32767,向右为正,向左为负,静止值为0
	int16_t Remote_Mouse_DU;//鼠标Y轴-鼠标前后速度,范围-32768~32767,向后为正,向前为负,静止值为0
	int16_t Remote_Mouse_Wheel;//鼠标Z轴-鼠标滚轮速度,范围-32768~32767,向前为正,向后为负,静止值为0
	uint8_t Remote_Mouse_KeyL;//鼠标左键,按下为1,未按下为0
	uint8_t Remote_Mouse_KeyR;//鼠标右键,按下为1,未按下为0
	
	uint8_t Remote_Key_W;//键盘W键,按下为1,未按下为0
	uint8_t Remote_Key_S;//键盘S键,按下为1,未按下为0
	uint8_t Remote_Key_A;//键盘A键,按下为1,未按下为0
	uint8_t Remote_Key_D;//键盘D键,按下为1,未按下为0
	uint8_t Remote_Key_Q;//键盘Q键,按下为1,未按下为0
	uint8_t Remote_Key_E;//键盘E键,按下为1,未按下为0
	uint8_t Remote_Key_Shift;//键盘Shift键,按下为1,未按下为0
	uint8_t Remote_Key_Ctrl;//键盘Ctrl键,按下为1,未按下为0
	
	uint8_t Remote_Mouse_KeyLastR;//上一次鼠标右键
	uint8_t Remote_KeyLast_Q;//上一次键盘Q键
	uint8_t Remote_KeyLast_E;//上一次键盘E键
	uint8_t Remote_KeyLast_Shift;//上一次键盘Shift键
	uint8_t Remote_KeyLast_Ctrl;//上一次键盘Ctrl键
	uint8_t Remote_Mouse_KeyPushR;//按下鼠标右键,按下瞬间为1,其他为0
	uint8_t Remote_KeyPush_Q;//按下键盘Q键,按下时0,1切换
	uint8_t Remote_KeyPush_E;//按下键盘E键,按下时0,1切换
	uint8_t Remote_KeyPush_Shift;//按下键盘Shift键,按下时0,1切换
	uint8_t Remote_KeyPush_Ctrl;//按下键盘Ctrl键,按下时0,1切换
	
	int16_t Remote_ThumbWheel;//保留字段-遥控器拨轮,范围-3278(最上端)~1684(最下端),默认值1024
}Remote_Data;//遥控器接收结构体

extern Remote_Data Remote_RxData;//遥控器接收数据
extern uint8_t Remote_Status;//遥控器连接状态,默认未连接(0)
extern uint8_t Remote_StartFlag;//遥控器启动标志位,0-未在启动阶段,1-准备启动,2-第一次接收到数据

void Remote_Init(void);//遥控器初始化
void Remote_ON(void);//遥控器开启
void Remote_OFF(void);//遥控器关闭

#endif
