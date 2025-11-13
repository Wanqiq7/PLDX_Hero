#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.csom
//修改日期:2011/6/14
//版本：V1.4
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
#define FRAME_HEADER      0X7B //Frame_header //??í·
#define FRAME_TAIL        0X7D //Frame_tail   //???2
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11
//void uart3_init(u32 bound);
//int USART3_IRQHandler(void);
float XYZ_Target_Speed_transition(u8 High,u8 Low);

extern int angle[3];
extern int angle_g[3];
extern int angle_v[3];
extern int angle_v_int[3];
extern int TV_target[3];

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);

	
typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1??×??ú
		float X_speed;	            //4 bytes //4??×??ú
		float Y_speed;              //4 bytes //4??×??ú
		float Z_speed;              //4 bytes //4??×??ú
		unsigned char Frame_Tail;   //1 bytes //1??×??ú
	}Control_Str;
}RECEIVE_DATA;

typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2??×??ú
	short Y_data; //2 bytes //2??×??ú
	short Z_data; //2 bytes //2??×??ú
}Mpu6050_Data;

typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1??×??ú
		short X_speed;	            //2 bytes //2??×??ú
		short Y_speed;              //2 bytes //2??×??ú
		short Z_speed;              //2 bytes //2??×??ú
		short Power_Voltage;        //2 bytes //2??×??ú
		Mpu6050_Data Accelerometer; //6 bytes //6??×??ú
		Mpu6050_Data Gyroscope;     //6 bytes //6??×??ú	
		unsigned char Frame_Tail;   //1 bytes //1??×??ú
	}Sensor_Str;
}SEND_DATA;

void data_transition(void);
void usart1_send(u8 data);
void USART1_SEND(void);
void uart6_init(u32 bound);
#endif


