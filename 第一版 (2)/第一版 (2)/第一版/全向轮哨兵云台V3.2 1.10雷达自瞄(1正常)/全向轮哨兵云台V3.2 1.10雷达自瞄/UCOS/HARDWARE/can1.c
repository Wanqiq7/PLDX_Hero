#include "can1.h"
#include "can2.h"
#include "sys.h"
#include "includes.h"					//ucos 使用	
#include "TIM7_FPS.h"
#include "AppTask.h"
#include <stdio.h>
#include <math.h>
//#include "arm_math.h"
#include <string.h>
#include <stdarg.h>

u16 Muzzle_heat0;		//枪口热量
u16 Muzzle_heat1;		//枪口热量
u16 Muzzle_heat_Limit;		//枪口热量上限
u16 Game_progress;	//比赛阶段：
										//	0：未开始		
										//	1：准备阶段		
										//	2：15秒自检			
										//	3：五秒倒计时		
										//	4：比赛中		
										//	5：比赛结算
										
										
u16 red_outpost_HP;		
u16 blue_outpost_HP;
u16 admit_num;
u16 time;

u16	Remian_hp;

uint8_t errcode;
float chassisPower;
uint16_t chassisPowerLimit;
uint8_t capEnergy;

float Motor_Power[4]={0};

uint8_t CAN1_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN1, &CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
		
		CAN_FilterInitStructure.CAN_FilterNumber = 14;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return 0;
}


void CAN_CMD_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x200;  //底盘所有ID
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CAN2, &TxMessage);
}





short	Velocity_Value_6020[2] = { 0 };
short	Position_Value_6020[2] = { 0 };
short	Torque_Value_6020[2] = { 0 };
unsigned char Temp_Value_6020[2] = { 0 };

short	Velocity_Value_2006[2] = { 0 };
short	Position_Value_2006[2] = { 0 };
short	Torque_Value_2006[2] = { 0 };
unsigned char Temp_Value_2006[2] = { 0 };

short morot1_test;
short morot2_test;
short morot3_test;
short morot4_test;

short can_test[4];

short Ros_Target_Speedx;		//接收来自底盘传来的树莓派指定的底盘目标速度（mm/s），将此装载至自动控制模式下的底盘目标速度变量
short Ros_Target_Speedy;
short Ros_Target_Speedw;



void CAN_CMD_2006(int pitch)
{

 CanTxMsg TxMessage;
    TxMessage.StdId = 0x200;  //底盘所有ID
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;  
    TxMessage.Data[4] = pitch >> 8;
    TxMessage.Data[5] = pitch;
    CAN_Transmit(CAN2, &TxMessage);

}
void CAN1_RX0_IRQHandler( void )
{
	
	CanRxMsg rx_message;
	OSIntEnter();//OS进入中断


	if ( CAN_GetITStatus( CAN1, CAN_IT_FMP0 ) != RESET )
	{
		
		CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
		CAN_Receive( CAN1, CAN_FIFO0, &rx_message );

//		if ( (rx_message.IDE == CAN_Id_Standard) && (rx_message.IDE == CAN_RTR_Data) && (rx_message.DLC == 8) ) /* 标准帧、数据帧、数据长度为8 */
//		{
//			if ( rx_message.StdId == 0x201 )                                                                /* 关于3508电机 */
//			{
//				Temp_Value_3508[0] = rx_message.Data[6];
//				Torque_Value_3508[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
//				Velocity_Value_3508[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
//				Position_Value_3508[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);
//				
//				
//				Velocity_3508_ID1=Velocity_Value_3508[0];	
//				Position_3508_ID1=Position_Value_3508[0];
//				Torque_3508_ID1=Torque_Value_3508[0];
//				Temp_Value_3508_ID1=Temp_Value_3508[0];
//			}
//		}

		
		if ( (rx_message.IDE == CAN_Id_Standard) && (rx_message.IDE == CAN_RTR_Data) && (rx_message.DLC == 8) ) /* 标准帧、数据帧、数据长度为8 */
		{

			if ( rx_message.StdId == 0x201 )                                                                /* 关于3508电机 */
			{
				Temp_Value_3508[0] = rx_message.Data[6];
				Torque_Value_3508[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);
				Motor_Power[0]=fabs(Velocity_Value_3508[0]*Torque_Value_3508[0]*0.0000025567f);				
				morot1_test = 5;					
			}
			
						if ( rx_message.StdId == 0x202 )                                                                /* 关于3508电机 */
			{
				Temp_Value_3508[1] = rx_message.Data[6];
				Torque_Value_3508[1]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508[1]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508[1]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);
				Motor_Power[1]=fabs(Velocity_Value_3508[1]*Torque_Value_3508[1]*0.0000025567f);				
				morot1_test = 5;					
			}
			
									if ( rx_message.StdId == 0x203 )                                                                /* 关于3508电机 */
			{
				Temp_Value_3508[2] = rx_message.Data[6];
				Torque_Value_3508[2]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508[2]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508[2]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);	
				Motor_Power[2]=fabs(Velocity_Value_3508[2]*Torque_Value_3508[2]*0.0000025567f);				
				morot1_test = 5;					
			}
                                    if ( rx_message.StdId == 0x209 )                                                                /* 关于3508电机 */
			{
				Temp_Value_6020[1] = rx_message.Data[6];
				Torque_Value_6020[1]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_6020[1]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_6020[1]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);				
			}
	
			
									if ( rx_message.StdId == 0x204 )                                                                /* 关于3508电机 */
			{
				Temp_Value_3508[3] = rx_message.Data[6];
				Torque_Value_3508[3]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508[3]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508[3]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);
				Motor_Power[3]=fabs(Velocity_Value_3508[3]*Torque_Value_3508[3]*0.0000025567f);
				morot1_test = 5;					
			}
            		 if ( rx_message.StdId == 0x205 )	/* 2006拨弹盘电机 */
			{
				Temp_Value_3508_FW[2] = rx_message.Data[6];
				Torque_Value_3508_FW[2]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508_FW[2]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508_FW[2]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
			}
		
			
			if ( rx_message.StdId == 0x222 )  //接受来自底盘主控的信息                                                         /* 关于3508电机 */
			{
				Game_progress = rx_message.Data[7];
				Muzzle_heat_Limit	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Muzzle_heat1	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);		
				Muzzle_heat0	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);	
					
				morot1_test = 5;					
			}
			
			if ( rx_message.StdId == 0x223 )  //接受来自底盘主控的信息                                                         /* 关于3508电机 */
			{
				time = rx_message.Data[7];
				admit_num	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);	
				blue_outpost_HP	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);		
				red_outpost_HP	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);	
					
				morot1_test = 5;	

			}
			
			if ( rx_message.StdId == 0x224 )  //接受来自底盘主控的信息                                                         /* 关于3508电机 */
			{
//				time = rx_message.Data[7];
//				admit_num	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);	
//				blue_outpost_HP	= (rx_messages.Data[2] << 8) | (rx_message.Data[3]);		
				Remian_hp	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);	
					
				morot1_test = 5;	
							
			}
		if ( (rx_message.IDE == CAN_Id_Standard) && (rx_message.IDE == CAN_RTR_Data) && (rx_message.DLC == 8) ) /* ?????????????????????8 */
		{
			if ( rx_message.StdId == 0x051 )    
			{
				errcode=rx_message.Data[0];
				uint32_t combined = (uint32_t)(rx_message.Data[4]) << 24 |(uint32_t)(rx_message.Data[3]) << 16 |(uint32_t)(rx_message.Data[2]) << 8 |(uint32_t)(rx_message.Data[1]);
				chassisPowerLimit=rx_message.Data[6]<<8|rx_message.Data[5];
				chassisPower=*((float*)&combined);
				capEnergy = rx_message.Data[7];

			}	
	  }
			
//			if ( rx_message.StdId == 0x220 )   //接收底盘数据：树莓派指定的目标速度，
//			{
//				Ros_Target_Speedw	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
//				Ros_Target_Speedy	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
//				Ros_Target_Speedx	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);		
//				morot1_test = 5;					
//			}
			
//						if ( rx_message.StdId == 0x203 )                                                                /* 关于3508电机 */
//			{
//				Temp_Value_2006[0] = rx_message.Data[6];
//				Torque_Value_2006[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
//				Velocity_Value_2006[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
//				Position_Value_2006[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);		
//				morot1_test = 5;					
//			}

			
						else if ( rx_message.StdId == 0x212 )                                                                /* 关于3508电机 */
			{
				SuperCap_power[3] = (rx_message.Data[7] << 8) | (rx_message.Data[6]);
				SuperCap_power[2]	= (rx_message.Data[5] << 8) | (rx_message.Data[4]);
				SuperCap_power[1]	= (rx_message.Data[3] << 8) | (rx_message.Data[2]);
				SuperCap_power[0]	= (rx_message.Data[1] << 8) | (rx_message.Data[0]);				
			}
			
			

			
		}
	}
	

	
	
	
	OSIntExit();    	//OS退出中断
}

