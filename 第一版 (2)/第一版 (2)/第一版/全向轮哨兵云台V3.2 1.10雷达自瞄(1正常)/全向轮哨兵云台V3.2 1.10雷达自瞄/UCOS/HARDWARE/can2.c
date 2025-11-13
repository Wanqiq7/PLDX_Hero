#include "can2.h"
#include "sys.h"
#include "includes.h"					//ucos 使用	  
#include <stdio.h>
#include <math.h>
//#include "arm_math.h"
#include <string.h>
#include <stdarg.h>

long int now_post1_switch,last_post1_switch,post_target1_switch;
long int post_erro1_switch;

uint8_t CAN2_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN2, &CAN_InitStructure);

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

    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return 0;
}

//void CAN_CMD_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
//{
//    CanTxMsg TxMessage;
//    TxMessage.StdId = 0x200;  //底盘所有ID
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 0x08;
//    TxMessage.Data[0] = motor1 >> 8;
//    TxMessage.Data[1] = motor1;
//    TxMessage.Data[2] = motor2 >> 8;
//    TxMessage.Data[3] = motor2;
//    TxMessage.Data[4] = motor3 >> 8;
//    TxMessage.Data[5] = motor3;
//    TxMessage.Data[6] = motor4 >> 8;
//    TxMessage.Data[7] = motor4;

//    CAN_Transmit(CAN1, &TxMessage);
//}


	
short	Velocity_Value_3508[4];
short	Position_Value_3508[4];
short	Torque_Value_3508[4];
unsigned char Temp_Value_3508[4];
 
short	Velocity_Value_3508_FW[4];
short	Position_Value_3508_FW[4];
short	Torque_Value_3508_FW[4];
short	SuperCap_power[4];
unsigned char Temp_Value_3508_FW[4];

int last_position;
extern int M2006_counter;
int position_error;
int number_of_turns;

extern long int position2006_target;
extern long int first_position2006_target;
char po_init;
extern char Barrel_init_state;

void CAN2_RX0_IRQHandler( void )
{
	CanRxMsg rx_message;
	OSIntEnter();//OS进入中断
	
	if ( CAN_GetITStatus( CAN2, CAN_IT_FMP0 ) != RESET )
	{
		CAN_ClearITPendingBit( CAN2, CAN_IT_FMP0 );
		CAN_Receive( CAN2, CAN_FIFO0, &rx_message );

		if ( (rx_message.IDE == CAN_Id_Standard) && (rx_message.IDE == CAN_RTR_Data) && (rx_message.DLC == 8) ) /* 标准帧、数据帧、数据长度为8 */
		{
			if ( rx_message.StdId == 0x201 ) /* 3508摩擦轮电机 */
			{
				Temp_Value_3508_FW[0] = rx_message.Data[6];
				Torque_Value_3508_FW[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508_FW[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508_FW[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);				
			}
		
		
		else if ( rx_message.StdId == 0x202 )	/* 3508摩擦轮电机 */
			{
				Temp_Value_3508_FW[1] = rx_message.Data[6];
				Torque_Value_3508_FW[1]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508_FW[1]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508_FW[1]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
			}
		
//		else if ( rx_message.StdId == 0x205 )	/* 2006拨弹盘电机 */
//			{
//				Temp_Value_3508_FW[2] = rx_message.Data[6];
//				Torque_Value_3508_FW[2]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
//				Velocity_Value_3508_FW[2]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
//				Position_Value_3508_FW[2]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
//			}
//		
		
		else if ( rx_message.StdId == 0x204 ) /* 2006枪管切换电机 */
			{
				Temp_Value_3508_FW[3] = rx_message.Data[6];
				Torque_Value_3508_FW[3]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508_FW[3]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508_FW[3]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
				
			//////计算2006转动角度//////
				
	now_post1_switch = Position_Value_3508_FW[3];	
		if(now_post1_switch >= last_post1_switch)
		{
			if((now_post1_switch-last_post1_switch) >= 4095)
			{
				post_erro1_switch = -last_post1_switch  + now_post1_switch - 8191;
			}	
			else if((now_post1_switch-last_post1_switch) < 4095)
			{
				post_erro1_switch = now_post1_switch - last_post1_switch ;
			}
		}	
		else if(now_post1_switch < last_post1_switch)
		{
			if((last_post1_switch-now_post1_switch) >= 4095)
			{
				post_erro1_switch =  now_post1_switch + 8191 - last_post1_switch;
			}	
						if((last_post1_switch-now_post1_switch) < 4095)
			{
				post_erro1_switch =  now_post1_switch - last_post1_switch;
			}	
		}
			M2006_counter=M2006_counter+post_erro1_switch;
		if(po_init==0&&Barrel_init_state==2)			//获取枪管初始位置 第一根枪管
				{
					position2006_target=M2006_counter+(int)(8912*17.6);		//减小偏左		//18.42  18.341
					first_position2006_target=M2006_counter+(int)(8912*17.6);
					po_init=1;
				}
			last_post1_switch = Position_Value_3508_FW[3];
		}
		///////end//////
			
		else if ( rx_message.StdId == 0x205 )                                                                /* 关于6020电机 */
			{
				Temp_Value_6020[0] = rx_message.Data[6];
				Torque_Value_6020[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_6020[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_6020[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
			}
		
//		else if ( rx_message.StdId == 0x206 )                                                                /* 关于3508电机 */
//			{
//				Temp_Value_6020[1] = rx_message.Data[6];
//				Torque_Value_6020[1]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
//				Velocity_Value_6020[1]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
//				Position_Value_6020[1]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
//			}
//			

			
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
