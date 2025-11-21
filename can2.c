#include "can2.h"
#include "sys.h"
#include "includes.h"					//ucos 使用	  
#include <stdio.h>
#include <math.h>
//#include "arm_math.h"
#include <string.h>
#include <stdarg.h>

Command received_command = {0};

// 3. 上位机CAN参数配置
#define QUATERNION_CANID    0x100  // 四元数帧ID(对应上位机 quaternion_canid_)
#define BULLET_MODE_CANID   0x101  // 子弹速度ID(对应上位机 bullet_speed_canid_)


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


short	Velocity_Value_3508[4];
short	Position_Value_3508[4];
short	Torque_Value_3508[4];
unsigned char Temp_Value_3508[4];
 
short	Velocity_Value_3508_FW[4];
short	Position_Value_3508_FW[4];
short Last_Position_Value_3508_FW[4];
short	Torque_Value_3508_FW[4];
short	SuperCap_power[4];
unsigned char Temp_Value_3508_FW[4];

// 用于存储6020型号电机的控制数据（2个电机）
short Velocity_Value_6020[2] = { 0 };    // 速度值（单位可能为RPM或自定义单位）
short Position_Value_6020[2] = { 0 };    // 位置值（单位可能为编码器脉冲或角度）
short Torque_Value_6020[2] = { 0 };     // 扭矩值（单位可能为N·m或百分比）
unsigned char Temp_Value_6020[2] = { 0 }; // 温度值（单位通常为℃）

// 用于存储2006型号电机的控制数据（2个电机）
short Velocity_Value_2006[2] = { 0 };    // 速度值
short Position_Value_2006[2] = { 0 };    // 位置值
short Torque_Value_2006[2] = { 0 };      // 扭矩值
unsigned char Temp_Value_2006[2] = { 0 }; // 温度值

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
			if ( rx_message.StdId == 0x201 )//左摩擦轮3508 id 200 + 1                                                               
			{
				Temp_Value_3508_FW[0] = rx_message.Data[6];
				Torque_Value_3508_FW[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508_FW[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508_FW[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);				
			}
			
		  else if ( rx_message.StdId == 0x202 )//右摩擦轮3508 id 200 + 2                                                               
			{
				Temp_Value_3508_FW[1] = rx_message.Data[6];
				Torque_Value_3508_FW[1]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508_FW[1]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508_FW[1]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
			}
		
		  else if ( rx_message.StdId == 0x203 )//拨弹盘2006 id 200 + 3                                                                
			{
				Temp_Value_3508_FW[2] = rx_message.Data[6];
				Torque_Value_3508_FW[2]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_3508_FW[2]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_3508_FW[2]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
			}
		
		  else if ( rx_message.StdId == 0x205 )//云台6020 id 204 + 1                                                               
			{
				Temp_Value_6020[0] = rx_message.Data[6];
				Torque_Value_6020[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_6020[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_6020[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);			
			}
					
			else if ( rx_message.StdId == 0x211 )                                                                /* 关于超级电容 */
			{
				SuperCap_power[3] = (rx_message.Data[7] << 8) | (rx_message.Data[6]);
				SuperCap_power[2]	= (rx_message.Data[5] << 8) | (rx_message.Data[4]);
				SuperCap_power[1]	= (rx_message.Data[3] << 8) | (rx_message.Data[2]);
				SuperCap_power[0]	= (rx_message.Data[1] << 8) | (rx_message.Data[0]);				
			}
      // 上位机的(ID: 0xff)
      else if (rx_message.StdId == 0xff)
      {
         // 控制与开火指令
         received_command.control = rx_message.Data[0];
         received_command.shoot = rx_message.Data[1];
                
         // yaw (int16_t -> float)
         int16_t yaw_raw = (rx_message.Data[2] << 8) | rx_message.Data[3];
         received_command.yaw = (float)yaw_raw / 1e4;
                
         // pitch
         int16_t pitch_raw = (rx_message.Data[4] << 8) | rx_message.Data[5];
         received_command.pitch = (float)pitch_raw / 1e4;
                
         // 水平距离
         int16_t distance_raw = (rx_message.Data[6] << 8) | rx_message.Data[7];
         received_command.horizon_distance = (float)distance_raw / 1e4;
      }
		}		 
	}
	OSIntExit();    	//OS退出中断
}
