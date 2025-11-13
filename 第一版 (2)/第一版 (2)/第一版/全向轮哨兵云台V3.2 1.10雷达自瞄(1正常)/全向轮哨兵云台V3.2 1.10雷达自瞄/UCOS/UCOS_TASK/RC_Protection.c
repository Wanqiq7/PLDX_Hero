#include "RC_Protection.h"

//#include "usart1.h"
#include "timer.h"

#include "can1.h"
#include "can2.h"
#include "GimbalTask.h"
#include "usart.h" 
#include "usart3_rc.h" 

extern short gyro_0,gyro_1,gyro_2;
extern short accel_0,accel_1,accel_2;

/**************************
遥控器保护任务
遥控器二级保护
主动分析通道是否正确
频率79HZ
**************************/
extern int Chassis_Mode,FW_Mode;
/**********************************************************************************、
任务名：RC_Protection_task(void *p_arg)
形式参数：无
作者：
时间：2022年2月3日
功能：
丢控时遥控器值清零，促使机器人进入保护模式
任务频率：79FPS
**********************************************************************************/
void RC_Protection_task(void *p_arg)
{
	OS_ERR err;

//	printf("遥控器保护任务 开始\r\n");


	while(1)
	{
		
		/*********
		技术样例
		*********/
//		GPIO_SetBits(GPIOI,GPIO_Pin_2);
//		if((RC_Ctl.rc.ch0 < 1685) && (RC_Ctl.rc.ch0 > 363)){RC_P.rc.ch0 = RC_Ctl.rc.ch0;}
//		if((RC_Ctl.rc.ch1 < 1685) && (RC_Ctl.rc.ch1 > 363)){RC_P.rc.ch1 = RC_Ctl.rc.ch1;}
//		if((RC_Ctl.rc.ch2 < 1685) && (RC_Ctl.rc.ch2 > 363)){RC_P.rc.ch2 = RC_Ctl.rc.ch2;}
//		if((RC_Ctl.rc.ch3 < 1685) && (RC_Ctl.rc.ch3 > 363)){RC_P.rc.ch3 = RC_Ctl.rc.ch3;}
//		
//		if((RC_Ctl.rc.s1 < 4) && (RC_Ctl.rc.s1 > 0)){RC_P.rc.s1 = RC_Ctl.rc.s1;}
//		if((RC_Ctl.rc.s2 < 4) && (RC_Ctl.rc.s2 > 0)){RC_P.rc.s2 = RC_Ctl.rc.s2;}
//						
//		
//		if((RC_Ctl.mouse.x < 32767) && (RC_Ctl.mouse.x > -32767)){RC_P.mouse.x = RC_Ctl.mouse.x;}
//		if((RC_Ctl.mouse.y < 32767) && (RC_Ctl.mouse.y > -32767)){RC_P.mouse.y = RC_Ctl.mouse.y;}
//		if((RC_Ctl.mouse.z < 32767) && (RC_Ctl.mouse.z > -32767)){RC_P.mouse.z = RC_Ctl.mouse.z;}

//		if((RC_Ctl.keep.che < 1685) && (RC_Ctl.keep.che > 363)){RC_P.keep.che = RC_Ctl.keep.che;}
//		
//		
////		GPIO_SetBits(GPIOI,GPIO_Pin_2);
////		delay_ms(100);
////		GPIO_ResetBits(GPIOI,GPIO_Pin_2);
		RC_Protection--;
		data_transition();
		USART1_SEND();

//	printf("g0=%d,g1=%d,g2=%d\r\n\r\n",gyro_0,gyro_1,gyro_2);
//	printf("%d,%d,%d\r\n\r\n",accel_0,accel_1,accel_2);
		CAN1_Send_queue_add(0x210,0x08,10000,0,0,0);//控制四个底盘电机

		if(RC_Protection < 1 )
		{
			RC_Ctl.rc.s1 = 0;
			RC_Ctl.rc.s2 = 0;
			
			RC_Ctl.mouse.x = 0;
			RC_Ctl.mouse.y = 0;
			RC_Ctl.mouse.z = 0;
			
			RC_Ctl.rc.ch0 = 1024;
			RC_Ctl.rc.ch1 = 1024;
			RC_Ctl.rc.ch2 = 1024;
			RC_Ctl.rc.ch3 = 1024;
						
			RC_Ctl.mouse.press_l = 0;                                        //!< Mouse Left Is Press      
			RC_Ctl.mouse.press_r = 0;                                        //!< Mouse Right Is Press 
			RC_Ctl.key.ass = 0;   			//!< KeyBoard value
			RC_Ctl.keep.che = 1024;
				
			RC_Ctl.key.w = RC_Ctl.key.ass & 0x01 ;
			RC_Ctl.key.s = (RC_Ctl.key.ass & 0x02) >> 1;
			RC_Ctl.key.a = (RC_Ctl.key.ass & 0x04) >> 2;
			RC_Ctl.key.d = (RC_Ctl.key.ass & 0x08) >> 3;
			RC_Ctl.key.shift = (RC_Ctl.key.ass & 0x10) >> 4;
			RC_Ctl.key.ctrl = (RC_Ctl.key.ass & 0x20) >> 5;
			RC_Ctl.key.q = (RC_Ctl.key.ass & 0x40) >> 6;
			RC_Ctl.key.e = (RC_Ctl.key.ass & 0x80) >> 7;
			RC_Ctl.key.r = (RC_Ctl.key.ass & 0x100) >> 8;
			RC_Ctl.key.f = (RC_Ctl.key.ass & 0x200) >> 9;
			RC_Ctl.key.g = (RC_Ctl.key.ass & 0x400) >> 10;
			RC_Ctl.key.z = (RC_Ctl.key.ass & 0x800) >> 11;
			RC_Ctl.key.x = (RC_Ctl.key.ass & 0x1000) >> 12;
			RC_Ctl.key.c = (RC_Ctl.key.ass & 0x2000) >>13 ;
			RC_Ctl.key.v = (RC_Ctl.key.ass & 0x4000) >>14;
			RC_Ctl.key.b = (RC_Ctl.key.ass & 0x8000) >>15;
			Chassis_Mode=0;
			FW_Mode =0;
		}
		
		
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms
		
		
	}
}



