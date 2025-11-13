#include "usart3_rc.h"

#include "FlickTask.h"
#include "AppTask.h"


#include "includes.h"					//ucos 使用	  
#include "AppTask.h"


/****************************
遥控器接收驱动初始化
USART3
RX:PC11
使用IDLE串口空闲中断处理
接收过程使用DMA传输数据，以减少主控负担
*****************************/
volatile unsigned char sbus_rx_buffer[25]; 
volatile  RC_Ctl_t RC_Ctl;
volatile unsigned short int last_target=0;
extern int Chassis_Mode,FW_Mode;

void USART3_Configuration(void)
{
    USART_InitTypeDef usart3;
	  GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef dma;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1,ENABLE);
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); 
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
	
	
	
    gpio.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC,&gpio);
		
		USART_DeInit(USART3);
    usart3.USART_BaudRate = 100000;
    usart3.USART_WordLength = USART_WordLength_8b;
    usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_Even;
		usart3.USART_Mode =USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3,&usart3);

    
    USART_Cmd(USART3,ENABLE);
		//USART_ClearFlag(USART1, USART_FLAG_IDLE);  
		USART_ClearFlag(USART3, USART_IT_IDLE); 
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
	  nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		DMA_Cmd(DMA1_Stream1, DISABLE);
    while (DMA1_Stream1->CR & DMA_SxCR_EN);
		DMA_DeInit(DMA1_Stream1);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = 18;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    //dma.DMA_MemoryBurst = DMA_Mode_Normal;
		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1,&dma);

		//DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,DISABLE);
    DMA_Cmd(DMA1_Stream1,ENABLE);
		
		
		//printf("串口3初始化完成");
}



/*
旧有DMA传输完成中断
*/
void DMA1_Stream1_IRQHandler(void)
{       

		OSIntEnter();//OS进入中断
	
    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
    {
			DMA_Cmd(DMA1_Stream1,DISABLE); //关闭DMA传输
			
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
			
//				RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
//				RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
//				RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
//				RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
//				RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
//				RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    

//				RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis        
//				RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis      
//				RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis         

//			
//				RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press      
//				RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press 
//				RC_Ctl.key.ass = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   			//!< KeyBoard value
//				RC_Ctl.keep.che = sbus_rx_buffer[16]| (sbus_rx_buffer[17] << 8);
//				
//				RC_Ctl.key.w = RC_Ctl.key.ass & 0x01 ;
//				RC_Ctl.key.s = (RC_Ctl.key.ass & 0x02) >> 1;
//				RC_Ctl.key.a = (RC_Ctl.key.ass & 0x04) >> 2;
//				RC_Ctl.key.d = (RC_Ctl.key.ass & 0x08) >> 3;
//				RC_Ctl.key.shift = (RC_Ctl.key.ass & 0x10) >> 4;
//				RC_Ctl.key.ctrl = (RC_Ctl.key.ass & 0x20) >> 5;
//				RC_Ctl.key.q = (RC_Ctl.key.ass & 0x40) >> 6;
//				RC_Ctl.key.e = (RC_Ctl.key.ass & 0x80) >> 7;
//				RC_Ctl.key.r = (RC_Ctl.key.ass & 0x100) >> 8;
//				RC_Ctl.key.f = (RC_Ctl.key.ass & 0x200) >> 9;
//				RC_Ctl.key.g = (RC_Ctl.key.ass & 0x400) >> 10;
//				RC_Ctl.key.z = (RC_Ctl.key.ass & 0x800) >> 11;
//				RC_Ctl.key.x = (RC_Ctl.key.ass & 0x1000) >> 12;
//				RC_Ctl.key.c = (RC_Ctl.key.ass & 0x2000) >>13 ;
//				RC_Ctl.key.v = (RC_Ctl.key.ass & 0x4000) >>14;
//				RC_Ctl.key.b = (RC_Ctl.key.ass & 0x8000) >>15;
				
				
				
					DMA_Cmd(DMA1_Stream1,ENABLE); //关闭DMA传输
    }
		
		OSIntExit();    	//OS退出中断
}

short RC_Protection;


short int Average_filtering(short int x,short int N)
{
	unsigned short int new_target;
	new_target=x;
	if (new_target-last_target>N || last_target-new_target>N )
	{
		last_target=new_target;
		return new_target;
	}
	else return last_target;
}
/*
串口中断函数
使用串口空闲中断
*/
//void flag_cheak()
//{
//	if(key_flag[0]==1)
//	{
//		RC_Ctl.rc.s2 = 2;
//	}
//}

void USART3_IRQHandler(void)
{
	
	u16 data;
 OS_ERR err;
	
	
/*
	旧有串口传输中断
*/
//    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//    {
//        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//        printf("1+1");
//    }
		OSIntEnter();//OS进入中断

		//printf("usart3\r\n");

		if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) 
       {
				 //printf("IDLE\r\n");
				 
				 
				 RC_Protection = 5;
				 
				 DMA_Cmd(DMA1_Stream1,DISABLE); //关闭DMA传输
				
				RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
				RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
				RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
				RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
				RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
				RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    

				RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis        
				RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis      
				RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis         

			
				RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press      
				RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press 
				RC_Ctl.key.ass = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   			//!< KeyBoard value
				RC_Ctl.keep.che = sbus_rx_buffer[16]| (sbus_rx_buffer[17] << 8);
				
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
				
				data = USART3->SR; //先读SR，然后读DR才能清除
				data = USART3->DR;
				
				
				RC_Ctl.rc.ch0 =  RC_Ctl.rc.ch0 + Average_filtering(RC_Ctl.mouse.x,2500);                     
				RC_Ctl.rc.ch1 =  RC_Ctl.rc.ch1 - Average_filtering(RC_Ctl.mouse.y,2500);  
				RC_Ctl.rc.ch2 = RC_Ctl.rc.ch2 - RC_Ctl.key.a*150 + RC_Ctl.key.d*150; 
				RC_Ctl.rc.ch3 = RC_Ctl.rc.ch3 + RC_Ctl.key.w*150 - RC_Ctl.key.s*150;
        
        if(RC_Ctl.mouse.press_l==1)
				{
					RC_Ctl.rc.s2 = 1;
				}
//				if(RC_Ctl.mouse.press_r==1)
//				{
//					RC_Ctl.rc.s2 = 3;
//				}
				 if(RC_Ctl.key.q==1)
				{
					Chassis_Mode = 1;
				}
				if(RC_Ctl.key.z==1)
				{
					Chassis_Mode = 2;
				}
				 if(RC_Ctl.key.e==1)
				{
					Chassis_Mode = 3;
				}
				
				
				//printf("1+2\r\n");
				USART_ClearITPendingBit(USART3,USART_IT_IDLE); // 清除标志位
				DMA_Cmd(DMA1_Stream1,ENABLE); //使能DMA传输


			 }
			 
//       flag_cheak();
			 OSIntExit();    	//OS退出中断
}






