#include "usart1.h"
#include "includes.h"					//ucos 使用	  
#include "AppTask.h"
/*-----USART1_TX-----PA9-----*/
/*-----USART1_RX-----PA10----*/
//cyq: for test

//		
////unsigned char sbus_rx_buffer[18];
//volatile unsigned char sbus_rx_buffer[25]; 
//volatile  RC_Ctl_t RC_Ctl;







//void USART1_Configuration(void)
//{
//    USART_InitTypeDef usart1;
//	  GPIO_InitTypeDef  gpio;
//    NVIC_InitTypeDef  nvic;
//		DMA_InitTypeDef dma;
//	
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2,ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
//    
//		GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); 
//	
//    gpio.GPIO_Pin = GPIO_Pin_7;
//    gpio.GPIO_Mode = GPIO_Mode_AF;
//    gpio.GPIO_OType = GPIO_OType_PP;
//    gpio.GPIO_Speed = GPIO_Speed_100MHz;
//    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOB,&gpio);
//		
//		USART_DeInit(USART1);
//    usart1.USART_BaudRate = 100000;
//    usart1.USART_WordLength = USART_WordLength_8b;
//    usart1.USART_StopBits = USART_StopBits_1;
//		usart1.USART_Parity = USART_Parity_Even;
//		usart1.USART_Mode =USART_Mode_Rx;
//    usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART1,&usart1);

//    
//    USART_Cmd(USART1,ENABLE);
//		//USART_ClearFlag(USART1, USART_FLAG_IDLE);  
//		//USART_ClearFlag(USART1, USART_IT_IDLE); 
//		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//		USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
//	  nvic.NVIC_IRQChannel = USART1_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//		
//		DMA_Cmd(DMA2_Stream5, DISABLE);
//    while (DMA2_Stream5->CR & DMA_SxCR_EN);
//		DMA_DeInit(DMA2_Stream5);
//    dma.DMA_Channel= DMA_Channel_4;
//    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//    dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
//    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//    dma.DMA_BufferSize = 18;
//    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma.DMA_Mode = DMA_Mode_Circular;
//    dma.DMA_Priority = DMA_Priority_VeryHigh;
//    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//    //dma.DMA_MemoryBurst = DMA_Mode_Normal;
//		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA2_Stream5,&dma);

//		//DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,DISABLE);
//    DMA_Cmd(DMA2_Stream5,ENABLE);
//		
//}



///*
//旧有DMA传输完成中断
//*/
//void DMA2_Stream5_IRQHandler(void)
//{       

//		OSIntEnter();//OS进入中断
//	
//    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
//    {
//			DMA_Cmd(DMA2_Stream5,DISABLE); //关闭DMA传输
//			
//        DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
//        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
//			
////				RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
////				RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
////				RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
////				RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
////				RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
////				RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    

////				RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis        
////				RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis      
////				RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis         

////			
////				RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press      
////				RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press 
////				RC_Ctl.key.ass = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   			//!< KeyBoard value
////				RC_Ctl.keep.che = sbus_rx_buffer[16]| (sbus_rx_buffer[17] << 8);
////				
////				RC_Ctl.key.w = RC_Ctl.key.ass & 0x01 ;
////				RC_Ctl.key.s = (RC_Ctl.key.ass & 0x02) >> 1;
////				RC_Ctl.key.a = (RC_Ctl.key.ass & 0x04) >> 2;
////				RC_Ctl.key.d = (RC_Ctl.key.ass & 0x08) >> 3;
////				RC_Ctl.key.shift = (RC_Ctl.key.ass & 0x10) >> 4;
////				RC_Ctl.key.ctrl = (RC_Ctl.key.ass & 0x20) >> 5;
////				RC_Ctl.key.q = (RC_Ctl.key.ass & 0x40) >> 6;
////				RC_Ctl.key.e = (RC_Ctl.key.ass & 0x80) >> 7;
////				RC_Ctl.key.r = (RC_Ctl.key.ass & 0x100) >> 8;
////				RC_Ctl.key.f = (RC_Ctl.key.ass & 0x200) >> 9;
////				RC_Ctl.key.g = (RC_Ctl.key.ass & 0x400) >> 10;
////				RC_Ctl.key.z = (RC_Ctl.key.ass & 0x800) >> 11;
////				RC_Ctl.key.x = (RC_Ctl.key.ass & 0x1000) >> 12;
////				RC_Ctl.key.c = (RC_Ctl.key.ass & 0x2000) >>13 ;
////				RC_Ctl.key.v = (RC_Ctl.key.ass & 0x4000) >>14;
////				RC_Ctl.key.b = (RC_Ctl.key.ass & 0x8000) >>15;
//				
//				
//				
//					DMA_Cmd(DMA2_Stream5,ENABLE); //关闭DMA传输
//    }
//		
//		OSIntExit();    	//OS退出中断
//}

//short Remote_control_test;



/*
串口中断函数
使用串口空闲中断
*/
//void USART1_IRQHandler(void)
//{
//	
//	u16 data;
//OS_ERR err;
//	
///*
//	旧有串口传输中断
//*/
//	
////    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
////    {
////        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
////        printf("1+1");
////    }
//		OSIntEnter();//OS进入中断

//	

//		if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) 
//       {
//				 Remote_control_test = 5;
//				 
//				 DMA_Cmd(DMA2_Stream5,DISABLE); //关闭DMA传输
//				
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
//				
//				data = USART1->SR; //先读SR，然后读DR才能清除
//				data = USART1->DR;
//				
//				
//				//printf("1+2\r\n");
//				USART_ClearITPendingBit(USART1,USART_IT_IDLE); // 清除标志位
//				DMA_Cmd(DMA2_Stream5,ENABLE); //关闭DMA传输


//			 }
//			 

//			 OSIntExit();    	//OS退出中断
//}




