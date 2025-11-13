#include "sys.h"
#include "usart.h"	
#include "can1.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
int TV_target[3];

uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

//uint8_t Serial_RxPacket[12];

int angle[3];
int angle_g[3];
int angle_v[3];
int angle_v_int[3];
short USART_RX_STA1; 
int USART_RX_BUF1[10];
int game_state;
extern int accel_souce[3],gyro_souce[3],temp;

#define Max_BUFF_Len 31
unsigned char Uart2_Buffer[Max_BUFF_Len];
unsigned int Uart2_Rx=0;
int Data,cx,cy,cw,ch;		//
extern int ss;
extern unsigned char Lose_Count;					//目标丢失时间计数器

extern float speed_x,speed_y,speed_w;		//xyz轴目标速度 单位：m/s

extern float Real_speed_x,Real_speed_y,Real_speed_w;		//xyz轴实际速度 单位：m/s
extern float V1,V2,V3,V4;							//换算出的每个电机转速

extern short gyro_0,gyro_1,gyro_2;
extern short accel_0,accel_1,accel_2;

extern int accel_souce[3],gyro_souce[3],temp,accel[3];

char arrive;	//雷达发送的指令

/*----地底盘与ROS通信---------------------------------------------start---------------------------------------------------------------*/

/*1.**********************************************************  
*  @Function      void uart_init(u32 bound)
*  @Decription    进行串口相关配置
*  @input         波特率
*  @output        无
*  @return        无
*  @Attention     
*************************************************************/
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;

void uart_init(u32 bound){
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
	
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 


	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、


}


/*2.**********************************************************  
*  @Function      float XYZ_Target_Speed_transition(u8 High,u8 Low)
*  @Decription    将发送的八位数据合成为16位
*  @input         高八位  低八位
*  @output        无
*  @return        转化后的16位
*  @Attention     
*************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s						
}


/*3.**********************************************************  
*  @Function      u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
*  @Decription    将数据进行BBC校验
*  @input         输入的数据      模式
*  @output        无
*  @return        校验位
*  @Attention     
*************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//对接收到的数据进行校验
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}



/*4.**********************************************************  
*  @Function      void USART1_IRQHandler(void)
*  @Decription    接受ROS下发的数据
*  @input         无
*  @output        无
*  @return        无
*  @Attention     
*************************************************************/
void USART1_IRQHandler(void)
{	
	static u8 Count=0;
	u8 Usart_Receive;

	
	OSIntEnter();//OS进入中断  
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //判断是否接收到数据
	{
		
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);      //清除中断标志
		Usart_Receive = USART_ReceiveData(USART1);          //读取数据
		//串口数据填入数组
		Receive_Data.buffer[Count]=Usart_Receive;
		//确保数组第一个数据为FRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else 
			Count=0;
		
		if (Count == 11)                              //验证数据包的长度
		{   
				Count=0;                                  //为串口数据重新填入数组做准备
				if(Receive_Data.buffer[10] == FRAME_TAIL) //验证数据包的帧尾
				{
					                                        //数据异或位校验计算，模式0是发送数据校验
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
				  {	
						//从串口数据求三轴目标速度， 单位m/s
						arrive=Receive_Data.buffer[1];
						speed_x=(XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]));
						speed_y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						speed_w=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
//						printf("x=%f\r\n",speed_x);
//						printf("y=%f\r\n",speed_y);
//						printf("z=%f\r\n",speed_w);
		        				
				  }
 
			  }
		}
	} 
	OSIntExit();    	//退出中断
}




/*5.**********************************************************  
*  @Function      void data_transition(void)
*  @Decription    将需要发送给ROS的数据装载
*  @input         无
*  @output        无
*  @return        无
*  @Attention     
*************************************************************/
void data_transition(void)
{
	//电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
	Send_Data.Sensor_Str.Power_Voltage = 1000;//Position_Value_6020[1]; 
	Send_Data.buffer[0]= 0X7B; //帧头
	Send_Data.buffer[1]= game_state;                                 //预留位
	//电池电压,拆分为两个8位数据发送
	Send_Data.buffer[2]=Send_Data.Sensor_Str.Power_Voltage>>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.Power_Voltage; 
  //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[4]=Check_Sum(4,1); 	
	Send_Data.buffer[5]= 0X7D;//帧尾
}
/*6..**********************************************************  
*  @Function      void USART1_SEND(void)
*  @Decription    串口发送函数，用于将装载后的数据发送
*  @input         无
*  @output        无
*  @return        无
*  @Attention     
*************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}

void USART1_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<6; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}

/*----地底盘与ROS通信---------------------------------------------end---------------------------------------------------------------*/



/*****************************
串口6：
功能：
1.实现串口数据发送
2.与视觉计算机通信
*****************************/
void uart6_init(u32 bound){
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOA时钟
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
 
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //GPIOA10复用为USART1
	
	//USART6端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PA9，PA10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PA9，PA10

   //USART6 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART6, ENABLE);  //使能串口1 


	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、


	
}

//uint8_t Serial_GetRxFlag(void)
//{
//	if (Serial_RxFlag == 1)
//	{
//		Serial_RxFlag = 0;
//		return 1;
//	}
//	return 0;
//}

//uint8_t Serial_GetRxData(void)
//{
//	return Serial_RxData;
//}


void USART6_IRQHandler(void)                	//串口6中断服务程序
{
		
//	static uint8_t RxState = 0;
//	static uint8_t pRxPacket = 0;
//	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
//	{
//		uint8_t RxData = USART_ReceiveData(USART1);
//		
//			if (RxState == 0)
//		{
//			if (RxData == 0xFF)
//			{
//				RxState = 1;
//				pRxPacket = 0;
//			}
//		}
//		else if (RxState == 1)
//			{
//			Serial_RxPacket[pRxPacket] = RxData;
//			pRxPacket ++;
//			if (pRxPacket >= 12)
//			{
//				RxState = 2;
//			}
//		}
//		else if (RxState == 2)
//		{
//			if (RxData == 0xFE)
//			{
//				RxState = 0;
//				Serial_RxFlag = 1;
//				int32_t centerX = (Serial_RxPacket[0] << 24) | (Serial_RxPacket[1] << 16) | (Serial_RxPacket[2] << 8) | Serial_RxPacket[3];
//				int32_t centerY = (Serial_RxPacket[4] << 24) | (Serial_RxPacket[5] << 16) | (Serial_RxPacket[6] << 8) | Serial_RxPacket[7];
//				int32_t distance = (Serial_RxPacket[8] << 24) | (Serial_RxPacket[9] << 16) | (Serial_RxPacket[10] << 8) | Serial_RxPacket[11];
//				cx = centerX;
//				cy = centerY;
//				cw = distance;
//				printf("%d,%d\r",cx,cy);
////		data[2] = distance;
//			}
//		}
//		
//		
//		
//		
//		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
//	}
//}
	u8 com_data; 
//		u8 i;
		static u8 RxCounter1=0;
		static uint16_t Serial_RxPacket[14];
		static u8 RxState = 0;	
		static u8 RxFlag1 = 0;
		
	OSIntEnter();//OS进入中断    

	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		
		{
				USART_ClearITPendingBit(USART6,USART_IT_RXNE);   //清除中断标志
			
			if(cx!=0)
			{
				Lose_Count=0;	//接收到数据证明openmv找到目标， 将计数器清零
			}
				
				com_data = USART_ReceiveData(USART6);
			
				if(RxState==0)  //
				{
					if(com_data == 0x0A)
					{
						RxState=1;
						RxCounter1=0;
					}
					//Serial_RxPacket[RxCounter1++]=com_data;
				}
				else if (RxState == 1)
					{
					Serial_RxPacket[RxCounter1]=com_data;
					RxCounter1 ++;	
					if (RxCounter1 >= 12)
					{
						RxState = 2;
					}
				}
//				else if(RxState==1&&com_data==0x12)  //0x12帧头
//				{
//					RxState=2;
//					RxBuffer1[RxCounter1++]=com_data;
//				}
		
				else if(RxState==2)
				{
					Serial_RxPacket[RxCounter1++]=com_data;

					if(com_data == 0x0B)       //RxBuffer1接受满了,接收数据结束
					{
						RxState=0;
						RxFlag1=1;
						int32_t centerX = (Serial_RxPacket[0] << 24) | (Serial_RxPacket[1] << 16) | (Serial_RxPacket[2] << 8) | Serial_RxPacket[3];
						int32_t centerY = (Serial_RxPacket[4] << 24) | (Serial_RxPacket[5] << 16) | (Serial_RxPacket[6] << 8) | Serial_RxPacket[7];
						int32_t distance = (Serial_RxPacket[8] << 24) | (Serial_RxPacket[9] << 16) | (Serial_RxPacket[10] << 8) | Serial_RxPacket[11];
						cx = centerX;
						cy = centerY;
						cw = distance;
	//					printf("%d,%d,%d\r",cx,cy,cw);						
//						cx=RxBuffer1[RxCounter1-5];
//						cy=RxBuffer1[RxCounter1-4];
//	
//						cw=RxBuffer1[RxCounter1-3];
//						ch=RxBuffer1[RxCounter1-2];
						

					}
				}
		
//				else if(RxState==3)		//检测是否接受到结束标志
//				{
//						if( com_data == 0xFE)
//						{
//									USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);//关闭DTSABLE中断
//									//printf("%d,%d\r",cx,cy);
////									if(RxFlag1)
////									{
//////										ss=cx;
////									//printf("%d,%d,%d,%d\r",cx,cy,cw);
//////									OLED_ShowNum(1,3,cx,4);
//////									OLED_ShowNum(2,3,cy,4);
//////									OLED_ShowNum(3,3,cw,4);
//////									OLED_ShowNum(4,3,ch,4);
////									}
////									RxFlag1 = 0;
////									RxCounter1 = 0;
////									RxState = 0;
////									USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
						}
//						else   //接收错误
//						{
//									RxState = 0;
//									RxCounter1=0;
//									for(i=0;i<12;i++)
//									{
//											Serial_RxPacket[i]=0x00;      //将存放数据数组清零
//									}
//						}
//				} 
//	
//				else   //接收异常
//				{
//						RxState = 0;
//						RxCounter1=0;
//						for(i=0;i<12;i++)
//						{
//								Serial_RxPacket[i]=0x00;      //将存放数据数组清零
//						}
//				}

//		}
	

	}
	
	
//USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	

	OSIntExit();    	//退出中断

} 



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void USART6_IRQHandler(void)                	//串口6中断服务程序
//{
//	u8 com_data; 
//		u8 i;
//		static u8 RxCounter1=0;
//		static u16 RxBuffer1[15]={0};
//		static u8 RxState = 0;	
//		static u8 RxFlag1 = 0;
//		
//	OSIntEnter();//OS进入中断    

//	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//		
//		{
//				USART_ClearITPendingBit(USART6,USART_IT_RXNE);   //清除中断标志
//			
//			//Lose_flag=0;	//接收到数据证明openmv找到目标， 将计数器清零
//				com_data = USART_ReceiveData(USART6);
//			
//				if(RxState==0&&com_data==0x2C)  //0x2c帧头
//				{
//					RxState=1;
//					RxBuffer1[RxCounter1++]=com_data;
//				}
//		
//				else if(RxState==1&&com_data==0x12)  //0x12帧头
//				{
//					RxState=2;
//					RxBuffer1[RxCounter1++]=com_data;
//				}
//		
//				else if(RxState==2)
//				{
//					RxBuffer1[RxCounter1++]=com_data;

//					if(RxCounter1>=15||com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
//					{
//						RxState=3;
//						RxFlag1=1;
//						//接收py视觉代码协议代码
////						cx=(RxBuffer1[RxCounter1-4]<<8)+RxBuffer1[RxCounter1-5];
////						cy=(RxBuffer1[RxCounter1-2]<<8)+RxBuffer1[RxCounter1-3];
////						
//						cx=(RxBuffer1[RxCounter1-13]<<24)|(RxBuffer1[RxCounter1-12]<<16)|(RxBuffer1[RxCounter1-11]<<8)|(RxBuffer1[RxCounter1-10]);
//						cy=(RxBuffer1[RxCounter1-9]<<24)|(RxBuffer1[RxCounter1-8]<<16)|(RxBuffer1[RxCounter1-7]<<8)|(RxBuffer1[RxCounter1-6]);
////						L=(RxBuffer1[RxCounter1-2]<<24)|(RxBuffer1[RxCounter1-3]<<16)|(RxBuffer1[RxCounter1-4]<<8)|(RxBuffer1[RxCounter1-5]);
//						

//						

//					}
//				}
//		
//				else if(RxState==3)		//检测是否接受到结束标志
//				{
//						if(RxBuffer1[RxCounter1-1] == 0x5b)
//						{
//									USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);//关闭DTSABLE中断
//									if(RxFlag1)
//									{
////										ss=cx;
//									//printf("%d,%d\r",cx,cy);
////									OLED_ShowNum(1,3,cx,4);
////									OLED_ShowNum(2,3,cy,4);
////									OLED_ShowNum(3,3,cw,4);
////									OLED_ShowNum(4,3,ch,4);
//									}
//									RxFlag1 = 0;
//									RxCounter1 = 0;
//									RxState = 0;
//									USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
//						}
//						else   //接收错误
//						{
//									RxState = 0;
//									RxCounter1=0;
//									for(i=0;i<10;i++)
//									{
//											RxBuffer1[i]=0x00;      //将存放数据数组清零
//									}
//						}
//				} 
//	
//				else   //接收异常
//				{
//						RxState = 0;
//						RxCounter1=0;
//						for(i=0;i<10;i++)
//						{
//								RxBuffer1[i]=0x00;      //将存放数据数组清零
//						}
//				}

//		}
//	

//	}
//	
//	
////USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//	

//	OSIntExit();    	//退出中断

//}
