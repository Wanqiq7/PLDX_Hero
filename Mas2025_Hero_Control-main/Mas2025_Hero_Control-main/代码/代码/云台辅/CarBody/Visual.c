#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "UART.h"
#include "AttitudeAlgorithms.h"
#include "DM_J4310.h"
#include "arm_math.h"
#include "USB.h"
#include "RefereeSystem.h"

//	0xFF 0x00 roll pitch yaw 0x00 0x0D
//	0xFF 0/1 pitch yaw dis 0x00 0x0D

float Visual_Yaw,Visual_Pitch,Visual_Fire;//视觉数据偏航角,视觉数据俯仰角
uint8_t Visual_RxHEXPacket[4],Visual_ReceiveFlag;//视觉数据接收缓冲区,视觉数据接收完成标志位
uint8_t RXData[16],RXSelect=0;

uint8_t Visual_TxDataRed[16]={0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD0};
uint8_t Visual_TxDataBlue[16]={0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD0};
uint8_t Visual_RxData0[16];//遥控器DMA数据存储器0
uint8_t Visual_RxData1[16];//遥控器DMA数据存储器1

uint8_t VisualSendFirstFlag=1;
uint8_t VisualErrorFlag=0;

/*
 *函数简介:视觉初始化
 *参数说明:无
 *返回类型:无
 *备注:初始化UART2(USART1)
 */
/*
 *函数简介:遥控器初始化
 *参数说明:无
 *返回类型:无
 *备注:默认接收引脚为PC11(USART3-Rx)
 *备注:采用串口DMA双缓冲接收
 *备注:配置定时中断为TIM7 25ms,用来检测遥控器连接情况
 *备注:加入独立看门狗,用来在数据接收错误时复位(只在上电的一刻可能数据错误)
 *备注:独立看门狗时钟为LSI(32kHz),预分频数默认为8,故IWDG时钟4kHz,重装载值为0x0FFF(4095)
 *备注:喂狗时间(溢出时间)T_OUT=Reload/(LSI/Prescaler)=4095/(32k/8)=1.02375s
 */
void Visual_Init(void)
{
	/*===============配置时钟===============*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);//开启时钟

	/*===============配置定时器===============*/
	TIM_InternalClockConfig(TIM12);//选择时基单元的时钟(此处为内部时钟),TIM_ITRxExternalClockConfig()接ITRx时钟,TIM_TIxExternalClockConfig()接TIx捕获通道时钟,TIM_ETRClockMode1Config()接ETR时钟(选择外部时钟模式1),TIM_ETRClockMode2Config()接ETR时钟(选择外部时钟模式2)
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元（配置参数）
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=420-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=1000-1;//配置分频值PSC,默认定时SPEED/4 ms
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseInitStructure);//初始化TIM12

	TIM_ClearFlag(TIM12,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位

	/*===============配置接收中断和定时器中断===============*/
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn;//选择中断通道为TIM2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//TIM12的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//TIM12的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	/*===============使能===============*/
	TIM_Cmd(TIM12,ENABLE);//启动定时器
	
	
	USB_Init();
}

/*
 *函数简介:遥控器数据处理
 *参数说明:无
 *返回类型:无
 *备注:遥控器接收数据共18Bytes(144bits)
 */
void Visual_DataProcess(void)
{
	static union
	{
		uint8_t Data[8];
		float INS[2];
	}Visual_Receive;


	uint8_t *Data=USB_ReceiveBuff;//选择存储器
	
	if(Data[15]==0x0D && (Data[1]==0x01 || Data[1]==0x00) && Data[14]==0x00)
	{
		Visual_Receive.Data[0]=Data[2];
		Visual_Receive.Data[1]=Data[3];
		Visual_Receive.Data[2]=Data[4];
		Visual_Receive.Data[3]=Data[5];

		Visual_Receive.Data[4]=Data[6];
		Visual_Receive.Data[5]=Data[7];
		Visual_Receive.Data[6]=Data[8];
		Visual_Receive.Data[7]=Data[9];
		
		Visual_Fire=Data[1];
		Visual_Yaw=Visual_Receive.INS[1]+360.0f*AttitudeAlgorithms_YawR;
		Visual_Pitch=-Visual_Receive.INS[0]/180.0f*3.1415926f;
		if(Visual_Receive.INS[1]!=0 || Visual_Receive.INS[0]!=0)Visual_ReceiveFlag=1;
	}
	else
		VisualErrorFlag=1;
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	static union
	{
		uint8_t Data[12];
		float INS[3];
	}Visual_Send;

	if(TIM_GetITStatus(TIM12,TIM_IT_Update)==SET)//检测TIM2更新
	{
		TIM_ClearITPendingBit(TIM12,TIM_IT_Update);//清除标志位
	
		Visual_Send.INS[0]=AttitudeAlgorithms_DegYaw;
		Visual_Send.INS[1]=-DM_J4310_MotorStatus[DM_J4310_7-0x01].Position/PI*180.0f;
		//Visual_Send.INS[1]=-AttitudeAlgorithms_DegPitch;
		Visual_Send.INS[2]=AttitudeAlgorithms_DegRoll;

		uint8_t *Visual_TxData;
		if(RefereeSystem_RobotID<100)Visual_TxData=Visual_TxDataRed;
		else Visual_TxData=Visual_TxDataBlue;
		
		Visual_TxData[2]=Visual_Send.Data[8];
		Visual_TxData[3]=Visual_Send.Data[9];
		Visual_TxData[4]=Visual_Send.Data[10];
		Visual_TxData[5]=Visual_Send.Data[11];
		Visual_TxData[6]=Visual_Send.Data[4];
		Visual_TxData[7]=Visual_Send.Data[5];
		Visual_TxData[8]=Visual_Send.Data[6];
		Visual_TxData[9]=Visual_Send.Data[7];
		Visual_TxData[10]=Visual_Send.Data[0];
		Visual_TxData[11]=Visual_Send.Data[1];
		Visual_TxData[12]=Visual_Send.Data[2];
		Visual_TxData[13]=Visual_Send.Data[3];
		              
		USB_Send(Visual_TxData,16);
		
		Visual_DataProcess();
	}
}
