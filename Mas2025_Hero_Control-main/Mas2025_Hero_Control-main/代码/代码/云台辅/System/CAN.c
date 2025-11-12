#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "CAN.h"
#include "M3508.h"
#include "GM6020.h"
#include "LinkCheck.h"
#include "Warming.h"
#include "CToC.h"
#include "DM_J4310.h"
#include "RefereeSystem.h"
#include "M2006.h"

uint8_t CAN_CAN1DeviceNumber=2;//CAN1总线上设备数量
uint8_t CAN_CAN2DeviceNumber=6;//CAN2总线上设备数量
uint8_t CAN_DeviceNumber=8;//CAN总线上设备数量
uint32_t CAN_CAN1IDList[10][2]={{CAN_RoboMasterC,CToC_SlaveID2},{CAN_RoboMasterC,CToC_SlaveID3},0};//CAN1总线上设备ID列表
uint32_t CAN_CAN2IDList[10][2]={{CAN_M3508,M3508_1},{CAN_M3508,M3508_2},{CAN_M3508,M3508_3},{CAN_M3508,M3508_4},{CAN_DM_J4310,DM_J4310_7+DM_J4310_IDReceiveOffset},{CAN_M2006,M2006_8},0};//CAN2总线上设备ID列表
int8_t CAN_IDSelect=0;//CAN总线上ID列表选择位

/*
 *函数简介:CAN总线初始化
 *参数说明:无
 *返回类型:无
 *备注:默认使用CAN1(CAN1-Tx为PD1,CAN1-Rx为PD0),CAN2(CAN2-Tx为PB6,CAN2-Rx为PB5)
 *备注:CAN波特率1M,默认1Tq=1/14us≈0.07us
 *备注:使用CAN2需要在打开CAN2时钟之前打开CAN1时钟,且CAN2筛选器编号为15~27
 */
void CAN_CANInit(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);//配置PD0(CAN1-Rx)和PD1(CAN1-Tx)
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//配置PB5(CAN2-Rx)和PB6(CAN2-Tx)

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1);//开启PD0的CAN1复用模式
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1);//开启PD1的CAN1复用模式
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2);//开启PB5的CAN2复用模式
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2);//开启PB6的CAN2复用模式
	
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;//正常模式
	CAN_InitStructure.CAN_Prescaler=3;//时钟分频,1Tq=Prescaler/T_PCLK=Prescaler/42M
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;//SJW极限值
	CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;//PBS1段长度
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;//PBS2段长度,1/Baudrate=T_1bit=(1+BS1+BS2)Tq
	CAN_InitStructure.CAN_TTCM=DISABLE;//关闭时间触发功能
	CAN_InitStructure.CAN_ABOM=ENABLE;//开启自动离线管理功能
	CAN_InitStructure.CAN_AWUM=ENABLE;//开启自动唤醒功能
	CAN_InitStructure.CAN_NART=ENABLE;//禁止自动重传功能
	CAN_InitStructure.CAN_RFLM=DISABLE;//关闭锁定FIFO功能
	CAN_InitStructure.CAN_TXFP=DISABLE;//配置报文优先级判定为标识符决定
	CAN_Init(CAN1,&CAN_InitStructure);
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber=0;//筛选器编号0(编号0~14属于CAN1,编号15~27属于CAN2)
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//存入FIFO0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;//筛选器尺度为16bits
	CAN_FilterInitStructure.CAN_FilterIdHigh=(CAN_CAN1IDList[0][1]<<5);//标识符=STID[15:5]+RTR[4:4](0)+IDE[3:3](0)+EXID[2:0](000)
	CAN_FilterInitStructure.CAN_FilterIdLow=(CAN_CAN1IDList[0][1]<<5);//筛选器尺度为16bits,故没有高低位之分
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFE3;//掩码(1111 1111 0000 0011),1对应位必须匹配,0对应位无需匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFE3;//筛选器尺度为16bits,故没有高低位之分
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	CAN_FilterInitStructure.CAN_FilterNumber=15;//筛选器编号15(编号0~14属于CAN1,编号15~27属于CAN2)
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;//存入FIFO1
	CAN_FilterInitStructure.CAN_FilterIdHigh=(0x000<<5);//标识符=STID[15:5](189/0001 1000 1001)+RTR[4:4](0)+IDE[3:3](0)+EXID[2:0](000)
	CAN_FilterInitStructure.CAN_FilterIdLow=(0x000<<5);//CAN2先给0,默认从CAN1开始串行接收
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//打通CAN1_FIFO0到NVIC的通道
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//打通CAN2_FIFO1到NVIC的通道
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级，2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;//选择CAN1_FIFO0接收中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化CAN1_FIFO0的NVIC
	NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX1_IRQn;//选择CAN2_FIFO1接收中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化CAN2_FIFO1的NVIC
}

/*
 *函数简介:CAN1总线接收报文
 *参数说明:报文存储数组
 *返回类型:报文ID
 *备注:默认8字节标准数据帧
 *备注:没有接收到数据,直接退出,返回0
 */
uint32_t CAN_CAN1Receive(uint8_t *Data)
{
	CanRxMsg RxMessage;
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;//没有接收到数据,直接退出
	CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);//读取数据
	for(uint32_t i=0;i<8;i++)
		Data[i]=RxMessage.Data[i];//储存数据
	return RxMessage.StdId;//返回ID
}

/*
 *函数简介:CAN2总线接收报文
 *参数说明:报文存储数组
 *返回类型:报文ID
 *备注:默认8字节数据
 *备注:没有接收到数据,直接退出,返回0
 */
uint32_t CAN_CAN2Receive(uint8_t *Data)
{
	CanRxMsg RxMessage;
	if(CAN_MessagePending(CAN2,CAN_FIFO1)==0)return 0;//没有接收到数据,直接退出
	CAN_Receive(CAN2,CAN_FIFO1,&RxMessage);//读取数据
	for(uint32_t i=0;i<8;i++)
		Data[i]=RxMessage.Data[i];
	return RxMessage.StdId;
}

/*
 *函数简介:CAN1总线更改接收ID
 *参数说明:接收ID
 *返回类型:无
 *备注:无
 */
void CAN_CAN1ChangeID(uint32_t ID)
{
	CAN1->FMR |= 0x00000001;//配置CAN_FMR寄存器FINIT位进入初始化模式
	CAN1->sFilterRegister[0].FR1 = ((uint32_t)0xFFE3<<16) | (ID<<5);//配置CAN_F0R1寄存器更改ID低位
	CAN1->sFilterRegister[0].FR2 = ((uint32_t)0xFFE3<<16) | (ID<<5);//配置CAN_F0R2寄存器更改ID高位(16bits尺度下无高低位区别)
	CAN1->FMR &= ~(0x00000001);//配置CAN_FMR寄存器FINIT位回到工作模式
}

/*
 *函数简介:CAN2总线更改接收ID
 *参数说明:接收ID
 *返回类型:无
 *备注:无
 */
void CAN_CAN2ChangeID(uint32_t ID)
{
	CAN1->FMR |= 0x00000001;
	CAN1->sFilterRegister[15].FR1 = ((uint32_t)0xFFE3<<16) | (ID<<5);
	CAN1->sFilterRegister[15].FR2 = ((uint32_t)0xFFE3<<16) | (ID<<5);
	CAN1->FMR &= ~(0x00000001);
}

/*
 *函数简介:CAN接收ID列表复位
 *参数说明:无
 *返回类型:无
 *备注:复位CAN_IDSelect,重新从CAN1的1号设备开始接收
 */
void CAN_CANIDReset(void)
{
	CAN_IDSelect=0;
	CAN_CAN1ChangeID(CAN_CAN1IDList[0][1]);
	CAN_CAN2ChangeID(0x000);
}

/*
 *函数简介:CAN接收获取裁判系统状态
 *参数说明:无
 *返回类型:无
 *备注:跳转到接收底盘C板的回传数据,主要用于发射机构掉电时的CAN设备隔离
 */
void CAN_CAN_GetRefereeSystemData(void)
{
	CAN_IDSelect=0;
	CAN_CAN1ChangeID(CAN_CAN1IDList[0][1]);	
	CAN_CAN2ChangeID(0x000);
}

/*
 *函数简介:CAN1_FIFO0接收中断函数
 *参数说明:无
 *返回类型:无
 *备注:进入中断时关闭连接检测计时,离开中断时重新打开连接检测计时
 *备注:某一设备掉线时,CAN_IDSelect会停留在当前设备在ID列表的索引
 *备注:从掉线到重新连接时会重启遥控器
 */
void CAN1_RX0_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)==SET)//查询接收中断标志位
	{
		uint8_t Data[8];//接收数据
		uint32_t ID=CAN_CAN1Receive(Data);//接收数据帧的ID
		
		LinkCheck_OFF();//关闭连接检测定时
		if(LinkCheck_ErrorID>=0 && LinkCheck_ErrorID<CAN_CAN1DeviceNumber)//接收的是掉线设备的数据
			if(ID==CAN_CAN1IDList[LinkCheck_ErrorID][1])
			{
				LinkCheck_ErrorID=-1;//清除掉线设备
				LinkCheck_Error=0;//回到正常连接状态
				LinkError_Count=0;
				Warming_BuzzerClean();//清除蜂鸣器报警
			}
		
		if(CAN_CAN1IDList[CAN_IDSelect][0]==CAN_M3508)M3508_CANDataProcess(ID,Data);//处理M3508的数据
		else if(CAN_CAN1IDList[CAN_IDSelect][0]==CAN_GM6020)GM6020_CANDataProcess(ID,Data);//处理GM6020的数据
		else if(CAN_CAN1IDList[CAN_IDSelect][0]==CAN_DM_J4310)DM_J4310_DataProcess(ID,Data);
		else if(CAN_CAN1IDList[CAN_IDSelect][0]==CAN_M2006)M2006_CANDataProcess(ID,Data);
		else if(CAN_CAN1IDList[CAN_IDSelect][0]==CAN_RoboMasterC)CToC_CANDataProcess(ID,Data);//处理板间通讯
		
		CAN_IDSelect=(CAN_IDSelect+1)%CAN_DeviceNumber;//更换CAN总线ID切换选择位
		if(CAN_IDSelect>=0 && CAN_IDSelect<CAN_CAN1DeviceNumber){CAN_CAN1ChangeID(CAN_CAN1IDList[CAN_IDSelect][1]);CAN_CAN2ChangeID(0x000);}//CAN1、CAN2更换接收ID
		else{CAN_CAN1ChangeID(0x000);CAN_CAN2ChangeID(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][1]);}//设置接收ID为0规定为关闭接收
		
//		if(RefereeSystem_ShooterOpenFlag==1)//检测到发射机构上电添加CAN设备
//		{
//			RefereeSystem_ShooterOpenCounter++;
//			if(RefereeSystem_ShooterOpenCounter>=7000)//计次5000下等待发射机构设备启动
//			{
//				RefereeSystem_ShooterOpenCounter=0;
//				RefereeSystem_ShooterOpenFlag=0;//发射机构离开上电期间
//				CAN_CAN1DeviceNumber=2;//添加CAN设备
//				CAN_CAN2DeviceNumber=6;//添加CAN设备
//				CAN_DeviceNumber=8;
//			}
//		}
		
		LinkCheck_ON();//连接检测重新计时
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);//清除CAN1的FIFO0接收中断标志位
	}
}

/*
 *函数简介:CAN2_FIFO1接收中断函数
 *参数说明:无
 *返回类型:无
 *备注:进入中断时关闭连接检测计时,离开中断时重新打开连接检测计时
 *备注:某一设备掉线时,CAN_IDSelect会停留在当前设备在ID列表的索引
 */
void CAN2_RX1_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP1)==SET)//查询接收中断标志位
	{
		uint8_t Data[8];//接收数据
		uint32_t ID=CAN_CAN2Receive(Data);//接收数据帧的ID
		
		LinkCheck_OFF();//关闭连接检测定时
		if(LinkCheck_ErrorID>=CAN_CAN1DeviceNumber && LinkCheck_ErrorID<CAN_DeviceNumber)//接收的是掉线设备的数据
			if(ID==CAN_CAN2IDList[LinkCheck_ErrorID-CAN_CAN1DeviceNumber][1])
			{
				LinkCheck_ErrorID=-1;//清除掉线设备
				LinkCheck_Error=0;//回到正常连接状态
				LinkError_Count=0;
				Warming_BuzzerClean();//清除蜂鸣器报警
			}

		if(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][0]==CAN_GM6020)GM6020_CANDataProcess(ID,Data);//处理6020的数据
		else if(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][0]==CAN_M3508)M3508_CANDataProcess(ID,Data);//处理M3508的数据
		else if(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][0]==CAN_M2006)M2006_CANDataProcess(ID,Data);//处理M3508的数据
		else if(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][0]==CAN_RoboMasterC)CToC_CANDataProcess(ID,Data);//处理板间通讯
		else if(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][0]==CAN_DM_J4310)DM_J4310_DataProcess(ID,Data);
		
		CAN_IDSelect=(CAN_IDSelect+1)%CAN_DeviceNumber;//更换CAN总线ID切换选择位
		if(CAN_IDSelect>=0 && CAN_IDSelect<CAN_CAN1DeviceNumber){CAN_CAN1ChangeID(CAN_CAN1IDList[CAN_IDSelect][1]);CAN_CAN2ChangeID(0x000);}//CAN1、CAN2更换接收ID
		else{CAN_CAN1ChangeID(0x000);CAN_CAN2ChangeID(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][1]);}//设置接收ID为0规定为关闭接收

//		if(RefereeSystem_ShooterOpenFlag==1)//检测到发射机构上电添加CAN设备
//		{
//			RefereeSystem_ShooterOpenCounter++;
//			if(RefereeSystem_ShooterOpenCounter>=7000)//计次5000下等待发射机构设备启动
//			{
//				RefereeSystem_ShooterOpenCounter=0;
//				RefereeSystem_ShooterOpenFlag=0;//发射机构离开上电期间
//				CAN_CAN1DeviceNumber=2;//添加CAN设备
//				CAN_CAN2DeviceNumber=6;//添加CAN设备
//				CAN_DeviceNumber=8;
//			}
//		}
				
		LinkCheck_ON();//连接检测重新计时
		CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);//清除CAN2的FIFO1接收中断标志位
	}
}
