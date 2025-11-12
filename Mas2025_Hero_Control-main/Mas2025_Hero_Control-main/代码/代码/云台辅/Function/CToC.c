#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Remote.h"
#include "CToC.h"
#include "Parameter.h"
#include "RefereeSystem.h"

/*
 *函数简介:板间通讯主机初始化
 *参数说明:无
 *返回类型:无
 *备注:默认使用CAN2(CAN2-Tx为PB6,CAN2-Rx为PB5)
 *备注:CAN波特率1M,默认1Tq=1/14us≈0.07us
 *备注:使用CAN2需要在打开CAN2时钟之前打开CAN1时钟,且CAN2筛选器编号为15~27
 *备注:主机控制从机的标识符为149,从机回报的标识符为189
 */
void CToC_MasterInit(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//配置PB5(CAN2-Rx)和PB6(CAN2-Tx)

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
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber=15;//筛选器编号15(编号0~14属于CAN1,编号15~27属于CAN2)
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;//存入FIFO1
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;//筛选器尺度为16bits
	CAN_FilterInitStructure.CAN_FilterIdHigh=(CToC_MasterID1<<5);//标识符=STID[15:5](189/0001 1000 1001)+RTR[4:4](0)+IDE[3:3](0)+EXID[2:0](000)
	CAN_FilterInitStructure.CAN_FilterIdLow=(CToC_MasterID1<<5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFE3;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFE3;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//打通CAN2_FIFO1到NVIC的通道
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级，2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX1_IRQn;//选择CAN2_FIFO1接收中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化CAN2_FIFO1的NVIC
}

/*
 *函数简介:板间通讯主机发送遥控器摇杆数据
 *参数说明:无
 *返回类型:1-发送成功,0-发送失败
 *备注:默认标准格式数据帧,8字节数据段
 *备注:使用从机ID1
 *发送数据:
 *	Data[0~1]-遥控器底盘左右数据(右摇杆右左或按键AD)
 *	Data[2~3]-遥控器底盘前后数据(右摇杆上下或按键WS)
 *	Data[4~5]-遥控器云台左右数据(左摇杆右左或鼠标右左)
 *	Data[6~7]-遥控器云台俯仰数据(左摇杆上下或鼠标下上)
 */
uint8_t CToC_MasterSendData(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=CToC_SlaveID1;
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	if((PC_Go==0 && PC_Back==0 && PC_Left==0 && PC_Right==0 && PC_Spin==0 && PC_Pitch==0) && RefereeSystem_Status==0)//遥控器控制
	{
		TxMessage.Data[0]=(uint8_t)((uint16_t)Remote_RxData.Remote_R_RL>>8);//右摇杆右左高八位
		TxMessage.Data[1]=(uint8_t)((uint16_t)Remote_RxData.Remote_R_RL & 0x00FF);//右摇杆右左低八位
		TxMessage.Data[2]=(uint8_t)((uint16_t)Remote_RxData.Remote_R_UD>>8);//右摇杆上下高八位
		TxMessage.Data[3]=(uint8_t)((uint16_t)Remote_RxData.Remote_R_UD & 0x00FF);//右摇杆上下低八位
		TxMessage.Data[4]=(uint8_t)(Remote_RxData.Remote_L_RL>>8);//左摇杆右左高八位
		TxMessage.Data[5]=(uint8_t)(Remote_RxData.Remote_L_RL & 0x00FF);//左摇杆右左低八位
		TxMessage.Data[6]=(uint8_t)(Remote_RxData.Remote_L_UD>>8);//左摇杆上下高八位
		TxMessage.Data[7]=(uint8_t)(Remote_RxData.Remote_L_UD & 0x00FF);//左摇杆上下低八位
	}
	else//键盘控制
	{
		TxMessage.Data[0]=(uint8_t)((1024+(PC_Right-PC_Left)*660)>>8);//按键AD,相当于右摇杆右左(高八位)
		TxMessage.Data[1]=(uint8_t)((1024+(PC_Right-PC_Left)*660) & 0x00FF);//按键AD,相当于右摇杆右左(低八位)
		TxMessage.Data[2]=(uint8_t)((1024+(PC_Go-PC_Back)*660)>>8);//按键WS,相当于右摇杆上下(高八位)
		TxMessage.Data[3]=(uint8_t)((1024+(PC_Go-PC_Back)*660) & 0x00FF);//按键WS,相当于右摇杆上下(低八位)
		TxMessage.Data[4]=(uint8_t)((uint16_t)(1024+PC_Spin*PC_Mouse_RLSensitivity)>>8);//鼠标右左,相当于左摇杆右左(高八位)
		TxMessage.Data[5]=(uint8_t)((uint16_t)(1024+PC_Spin*PC_Mouse_RLSensitivity) & 0x00FF);//鼠标右左,相当于左摇杆右左(低八位)
		TxMessage.Data[6]=(uint8_t)((uint16_t)(1024+PC_Pitch*PC_Mouse_DUSensitivity)>>8);//鼠标下上,相当于左摇杆上下(高八位)
		TxMessage.Data[7]=(uint8_t)((uint16_t)(1024+PC_Pitch*PC_Mouse_DUSensitivity) & 0x00FF);//鼠标下上,相当于左摇杆上下(低八位)
	}
	
	uint8_t mbox=CAN_Transmit(CAN2,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN2,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:板间通讯主机发送遥控器控制数据
 *参数说明:无
 *返回类型:1-发送成功,0-发送失败
 *备注:默认标准格式数据帧,8字节数据段
 *备注:使用从机ID2
 *发送数据:
 *	Data[0]-遥控器连接状态
 *	Data[1]-遥控器右侧拨动开关
 *	Data[2]-键盘Ctrl状态(Remote_KeyPush_Ctrl)
 *	Data[3]-键盘Shift状态(Remote_KeyPush_Shift)
 *	Data[4]-遥控器启动标志位
 *	Data[5]-遥控器左侧拨动开关
 */
uint8_t CToC_MasterSendControl(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=CToC_SlaveID2;
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Remote_Status;//遥控器连接状态
	TxMessage.Data[1]=Remote_RxData.Remote_RS;//遥控器右侧拨动开关
	TxMessage.Data[2]=Remote_RxData.Remote_KeyPush_Ctrl;//键盘Ctrl状态
	TxMessage.Data[3]=Remote_RxData.Remote_KeyPush_Shift;//键盘Shift状态
	TxMessage.Data[4]=Remote_StartFlag;//遥控器启动标志位
	TxMessage.Data[5]=Remote_RxData.Remote_LS;//遥控器左侧拨动开关
	TxMessage.Data[6]=Remote_RxData.Remote_Mouse_KeyR;
	TxMessage.Data[7]=Remote_RxData.Remote_KeyPush_Q;
	
	uint8_t mbox=CAN_Transmit(CAN2,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN2,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:板间通讯数据处理
 *参数说明:CAN数据帧ID号,详情见CToC.h的宏定义
 *参数说明:反馈数据(8字节)
 *返回类型:无
 *备注:无
 *接收数据:
 *	Data[0]-发射机构状态
 */
void CToC_CANDataProcess(uint32_t ID,uint8_t *Data)
{
	if(ID==CToC_SlaveID2)//接收遥控器控制数据
	{
		Remote_Status=Data[0];//遥控器连接状态
		Remote_RxData.Remote_RS=Data[1];//遥控器右侧拨动开关
		Remote_RxData.Remote_KeyPush_Ctrl=Data[2];//键盘Ctrl状态
		Remote_RxData.Remote_KeyPush_Shift=Data[3];//键盘Shift状态
		Remote_StartFlag=Data[4];//遥控器启动标志位
		Remote_RxData.Remote_LS=Data[5];//遥控器左侧拨动开关
		Remote_RxData.Remote_Mouse_KeyR=Data[6];
		Remote_RxData.Remote_KeyPush_Q=Data[7];
	}
	else if(ID==CToC_SlaveID3)//接收遥控器控制数据
	{
		RefereeSystem_ShooterStatus=Data[0];//遥控器连接状态
		RefereeSystem_GameStatus=Data[1];
		Remote_RxData.Remote_ThumbWheel=(int16_t)((uint16_t)Data[2]<<8 | Data[3]);//右摇杆右左
	}
}
