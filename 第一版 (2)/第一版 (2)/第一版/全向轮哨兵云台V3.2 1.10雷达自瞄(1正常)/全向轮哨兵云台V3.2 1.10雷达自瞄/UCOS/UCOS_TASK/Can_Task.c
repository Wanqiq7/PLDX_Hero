#include "Can_Task.h"
#include "can2.h"
#include "usart1.h"
#include "RC_Protection.h"

int CAN1_Task_number;
int CAN1_Location[100];
int CAN1_DLC[100];
int CAN1_Data[100][8];
int i_task_1;

int CAN2_Task_number;
int CAN2_Location[100];
int CAN2_DLC[100];
int CAN2_Data[100][8];
int i_task_2;

uint8_t SCAP_Control(float power_limit, float buffer, uint8_t dcdc, uint8_t restart)
{
    if (power_limit < SCAP_MIN_POWER_LIMIT) {
        power_limit = SCAP_MIN_POWER_LIMIT;
    } else if (power_limit > SCAP_MAX_POWER_LIMIT) {
        power_limit = SCAP_MAX_POWER_LIMIT;
    }
    if (buffer < SCAP_MIN_ENERGY_BUFFER) {
        buffer = SCAP_MIN_ENERGY_BUFFER;
    } else if (buffer > SCAP_MAX_ENERGY_BUFFER) {
        buffer = SCAP_MAX_ENERGY_BUFFER;
    }
    // 确保dcdc和restart是0或1
    dcdc = (dcdc != 0) ? 1 : 0;
    restart = (restart != 0) ? 1 : 0;
    // 填充发送数据结构
    SCAP_Tx txData;
    txData.enableDCDC = dcdc;
    txData.systemRestart = restart;
    txData.resv0 = 0; // 保留位设为0
    txData.feedbackRefereePowerLimit = (uint16_t)roundf(power_limit);
    txData.feedbackRefereeEnergyBuffer = (uint16_t)roundf(buffer);
    txData.resv1[0] = 0;
    txData.resv1[1] = 0;
    txData.resv1[2] = 0;
    // 准备CAN消息
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x061;          // 目标CAN ID
    TxMessage.ExtId = 0x00;
    TxMessage.IDE = CAN_ID_STD;        // 标准ID
    TxMessage.RTR = CAN_RTR_DATA;      // 数据帧
    TxMessage.DLC = sizeof(SCAP_Tx);   // 数据长度(应为8)
    // 复制数据到发送缓冲区
    memcpy(TxMessage.Data,(uint8_t*)&txData,sizeof(SCAP_Tx));
    // 发送CAN消息到CAN1
    uint8_t transmitStatus = CAN_Transmit(CAN1, &TxMessage);
    // 检查发送状态
    if (transmitStatus == CAN_TxStatus_NoMailBox) {
        return 1; // 无可用邮箱
    }
    return 0; // 成功
}

/**********************************************************************************、
函数名：CAN1_sand(int CAN2_Location,int CAN2_DLC,int data_1,int data_2,int data_3,int data_4)
形式参数：
1.发送的地址
2.发送字长
3.内容数据位1
4.内容数据位2
5.内容数据位3
6.内容数据位4
作者：
时间：2021年8月13日
功能：将要发送的数据加入CAN1发送邮箱
**********************************************************************************/
void CAN1_sand(int CAN1_Location,int CAN1_DLC,int data_1,int data_2,int data_3,int data_4)
{
		CanTxMsg TxMessage;
    
		TxMessage.StdId = CAN1_Location;  //底盘所有ID
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = CAN1_DLC;
   
		TxMessage.Data[0] = data_1 >> 8;
    TxMessage.Data[1] = data_1;
    TxMessage.Data[2] = data_2 >> 8;
    TxMessage.Data[3] = data_2;
    TxMessage.Data[4] = data_3 >> 8;
    TxMessage.Data[5] = data_3;
    TxMessage.Data[6] = data_4 >> 8;
    TxMessage.Data[7] = data_4;

    CAN_Transmit(CAN1, &TxMessage);
		
		delay_us(45);	
}


/**********************************************************************************、
函数名：CAN2_sand(int CAN2_Location,int CAN2_DLC,int data_1,int data_2,int data_3,int data_4)
形式参数：
1.发送的地址
2.发送字长
3.内容数据位1
4.内容数据位2
5.内容数据位3
6.内容数据位4
作者：
时间：2021年8月13日
功能：将要发送的数据加入CAN2发送邮箱
**********************************************************************************/
void CAN2_sand(int CAN2_Location,int CAN2_DLC,int data_1,int data_2,int data_3,int data_4)
{
		CanTxMsg TxMessage;
    TxMessage.StdId = CAN2_Location;  //底盘所有ID
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = CAN2_DLC;
    TxMessage.Data[0] = data_1 >> 8;
    TxMessage.Data[1] = data_1;
    TxMessage.Data[2] = data_2 >> 8;
    TxMessage.Data[3] = data_2;
    TxMessage.Data[4] = data_3 >> 8;
    TxMessage.Data[5] = data_3;
    TxMessage.Data[6] = data_4 >> 8;
    TxMessage.Data[7] = data_4;
		
    CAN_Transmit(CAN2, &TxMessage);
		delay_us(45);	
}


/**********************************************************************************、
函数名：CAN1_Send_queue_add(int CAN2_Location_add,int CAN2_DLC_add,int CAN2_Data_1,int CAN2_Data_2,int CAN2_Data_3,int CAN2_Data_4)
形式参数：
1.发送的地址
2.发送字长
3.内容数据位1
4.内容数据位2
5.内容数据位3
6.内容数据位4
作者：
时间：2021年8月13日
功能：将CAN1要发送的信息进行排队
**********************************************************************************/

void CAN1_Send_queue_add(int CAN1_Location_add,int CAN1_DLC_add,int CAN1_Data_1,int CAN1_Data_2,int CAN1_Data_3,int CAN1_Data_4)
{
	int add;
	
	add = i_task_1 + 1;
	
	while(CAN1_Location[add] > 0)
	{	
		add++;
				if(add > 30)
		{
		add = 0;
		}
	}
	

	
	CAN1_Location[add] = CAN1_Location_add;
	CAN1_DLC[add] = CAN1_DLC_add;
	CAN1_Data[add][0] = CAN1_Data_1;
	CAN1_Data[add][1] = CAN1_Data_2;
	CAN1_Data[add][2] = CAN1_Data_3;
	CAN1_Data[add][3] = CAN1_Data_4;
	CAN1_Task_number ++ ;
}


/**********************************************************************************、
函数名：CAN2_Send_queue_add(int CAN2_Location_add,int CAN2_DLC_add,int CAN2_Data_1,int CAN2_Data_2,int CAN2_Data_3,int CAN2_Data_4)
形式参数：
1.发送的地址
2.发送字长
3.内容数据位1
4.内容数据位2
5.内容数据位3
6.内容数据位4
作者：
时间：2021年8月13日
功能：将CAN2要发送的信息进行排队
**********************************************************************************/
void CAN2_Send_queue_add(int CAN2_Location_add,int CAN2_DLC_add,int CAN2_Data_1,int CAN2_Data_2,int CAN2_Data_3,int CAN2_Data_4)
{
	int add2;
	
	add2 = i_task_2 + 1;
	
	while(CAN2_Location[add2] > 0)
	{	
	add2++;
	}
	
	if(add2 > 30)
	{
		add2 = 0;
	}
	
	CAN2_Location[add2] = CAN2_Location_add;
	CAN2_DLC[add2] = CAN2_DLC_add;
	CAN2_Data[add2][0] = CAN2_Data_1;
	CAN2_Data[add2][1] = CAN2_Data_2;
	CAN2_Data[add2][2] = CAN2_Data_3;
	CAN2_Data[add2][3] = CAN2_Data_4;
	CAN2_Task_number ++ ;
}

/**********************************************************************************、
函数名：CAN1_Send_queue(void)
形式参数：无
作者：
时间：2021年8月13日
功能：排队控制CAN1数据发送
**********************************************************************************/
void CAN1_Send_queue(void)
{
	int kk;
	kk = 3;
	
		while(kk)
		{
			//GPIO_SetBits(GPIOI,GPIO_Pin_2);
			while(CAN1_Location[i_task_1] == 0)
			{
			
				i_task_1++;
				if(i_task_1 > 30)
				{
					i_task_1 = 0;
					break;
				}
			}
			
			if(CAN1_Location[i_task_1] > 0)
			{
				//printf("%d\r\n",CAN1_DLC[i_task_1]);
				CAN1_sand(CAN1_Location[i_task_1],CAN1_DLC[i_task_1],CAN1_Data[i_task_1][0],CAN1_Data[i_task_1][1],CAN1_Data[i_task_1][2],CAN1_Data[i_task_1][3]);
				CAN1_Location[i_task_1] = 0;
				CAN1_DLC[i_task_1] = 0;
				CAN1_Data[i_task_1][0] = 0;
				CAN1_Data[i_task_1][1] = 0;
				CAN1_Data[i_task_1][2] = 0;
				CAN1_Data[i_task_1][3] = 0;
			if(CAN1_Task_number > 0){	CAN1_Task_number--;}
			}
		kk--;
			
	
		}
		
				//GPIO_ResetBits(GPIOI,GPIO_Pin_2);

}

/**********************************************************************************、
函数名：CAN2_Send_queue(void)
形式参数：无
作者：
时间：2021年8月13日
功能：排队控制CAN2数据发送
**********************************************************************************/
void CAN2_Send_queue(void)
{
	int kk2;
	kk2 = 3;
	
			while(kk2)
		{

		while(CAN2_Location[i_task_2] == 0)
			{
			
				i_task_2++;
				if(i_task_2 > 30)
				{
					i_task_2 = 0;
					break;
				}
			}
		if(CAN2_Location[i_task_2] > 0)
			{
				//printf("%d\r\n",CAN1_DLC[i_task_1]);
				CAN2_sand(CAN2_Location[i_task_2],CAN2_DLC[i_task_2],CAN2_Data[i_task_2][0],CAN2_Data[i_task_2][1],CAN2_Data[i_task_2][2],CAN2_Data[i_task_2][3]);
				CAN2_Location[i_task_2] = 0;
				CAN2_DLC[i_task_2] = 0;
				CAN2_Data[i_task_2][0] = 0;
				CAN2_Data[i_task_2][1] = 0;
				CAN2_Data[i_task_2][2] = 0;
				CAN2_Data[i_task_2][3] = 0;
			}
		kk2--;
		if(CAN2_Task_number > 0){	CAN2_Task_number--;}
		}
	
}

//void send_accel(void)		//将陀螺仪数据发给云台，ID：220
//{
//	CanTxMsg TxMessage;
//    TxMessage.StdId = 0x221;  
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 0x08;  
//     TxMessage.Data[0] = Attack_plan >> 8;
//    TxMessage.Data[1] = Attack_plan;
//	  TxMessage.Data[2] = Attack_plan >> 8;
//    TxMessage.Data[3] = Attack_plan;
//	  TxMessage.Data[4] = Attack_plan >> 8;
//    TxMessage.Data[5] = Attack_plan;
//	
//    CAN_Transmit(CAN1, &TxMessage);
//	delay_us(45);	
//}

/**********************************************************************************、
任务名：CAN_Send_Task(void)
形式参数：无
作者：
时间：2021年8月13日
功能：使CAN的发送频率接近到8100HZ的理论值，实测能达到8050HZ
技术实现：
1.排队控制CAN1.CAN2发送邮箱的数据导入
2.控制数据能塞满3个邮箱深度
任务频率：4132FPS
**********************************************************************************/
void CAN_Send_Task(void)
{
//	OS_ERR err;
	
	//printf("创建CAN发送消息队列任务\r\n");
	while(1)
	{
		//GPIO_SetBits(GPIOI,GPIO_Pin_2);
		//GPIO_SetBits(GPIOI,GPIO_Pin_2);
	CAN1_Send_queue();//发送CAN1数据		
	CAN2_Send_queue();//发送CAN2数据
		
	//send_accel();
	delay_us(242);	
  //OSTimeDlyHMSM(0,0,0,1,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms

		
	}


}

