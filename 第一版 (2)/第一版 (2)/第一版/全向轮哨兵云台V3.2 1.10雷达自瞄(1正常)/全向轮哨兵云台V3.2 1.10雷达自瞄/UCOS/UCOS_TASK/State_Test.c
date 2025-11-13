
#include "State_Test.h"
#include "can1.h"
#include "can2.h"
#include "usart1.h"
#include "timer.h"
#include "spi.h"

extern OS_FLAG_GRP	KeyEventFlags;		//定义一个事件标志组

#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "usart.h"
//fp32 gyro[3], accel[3];

double word_angle[3];

double temp2,temp4,temp5,temp6;
int data;
double temp3 = 0;
int accel_souce[3],gyro_souce[3],temp,accel[3];
unsigned int buf[16];

int temp_Erro,temp_I,temp_out;



/*
输入捕获函数封装
*/

void cap_test(void)
{

		//double temp3 = 0;
		long long temp2=0; 
	  
		if(TIM8CH2_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp2=TIM8CH2_CAPTURE_STA&0X3F; 
			temp2*=0XFFFFFFFF;		 		         //溢出时间总和
			temp2+=TIM8CH2_CAPTURE_VAL;		   //得到总的高电平时间
			temp3 = temp2*0.93679834;
			//printf("CH2_HIGH:%lf us\r\n",temp3); //打印总的高点平时间
			TIM8CH2_CAPTURE_STA=0;			     //开启下一次捕获
		}
		
}

/**********************************************************************************、
函数名：KalmanFilter_1(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
形式参数：
1.被滤波数据;
2.卡尔曼滤波Q值;
3.卡尔曼滤波R值;
4.无效形参
作者：
时间：2022年2月3日
功能：
对陀螺仪X轴角速度数据进行卡尔曼滤波
**********************************************************************************/
double KalmanFilter_1(const double ResrcData,
                                        double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
{
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;

        static        double x_last;

        double x_mid = x_last;
        double x_now;

        static        double p_last;

        double p_mid ;
        double p_now;
        double kg;        

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
        kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
        x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
        p_now=(1-kg)*p_mid;//最优值对应的covariance        

        p_last = p_now; //更新covariance值
        x_last = x_now; //更新系统状态值

        return x_now;                
}

/**********************************************************************************、
函数名：KalmanFilter_2(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
形式参数：
1.被滤波数据;
2.卡尔曼滤波Q值;
3.卡尔曼滤波R值;
4.无效形参
作者：
时间：2022年2月3日
功能：
对陀螺仪Y轴角速度数据进行卡尔曼滤波
**********************************************************************************/
double KalmanFilter_2(const double ResrcData,
                                        double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
{
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;

        static        double x_last;

        double x_mid = x_last;
        double x_now;

        static        double p_last;

        double p_mid ;
        double p_now;
        double kg;        

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
        kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
        x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
        p_now=(1-kg)*p_mid;//最优值对应的covariance        

        p_last = p_now; //更新covariance值
        x_last = x_now; //更新系统状态值

        return x_now;                
}


/**********************************************************************************、
函数名：KalmanFilter_3(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
形式参数：
1.被滤波数据;
2.卡尔曼滤波Q值;
3.卡尔曼滤波R值;
4.无效形参
作者：
时间：2022年2月3日
功能：
对陀螺仪Z轴角速度数据进行卡尔曼滤波
**********************************************************************************/
double KalmanFilter_3(const double ResrcData,
                                        double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
{
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;

        static        double x_last;

        double x_mid = x_last;
        double x_now;

        static        double p_last;

        double p_mid ;
        double p_now;
        double kg;        

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
        kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
        x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
        p_now=(1-kg)*p_mid;//最优值对应的covariance        

        p_last = p_now; //更新covariance值
        x_last = x_now; //更新系统状态值

        return x_now;                
}


/**********************************************************************************、
函数名：Get_IMU_Value()
形式参数：无
作者：
时间：2022年2月3日
功能：
1.陀螺仪6轴数据刷新，3轴角速度，3轴加速度

**********************************************************************************/



void Get_IMU_Value()
{
	
	//////////////////////////////////////////////////////
		GPIO_ResetBits(GPIOB,GPIO_Pin_0); //GPIOF 高电平
		SPI_I2S_SendData(SPI1, 0x02 | 0x80); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		data = SPI_I2S_ReceiveData(SPI1);
		data = 6;			
		while(data)
		{
			SPI_I2S_SendData(SPI1, 0x55); //通过外设SPIx发送一个byte  数据
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
			buf[6-data] = SPI_I2S_ReceiveData(SPI1);	
			data--;
		}
		GPIO_SetBits(GPIOB,GPIO_Pin_0); //GPIOF 高电平
	
	gyro_souce[0] = buf[1] * 256 + buf[0];		 
	gyro_souce[1] = buf[3] * 256 + buf[2];	
	gyro_souce[2] = buf[5] * 256 + buf[4];	
		
	if(gyro_souce[0] > 32767)
	{
		gyro_souce[0] = gyro_souce[0] - 65535;
	}	
		
		if(gyro_souce[1] > 32767)
	{
		gyro_souce[1] = gyro_souce[1] - 65535;
	}
		
	if(gyro_souce[2] > 32767)
	{
		gyro_souce[2] = gyro_souce[2] - 65535;
	}
	delay_ms(1);
	//////////////////////////////////////////////////////
	
	
			GPIO_ResetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平
		SPI_I2S_SendData(SPI1, 0x11 | 0x80); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		data = SPI_I2S_ReceiveData(SPI1);
		data = 6;			
		while(data)
		{
			SPI_I2S_SendData(SPI1, 0x55); //通过外设SPIx发送一个byte  数据
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
			buf[12-data] = SPI_I2S_ReceiveData(SPI1);	
			data--;
		}
		GPIO_SetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平
	
	
	accel_souce[0] = buf[7] * 256 + buf[6];		 
	accel_souce[1] = buf[9] * 256 + buf[8];	
	accel_souce[2] = buf[11] * 256 + buf[10];	
	
		if(accel_souce[0] > 32767)
	{
		accel_souce[0] = accel_souce[0] - 65535;
	}	
		
		if(accel_souce[1] > 32767)
	{
		accel_souce[1] = accel_souce[1] - 65535;
	}
		
	if(accel_souce[2] > 32767)
	{
		accel_souce[2] = accel_souce[2] - 65535;
	}
	

	GPIO_ResetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平
		SPI_I2S_SendData(SPI1, 0x11 | 0x80); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		data = SPI_I2S_ReceiveData(SPI1);
		data = 8;			
		while(data)
		{
			SPI_I2S_SendData(SPI1, 0x55); //通过外设SPIx发送一个byte  数据
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
			buf[14-data] = SPI_I2S_ReceiveData(SPI1);	
			data--;
		}
		GPIO_SetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平
	
	
	accel[0] = buf[9] * 256 + buf[8];		 
	accel[1] = buf[11] * 256 + buf[10];	
	accel[2] = buf[13] * 256 + buf[12];	
	
		if(accel[0] > 32767)
	{
		accel[0] = accel[0] - 65535;
	}	
		
		if(accel[1] > 32767)
	{
		accel[1] = accel[1] - 65535;
	}
		
	if(accel[2] > 32767)
	{
		accel[2] = accel[2] - 65535;
	}

//	//////////////////////////////////////////////////////
	
			
			
		GPIO_ResetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平
		
		SPI_I2S_SendData(SPI1, 0x22 | 0x80); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		data = SPI_I2S_ReceiveData(SPI1);

		SPI_I2S_SendData(SPI1, 0x55); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		buf[14] = SPI_I2S_ReceiveData(SPI1);
		
		SPI_I2S_SendData(SPI1, 0x55); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		buf[14] = SPI_I2S_ReceiveData(SPI1);	

		SPI_I2S_SendData(SPI1, 0x55); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		buf[13] = SPI_I2S_ReceiveData(SPI1);	
			
		GPIO_SetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平
		
		
		temp = (int16_t)((buf[14] << 3) | (buf[13] >> 5));
		

	 
	if(temp > 1023)
	{
		temp = temp - 2048;
	}
	temp2 = temp*0.125 + 23;
	
	
	temp_Erro = 168 - temp;
	

			temp_I = temp_I + temp_Erro;
	
	if(temp_I > 20000)
		{
			temp_I = 20000;
		}
		if(temp_I < -1)
		{
			temp_I = -1;
		}
			temp_out = temp_Erro*421 + temp_I/10 + 3773;
				
		if(temp_out > 20000)
		{
			temp_out = 20000;
		}
		if(temp_out < 0)
		{
			temp_out = 0;
		}
		
		TIM_SetCompare1(TIM10,temp_out);//PD12



}



/**********************************************************************************、
任务名：State_Test_task(void *p_arg)
形式参数：无
作者：
时间：2022年2月3日
功能：
1.陀螺仪9轴数据刷新，3轴角速度，3轴加速度，3轴世界角度
2.程序执行时间统计
任务频率：500FPS
**********************************************************************************/

void State_Test_task(void *p_arg)
{

	OS_ERR err;
	p_arg = p_arg;
	
	//printf("状态机任务 开始\r\n");
	

while(1)
	{

	cap_test(); //输入捕获
	Get_IMU_Value();//更新陀螺仪加速计数据

  temp4 = KalmanFilter_1(  gyro_souce[0], 1,	12 , 1);//X轴角度卡尔曼滤波
	temp5 = KalmanFilter_2(  gyro_souce[1], 1,	12 , 1);//Y轴角度卡尔曼滤波
	temp6 = KalmanFilter_3(  gyro_souce[2], 1,	12 , 1);//Z轴角度卡尔曼滤波
				
	word_angle[0] = word_angle[0] + (double)temp4-3.6237;//X轴角度积分//-3.6237 50

	word_angle[1] = word_angle[1] + (double)temp5-3.7237;//Y轴角度积分//3.7237

	word_angle[2] = word_angle[2] + (double)temp6-3.7337;//Z轴角度积分//3.7337
	
//		if(word_angle[0] > 449036){word_angle[0] = word_angle[0] - 449036;}
//	else if(word_angle[0] < 0){word_angle[0] = word_angle[0] + 449036;}
	
		if(word_angle[1] > 449036){word_angle[1] = word_angle[1] - 449036;}
	else if(word_angle[1] < 0){word_angle[1] = word_angle[1] + 449036;}
	
		if(word_angle[2] > 449036){word_angle[2] = word_angle[2] - 449036;}
	else if(word_angle[2] < 0){word_angle[2] = word_angle[2] + 449036;}
	
//	printf("data1=%f,data2=%f,",word_angle[0],word_angle[0]);
	//printf("data1=%d,data2=%d,data3=%d\r\n",accel[0],accel[1],accel[2]);
	

	
	OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err); //延时1ms
	}
}
/*
底盘M3508电机检测函数封装
*/
/*
void Chassis_M3508_Test(void)
{
		morot1_test--;
		morot2_test--;
		morot3_test--;
		morot4_test--;
		if(morot1_test < 0){morot1_test = 0;}
		if(morot2_test < 0){morot2_test = 0;}
		if(morot3_test < 0){morot3_test = 0;}
		if(morot4_test < 0){morot4_test = 0;}
		
		
				if(morot1_test < 2)	{GPIO_SetBits(GPIOG,GPIO_Pin_1);}
		else{GPIO_ResetBits(GPIOG,GPIO_Pin_1);}
				if(morot2_test < 2)	{GPIO_SetBits(GPIOG,GPIO_Pin_2);}
		else{GPIO_ResetBits(GPIOG,GPIO_Pin_2);}
				if(morot3_test < 2)	{GPIO_SetBits(GPIOG,GPIO_Pin_3);}
		else{GPIO_ResetBits(GPIOG,GPIO_Pin_3);}
				if(morot4_test < 2)	{GPIO_SetBits(GPIOG,GPIO_Pin_4);}
		else{GPIO_ResetBits(GPIOG,GPIO_Pin_4);}
}
*/
