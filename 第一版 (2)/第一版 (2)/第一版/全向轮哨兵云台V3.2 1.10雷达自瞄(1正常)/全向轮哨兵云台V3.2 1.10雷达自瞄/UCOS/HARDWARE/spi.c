#include "spi.h"
#include "Bsp_Init.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//SPI 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define BMI088_TEMP_FACTOR 0.125f

#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150


#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        GPIO_ResetBits(GPIOA,GPIO_Pin_4);                  \
        SPI1_ReadWriteByte((reg));				\
				SPI1_ReadWriteByte((data)); 			\
        GPIO_SetBits(GPIOA,GPIO_Pin_4);                      \
    }
		
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        GPIO_ResetBits(GPIOA,GPIO_Pin_4);                   \
        SPI1_ReadWriteByte((reg) | 0x80);   \
        SPI1_ReadWriteByte(0x55);           \
        (data) = SPI1_ReadWriteByte(0x55);  \
        GPIO_SetBits(GPIOA,GPIO_Pin_4);                     \
    }
		
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);                       \
        SPI1_ReadWriteByte(reg); \
			SPI1_ReadWriteByte(data);	\
			\
      GPIO_SetBits(GPIOB,GPIO_Pin_0);                  \
    }
		
		
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);                      \
        BMI088_read_single_reg((reg), &(data));						\
        GPIO_SetBits(GPIOB,GPIO_Pin_0);                    \
    }																												\
		
		
		
		#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        GPIO_ResetBits(GPIOA,GPIO_Pin_4);                      \
        SPI1_ReadWriteByte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        GPIO_SetBits(GPIOA,GPIO_Pin_4);                       \
    }
		
		enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

		static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
		
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI口初始化
//这里针是对SPI1的初始化
		static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    SPI1_ReadWriteByte(reg | 0x80);
    *return_data = SPI1_ReadWriteByte(0x55);
}
		

void SPI1_Init(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
//	

//			 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PA4普通引脚功能输出,作为加速度的片选
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化
	//GPIO_SetBits(GPIOF,GPIO_Pin_6); //GPIOF 高电平
	
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟
 
  //GPIOFB3,4,5初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;//PB3~4复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PB7复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PB7复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //PB3复用为 SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1); //PB4复用为 SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PB7复用为 SPI1
 
	//这里只针对SPI口初始化
		 
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PA4普通引脚功能输出,作为加速度的片选
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	GPIO_SetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平		 
			 
			 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0普通引脚功能输出,作为角速度的片选
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	GPIO_SetBits(GPIOB,GPIO_Pin_0); //GPIOF 高电平	
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	
	SPI_InitStructure.SPI_CRCPolynomial = 10;	//CRC值计算的多项式
	
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设


	
	
	
	
	

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOB时钟	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	//TIM1时钟使能    
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10); //GPIOE9复用为定时器1
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOE9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure);              //初始化GPIOE
	
		TIM_TimeBaseStructure.TIM_Prescaler=45-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
		//初始化TIM4 通道1 PWM模式 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM10,ENABLE);//ARPE使能 
	TIM_Cmd(TIM10, ENABLE);  //使能TIM4
	
		
	TIM_TimeBaseInit(TIM10 ,&TIM_TimeBaseStructure);//初始化定时器1
	
	TIM_CtrlPWMOutputs(TIM10, ENABLE);    //使能PWM输出
	
//	DMA_InitTypeDef dma;
//	DMA_Cmd(DMA2_Stream2, DISABLE);
//    //while (DMA2_Stream2->CR & DMA_SxCR_EN);
//		DMA_DeInit(DMA2_Stream2);
//    dma.DMA_Channel= DMA_Channel_3;
//    //dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//    //dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
//    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//    //dma.DMA_BufferSize = 18;
//    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma.DMA_Mode = DMA_Mode_Normal;
//    dma.DMA_Priority = DMA_Priority_VeryHigh;
//    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//    //dma.DMA_MemoryBurst = DMA_Mode_Normal;
//		//dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//    //dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA2_Stream5,&dma);

//		//DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,DISABLE);
//    DMA_Cmd(DMA2_Stream5,ENABLE);


	//SPI1_ReadWriteByte(0xff);//启动传输		 
	GPIO_SetBits(GPIOB,GPIO_Pin_0); //GPIOF 高电平
	GPIO_SetBits(GPIOA,GPIO_Pin_4); //GPIOF 高电平
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
	delay_ms(175);
	bmi088_gyro_init();
	bmi088_accel_init();
}   
//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
} 
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
	
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
	//printf("TX = %d,RX = %d\r\n",TxData,SPI_I2S_ReceiveData(SPI1));
//	printf("1+1");
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
 		    
}

unsigned char bmi088_accel_init(void)
{
		uint8_t res = 0;
    uint8_t write_reg_num = 0;
	
	
	//check commiunication
    BMI088_accel_read_single_reg(0x00, res);
    delay_us(222);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(222);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(222);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(222);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        delay_us(222);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        delay_us(222);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
	
	
	
	
	
	
}

	
unsigned char bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(0x00, res);
    delay_us(222);
    BMI088_gyro_read_single_reg(0x00, res);
    delay_us(222);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(222);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(0x00, res);
    delay_us(222);
    BMI088_gyro_read_single_reg(0x00, res);
    delay_us(222);

	if (res != 0x0F)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        delay_us(222);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        delay_us(222);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }
		
} 

