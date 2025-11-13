#include "BMI088Middleware.h"
#include "spi.h"

//extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}







uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    rx_data = SPI1_ReadWriteByte(txdata);
    return rx_data;
}

