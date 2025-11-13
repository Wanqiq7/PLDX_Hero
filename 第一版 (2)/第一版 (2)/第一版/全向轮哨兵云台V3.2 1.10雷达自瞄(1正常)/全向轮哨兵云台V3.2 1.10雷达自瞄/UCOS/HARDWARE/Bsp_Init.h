#ifndef _BSP_INIT_H
#define _BSP_INIT_H


#include "led.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "bsp_key.h"

#include "usart.h"
//#include "usart1.h"
#include "usart2.h"
#include "usart3_rc.h"
//#include "usart6.h"

#include "pwm.h"
#include "timer.h"
#include "TIM7_FPS.h"

#include "can1.h"
#include "can2.h"

#include "iwdg.h"

#include "spi.h"
#include "light_short.h"

#include "buzzer.h"

void Bsp_Init(void);

#endif
