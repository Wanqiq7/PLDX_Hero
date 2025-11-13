#ifndef __TIME1_H_
#define __TIME1_H_

#include "delay.h"
#include "includes.h"
#include "includes.h"


void tmr1_callback(void *p_tmr, void *p_arg);	//定时器1回调函数


extern OS_TMR tmr1;

#endif
