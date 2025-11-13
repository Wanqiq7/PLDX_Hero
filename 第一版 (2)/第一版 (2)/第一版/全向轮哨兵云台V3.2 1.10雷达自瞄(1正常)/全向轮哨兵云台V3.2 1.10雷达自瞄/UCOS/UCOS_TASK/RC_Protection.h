#ifndef _LED0TASK_H
#define _LED0TASK_H

#include "delay.h"
#include "led.h"
#include "AppTask.h"
#include "includes.h"
#include <stdint.h>
void RC_Protection_task(void *p_arg);
static void KeyEventCtrl(uint32_t event);

//typedef struct
//{
//	struct
//	{ 
//		unsigned short ch0;
//		unsigned short ch1;
//		unsigned short ch2;
//		unsigned short ch3;
//		unsigned char s1;
//		unsigned char s2;
//	}rc;
//	
//	struct 
//	{
//		 short x;
//		 short y;
//		 short z;
//		unsigned char press_l;
//		unsigned char press_r;
//	}mouse;
//	
//	struct
//	{
//		unsigned short w;
//		unsigned short s;
//		unsigned short a;
//		unsigned short d;
//		unsigned short q;
//		unsigned short e;
//		unsigned short shift;
//		unsigned short ctrl;
//		unsigned short r;
//		unsigned short f;
//		unsigned short g;
//		unsigned short z;
//		unsigned short x;
//		unsigned short c;
//		unsigned short v;
//		unsigned short b;
//		unsigned short ass;
//	}key;
//	
//	struct
//	{
//		unsigned short che;
//	}keep;

//}RC_Ctl_t_peotect;


//extern volatile  RC_Ctl_t_peotect RC_P;

#endif

