#ifndef __USART3_H__
#define __USART3_H__

#include <stm32f4xx.h>
#include <stdio.h>

#define RC_S1Up 1
#define RC_S1Down 2
#define RC_S1Middle 3

#define RC_S2Up 1
#define RC_S2Down 2
#define RC_S2Middle 3

extern short RC_Protection;

typedef struct
{
	struct
	{ 
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned char s1;
		unsigned char s2;
	}rc;
	
	struct 
	{
		 short x;
		 short y;
		 short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	
	struct
	{
		unsigned short w;
		unsigned short s;
		unsigned short a;
		unsigned short d;
		unsigned short q;
		unsigned short e;
		unsigned short shift;
		unsigned short ctrl;
		unsigned short r;
		unsigned short f;
		unsigned short g;
		unsigned short z;
		unsigned short x;
		unsigned short c;
		unsigned short v;
		unsigned short b;
		unsigned short ass;
	}key;
	
	struct
	{
		unsigned short che;
	}keep;

}RC_Ctl_t;

extern volatile  RC_Ctl_t RC_Ctl;


void USART3_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void USART3_Configuration(void);
#endif
