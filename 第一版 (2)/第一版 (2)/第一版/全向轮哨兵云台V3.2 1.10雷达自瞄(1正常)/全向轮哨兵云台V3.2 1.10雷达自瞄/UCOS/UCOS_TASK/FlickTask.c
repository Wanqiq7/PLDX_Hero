#include "FlickTask.h"
#include "AppTask.h"
#include "usart3_rc.h"
#include "FW_Keep.h"

/**********
拨弹任务
//待填
**********/

void Flick_TASK(void)
{
	OS_ERR err;
	//printf("创建拨弹保持任务\r\n");
	
	while(1)
	{
	


//		if(i2>0)
//		{
//			post_target = post_target-32766;
//			i2--;
//		}
		
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms
	}
}

