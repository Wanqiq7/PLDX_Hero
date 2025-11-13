#include "State_Test.h"
#include "time1.h"

OS_TMR tmr1;	//定时器1




//定时器1的回调函数
void tmr1_callback(void *p_tmr, void *p_arg)
{
	//OS_ERR err;
	static u8 tmr1_num=0;
	
	//OSSemPend(&MY_SEM, 0, OS_OPT_PEND_BLOCKING, 0, &err);//请求信号量
	tmr1_num++;		//定时器1执行次数加1
	//printf("tmr1=%d\r\n", tmr1_num);
	
	//printf("%d\r\n",test);
	//test = 0;
	
	//OSTaskSemPost(&KeyTaskTCB, OS_OPT_POST_NONE, &err);	//发送内嵌信号量
	//OSSemPost(&MY_SEM, OS_OPT_POST_1, &err);	//发送信号量
}


