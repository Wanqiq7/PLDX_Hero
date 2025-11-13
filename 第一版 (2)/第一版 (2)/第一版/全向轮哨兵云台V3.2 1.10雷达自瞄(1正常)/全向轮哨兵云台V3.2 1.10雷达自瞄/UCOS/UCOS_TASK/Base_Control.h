#ifndef __BASE_CONTROL_H
#define __BASE_CONTROL_H
#include <sys.h>
#include <includes.h>
#include <os_app_hooks.h>
#include "Time1.h"
#include "RC_Protection.h"
#include "GimbalTask.h"
#include "chassisTask.h"
#include "State_Test.h"
#include "Can_Task.h"
#include "FW_Keep.h"
#include "FlickTask.h"
#include "Referee_System.h"

void Base_Control_task(void *p_arg);


typedef struct _Base_Control_Param_
{
	u8 Flag;
  u32 Count;
	u8 Flag1;
  u32 Count1;
}_Base_Control_Param;



#endif
