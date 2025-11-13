#ifndef _KEYTASK_H
#define _KEYTASK_H

#include "AppTask.h"
#include "includes.h"
#include "bsp_key.h"

void State_Test_task(void *p_arg);
extern int accel_souce[3],gyro_souce[3],temp;
extern double word_angle[3];
#endif
