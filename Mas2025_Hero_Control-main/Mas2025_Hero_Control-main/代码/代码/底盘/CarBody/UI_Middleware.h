#ifndef __UI_MIDDLEWARE_H
#define __UI_MIDDLEWARE_H

#include "stdint.h"

void UI_Middleware_Init(uint8_t Flag);
void UI_Middleware_Updata(float angle_data,float V_,float pitch_data,float d_data);

#endif
