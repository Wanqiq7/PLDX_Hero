//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_hero_H
#define UI_hero_H
#include "UI_Base.h"
//#include "ui_interface.h"
#define SEND_MESSAGE(message, length) UART1_SendArray(message, length);
void _ui_init_hero_Ungroup_0(void);
void _ui_init_hero_Ungroup_1(void);
void _ui_init_hero_Ungroup_2(void);
void _ui_init_hero_Ungroup_3(void);
void _ui_init_hero_Ungroup_4(void);
void _ui_init_hero_Ungroup_5(void);

void _ui_init_hero_update_0(void);


void _ui_update_hero_change(float angle,float V);
void _ui_update_hero_X_point_on(void);
void _ui_update_hero_X_point_off(void);
void _ui_update_hero_Q_point_on(void);
void _ui_update_hero_Q_point_off(void);
void _ui_update_hero_ctrl_point_on(void);
void _ui_update_hero_ctrl_point_off(void);
void _ui_update_hero_shift_point_on(void);
void _ui_update_hero_shift_point_off(void);

void _ui_update_default_Ungroup_2_NOCheck(void);
void _ui_update_default_Ungroup_2_ERROR(uint8_t flag);
#endif // UI_hero_H
