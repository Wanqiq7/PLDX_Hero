#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <string.h>
#include "UI_Library.h"
#include "UART.h"
#include "Delay.h"
float map_linear(float x) {
    float x_min = 0.0f;
    float x_max = 255.0f;
    float y_min = 340.0f;
    float y_max = 200.0f;

    // 线性映射
    float y = y_min + (x - x_min) * (y_max - y_min) / (x_max - x_min);
    return y;
}
ui_7_frame_t ui_hero_Ungroup_0;

ui_interface_line_t *ui_hero_Ungroup_vertical = (ui_interface_line_t*)&(ui_hero_Ungroup_0.data[0]);
ui_interface_line_t *ui_hero_Ungroup_vertical_1 = (ui_interface_line_t*)&(ui_hero_Ungroup_0.data[1]);
ui_interface_rect_t *ui_hero_Ungroup_NewRect = (ui_interface_rect_t*)&(ui_hero_Ungroup_0.data[2]);
ui_interface_line_t *ui_hero_Ungroup_line_1 = (ui_interface_line_t*)&(ui_hero_Ungroup_0.data[3]);
ui_interface_arc_t *ui_hero_Ungroup_edge1 = (ui_interface_arc_t*)&(ui_hero_Ungroup_0.data[4]);
ui_interface_arc_t *ui_hero_Ungroup_edge = (ui_interface_arc_t*)&(ui_hero_Ungroup_0.data[5]);
ui_interface_line_t *ui_hero_Ungroup_line_2 = (ui_interface_line_t*)&(ui_hero_Ungroup_0.data[6]);

void _ui_init_hero_Ungroup_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_hero_Ungroup_0.data[i].figure_name[0] = 0;
        ui_hero_Ungroup_0.data[i].figure_name[1] = 0;
        ui_hero_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_hero_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_hero_Ungroup_0.data[i].operate_type = 0;
    }

    ui_hero_Ungroup_vertical->figure_type = 0;
    ui_hero_Ungroup_vertical->operate_type = 1;
    ui_hero_Ungroup_vertical->layer = 0;
    ui_hero_Ungroup_vertical->color = 6;
    ui_hero_Ungroup_vertical->start_x = 1070;
    ui_hero_Ungroup_vertical->start_y = 952;
    ui_hero_Ungroup_vertical->width = 2;
    ui_hero_Ungroup_vertical->end_x = 1070;
    ui_hero_Ungroup_vertical->end_y = 273;

    ui_hero_Ungroup_vertical_1->figure_type = 0;
    ui_hero_Ungroup_vertical_1->operate_type = 1;
    ui_hero_Ungroup_vertical_1->layer = 0;
    ui_hero_Ungroup_vertical_1->color = 1;
    ui_hero_Ungroup_vertical_1->start_x = 988;
    ui_hero_Ungroup_vertical_1->start_y = 937;
    ui_hero_Ungroup_vertical_1->width = 3;
    ui_hero_Ungroup_vertical_1->end_x = 988;
    ui_hero_Ungroup_vertical_1->end_y = 258;

    ui_hero_Ungroup_NewRect->figure_type = 1;
    ui_hero_Ungroup_NewRect->operate_type = 1;
    ui_hero_Ungroup_NewRect->layer = 0;
    ui_hero_Ungroup_NewRect->color = 2;
    ui_hero_Ungroup_NewRect->start_x = 676;
    ui_hero_Ungroup_NewRect->start_y = 353;
    ui_hero_Ungroup_NewRect->width = 2;
    ui_hero_Ungroup_NewRect->end_x = 1257;
    ui_hero_Ungroup_NewRect->end_y = 758;

    ui_hero_Ungroup_line_1->figure_type = 0;
    ui_hero_Ungroup_line_1->operate_type = 1;
    ui_hero_Ungroup_line_1->layer = 0;
    ui_hero_Ungroup_line_1->color = 1;
    ui_hero_Ungroup_line_1->start_x = 887;
    ui_hero_Ungroup_line_1->start_y = 839;
    ui_hero_Ungroup_line_1->width = 2;
    ui_hero_Ungroup_line_1->end_x = 1087;
    ui_hero_Ungroup_line_1->end_y = 839;

    ui_hero_Ungroup_edge1->figure_type = 4;
    ui_hero_Ungroup_edge1->operate_type = 1;
    ui_hero_Ungroup_edge1->layer = 0;
    ui_hero_Ungroup_edge1->color = 8;
    ui_hero_Ungroup_edge1->start_x = 567;
    ui_hero_Ungroup_edge1->start_y = 512;
    ui_hero_Ungroup_edge1->width = 3;
    ui_hero_Ungroup_edge1->start_angle = 200;
    ui_hero_Ungroup_edge1->end_angle = 340;
    ui_hero_Ungroup_edge1->rx = 199;
    ui_hero_Ungroup_edge1->ry = 276;

    ui_hero_Ungroup_edge->figure_type = 4;
    ui_hero_Ungroup_edge->operate_type = 1;
    ui_hero_Ungroup_edge->layer = 0;
    ui_hero_Ungroup_edge->color = 8;
    ui_hero_Ungroup_edge->start_x = 583;
    ui_hero_Ungroup_edge->start_y = 511;
    ui_hero_Ungroup_edge->width = 3;
    ui_hero_Ungroup_edge->start_angle = 202;
    ui_hero_Ungroup_edge->end_angle = 338;
    ui_hero_Ungroup_edge->rx = 193;
    ui_hero_Ungroup_edge->ry = 256;

    ui_hero_Ungroup_line_2->figure_type = 0;
    ui_hero_Ungroup_line_2->operate_type = 1;
    ui_hero_Ungroup_line_2->layer = 0;
    ui_hero_Ungroup_line_2->color = 1;
    ui_hero_Ungroup_line_2->start_x = 837;
    ui_hero_Ungroup_line_2->start_y = 730;
    ui_hero_Ungroup_line_2->width = 2;
    ui_hero_Ungroup_line_2->end_x = 1137;
    ui_hero_Ungroup_line_2->end_y = 730;


    ui_proc_7_frame(&ui_hero_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_hero_Ungroup_0, sizeof(ui_hero_Ungroup_0));
}


ui_5_frame_t ui_hero_Ungroup_1;

ui_interface_line_t *ui_hero_Ungroup_line_3 = (ui_interface_line_t*)&(ui_hero_Ungroup_1.data[0]);
ui_interface_line_t *ui_hero_Ungroup_line_4 = (ui_interface_line_t*)&(ui_hero_Ungroup_1.data[1]);
ui_interface_line_t *ui_hero_Ungroup_line_5 = (ui_interface_line_t*)&(ui_hero_Ungroup_1.data[2]);
ui_interface_line_t *ui_hero_Ungroup_line_6 = (ui_interface_line_t*)&(ui_hero_Ungroup_1.data[3]);

void _ui_init_hero_Ungroup_1(void) {
    for (int i = 0; i < 4; i++) {
        ui_hero_Ungroup_1.data[i].figure_name[0] = 0;
        ui_hero_Ungroup_1.data[i].figure_name[1] = 0;
        ui_hero_Ungroup_1.data[i].figure_name[2] = i + 7;
        ui_hero_Ungroup_1.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_hero_Ungroup_1.data[i].operate_type = 0;
    }

    ui_hero_Ungroup_line_3->figure_type = 0;
    ui_hero_Ungroup_line_3->operate_type = 1;
    ui_hero_Ungroup_line_3->layer = 0;
    ui_hero_Ungroup_line_3->color = 1;
    ui_hero_Ungroup_line_3->start_x = 837;
    ui_hero_Ungroup_line_3->start_y = 640;
    ui_hero_Ungroup_line_3->width = 3;
    ui_hero_Ungroup_line_3->end_x = 1143;
    ui_hero_Ungroup_line_3->end_y = 640;

    ui_hero_Ungroup_line_4->figure_type = 0;
    ui_hero_Ungroup_line_4->operate_type = 1;
    ui_hero_Ungroup_line_4->layer = 0;
    ui_hero_Ungroup_line_4->color = 1;
    ui_hero_Ungroup_line_4->start_x = 849;
    ui_hero_Ungroup_line_4->start_y = 522;
    ui_hero_Ungroup_line_4->width = 2;
    ui_hero_Ungroup_line_4->end_x = 1131;
    ui_hero_Ungroup_line_4->end_y = 522;

    ui_hero_Ungroup_line_5->figure_type = 0;
    ui_hero_Ungroup_line_5->operate_type = 1;
    ui_hero_Ungroup_line_5->layer = 0;
    ui_hero_Ungroup_line_5->color = 6;
    ui_hero_Ungroup_line_5->start_x = 900;
    ui_hero_Ungroup_line_5->start_y = 457;
    ui_hero_Ungroup_line_5->width = 1;
    ui_hero_Ungroup_line_5->end_x = 1090;
    ui_hero_Ungroup_line_5->end_y = 457;

    ui_hero_Ungroup_line_6->figure_type = 0;
    ui_hero_Ungroup_line_6->operate_type = 1;
    ui_hero_Ungroup_line_6->layer = 0;
    ui_hero_Ungroup_line_6->color = 6;
    ui_hero_Ungroup_line_6->start_x = 900;
    ui_hero_Ungroup_line_6->start_y = 411;
    ui_hero_Ungroup_line_6->width = 1;
    ui_hero_Ungroup_line_6->end_x = 1090;
    ui_hero_Ungroup_line_6->end_y = 411;


    ui_proc_5_frame(&ui_hero_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_hero_Ungroup_1, sizeof(ui_hero_Ungroup_1));
}

ui_string_frame_t ui_hero_Ungroup_2;
ui_interface_string_t* ui_hero_Ungroup_Q = &(ui_hero_Ungroup_2.option);

void _ui_init_hero_Ungroup_2(void) {
    ui_hero_Ungroup_2.option.figure_name[0] = 0;
    ui_hero_Ungroup_2.option.figure_name[1] = 0;
    ui_hero_Ungroup_2.option.figure_name[2] = 11;
    ui_hero_Ungroup_2.option.operate_type = 1;

    ui_hero_Ungroup_Q->figure_type = 7;
    ui_hero_Ungroup_Q->operate_type = 1;
    ui_hero_Ungroup_Q->layer = 0;
    ui_hero_Ungroup_Q->color = 1;
    ui_hero_Ungroup_Q->start_x = 100;
    ui_hero_Ungroup_Q->start_y = 792;
    ui_hero_Ungroup_Q->width = 2;
    ui_hero_Ungroup_Q->font_size = 20;
    ui_hero_Ungroup_Q->str_length = 2;
    strcpy(ui_hero_Ungroup_Q->string, "Q:");


    ui_proc_string_frame(&ui_hero_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_hero_Ungroup_2, sizeof(ui_hero_Ungroup_2));
}


ui_string_frame_t ui_hero_Ungroup_3;
ui_interface_string_t* ui_hero_Ungroup_Shift = &(ui_hero_Ungroup_3.option);

void _ui_init_hero_Ungroup_3(void) {
    ui_hero_Ungroup_3.option.figure_name[0] = 0;
    ui_hero_Ungroup_3.option.figure_name[1] = 0;
    ui_hero_Ungroup_3.option.figure_name[2] = 12;
    ui_hero_Ungroup_3.option.operate_type = 1;

    ui_hero_Ungroup_Shift->figure_type = 7;
    ui_hero_Ungroup_Shift->operate_type = 1;
    ui_hero_Ungroup_Shift->layer = 0;
    ui_hero_Ungroup_Shift->color = 1;
    ui_hero_Ungroup_Shift->start_x = 100;
    ui_hero_Ungroup_Shift->start_y = 742;
    ui_hero_Ungroup_Shift->width = 2;
    ui_hero_Ungroup_Shift->font_size = 20;
    ui_hero_Ungroup_Shift->str_length = 6;
    strcpy(ui_hero_Ungroup_Shift->string, "Shift:");


    ui_proc_string_frame(&ui_hero_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_hero_Ungroup_3, sizeof(ui_hero_Ungroup_3));
}

ui_string_frame_t ui_hero_Ungroup_4;
ui_interface_string_t* ui_hero_Ungroup_ctrl = &(ui_hero_Ungroup_4.option);

void _ui_init_hero_Ungroup_4(void) {
    ui_hero_Ungroup_4.option.figure_name[0] = 0;
    ui_hero_Ungroup_4.option.figure_name[1] = 0;
    ui_hero_Ungroup_4.option.figure_name[2] = 13;
    ui_hero_Ungroup_4.option.operate_type = 1;

    ui_hero_Ungroup_ctrl->figure_type = 7;
    ui_hero_Ungroup_ctrl->operate_type = 1;
    ui_hero_Ungroup_ctrl->layer = 0;
    ui_hero_Ungroup_ctrl->color = 1;
    ui_hero_Ungroup_ctrl->start_x = 100;
    ui_hero_Ungroup_ctrl->start_y = 692;
    ui_hero_Ungroup_ctrl->width = 2;
    ui_hero_Ungroup_ctrl->font_size = 20;
    ui_hero_Ungroup_ctrl->str_length = 5;
    strcpy(ui_hero_Ungroup_ctrl->string, "Ctrl:");


    ui_proc_string_frame(&ui_hero_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_hero_Ungroup_4, sizeof(ui_hero_Ungroup_4));
}

ui_string_frame_t ui_hero_Ungroup_5;
ui_interface_string_t* ui_hero_Ungroup_X = &(ui_hero_Ungroup_5.option);

void _ui_init_hero_Ungroup_5(void) {
    ui_hero_Ungroup_5.option.figure_name[0] = 0;
    ui_hero_Ungroup_5.option.figure_name[1] = 0;
    ui_hero_Ungroup_5.option.figure_name[2] = 14;
    ui_hero_Ungroup_5.option.operate_type = 1;

    ui_hero_Ungroup_X->figure_type = 7;
    ui_hero_Ungroup_X->operate_type = 1;
    ui_hero_Ungroup_X->layer = 0;
    ui_hero_Ungroup_X->color = 1;
    ui_hero_Ungroup_X->start_x = 100;
    ui_hero_Ungroup_X->start_y = 642;
    ui_hero_Ungroup_X->width = 2;
    ui_hero_Ungroup_X->font_size = 20;
    ui_hero_Ungroup_X->str_length = 2;
    strcpy(ui_hero_Ungroup_X->string, "X:");


    ui_proc_string_frame(&ui_hero_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_hero_Ungroup_5, sizeof(ui_hero_Ungroup_5));
}

ui_7_frame_t ui_hero_update_0;

ui_interface_arc_t *ui_hero_update_NewArc = (ui_interface_arc_t*)&(ui_hero_update_0.data[0]);
ui_interface_round_t *ui_hero_update_Q_point = (ui_interface_round_t*)&(ui_hero_update_0.data[1]);
ui_interface_round_t *ui_hero_update_X_point = (ui_interface_round_t*)&(ui_hero_update_0.data[2]);
ui_interface_round_t *ui_hero_update_ctrl_point = (ui_interface_round_t*)&(ui_hero_update_0.data[3]);
ui_interface_round_t *ui_hero_update_shift_point = (ui_interface_round_t*)&(ui_hero_update_0.data[4]);
ui_interface_arc_t *ui_hero_update_Energy_bar = (ui_interface_arc_t*)&(ui_hero_update_0.data[5]);

void _ui_init_hero_update_0(void) {
    for (int i = 0; i < 6; i++) {
        ui_hero_update_0.data[i].figure_name[0] = 0;
        ui_hero_update_0.data[i].figure_name[1] = 1;
        ui_hero_update_0.data[i].figure_name[2] = i + 0;
        ui_hero_update_0.data[i].operate_type = 1;
    }
    for (int i = 6; i < 7; i++) {
        ui_hero_update_0.data[i].operate_type = 0;
    }

    ui_hero_update_NewArc->figure_type = 4;
    ui_hero_update_NewArc->operate_type = 1;
    ui_hero_update_NewArc->layer = 0;
    ui_hero_update_NewArc->color = 4;
    ui_hero_update_NewArc->start_x = 959;
    ui_hero_update_NewArc->start_y = 495;
    ui_hero_update_NewArc->width = 4;
    ui_hero_update_NewArc->start_angle = 120;
    ui_hero_update_NewArc->end_angle = 240;
    ui_hero_update_NewArc->rx = 200;
    ui_hero_update_NewArc->ry = 200;

    ui_hero_update_Q_point->figure_type = 2;
    ui_hero_update_Q_point->operate_type = 1;
    ui_hero_update_Q_point->layer = 0;
    ui_hero_update_Q_point->color = 8;
    ui_hero_update_Q_point->start_x = 162;
    ui_hero_update_Q_point->start_y = 773;
    ui_hero_update_Q_point->width = 6;
    ui_hero_update_Q_point->r = 15;

    ui_hero_update_X_point->figure_type = 2;
    ui_hero_update_X_point->operate_type = 1;
    ui_hero_update_X_point->layer = 0;
    ui_hero_update_X_point->color = 8;
    ui_hero_update_X_point->start_x = 160;
    ui_hero_update_X_point->start_y = 625;
    ui_hero_update_X_point->width = 6;
    ui_hero_update_X_point->r = 15;

    ui_hero_update_ctrl_point->figure_type = 2;
    ui_hero_update_ctrl_point->operate_type = 1;
    ui_hero_update_ctrl_point->layer = 0;
    ui_hero_update_ctrl_point->color = 8;
    ui_hero_update_ctrl_point->start_x = 217;
    ui_hero_update_ctrl_point->start_y = 675;
    ui_hero_update_ctrl_point->width = 6;
    ui_hero_update_ctrl_point->r = 15;

    ui_hero_update_shift_point->figure_type = 2;
    ui_hero_update_shift_point->operate_type = 1;
    ui_hero_update_shift_point->layer = 0;
    ui_hero_update_shift_point->color = 8;
    ui_hero_update_shift_point->start_x = 240;
    ui_hero_update_shift_point->start_y = 723;
    ui_hero_update_shift_point->width = 6;
    ui_hero_update_shift_point->r = 15;

    ui_hero_update_Energy_bar->figure_type = 4;
    ui_hero_update_Energy_bar->operate_type = 1;
    ui_hero_update_Energy_bar->layer = 0;
    ui_hero_update_Energy_bar->color = 5;
    ui_hero_update_Energy_bar->start_x = 567;
    ui_hero_update_Energy_bar->start_y = 501;
    ui_hero_update_Energy_bar->width = 20;
    ui_hero_update_Energy_bar->start_angle = 180.0f;
    ui_hero_update_Energy_bar->end_angle = 340;
    ui_hero_update_Energy_bar->rx = 196;
    ui_hero_update_Energy_bar->ry = 265;


    ui_proc_7_frame(&ui_hero_update_0);
    SEND_MESSAGE((uint8_t *) &ui_hero_update_0, sizeof(ui_hero_update_0));
}
//===========================shift¿ª
void _ui_update_hero_shift_point_on(void) {
    ui_hero_update_0.data[0].operate_type = 0;
    ui_hero_update_0.data[1].operate_type = 0;
    ui_hero_update_0.data[2].operate_type = 0;
    ui_hero_update_0.data[3].operate_type = 0;
    ui_hero_update_0.data[4].operate_type = 2;
    ui_hero_update_0.data[5].operate_type = 0;

    ui_hero_update_shift_point->color = 4;

    ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}

//===========================shift¹؍
void _ui_update_hero_shift_point_off(void) {
    ui_hero_update_0.data[0].operate_type = 0;
    ui_hero_update_0.data[1].operate_type = 0;
    ui_hero_update_0.data[2].operate_type = 0;
    ui_hero_update_0.data[3].operate_type = 0;
    ui_hero_update_0.data[4].operate_type = 2;
    ui_hero_update_0.data[5].operate_type = 0;

    ui_hero_update_shift_point->color = 8;

    ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}
void _ui_update_hero_change(float angle,float V) {
	ui_hero_update_0.data[0].operate_type = 2;
	ui_hero_update_0.data[1].operate_type = 0;
	ui_hero_update_0.data[2].operate_type = 0;
	ui_hero_update_0.data[3].operate_type = 0;
	ui_hero_update_0.data[4].operate_type = 0;
	ui_hero_update_0.data[5].operate_type = 2;
	
	if(angle-60.0f<0.0f)ui_hero_update_NewArc->start_angle=360.0f+angle-60.0f;
    else ui_hero_update_NewArc->start_angle = angle-60.0f;
    if(angle+60.0f>360.0f)ui_hero_update_NewArc->end_angle =angle+60.0f-360.0f;
	else ui_hero_update_NewArc->end_angle =angle+60.0f;
	ui_hero_update_Energy_bar->start_angle = 180.0f; 
	ui_hero_update_Energy_bar->end_angle=map_linear(V)+180.0f;
	ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}
//=======================================X¿ª
void _ui_update_hero_X_point_on(void) {
	ui_hero_update_0.data[0].operate_type = 0;
	ui_hero_update_0.data[1].operate_type = 0;
	ui_hero_update_0.data[2].operate_type = 2;
	ui_hero_update_0.data[3].operate_type = 0;
	ui_hero_update_0.data[4].operate_type = 0;
	ui_hero_update_0.data[5].operate_type = 0;

    ui_hero_update_X_point->color = 4;
	
	 ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}
//=======================================X¹؍
void _ui_update_hero_X_point_off(void) {
	ui_hero_update_0.data[0].operate_type = 0;
	ui_hero_update_0.data[1].operate_type = 0;
	ui_hero_update_0.data[2].operate_type = 2;
	ui_hero_update_0.data[3].operate_type = 0;
	ui_hero_update_0.data[4].operate_type = 0;
	ui_hero_update_0.data[5].operate_type = 0;
	
    ui_hero_update_X_point->color = 8;

	 ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}

//=======================================Q¿ª
void _ui_update_hero_Q_point_on(void) {
	ui_hero_update_0.data[0].operate_type = 0;
	ui_hero_update_0.data[1].operate_type = 2;
	ui_hero_update_0.data[2].operate_type = 0;
	ui_hero_update_0.data[4].operate_type = 0;
	ui_hero_update_0.data[5].operate_type = 0;
    ui_hero_update_0.data[3].operate_type = 0;
	
    ui_hero_update_Q_point->color = 4;
	
    ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}

//=======================================Q¹؍
void _ui_update_hero_Q_point_off(void) {
	ui_hero_update_0.data[0].operate_type = 0;
	ui_hero_update_0.data[1].operate_type = 2;
	ui_hero_update_0.data[2].operate_type = 0;
	ui_hero_update_0.data[4].operate_type = 0;
	ui_hero_update_0.data[5].operate_type = 0;
    ui_hero_update_0.data[3].operate_type = 0;

    ui_hero_update_Q_point->color = 8;
	
    ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}
//=======================================ctrl¿ª
void _ui_update_hero_ctrl_point_on(void) {
	ui_hero_update_0.data[0].operate_type = 0;
	ui_hero_update_0.data[1].operate_type = 0;
	ui_hero_update_0.data[2].operate_type = 0;
	ui_hero_update_0.data[3].operate_type = 2;
	ui_hero_update_0.data[4].operate_type = 0;
	ui_hero_update_0.data[5].operate_type = 0;

    ui_hero_update_ctrl_point->color = 4;

    ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}
//=======================================ctrl¹؍
void _ui_update_hero_ctrl_point_off(void) {
	ui_hero_update_0.data[0].operate_type = 0;
	ui_hero_update_0.data[1].operate_type = 0;
	ui_hero_update_0.data[2].operate_type = 0;
	ui_hero_update_0.data[3].operate_type = 2;
	ui_hero_update_0.data[4].operate_type = 0;
	ui_hero_update_0.data[5].operate_type = 0;

    ui_hero_update_ctrl_point->color = 8;

    ui_proc_7_frame(&ui_hero_update_0);
    UART1_SendArray((uint8_t *)&ui_hero_update_0,sizeof(ui_hero_update_0));
}

