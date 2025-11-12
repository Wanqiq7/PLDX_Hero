#ifndef __UI_BASE_H
#define __UI_BASE_H
#include "stm32f4xx.h"                  // Device header


#define PRIMITIVE_CAT(x, y) x ## y
#define MESSAGE_PACKED __attribute__((packed))
#define DEFINE_MESSAGE(name, p_a, p_b, p_c, p_d, p_e)   \
typedef struct {                                        \
uint8_t figure_name[3];                                 \
uint32_t operate_type:3;                               \
uint32_t figure_type:3;                                 \
uint32_t layer:4;                                       \
uint32_t color:4;                                       \
uint32_t PRIMITIVE_CAT(,p_a) :9;                        \
uint32_t PRIMITIVE_CAT(,p_b):9;                         \
uint32_t width:10;                                      \
uint32_t start_x:11;                                    \
uint32_t start_y:11;                                    \
uint32_t PRIMITIVE_CAT(,p_c):10;                        \
uint32_t PRIMITIVE_CAT(,p_d):11;                        \
uint32_t PRIMITIVE_CAT(,p_e):11;                        \
} MESSAGE_PACKED ui_interface_ ## name ##_t

DEFINE_MESSAGE(figure, _a, _b, _c, _d, _e);
DEFINE_MESSAGE(line, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(rect, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(round, _a, _b, r, _d, _e);
DEFINE_MESSAGE(ellipse, _a, _b, _c, rx, ry);
DEFINE_MESSAGE(arc, start_angle, end_angle, _c, rx, ry);

typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type: 3;
    uint32_t figure_type: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t _b: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    int32_t number;
} MESSAGE_PACKED ui_interface_number_t;

typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type: 3;
    uint32_t figure_type: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t str_length: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t _c: 10;
    uint32_t _d: 11;
    uint32_t _e: 11;
    char string[30];
} MESSAGE_PACKED ui_interface_string_t;

typedef struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
	uint8_t CRC8;
    uint16_t cmd_id;
	uint16_t dada_cmd_id;
    uint16_t sender_id;
	uint16_t receiver_id;
}MESSAGE_PACKED ui_frame_header_t;

#define DEFINE_FIGURE_MESSAGE(num)      \
typedef struct {                        \
ui_frame_header_t header;               \
ui_interface_figure_t data[num];        \
uint16_t CRC16;                         \
} MESSAGE_PACKED ui_ ## num##_frame_t

DEFINE_FIGURE_MESSAGE(1);
DEFINE_FIGURE_MESSAGE(2);
DEFINE_FIGURE_MESSAGE(5);
DEFINE_FIGURE_MESSAGE(7);

typedef struct {
    ui_frame_header_t header;
    ui_interface_string_t option;
    uint16_t CRC16;
} MESSAGE_PACKED ui_string_frame_t;

typedef struct {
    ui_frame_header_t header;
    uint8_t delete_type;
    uint8_t layer;
    uint16_t CRC16;
} MESSAGE_PACKED ui_delete_frame_t;

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void ui_proc_delete_frame(ui_delete_frame_t *msg);

extern ui_string_frame_t _ui_string_frame;
extern ui_1_frame_t _ui_1_frame;
extern ui_2_frame_t _ui_2_frame;
extern ui_5_frame_t _ui_5_frame;
extern ui_7_frame_t _ui_7_frame;

#endif //UI_TYPES_H
