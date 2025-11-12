#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "UI_Base.h"
#include <string.h>
#include "RefereeSystem.h"
#include "RefereeSystem_CRCTable.h"

uint8_t UI_seq=0;

#define DEFINE_FRAME_PROC(num,id)														\
void ui_proc_##num##_frame(ui_##num##_frame_t *msg)										\
{   																					\
    msg->header.SOF=0xA5;																\
    msg->header.data_length=6+15*num;													\
    msg->header.seq=UI_seq++;															\
    msg->header.CRC8=RefereeSystem_GetCRC8CheckSum((uint8_t*)msg,4,CRC8_Initial);		\
    msg->header.cmd_id=0x0301;															\
    msg->header.dada_cmd_id=id;															\
    msg->header.sender_id=RefereeSystem_RobotID;													\
    msg->header.receiver_id=RefereeSystem_RobotID+256;												\
    msg->CRC16=RefereeSystem_GetCRC16CheckSum((uint8_t*)msg,13+15*num,CRC16_Initial);	\
}

DEFINE_FRAME_PROC(1,0x0101)
DEFINE_FRAME_PROC(2,0x0102)
DEFINE_FRAME_PROC(5,0x0103)
DEFINE_FRAME_PROC(7,0x0104)

void ui_proc_string_frame(ui_string_frame_t *msg)
{
    msg->header.SOF=0xA5;
    msg->header.data_length=51;
    msg->header.seq=UI_seq++;
    msg->header.CRC8=RefereeSystem_GetCRC8CheckSum((uint8_t*)msg,4,CRC8_Initial);
    msg->header.cmd_id=0x0301;
    msg->header.dada_cmd_id=0x0110;
    msg->header.sender_id=RefereeSystem_RobotID;
    msg->header.receiver_id=RefereeSystem_RobotID+256;
    msg->option.str_length=strlen(msg->option.string);
    msg->CRC16=RefereeSystem_GetCRC16CheckSum((uint8_t *)msg,58,CRC16_Initial);
}
