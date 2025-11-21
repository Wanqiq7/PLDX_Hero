#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

/* CAN视觉通信配置 */
#define VISION_CAN_CTRL_ID 0xff  // 云台控制指令CAN ID（默认值）

/**
 * @brief 视觉模块初始化配置结构体
 * @note 参考DJI电机模块的设计模式，提供灵活的配置接口
 *       支持双向通信：接收视觉控制指令(rx_id)和发送IMU四元数(tx_id)
 */
typedef struct {
    CAN_HandleTypeDef *can_handle;  // CAN总线句柄 (hcan1/hcan2)
    uint16_t rx_id;                 // 视觉控制帧接收ID (通常为0xff)
    uint16_t tx_id;                 // 四元数发送ID (推荐0x100)
    uint16_t reload_count;          // 离线检测超时计数 (默认10 = 100ms @ 10ms daemon周期)
} Vision_Init_Config_s;

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch;
	float yaw;
} Vision_Recv_s;

/* CAN视觉数据结构（对应0xff帧） */
typedef struct
{
	uint8_t control;          // 控制标志
	uint8_t shoot;            // 射击标志
	float yaw;                // yaw角度
	float pitch;              // pitch角度
	float horizon_distance;   // 水平距离
} Vision_CAN_Ctrl_s;

/* 保留枚举定义（robot_def.h中使用） */
typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;
#pragma pack()

/**
 * @brief 初始化视觉CAN通信模块
 * @note 参考DJI电机模块的初始化模式，使用配置结构体
 *
 * @param config 视觉模块初始化配置结构体指针
 * @return Vision_Recv_s* 返回视觉接收数据指针
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *config);

/**
 * @brief 查询视觉模块在线状态
 *
 * @return uint8_t 1-在线，0-离线或未初始化
 */
uint8_t VisionIsOnline(void);

/**
 * @brief 发送四元数数据到视觉小电脑
 * @note 使用int16压缩编码，单帧8字节传输，大端序
 *       数据格式: [qx_high, qx_low, qy_high, qy_low, qz_high, qz_low, qw_high, qw_low]
 *       解码公式: float = int16_val / 32767.0f
 *
 * @param x 四元数x分量 (范围[-1, 1])
 * @param y 四元数y分量 (范围[-1, 1])
 * @param z 四元数z分量 (范围[-1, 1])
 * @param w 四元数w分量 (范围[-1, 1])
 */
void VisionSendQuaternion(float x, float y, float z, float w);

#endif // !MASTER_PROCESS_H