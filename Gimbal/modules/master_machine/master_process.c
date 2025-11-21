/**
 * @file master_process.c
 * @brief 视觉CAN通信模块
 * @note 实现与视觉小电脑的双向CAN通信：
 *       - 接收视觉控制指令（0xff帧）
 *       - 发送IMU四元数数据
 */

#include "master_process.h"
#include "bsp_can.h"
#include "bsp_log.h"
#include "daemon.h"
#include "stdlib.h"
#include "string.h"

/* 模块私有变量 */
static CANInstance *vision_can_ins = NULL;   // CAN实例
static DaemonInstance *vision_daemon = NULL; // 守护进程实例
static Vision_Recv_s vision_recv_data;       // 视觉接收数据
static uint16_t vision_tx_id = 0x100;        // 四元数发送ID（默认值）
static float vision_horizon_distance = 0.0f; // 目标水平距离（暂存，供后续扩展）

/* 前向声明 */
static void VisionCANRxCallback(CANInstance *can_ins);
static void VisionOfflineCallback(void *owner);

/**
 * @brief 视觉CAN接收回调函数
 * @param can_ins CAN实例指针
 * @note 解析0xff帧的视觉控制数据
 */
static void VisionCANRxCallback(CANInstance *can_ins) {
  // 解析接收到的数据（8字节）
  // 数据格式参考 Vision_CAN_Ctrl_s 和 can2.c:
  // [0]: control - 控制标志
  // [1]: shoot - 射击标志
  // [2-3]: yaw (int16, 大端序, 精度0.0001°)
  // [4-5]: pitch (int16, 大端序, 精度0.0001°)
  // [6-7]: horizon_distance (int16, 大端序, 精度0.0001m)

  uint8_t *rx_data = can_ins->rx_buff;

  // 解析控制标志
  uint8_t control = rx_data[0];
  uint8_t shoot = rx_data[1];

  // 解析角度数据（int16大端序，高字节在前）
  // 精度：0.0001° (与can2.c保持一致，使用/1e4转换)
  int16_t yaw_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
  int16_t pitch_raw = (int16_t)((rx_data[4] << 8) | rx_data[5]);

  // 转换为浮点角度（精度0.0001°，即/1e4）
  vision_recv_data.yaw = (float)yaw_raw / 1e4f;
  vision_recv_data.pitch = (float)pitch_raw / 1e4f;

  // 解析水平距离（暂存，供后续扩展使用）
  int16_t distance_raw = (int16_t)((rx_data[6] << 8) | rx_data[7]);
  vision_horizon_distance = (float)distance_raw / 1e4f;

  // 解析目标状态
  // control字段映射：
  // bit0-1: fire_mode (NO_FIRE=0, AUTO_FIRE=1, AUTO_AIM=2)
  // bit2-3: target_state (NO_TARGET=0, TARGET_CONVERGING=1, READY_TO_FIRE=2)
  vision_recv_data.fire_mode = (Fire_Mode_e)(control & 0x03);
  vision_recv_data.target_state = (Target_State_e)((control >> 2) & 0x03);

  // shoot字段映射目标类型
  vision_recv_data.target_type = (Target_Type_e)(shoot & 0x0F);

  // 喂狗，表示视觉模块在线
  DaemonReload(vision_daemon);
}

/**
 * @brief 视觉模块离线回调
 * @param owner 模块拥有者指针（未使用）
 */
static void VisionOfflineCallback(void *owner) {
  UNUSED(owner);
  // 视觉离线时，重置接收数据
  memset(&vision_recv_data, 0, sizeof(Vision_Recv_s));
  vision_recv_data.target_state = NO_TARGET;
  vision_recv_data.fire_mode = NO_FIRE;
  LOGWARNING("[vision] Vision module offline!");
}

/**
 * @brief 初始化视觉CAN通信模块
 * @param config 视觉模块初始化配置结构体指针
 * @return Vision_Recv_s* 返回视觉接收数据指针
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *config) {
  // 参数检查
  if (config == NULL || config->can_handle == NULL) {
    LOGERROR("[vision] VisionInit: Invalid config!");
    return NULL;
  }

  // 保存发送ID
  vision_tx_id = config->tx_id;

  // 初始化接收数据结构
  memset(&vision_recv_data, 0, sizeof(Vision_Recv_s));

  // 注册CAN实例
  CAN_Init_Config_s can_config = {
      .can_handle = config->can_handle,
      .tx_id = config->tx_id, // 四元数发送ID
      .rx_id = config->rx_id, // 视觉控制帧接收ID
      .can_module_callback = VisionCANRxCallback,
      .id = NULL, // 本模块不需要额外的id标识
  };
  vision_can_ins = CANRegister(&can_config);

  if (vision_can_ins == NULL) {
    LOGERROR("[vision] VisionInit: CAN register failed!");
    return NULL;
  }

  // 注册守护进程（离线检测）
  Daemon_Init_Config_s daemon_config = {
      .reload_count = config->reload_count,
      .callback = VisionOfflineCallback,
      .owner_id = NULL,
  };
  vision_daemon = DaemonRegister(&daemon_config);

  if (vision_daemon == NULL) {
    LOGERROR("[vision] VisionInit: Daemon register failed!");
    return NULL;
  }

  LOGINFO("[vision] Vision CAN module initialized. RX_ID=0x%X, TX_ID=0x%X",
          config->rx_id, config->tx_id);

  return &vision_recv_data;
}

/**
 * @brief 查询视觉模块在线状态
 * @return uint8_t 1-在线，0-离线或未初始化
 */
uint8_t VisionIsOnline(void) {
  if (vision_daemon == NULL) {
    return 0;
  }
  return DaemonIsOnline(vision_daemon);
}

/**
 * @brief 发送四元数数据到视觉小电脑
 * @note 使用int16压缩编码，单帧8字节传输，大端序
 *       数据格式: [qx_high, qx_low, qy_high, qy_low, qz_high, qz_low, qw_high,
 * qw_low] 解码公式: float = int16_val / 32767.0f
 *
 * @param x 四元数x分量 (范围[-1, 1])
 * @param y 四元数y分量 (范围[-1, 1])
 * @param z 四元数z分量 (范围[-1, 1])
 * @param w 四元数w分量 (范围[-1, 1])
 */
void VisionSendQuaternion(float x, float y, float z, float w) {
  if (vision_can_ins == NULL) {
    return;
  }

  // 四元数压缩：float -> int16 (范围[-32767, 32767])
  // 编码公式：int16_val = float_val * 32767.0f
  int16_t qx = (int16_t)(x * 32767.0f);
  int16_t qy = (int16_t)(y * 32767.0f);
  int16_t qz = (int16_t)(z * 32767.0f);
  int16_t qw = (int16_t)(w * 32767.0f);

  // 填充发送缓冲区（大端序）
  // [qx_high, qx_low, qy_high, qy_low, qz_high, qz_low, qw_high, qw_low]
  vision_can_ins->tx_buff[0] = (uint8_t)(qx >> 8);   // qx high
  vision_can_ins->tx_buff[1] = (uint8_t)(qx & 0xFF); // qx low
  vision_can_ins->tx_buff[2] = (uint8_t)(qy >> 8);   // qy high
  vision_can_ins->tx_buff[3] = (uint8_t)(qy & 0xFF); // qy low
  vision_can_ins->tx_buff[4] = (uint8_t)(qz >> 8);   // qz high
  vision_can_ins->tx_buff[5] = (uint8_t)(qz & 0xFF); // qz low
  vision_can_ins->tx_buff[6] = (uint8_t)(qw >> 8);   // qw high
  vision_can_ins->tx_buff[7] = (uint8_t)(qw & 0xFF); // qw low

  // 设置DLC为8字节并发送
  CANSetDLC(vision_can_ins, 8);
  CANTransmit(vision_can_ins, 1);
}
