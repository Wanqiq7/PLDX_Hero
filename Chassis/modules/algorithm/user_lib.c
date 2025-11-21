/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "arm_math.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"
#include "stm32f407xx.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 闁兼眹鍎寸换宥囩不濡ゅ應鍋撻悢宄邦唺濞戞挸绉撮¨锟�,闁告瑯鍨禍鎺撴媴鐠恒劍鏆弎31濞寸媴绲惧ù娌�32,濞达絽妫欏Σ鍝ュ垝閹冾唺濞村吋宀稿閿嬫媴閿燂拷
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
void MatInit(mat *m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max)                                               \
  do {                                                                         \
    if ((val) <= (min)) {                                                      \
      (val) = (min);                                                           \
    } else if ((val) >= (max)) {                                               \
      (val) = (max);                                                           \
    }                                                                          \
  } while (0)

#define ANGLE_LIMIT_360(val, angle)                                            \
  do {                                                                         \
    (val) = (angle) - (int)(angle);                                            \
    (val) += (int)(angle) % 360;                                               \
  } while (0)

#define ANGLE_LIMIT_360_TO_180(val)                                            \
  do {                                                                         \
    if ((val) > 180)                                                           \
      (val) -= 360;                                                            \
  } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief
 * 閺夆晜鏌ㄥú鏍ㄧ▔閳ь剟宕稿Δ鈧崗閬嶅礄閳ь剟鎯冮崟顐㈡暥闁跨噦鎷�?,濞戞挸绉风换鍐╃瀹ュ洤濮ч梻鍥ｅ亾閻熸洑绀佸閬嶅礆閹増绁柨鐕傛嫹?濞戞捁妗ㄧ紞姗€妫侀埀顒傛啺娴ｇ儤鐣辩紒顐ヮ嚙閻庯拷
 *
 * @param size 闁告帒妫濋崢銈嗗緞瑜嶉惃锟�
 * @return void*
 */
void *zmalloc(size_t size);

// 闁跨喐鏋婚幏鐑芥晸閸旑偄灏呴幏鐑芥晸閺傘倖瀚�
float Sqrt(float x);
// 闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
float abs_limit(float num, float Limit);
// 闁跨喎褰ㄩ弬顓炲殩閹风兘鏁撻弬銈嗗娴ｏ拷
float sign(float value);
// 闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
float float_deadband(float Value, float minValue, float maxValue);
// 闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
float float_constrain(float Value, float minValue, float maxValue);
// 闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 瀵邦亪鏁撻弬銈嗗闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
float loop_float_constrain(float Input, float minValue, float maxValue);
// 闁跨喕顫楃拋瑙勫 闁跨喐鏋婚幏鐑芥晸閻偄鍤栭幏锟� 180 ~
// -180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

uint16_t LowPassFilter(uint16_t Out, float K);

int16_t sign_with_deadband(float value, float deadband);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
// 寰幆闄愬箙鍑芥暟
float loop_float_constrain(float Input, float minValue, float maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    float len = maxValue - minValue;
    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    float len = maxValue - minValue;
    while (Input < minValue) {
      Input += len;
    }
  }
  return Input;
}

// 瀵冨閺嶇厧绱￠崠鏍﹁礋-PI~PI

// 鐟欐帒瀹抽弽鐓庣础閸栨牔璐�-180~180
float theta_format(float Ang) {
  return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw) {
  static int integer;
  static float decimal;
  integer = (int)raw;
  decimal = raw - integer;
  if (decimal > 0.5f)
    integer++;
  return integer;
}

// 娑撳娣崥鎴﹀櫤瑜版帊绔撮崠锟�
float *Norm3d(float *v) {
  float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
  return v;
}

// 鐠侊紕鐣诲Ο锟犳毐
float NormOf3d(float *v) {
  return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 娑撳娣崥鎴﹀櫤閸欏绠籿1 x v2
void Cross3d(float *v1, float *v2, float *res) {
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 娑撳娣崥鎴﹀櫤閻愰€涚
float Dot3d(float *v1, float *v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 閸у洤鈧吋鎶ゅ▔锟�,閸掔娀娅巄uffer娑擃厾娈戦張鈧崥搴濈娑擃亜鍘撶槐锟�,婵夘偄鍙嗛弬鎵畱閸忓啰绀岄獮鑸电湴楠炲啿娼庨崐锟�
float AverageFilter(float new_data, float *buf, uint8_t len) {
  float sum = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    buf[i] = buf[i + 1];
    sum += buf[i];
  }
  buf[len - 1] = new_data;
  sum += new_data;
  return sum / len;
}

void MatInit(mat *m, uint8_t row, uint8_t col) {
  m->numCols = col;
  m->numRows = row;
  m->pData = (float *)zmalloc(row * col * sizeof(float));
}

uint16_t LowPassFilter(uint16_t Out, float K) {
  // 鐎规矮绠熸稉濠佺濞嗭紕娈戞潏鎾冲毉
  static uint16_t Last_Out = 0;
  // 娴ｅ酣鈧碍鎶ゅ▔锟�
  Out = K * Out + (1 - K) * Last_Out;
  // 閺囧瓨鏌婃稉濠佺濞嗭紕娈戞潏鎾冲毉閸婏拷
  Last_Out = Out;
  return Out;
}

/**
 * @brief 绗﹀彿鍑芥暟 (甯︽鍖�)
 * @param value 杈撳叆鍊�
 * @param deadband 姝诲尯鑼冨洿
 * @return -1, 0, or 1
 * @note 褰撹緭鍏ュ€煎湪[-deadband, deadband]鑼冨洿鍐呮椂, 杩斿洖0;
 * 鍚﹀垯杩斿洖杈撳叆鍊肩殑绗﹀彿锛岀敤浜庡簳鐩橀樆鍔涜ˉ鍋�
 */
int16_t sign_with_deadband(float value, float deadband) {
  if (value > deadband)
    return 1;
  else if (value < -deadband)
    return -1;
  else
    return 0;
}

/**
 * @brief 娴偣鏁颁綆閫氭护娉㈠櫒
 * @param new_value 鏂扮殑杈撳叆鍊�
 * @param K 婊ゆ尝绯绘暟 (0.0~1.0), 瓒婂皬婊ゆ尝鏁堟灉瓒婂己
 * @param last_value 涓婁竴娆＄殑杈撳嚭鍊�
 * @return 婊ゆ尝鍚庣殑杈撳嚭鍊�
 */
float LowPassFilter_Float(float new_value, float K, float *last_value) {
  float out = K * new_value + (1.0f - K) * (*last_value);
  *last_value = out;
  return out;
}

/**
 * @brief  鏂滃潯杞鍒掑櫒鏍稿績鍑芥暟 (鍗曚綅:
 * CAN鎸囦护鍊�),鐢ㄤ簬閿洏鎺у埗
 * @param  target      鐩爣鎸囦护鍊�
 * @param  current     褰撳墠瑙勫垝鍣ㄧ殑杈撳嚭鎸囦护鍊� (涓婁竴鍛ㄦ湡鐨勭粨鏋�)
 * @param  accel       鍔犻€熷害 (鎸囦护鍊�/绉�)
 * @param  decel       鍑忛€熷害 (鎸囦护鍊�/绉�)
 * @param  brake_decel 鍙嶅悜鍒跺姩鍑忛€熷害 (鎸囦护鍊�/绉�)
 * @param  dt          鎺у埗鍛ㄦ湡 (s)
 * @retval             褰撳墠鍛ㄦ湡瑙勫垝濂界殑杈撳嚭鎸囦护鍊�
 */
static float SoftRamp_CMD(float target, float current, float accel, float decel,
                          float brake_decel, float dt) {
  float ramp_out = current;
  float error = target - current;

  // 鍒ゆ柇鏄姞閫熴€佸噺閫熻繕鏄弽鍚戝埗鍔�
  if (target * current >= 0) // 鐩爣鍊煎拰褰撳墠鍊煎悓鍙�
  {
    if (fabsf(target) > fabsf(current)) // 鍔犻€�
    {
      ramp_out += accel * dt * sign(error);
    } else // 鍑忛€�
    {
      ramp_out += decel * dt * sign(error);
    }
  } else // 鐩爣鍊煎拰褰撳墠鍊煎紓鍙� (鍙嶅悜鍒跺姩)
  {
    ramp_out += brake_decel * dt * sign(error);
  }

  // 闄愬埗瑙勫垝鍊间笉鑳借秴杩囩洰鏍囧€�
  if (sign(error) > 0) {
    ramp_out = float_constrain(ramp_out, current, target);
  } else {
    ramp_out = float_constrain(ramp_out, target, current);
  }

  // 鏈€缁堝啀瀵硅緭鍑鸿繘琛屼竴娆℃€昏寖鍥寸殑闄愬埗
  return float_constrain(ramp_out, -16384.0f, 16384.0f);
}

float Sqrt(float x) {
  if (x <= 0.0f)
    return 0.0f;
  float out = 0.0f;
  arm_sqrt_f32(x, &out);
  return out;
}

float float_constrain(float Value, float minValue, float maxValue) {
  if (Value < minValue)
    return minValue;
  if (Value > maxValue)
    return maxValue;
  return Value;
}