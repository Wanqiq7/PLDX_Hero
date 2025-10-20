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
 // 閼汇儴绻嶇粻妤呪偓鐔峰娑撳秴顧�,閸欘垯浜掓担璺ㄦ暏q31娴狅絾娴沠32,娴ｅ棙妲哥划鎯у娴兼岸妾锋担锟�
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
  * 鏉╂柨娲栨稉鈧崸妤€鍏遍崙鈧惃鍕敶閿燂拷?,娑撳秷绻冩禒宥囧姧闂団偓鐟曚礁宸遍崚鎯版祮閿燂拷?娑撹桨缍橀棁鈧憰浣烘畱缁鐎�
  *
  * @param size 閸掑棝鍘ゆ径褍鐨�
  * @return void*
  */
 void *zmalloc(size_t size);
 
 // 閿熸枻鎷烽敓鍔尅鎷烽敓鏂ゆ嫹
 float Sqrt(float x);
 // 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
 float abs_limit(float num, float Limit);
 // 閿熷彨鏂嚖鎷烽敓鏂ゆ嫹浣�
 float sign(float value);
 // 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
 float float_deadband(float Value, float minValue, float maxValue);
 // 閿熺潾鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
 float float_constrain(float Value, float minValue, float maxValue);
 // 閿熺潾鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
 int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
 // 寰敓鏂ゆ嫹閿熺潾鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
 float loop_float_constrain(float Input, float minValue, float maxValue);
 // 閿熻璁规嫹 閿熸枻鎷烽敓鐫嚖鎷� 180 ~ -180
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
 // 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}
 
 // 寮у害鏍煎紡鍖栦负-PI~PI
 
 // 瑙掑害鏍煎紡鍖栦负-180~180
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
 
 // 涓夌淮鍚戦噺褰掍竴鍖�
 float *Norm3d(float *v) {
   float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
   v[0] /= len;
   v[1] /= len;
   v[2] /= len;
   return v;
 }
 
 // 璁＄畻妯￠暱
 float NormOf3d(float *v) {
   return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
 }
 
 // 涓夌淮鍚戦噺鍙変箻v1 x v2
 void Cross3d(float *v1, float *v2, float *res) {
   res[0] = v1[1] * v2[2] - v1[2] * v2[1];
   res[1] = v1[2] * v2[0] - v1[0] * v2[2];
   res[2] = v1[0] * v2[1] - v1[1] * v2[0];
 }
 
 // 涓夌淮鍚戦噺鐐逛箻
 float Dot3d(float *v1, float *v2) {
   return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
 }
 
 // 鍧囧€兼护娉�,鍒犻櫎buffer涓殑鏈€鍚庝竴涓厓绱�,濉叆鏂扮殑鍏冪礌骞舵眰骞冲潎鍊�
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
   // 瀹氫箟涓婁竴娆＄殑杈撳嚭
   static uint16_t Last_Out = 0;
   // 浣庨€氭护娉�
   Out = K * Out + (1 - K) * Last_Out;
   // 鏇存柊涓婁竴娆＄殑杈撳嚭鍊�
   Last_Out = Out;
   return Out;
 }
 
 /**
  * @brief 符号函数 (带死区)
  * @param value 输入值
  * @param deadband 死区范围
  * @return -1, 0, or 1
  * @note 当输入值在[-deadband, deadband]范围内时, 返回0;
  * 否则返回输入值的符号，用于底盘阻力补偿
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
 * @brief 浮点数低通滤波器
 * @param new_value 新的输入值
 * @param K 滤波系数 (0.0~1.0), 越小滤波效果越强
 * @param last_value 上一次的输出值
 * @return 滤波后的输出值
 */
float LowPassFilter_Float(float new_value, float K, float *last_value) {
  float out = K * new_value + (1.0f - K) * (*last_value);
  *last_value = out;
  return out;
}
 
 /**
  * @brief  斜坡软规划器核心函数 (单位: CAN指令值),用于键盘控制
  * @param  target      目标指令值
  * @param  current     当前规划器的输出指令值 (上一周期的结果)
  * @param  accel       加速度 (指令值/秒)
  * @param  decel       减速度 (指令值/秒)
  * @param  brake_decel 反向制动减速度 (指令值/秒)
  * @param  dt          控制周期 (s)
  * @retval             当前周期规划好的输出指令值
  */
 static float SoftRamp_CMD(float target, float current, float accel, float decel,
                           float brake_decel, float dt) {
   float ramp_out = current;
   float error = target - current;
 
   // 判断是加速、减速还是反向制动
   if (target * current >= 0) // 目标值和当前值同号
   {
     if (fabsf(target) > fabsf(current)) // 加速
     {
       ramp_out += accel * dt * sign(error);
     } else // 减速
     {
       ramp_out += decel * dt * sign(error);
     }
   } else // 目标值和当前值异号 (反向制动)
   {
     ramp_out += brake_decel * dt * sign(error);
   }
 
   // 限制规划值不能超过目标值
   if (sign(error) > 0) {
     ramp_out = float_constrain(ramp_out, current, target);
   } else {
     ramp_out = float_constrain(ramp_out, target, current);
   }
 
   // 最终再对输出进行一次总范围的限制
   return float_constrain(ramp_out, -16384.0f, 16384.0f);
 }

 float Sqrt(float x)
{
  if (x <= 0.0f) return 0.0f;
  float out = 0.0f;
  arm_sqrt_f32(x, &out);
  return out;
}

float float_constrain(float Value, float minValue, float maxValue)
{
  if (Value < minValue) return minValue;
  if (Value > maxValue) return maxValue;
  return Value;
}