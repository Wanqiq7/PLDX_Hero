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
// 闂佸吋鐪归崕瀵告崲瀹ュ洨涓嶆俊銈呮噳閸嬫捇鎮㈠畡閭﹀敽婵炴垶鎸哥粔鎾ㄩ敓锟�,闂佸憡鐟崹顖涚閹烘挻濯撮悹鎭掑妽閺嗗紟31婵炲濯寸徊鎯瑰▽锟�32,婵炶揪绲藉Λ娆徫ｉ崫銉ュ灊闁诡垎鍐惧敽婵炴潙鍚嬪畝绋款瀶闁垮濯撮柨鐕傛嫹
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
 * 闁哄鏅滈弻銊ッ洪弽銊р枖闁逞屽墴瀹曠ǹ螖閳ь剟宕楅柆宥呯闁逞屽墴閹啴宕熼銏℃殽闂佽法鍣﹂幏锟�?,婵炴垶鎸哥粔椋庢崲閸愨晝顩风€广儱娲ゆ慨褔姊婚崶锝呬壕闁荤喐娲戠粈浣割啅闁秴绀嗛柟顖滃缁侇噣鏌ㄩ悤鍌涘?婵炴垶鎹佸銊х礊濮椻偓濡線鍩€椤掑倹鍟哄ù锝囧劋閻ｈ京绱掗銉殭闁诲函鎷�
 *
 * @param size 闂佸憡甯掑Λ婵嬪储閵堝棗绶炵憸宥夋儍閿燂拷
 * @return void*
 */
void *zmalloc(size_t size);

// 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柛鏃戝亜鐏忓懘骞忛悜鑺ユ櫢闁哄倶鍊栫€氾拷
float Sqrt(float x);
// 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
float abs_limit(float num, float Limit);
// 闂佽法鍠庤ぐ銊╁棘椤撶偛娈╅柟椋庡厴閺佹捇寮妶鍡楊伓濞达綇鎷�
float sign(float value);
// 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏�
float float_deadband(float Value, float minValue, float maxValue);
// 闂佽法鍠撳楣冨礄閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫€氾拷
float float_constrain(float Value, float minValue, float maxValue);
// 闂佽法鍠撳楣冨礄閵堝棗顏堕梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫€氾拷
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 鐎甸偊浜弫鎾诲棘閵堝棗顏堕梺璺ㄥ枔濞奸箖宕欓妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氾拷
float loop_float_constrain(float Input, float minValue, float maxValue);
// 闂佽法鍠曢～妤冩媼鐟欏嫬顏�
// 闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柣顐亜閸ゆ牠骞忛敓锟� 180
// ~ -180
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
// 瀵邦亞骞嗛梽鎰畽閸戣姤鏆�
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

// 鐎殿啫鍐唺闁哄秶鍘х槐锟犲礌閺嶏箒绀�-PI~PI

// 閻熸瑦甯掔€规娊寮介悡搴ｇ闁告牗鐗旂拹锟�-180~180
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

// 濞戞挸顦卞ǎ顕€宕ラ幋锕€娅ょ憸鐗堝笂缁旀挳宕犻敓锟�
float *Norm3d(float *v) {
  float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
  return v;
}

// 閻犱緤绱曢悾璇参熼敓鐘虫瘣
float NormOf3d(float *v) {
  return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 濞戞挸顦卞ǎ顕€宕ラ幋锕€娅ら柛娆忣槷缁犵笨1 x v2
void Cross3d(float *v1, float *v2, float *res) {
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 濞戞挸顦卞ǎ顕€宕ラ幋锕€娅ら柣鎰扳偓娑氼啋
float Dot3d(float *v1, float *v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 闁秆冩搐閳ь剙鍚嬮幎銈呪枖閿燂拷,闁告帞濞€濞呭穭uffer濞戞搩鍘惧▓鎴﹀嫉閳ь剟宕ユ惔婵堫伇濞戞搩浜滈崢鎾舵閿燂拷,濠靛鍋勯崣鍡涘棘閹殿喗鐣遍柛蹇撳暟缁€宀勭嵁閼哥數婀存鐐插暱濞煎酣宕愰敓锟�
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
  // 閻庤鐭粻鐔哥▔婵犱胶顏辨繛鍡磿濞堟垶娼忛幘鍐叉瘔
  static uint16_t Last_Out = 0;
  // 濞达絽閰ｉ埀顒佺閹躲倕鈻旈敓锟�
  Out = K * Out + (1 - K) * Last_Out;
  // 闁哄洤鐡ㄩ弻濠冪▔婵犱胶顏辨繛鍡磿濞堟垶娼忛幘鍐叉瘔闁稿⿵鎷�
  Last_Out = Out;
  return Out;
}

/**
 * @brief 缁楋箑褰块崙鑺ユ殶 (鐢附顒撮崠锟�)
 * @param value 鏉堟挸鍙嗛崐锟�
 * @param deadband 濮濊灏懠鍐ㄦ纯
 * @return -1, 0, or 1
 * @note 瑜版捁绶崗銉モ偓鐓庢躬[-deadband, deadband]閼煎啫娲块崘鍛,
 * 鏉╂柨娲�0;
 * 閸氾箑鍨潻鏂挎礀鏉堟挸鍙嗛崐鑲╂畱缁楋箑褰块敍宀€鏁ゆ禍搴＄俺閻╂﹢妯嗛崝娑溗夐崑锟�
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
 * @brief 濞搭喚鍋ｉ弫棰佺秵闁碍鎶ゅ▔銏犳珤
 * @param new_value 閺傛壆娈戞潏鎾冲弳閸婏拷
 * @param K 濠娿倖灏濈化缁樻殶 (0.0~1.0),
 * 鐡掑﹤鐨⿰銈嗗皾閺佸牊鐏夌搾濠傚繁
 * @param last_value 娑撳﹣绔村▎锛勬畱鏉堟挸鍤崐锟�
 * @return 濠娿倖灏濋崥搴ｆ畱鏉堟挸鍤崐锟�
 */
float LowPassFilter_Float(float new_value, float K, float *last_value) {
  float out = K * new_value + (1.0f - K) * (*last_value);
  *last_value = out;
  return out;
}

/**
 * @brief  閺傛粌娼潪顖濐潐閸掓帒娅掗弽绋跨妇閸戣姤鏆�
 * (閸楁洑缍�:
 * CAN閹稿洣鎶ら崐锟�),閻€劋绨柨顔炬磸閹貉冨煑
 * @param  target      閻╊喗鐖ｉ幐鍥︽姢閸婏拷
 * @param  current     瑜版挸澧犵憴鍕灊閸ｃ劎娈戞潏鎾冲毉閹稿洣鎶ら崐锟�
 * (娑撳﹣绔撮崨銊︽埂閻ㄥ嫮绮ㄩ弸锟�)
 * @param  accel       閸旂娀鈧喎瀹�
 * (閹稿洣鎶ら崐锟�/缁夛拷)
 * @param  decel       閸戝繘鈧喎瀹�
 * (閹稿洣鎶ら崐锟�/缁夛拷)
 * @param  brake_decel 閸欏秴鎮滈崚璺哄З閸戝繘鈧喎瀹�
 * (閹稿洣鎶ら崐锟�/缁夛拷)
 * @param  dt          閹貉冨煑閸涖劍婀� (s)
 * @retval
 * 瑜版挸澧犻崨銊︽埂鐟欏嫬鍨濇總鐣屾畱鏉堟挸鍤幐鍥︽姢閸婏拷
 */
static float SoftRamp_CMD(float target, float current, float accel, float decel,
                          float brake_decel, float dt) {
  float ramp_out = current;
  float error = target - current;

  // 閸掋倖鏌囬弰顖氬闁喆鈧礁鍣洪柅鐔荤箷閺勵垰寮介崥鎴濆煑閸旓拷
  if (target * current >= 0) // 閻╊喗鐖ｉ崐鐓庢嫲瑜版挸澧犻崐鐓庢倱閸欙拷
  {
    if (fabsf(target) > fabsf(current)) // 閸旂娀鈧拷
    {
      ramp_out += accel * dt * sign(error);
    } else // 閸戝繘鈧拷
    {
      ramp_out += decel * dt * sign(error);
    }
  } else // 閻╊喗鐖ｉ崐鐓庢嫲瑜版挸澧犻崐鐓庣磽閸欙拷 (閸欏秴鎮滈崚璺哄З)
  {
    ramp_out += brake_decel * dt * sign(error);
  }

  // 闂勬劕鍩楃憴鍕灊閸婇棿绗夐懗鍊熺Т鏉╁洨娲伴弽鍥р偓锟�
  if (sign(error) > 0) {
    ramp_out = float_constrain(ramp_out, current, target);
  } else {
    ramp_out = float_constrain(ramp_out, target, current);
  }

  // 閺堚偓缂佸牆鍟€鐎电绶崙楦跨箻鐞涘奔绔村▎鈩冣偓鏄忓瘱閸ュ娈戦梽鎰煑
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
} // 浣跨敤鍕捐偂瀹氱悊: r = sqrt(x^2 + y^2)
// WHEEL_BASE鍜孴RACK_WIDTH鍦╮obot_def.h涓凡缁忔槸m锛屾棤闇€杞崲
#define LF_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define RF_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define LB_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define RB_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y, 2.0f))

/* ==================================================== */
/* ----------------鍔涙帶绛栫暐鏍稿績鍑芥暟---------------- */
/* ==================================================== */

/**
 * @brief 楹﹁疆姝ｈ繍鍔ㄥ瑙ｇ畻 -
 * 璁＄畻鍚勮疆鐩爣瑙掗€熷害锛堢敤浜庨€熷害鍐呯幆鍙嶉锛�
 * @note  鍙傝€價obowalker鐨凨inematics_Inverse_Resolution鎬濇兂
 *        浠庡簳鐩橀€熷害(vx,vy,wz)瑙ｇ畻鍑哄悇杞殑鐩爣瑙掗€熷害
 *        杩欐槸鍔涙帶+閫熷害鍐呯幆鐨勫叧閿細鎻愪緵杞€熺洰鏍囧€肩敤浜庡姩鎬佽ˉ鍋�
 */
static void MecanumKinematicsCalculate() {
  // 楹﹁疆姝ｈ繍鍔ㄥ锛氫粠搴曠洏閫熷害瑙ｇ畻鍚勮疆绾块€熷害
  // v_wheel = v_chassis + 蠅 脳 r_vec
  // 瀵逛簬45搴﹂害杞紝涓庡姏鍒嗛厤鍏紡瀵瑰簲

  float vx = chassis_vx;
  float vy = chassis_vy;
  float wz = chassis_cmd_recv.wz;

  // 璁＄畻鍚勮疆绾块€熷害锛坢/s锛�-
  // 蹇呴』涓庡姏鍒嗛厤鍏紡鐨勭鍙蜂竴鑷�
  float wheel_linear_vel[4];
  wheel_linear_vel[0] = (-vx - vy) + wz * LF_CENTER; // 宸﹀墠 +wz
  wheel_linear_vel[1] = (-vx + vy) - wz * RF_CENTER; // 鍙冲墠 -wz
  wheel_linear_vel[2] = (vx - vy) - wz * LB_CENTER;  // 宸﹀悗 -wz
  wheel_linear_vel[3] = (vx + vy) + wz * RB_CENTER;  // 鍙冲悗 +wz

  // 杞崲涓虹數鏈鸿閫熷害锛坮ad/s锛�
  // 鍏抽敭锛氱嚎閫熷害 鈫� 杞瓙瑙掗€熷害 鈫�
  // 鐢垫満瑙掗€熷害锛堣€冭檻鍑忛€熸瘮锛�
  for (int i = 0; i < 4; i++) {
    // 杞瓙瑙掗€熷害 = 绾块€熷害 / 杞瓙鍗婂緞
    float wheel_omega = wheel_linear_vel[i] / RADIUS_WHEEL;
    // 鐢垫満瑙掗€熷害 = 杞瓙瑙掗€熷害 脳 鍑忛€熸瘮
    target_wheel_omega[i] = wheel_omega * REDUCTION_RATIO_WHEEL;
  }
}

/**
 * @brief 浼扮畻搴曠洏褰撳墠閫熷害锛堜紭鍖栫増锛�
 * @detail 浣跨敤涓€闃朵綆閫氭护娉㈠钩婊戦€熷害浼扮畻
 *
 * 绠楁硶娴佺▼锛�
 * 1. 璇诲彇鍥涗釜鐢垫満瑙掗€熷害 鈫�
 * 杞崲涓虹嚎閫熷害
 * 2. 搴旂敤楹﹁疆閫嗚繍鍔ㄥ鍏紡璁＄畻搴曠洏閫熷害
 * 3. 涓€闃朵綆閫氭护娉㈡秷闄ょ紪鐮佸櫒鍣０
 *
 * 鍏紡璇存槑锛�
 *   vx = (-vlf - vrf + vlb + vrb) / 4
 *   vy = (-vlf + vrf - vlb + vrb) / 4
 *   wz = (vlf - vrf - vlb + vrb) / (4脳r)
 *   鍏朵腑 r = sqrt( (L/2)虏 + (W/2)虏 )
 */
static void EstimateChassisVelocity(void) {
  /* ===== 绗竴姝ワ細鐢垫満瑙掗€熷害 鈫� 杞瓙绾块€熷害 =====
   */
  // 杞崲鍏紡锛氱嚎閫熷害 = 瑙掗€熷害(rad/s) 脳 鍗婂緞
  // 娉ㄦ剰锛氬弽杞數鏈洪渶瑕佸彇鍙嶄互鍖归厤鍧愭爣绯�
  const float speed_to_linear =
      DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
  float lf_linear_vel = -motor_lf->measure.speed_aps * speed_to_linear;
  float rf_linear_vel = motor_rf->measure.speed_aps * speed_to_linear;
  float lb_linear_vel = motor_lb->measure.speed_aps * speed_to_linear;
  float rb_linear_vel = -motor_rb->measure.speed_aps * speed_to_linear;

  /* ===== 绗簩姝ワ細楹﹁疆閫嗚繍鍔ㄥ瑙ｇ畻 ===== */
  // 璁＄畻鏃嬭浆鍗婂緞锛氫粠搴曠洏涓績鍒拌疆瀛愮殑璺濈锛堥璁＄畻閬垮厤閲嶅璁＄畻锛�
  static const float rotation_radius = sqrtf(
      HALF_WHEEL_BASE * HALF_WHEEL_BASE + HALF_TRACK_WIDTH * HALF_TRACK_WIDTH);

  // 搴旂敤閫嗚繍鍔ㄥ鍏紡璁＄畻鐬椂閫熷害
  const float inv_4 = 0.25f; // 棰勮绠�1/4锛岄伩鍏嶉櫎娉�
  float instant_vx =
      (-lf_linear_vel - rf_linear_vel + lb_linear_vel + rb_linear_vel) * inv_4;
  float instant_vy =
      (-lf_linear_vel + rf_linear_vel - lb_linear_vel + rb_linear_vel) * inv_4;
  float instant_wz =
      (lf_linear_vel - rf_linear_vel - lb_linear_vel + rb_linear_vel) /
      (4.0f * rotation_radius);

  /* ===== 绗笁姝ワ細浣庨€氭护娉㈠钩婊戝櫔澹� ===== */
  // 浣跨敤涓€闃朵綆閫氭护娉細y[n] = 伪脳x[n] +
  // (1-伪)脳y[n-1] 伪 = 0.85
  // 瀵瑰簲鎴棰戠巼绾�30.9Hz锛�200Hz閲囨牱锛�
  static float filtered_vx = 0.0f; // 闈欐€佸彉閲忛伩鍏嶉噸澶嶅垵濮嬪寲
  static float filtered_vy = 0.0f;
  static float filtered_wz = 0.0f;

  chassis_estimated_vx = LowPassFilter_Float(
      instant_vx, chassis_config.kinematics.velocity_lpf_alpha, &filtered_vx);
  chassis_estimated_vy = LowPassFilter_Float(
      instant_vy, chassis_config.kinematics.velocity_lpf_alpha, &filtered_vy);
  chassis_estimated_wz = LowPassFilter_Float(
      instant_wz, chassis_config.kinematics.velocity_lpf_alpha, &filtered_wz);
}

/**
 * @brief 璁＄畻鐩爣鎵煩鍓嶉琛ュ伩
 * @param target_wz 鐩爣瑙掗€熷害 (rad/s)
 * @return 鍓嶉鎵煩 (N路m)
 * @detail
 * 鍓嶉鍘熺悊锛氶浼扮淮鎸佽閫熷害鎵€闇€鐨勫熀纭€鎵煩
 */
static inline float CalculateTorqueFeedforward(float target_wz) {
  return chassis_config.force.torque_feedforward_coeff * target_wz;
}

/**
 * @brief 閫熷害闂幆鎺у埗 鈫�
 * 鍔�/鎵煩杈撳嚭锛堥噸鏋勭増锛�
 *
 * 鎺у埗閾捐矾锛�
 * 鐩爣閫熷害 鈫� PID鎺у埗鍣� 鈫� 闇€姹傚姏/鎵煩 鈫� 鍓嶉琛ュ伩 鈫�
 * 鏈€缁堣緭鍑�
 */
static void VelocityToForceControl(void) {
  /* ===== 姝ラ1锛氭洿鏂伴€熷害浼扮畻 ===== */
  EstimateChassisVelocity();

  /* ===== 姝ラ2锛氳幏鍙栫洰鏍囬€熷害 ===== */
  float target_vx = chassis_vx;
  float target_vy = chassis_vy;
  float target_wz = chassis_cmd_recv.wz; // rad/s

  /* ===== 姝ラ3锛歅ID閫熷害鎺у埗 鈫� 鍔�/鎵煩 ===== */
  // PID杈撳嚭锛氭牴鎹€熷害璇樊璁＄畻鎵€闇€鐨勫悎鍔�(N)鍜屾壄鐭�(N路m)
  force_x = PIDCalculate(&chassis_force_x_pid, chassis_estimated_vx, target_vx);
  force_y = PIDCalculate(&chassis_force_y_pid, chassis_estimated_vy, target_vy);
  float torque_feedback =
      PIDCalculate(&chassis_torque_pid, chassis_estimated_wz, target_wz);

  /* ===== 姝ラ4锛氬墠棣堣ˉ鍋� ===== */
  // 鍓嶉鍑忓皯PID璐熸媴锛屾彁楂樺搷搴旈€熷害
  float torque_feedforward = CalculateTorqueFeedforward(target_wz);

  /* ===== 姝ラ5锛氬悎鎴愭渶缁堣緭鍑� ===== */
  torque_z = torque_feedback + torque_feedforward;

  /* ===== 姝ラ6锛氶檺骞呬繚鎶� ===== */
  force_x = float_constrain(force_x, -MAX_CONTROL_FORCE, MAX_CONTROL_FORCE);
  force_y = float_constrain(force_y, -MAX_CONTROL_FORCE, MAX_CONTROL_FORCE);
  torque_z = float_constrain(torque_z, -MAX_CONTROL_TORQUE, MAX_CONTROL_TORQUE);
}
/**
 * @brief
 * 鍔涚殑鍔ㄥ姏瀛﹂€嗚В绠楋紙鍔涙帶鏍稿績鐜妭2锛�
 * @note
 * 灏嗗簳鐩樺悎鍔涘垎閰嶅埌鍚勪釜杞瓙锛屼娇鐢ㄧ嫭绔嬭疆璺濆姏鑷傦紙淇鎸崱闂锛�
 *       淇锛氫娇鐢ㄥ悇杞嫭绔嬪姏鑷傝€岄潪缁熶竴rotation_radius锛屼笌姝ｈВ绠椾繚鎸佷竴鑷�
 */
static void ForceDynamicsInverseResolution() {
  // 楹﹁疆鍔涘垎閰嶏紙鍙傝€價obowalker鎬濇兂锛屼慨姝ｄ负鐙珛杞窛锛�
  // 瀵逛簬楹﹁疆锛屾瘡涓疆瀛愬彈鍒扮殑鍔� = 骞崇Щ鍔涘垎閲� +
  // 鏃嬭浆鍔涚煩鍒嗛噺
  // 淇锛氬繀椤讳娇鐢ㄤ笌姝ｈВ绠椾竴鑷寸殑鐙珛杞窛鍔涜噦锛岄伩鍏嶅弬鏁颁笉鍖归厤瀵艰嚧鎸崱

  // 鍔涘垎閰嶇煩闃碉紙楹﹁疆45搴﹁緤瀛愰厤缃級-
  // 淇锛氫娇鐢ㄧ嫭绔嬭疆璺濆姏鑷� wheel_force[i] = f_x *
  // cos(angle) + f_y * sin(angle) + torque / r_individual
  // 鍏抽敭淇锛氱鍙峰拰鍔涜噦蹇呴』涓庢瑙ｇ畻涓殑wz绗﹀彿涓€鑷达紒
  //
  // 姝ｈВ绠楃鍙峰弬鑰冿紙333-337琛岋級锛�
  // vt_lf = ... + wz * LF_CENTER   鈫� +wz锛屼娇鐢↙F_CENTER鍔涜噦
  // vt_rf = ... - wz * RF_CENTER   鈫� -wz锛屼娇鐢≧F_CENTER鍔涜噦
  // vt_lb = ... - wz * LB_CENTER   鈫� -wz锛屼娇鐢↙B_CENTER鍔涜噦
  // vt_rb = ... + wz * RB_CENTER   鈫� +wz锛屼娇鐢≧B_CENTER鍔涜噦

  // 宸﹀墠杞� (45搴﹂厤缃�) 鈫�
  // +wz锛屼娇鐢↙F_CENTER鐙珛鍔涜噦
  wheel_force[0] = (-force_x - force_y) / 4.0f + torque_z / (4.0f * LF_CENTER);

  // 鍙冲墠杞� 鈫� -wz锛屼娇鐢≧F_CENTER鐙珛鍔涜噦
  wheel_force[1] = (-force_x + force_y) / 4.0f - torque_z / (4.0f * RF_CENTER);

  // 宸﹀悗杞� 鈫� -wz锛屼娇鐢↙B_CENTER鐙珛鍔涜噦
  wheel_force[2] = (force_x - force_y) / 4.0f - torque_z / (4.0f * LB_CENTER);

  // 鍙冲悗杞� 鈫� +wz锛屼娇鐢≧B_CENTER鐙珛鍔涜噦
  wheel_force[3] = (force_x + force_y) / 4.0f + torque_z / (4.0f * RB_CENTER);
}

/**
 * @brief 璁＄畻閫熷害鍙嶉琛ュ伩鐢垫祦
 * @param target_omega 鐩爣瑙掗€熷害 (rad/s)
 * @param actual_omega 瀹為檯瑙掗€熷害 (rad/s)
 * @param motor_index 鐢垫満绱㈠紩
 * @return 琛ュ伩鐢垫祦 (A)
 */
static float CalculateSpeedFeedback(float target_omega, float actual_omega,
                                    uint8_t motor_index) {
  // 璁＄畻瑙掗€熷害璇樊
  float omega_error = target_omega - actual_omega;

  // 浜岀骇婊ゆ尝鍑忓皯鍣０鏀惧ぇ
  static float filtered_omega_error[4] = {
      0.0f}; // 姣忎釜杞瓙鐙珛婊ゆ尝鐘舵€�
  omega_error = LowPassFilter_Float(omega_error,
                                    chassis_config.force.omega_error_lpf_alpha,
                                    &filtered_omega_error[motor_index]);

  // 杩囬浂淇濇姢锛氫綆閫熸椂鍑忓皯琛ュ伩閬垮厤闇囪崱
  // 淇锛氭敼涓�"鎴�"閫昏緫锛屽彧瑕佷换涓€涓帴杩戦浂灏变笉琛ュ伩锛堥槻姝㈡护娉㈠櫒婊炲悗瀵艰嚧鐨勬柟鍚戝彉鍖栬鍒わ級
  if (fabsf(target_omega) < chassis_config.force.omega_threshold ||
      fabsf(actual_omega) < chassis_config.force.omega_threshold) {
    return 0.0f;
  }

  // 鏂瑰悜鍙樺寲鏃朵娇鐢ㄥ急琛ュ伩锛堟敞鎰忥細姝ゆ椂宸茬粡鎺掗櫎浜嗕綆閫熷満鏅級
  if (target_omega * actual_omega < 0.0f) {
    return 0.05f * chassis_config.force.wheel_speed_feedback_coeff *
           omega_error;
  }

  // 姝ｅ父閫熷害鍙嶉
  return chassis_config.force.wheel_speed_feedback_coeff * omega_error;
}

/**
 * @brief 璁＄畻鎽╂摝琛ュ伩鐢垫祦
 * @param target_omega 鐩爣瑙掗€熷害 (rad/s)
 * @return 鎽╂摝琛ュ伩鐢垫祦 (A)
 * @detail 浣跨敤tanh鍑芥暟瀹炵幇骞虫粦杩囨浮锛岄伩鍏嶉樁璺冧笉杩炵画鎬�
 */
static inline float CalculateFrictionCompensation(float target_omega) {
  // tanh鍑芥暟鍦�3鑼冨洿鍐呰繎浼肩嚎鎬э紝瓒呭嚭鍒欓ケ鍜�
  float normalized_speed =
      target_omega / chassis_config.force.friction_threshold_omega;
  float smooth_factor = tanhf(normalized_speed);

  // 鍔ㄦ懇鎿� + 骞虫粦鐨勯潤鎽╂摝琛ュ伩
  return FRICTION_DYNAMIC_CURRENT +
         smooth_factor * (FRICTION_STATIC_CURRENT - FRICTION_DYNAMIC_CURRENT);
}

/**
 * @brief 鍔涒啋鐢垫祦杞崲锛堥噸鏋勭増锛�
 *
 * 澶勭悊娴佺▼锛�
 * 1. 鍩虹鍔涒啋鐢垫祦杞崲
 * 2. 閫熷害鍙嶉琛ュ伩
 * 3. 鎽╂摝琛ュ伩
 * 4. 闄愬箙淇濇姢
 */
static void ForceToCurrentConversion(void) {
  // 鐢垫満鏁扮粍锛堜笉鑳戒娇鐢╯tatic
  // const锛屽洜涓簃otor_lf绛夋槸杩愯鏃跺垵濮嬪寲鐨勶級
  DJIMotorInstance *motors[4] = {motor_lf, motor_rf, motor_lb, motor_rb};

  // 棰勮绠楄浆鎹㈢郴鏁帮紝閬垮厤閲嶅璁＄畻
  static const float force_to_current = RADIUS_WHEEL / M3508_TORQUE_CONSTANT;

  for (int i = 0; i < 4; i++) {
    /* ===== 姝ラ1锛氬熀纭€鍔涒啋鐢垫祦杞崲 ===== */
    // 鍏紡锛氱數娴� = 鍔� 脳 鍗婂緞 / 杞煩甯告暟
    float base_current = wheel_force[i] * force_to_current;

    /* ===== 姝ラ2锛氳幏鍙栧疄闄呰閫熷害 ===== */
    float actual_omega = (i == 0 || i == 3)
                             ? -motors[i]->measure.speed_aps * DEGREE_2_RAD
                             : motors[i]->measure.speed_aps * DEGREE_2_RAD;

    /* ===== 姝ラ3锛氶€熷害鍙嶉琛ュ伩 ===== */
    float speed_feedback =
        CalculateSpeedFeedback(target_wheel_omega[i], actual_omega, i);

    /* ===== 姝ラ4锛氭懇鎿﹁ˉ鍋� ===== */
    float friction_comp = CalculateFrictionCompensation(target_wheel_omega[i]);

    /* ===== 姝ラ5锛氬悎鎴愭€荤數娴� ===== */
    wheel_current[i] = base_current + speed_feedback + friction_comp;

    /* ===== 姝ラ6锛氶檺骞呬繚鎶� ===== */
    wheel_current[i] = float_constrain(wheel_current[i], -MAX_WHEEL_CURRENT,
                                       MAX_WHEEL_CURRENT);
  }
}

/* 鏈哄櫒浜哄簳鐩樻帶鍒舵牳蹇冧换鍔� */
void ChassisTask() {
#if ENABLE_CHASSIS_SYSID
  // 绯荤粺杈ㄨ瘑瀹忓紑鍏宠Е鍙戯紙ENABLE_CHASSIS_SYSID=1鏃舵湁鏁堬級
  ChassisSystemIDSwitch();
#endif

  // 妫€鏌ョ郴缁熻鲸璇嗘槸鍚︽縺娲�
  if (Chassis_SysIDIsActive()) {
    // 绯荤粺杈ㄨ瘑婵€娲绘椂锛屽畬鍏ㄧ敱杈ㄨ瘑浠诲姟鎺у埗鐢垫満
    // 搴曠洏鎺у埗浠诲姟涓嶅共棰勶紝鐩存帴杩斿洖
    return;
  }

  // 鍚庣画澧炲姞娌℃敹鍒版秷鎭殑澶勭悊(鍙屾澘鐨勬儏鍐�)
  // 鑾峰彇鏂扮殑鎺у埗淇℃伅
#ifdef ONE_BOARD
  SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
  chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

  // === 搴旂敤閬ユ帶鍣ㄩ€熷害澧炵泭 ===
  // 浠嶨imbal鎺ユ敹鍒扮殑vx/vy鏄綊涓€鍖栧€�(-1.0~1.0)
  // 闇€瑕佷箻浠ュ鐩婅浆鎹负瀹為檯閫熷害(m/s)
  chassis_cmd_recv.vx *= chassis_config.rc.max_linear_speed;
  chassis_cmd_recv.vy *= chassis_config.rc.max_linear_speed;
  // 娉ㄦ剰:
  // wz(瑙掗€熷害)鐢卞簳鐩樻牴鎹ā寮忚嚜鍔ㄨ瀹氾紝涓嶉渶瑕佸湪杩欓噷澶勭悊

  if (chassis_cmd_recv.chassis_mode ==
      CHASSIS_ZERO_FORCE) { // 濡傛灉鍑虹幇閲嶈妯″潡绂荤嚎鎴栭仴鎺у櫒璁剧疆涓烘€ュ仠,璁╃數鏈哄仠姝�
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
  } else { // 姝ｅ父宸ヤ綔
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);
  }

  // 鏍规嵁鎺у埗妯″紡璁惧畾鏃嬭浆閫熷害
  switch (chassis_cmd_recv.chassis_mode) {
  case CHASSIS_NO_FOLLOW: // 搴曠洏涓嶆棆杞�,浣嗙淮鎸佸叏鍚戞満鍔�,涓€鑸敤浜庤皟鏁翠簯鍙板Э鎬�
    chassis_cmd_recv.wz = 0;
    last_follow_wz = 0; // 閲嶇疆婊ゆ尝鐘舵€�
    break;

  case CHASSIS_FOLLOW_GIMBAL_YAW: { // 璺熼殢浜戝彴,缁熶竴鍔涙帶閾捐矾
    // PID杈撳嚭鐩爣瑙掗€熷害锛坮ad/s锛�
    // 缁熶竴杩涘叆鍔涙帶PID杞崲涓烘壄鐭�
    // 杈撳叆锛氳搴﹁宸紙搴︼級
    // 杈撳嚭锛氱洰鏍囪閫熷害锛坮ad/s锛�
    float follow_angular_vel = -PIDCalculate(
        &chassis_follow_pid, chassis_cmd_recv.near_center_error, 0.0f);

    // 涓€闃舵护娉㈠櫒骞虫粦锛堜慨澶嶏細浣跨敤鍏ㄥ眬闈欐€佸彉閲忥紝涓嶅啀閲嶅瀹氫箟锛�
    chassis_cmd_recv.wz = LowPassFilter_Float(
        follow_angular_vel, chassis_config.kinematics.follow_lpf_alpha,
        &last_follow_wz);
    // wz缁熶竴涓鸿閫熷害锛坮ad/s锛夛紝鍚庣画閫氳繃chassis_torque_pid杞崲涓烘壄鐭�
    break;
  }

  case CHASSIS_ROTATE: // 鑷棆,鍚屾椂淇濇寔鍏ㄥ悜鏈哄姩;褰撳墠wz缁存寔瀹氬€�,鍚庣画澧炲姞涓嶈鍒欑殑鍙橀€熺瓥鐣�
    chassis_cmd_recv.wz = chassis_config.kinematics.rotate_speed; // rad/s
    last_follow_wz =
        chassis_cmd_recv.wz; // 淇濆瓨褰撳墠wz,渚夸簬鍒囨崲鍚庡钩婊戣繃娓�
    break;

  default:
    break;
  }

  // 鏍规嵁浜戝彴鍜屽簳鐩樼殑瑙掑害offset灏嗘帶鍒堕噺鏄犲皠鍒板簳鐩樺潗鏍囩郴涓�
  // 搴曠洏閫嗘椂閽堟棆杞负瑙掑害姝ｆ柟鍚�;浜戝彴鍛戒护鐨勬柟鍚戜互浜戝彴鎸囧悜鐨勬柟鍚戜负x,閲囩敤鍙虫墜绯�(x鎸囧悜姝ｅ寳鏃秠鍦ㄦ涓�)
  static float sin_theta, cos_theta;
  cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  chassis_vx =
      chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
  chassis_vy =
      chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

  /* ================== 鍔涙帶绛栫暐鎺у埗娴佺▼ ================== */
  // 鍙傝€價obowalker鐨勫姏鎺ф€濇兂锛屽疄鐜颁粠閫熷害鍒扮數娴佺殑瀹屾暣閾捐矾

  // 0.
  // 姝ｈ繍鍔ㄥ瑙ｇ畻锛氳绠楀悇杞洰鏍囪閫熷害锛堢敤浜庨€熷害鍐呯幆锛�
  //    robowalker鐨勫叧閿細鎻愪緵杞€熺洰鏍囧€肩敤浜庡姩鎬佽ˉ鍋�
  MecanumKinematicsCalculate();

  // 1. 閫熷害闂幆 鈫� 鍔�/鎵煩
  // (鍔涙帶鏍稿績鐜妭1)
  //    閫熷害鎺у埗鍣ㄨ緭鍑虹殑涓嶆槸閫熷害鍙傝€冨€硷紝鑰屾槸闇€瑕佺殑鍚堝姏鍜屽悎鎵煩
  //    缁熶竴澶勭悊锛氭墍鏈夋ā寮忕殑wz锛堣閫熷害锛夐兘閫氳繃chassis_torque_pid杞崲涓烘壄鐭�
  VelocityToForceControl();

  // 2. 鍔涚殑鍔ㄥ姏瀛﹂€嗚В绠� (鍔涙帶鏍稿績鐜妭2)
  //    灏嗗簳鐩樺悎鍔涘垎閰嶅埌鍚勪釜杞瓙
  ForceDynamicsInverseResolution();

  // 3. 鍔涒啋鐢垫祦杞崲 + 閫熷害鍐呯幆 + 鎽╂摝琛ュ伩
  // (鍔涙帶鏍稿績鐜妭3)
  //    robowalker鏍稿績锛歜ase_current + speed_feedback + friction_comp
  ForceToCurrentConversion();

#if POWER_CONTROLLER_ENABLE
  // 4.
  // 鍔熺巼闄愬埗锛堝繀椤诲湪鍙戦€佺數鏈烘寚浠ゅ墠鎵ц锛�
  // 4.1 鑾峰彇瓒呯骇鐢靛鏁版嵁
  SuperCap_Rx_Data_s cap_rx_data = SuperCapGetData(cap);

  // 4.2
  // 鏇存柊鍔熺巼鎺у埗鍣ㄦ暟鎹紙鏆傛椂鏃犺鍒ょ郴缁燂紝浣跨敤瓒呯骇鐢靛鏁版嵁锛�
  // 鎵嬪姩璁惧畾鍔熺巼闄愬埗锛堟牴鎹疄闄呮祴璇曡皟鏁达紝鍗曚綅锛歐锛�
  float manual_power_limit = 95.0f;

  // 浣跨敤瓒呯骇鐢靛娴嬮噺鐨勬暟鎹�
  float cap_energy_buffer =
      (float)cap_rx_data.cap_energy; // 鐢靛鑳介噺 (0-255)
  float measured_power = cap_rx_data.chassis_power; // 搴曠洏鍔熺巼 (W)

  PowerUpdateRefereeData(manual_power_limit, cap_energy_buffer, measured_power);

  // 4.3 鏇存柊瓒呯骇鐢靛鍦ㄧ嚎鐘舵€�
  // 鍦ㄧ嚎鍒ゆ柇锛欳AN鏈夋暟鎹� 涓�
  // 閿欒鐮乥it7=0锛堣緭鍑烘湭鍏抽棴锛�
  uint8_t cap_online =
      (cap->can_ins->rx_len > 0 && (cap_rx_data.error_code & 0x80) == 0) ? 1
                                                                         : 0;
  PowerUpdateCapData(cap_rx_data.cap_energy, cap_online);

  // 4.4
  // 鏇存柊鐢垫満鍙嶉鏁版嵁锛堢敤浜嶳LS鍙傛暟杈ㄨ瘑锛�
  // 鈿狅笍 閲忕翰缁熶竴鍒拌緭鍑鸿酱渚э紙涓庡姏鎺у簳鐩樹竴鑷达級
  // 杈撳嚭杞磋閫熷害 = 杞瓙瑙掗€熷害 / 鍑忛€熸瘮
  float motor_speeds[4] = {
      motor_lf->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL,
      motor_rf->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL,
      motor_lb->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL,
      motor_rb->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL,
  };

  // 浣跨敤鐢垫満瀹炴祴鐢垫祦鍙嶉璁＄畻杞煩锛堣€岄潪鎺у埗鎸囦护锛�
  // 杩欏RLS鍙傛暟杈ㄨ瘑鑷冲叧閲嶈锛氬繀椤讳娇鐢ㄧ湡瀹炴秷鑰楃殑鐢垫祦锛岃€岄潪鏈熸湜鎸囦护
  // real_current鑼冨洿锛�-16384~16384锛岄渶涔樹互M3508_CMD_TO_CURRENT_COEFF杞崲涓哄畨鍩�
  // 鈿狅笍
  // 浣跨敤杈撳嚭杞磋浆鐭╁父鏁帮紙涓巑otor_speeds鐨勮緭鍑鸿酱瑙掗€熷害鍖归厤锛�
  const float TORQUE_CONSTANT =
      M3508_TORQUE_CONSTANT; // 0.3 Nm/A锛堣緭鍑鸿酱锛�
  float motor_torques[4] = {
      motor_lf->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT,
      motor_rf->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT,
      motor_lb->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT,
      motor_rb->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT,
  };
  PowerUpdateMotorFeedback(motor_speeds, motor_torques);

  // 4.5 鎵ц鑳介噺鐜帶鍒跺拰RLS鏇存柊
  PowerControllerTask();

  // 4.6 鍔熺巼闄愬埗锛氬鐢垫祦杩涜闄愬箙
  // 鈿狅笍 閲忕翰璇存槑锛�
  //   - target_wheel_omega 鏄洰鏍囪浆瀛愯閫熷害(rad/s)
  //   - motor_speeds 鏄綋鍓嶈緭鍑鸿酱瑙掗€熷害(rad/s)
  //   -
  //   鍔熺巼鎺у埗鍣ㄩ渶瑕佺粺涓€閲忕翰锛岄兘杞崲涓鸿緭鍑鸿酱瑙掗€熷害
  PowerMotorObj_t motor_objs[4] = {
      {.pid_output = wheel_current[0] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[0],
       .target_av = target_wheel_omega[0] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
      {.pid_output = wheel_current[1] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[1],
       .target_av = target_wheel_omega[1] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
      {.pid_output = wheel_current[2] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[2],
       .target_av = target_wheel_omega[2] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
      {.pid_output = wheel_current[3] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[3],
       .target_av = target_wheel_omega[3] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
  };
  float limited_output[4];
  PowerGetLimitedOutput(motor_objs, limited_output);

  // 5. 涓嬪彂鐢垫満鐢垫祦鎸囦护锛堜娇鐢ㄥ姛鐜囬檺鍒跺悗鐨勫€硷級
  DJIMotorSetRef(motor_lf, limited_output[0]);
  DJIMotorSetRef(motor_rf, limited_output[1]);
  DJIMotorSetRef(motor_lb, limited_output[2]);
  DJIMotorSetRef(motor_rb, limited_output[3]);
#else
  // 5. 涓嬪彂鐢垫満鐢垫祦鎸囦护锛堟棤鍔熺巼闄愬埗锛�
  //    娉ㄦ剰锛氳繖閲岀洿鎺ヤ娇鐢╳heel_current鏁扮粍锛屽崟浣嶄负瀹夊煿(A)
  //    闇€瑕佽浆鎹负CAN鎸囦护鍊硷細cmd = current /
  //    M3508_CMD_TO_CURRENT_COEFF
  DJIMotorSetRef(motor_lf, wheel_current[0] / M3508_CMD_TO_CURRENT_COEFF);
  DJIMotorSetRef(motor_rf, wheel_current[1] / M3508_CMD_TO_CURRENT_COEFF);
  DJIMotorSetRef(motor_lb, wheel_current[2] / M3508_CMD_TO_CURRENT_COEFF);
  DJIMotorSetRef(motor_rb, wheel_current[3] / M3508_CMD_TO_CURRENT_COEFF);
#endif

  // 5.
  // 鏍规嵁瑁佸垽绯荤粺鐨勫弽棣堟暟鎹拰鐢靛鏁版嵁瀵硅緭鍑洪檺骞呭苟璁惧畾闂幆鍙傝€冨€�
  // LimitChassisOutput();  //
  // 鍔涙帶绛栫暐涓凡鍦‵orceToCurrentConversion涓畬鎴愰檺骞�

  // 6. 鏍规嵁鐢垫満鐨勫弽棣堥€熷害鍜孖MU(濡傛灉鏈�)璁＄畻鐪熷疄閫熷害
  // EstimateSpeed();

  // // 鑾峰彇瑁佸垽绯荤粺鏁版嵁
  // 寤鸿灏嗚鍒ょ郴缁熶笌搴曠洏鍒嗙锛屾墍浠ユ澶勬暟鎹簲浣跨敤娑堟伅涓績鍙戦€�
  // //
  // 鎴戞柟棰滆壊id灏忎簬7鏄孩鑹�,澶т簬7鏄摑鑹�,娉ㄦ剰杩欓噷鍙戦€佺殑鏄鏂圭殑棰滆壊,
  // 0:blue , 1:red chassis_feedback_data.enemy_color =
  // referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
  // //
  // 褰撳墠鍙仛浜�17mm鐑噺鐨勬暟鎹幏鍙�,鍚庣画鏍规嵁robot_def涓殑瀹忓垏鎹㈠弻鏋鍜岃嫳闆�42mm鐨勬儏鍐�
  // chassis_feedback_data.bullet_speed =
  // referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
  // chassis_feedback_data.rest_heat =
  // referee_data->PowerHeatData.shooter_heat0;

  // 鎺ㄩ€佸弽棣堟秷鎭�
#ifdef ONE_BOARD
  PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
  CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}