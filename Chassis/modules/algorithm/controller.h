/**
 ******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "main.h"
#include "stdint.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
    PID_Integral_Limit = 0b00000001,              // 0000 0001
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
    PID_OutputFilter = 0b00010000,                // 0001 0000
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
    PID_DerivativeFilter = 0b01000000,            // 0100 0000
    PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config_s;

/**
 * @brief 初始化PID实例
 * @todo 待修改为统一的PIDRegister风格
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

/* ======================== RLS递归最小二乘算法 ======================== */

/**
 * @brief RLS算法实例，固定为2维
 * @note  用于电机功率模型参数识别: P = τΩ + k1|Ω| + k2τ^2 + k3
 *        识别参数: k1(转速损耗系数), k2(力矩损耗系数)
 */
typedef struct
{
    float lambda;                    // 遗忘因子(0.9~1.0)，越接近1收敛越慢但越稳定
    float delta;                     // 初始化转移矩阵的非奇异值
    
    float trans_matrix[4];           // 2x2转移矩阵 P(k)
    float gain_vector[2];            // 2x1增益向量 K(k)
    float params_vector[2];          // 2x1参数向量 [k1, k2]
    
    uint32_t update_cnt;             // 更新计数
} RLSInstance;

/**
 * @brief RLS算法初始化配置
 */
typedef struct
{
    float lambda;                    // 遗忘因子，推荐0.9999
    float delta;                     // 初始化值，推荐1e-5
    float init_k1;                   // k1初始值，推荐0.22
    float init_k2;                   // k2初始值，推荐1.2
} RLS_Init_Config_s;

/**
 * @brief 初始化RLS实例
 * 
 * @param rls RLS实例指针
 * @param config RLS初始化配置
 */
void RLSInit(RLSInstance *rls, RLS_Init_Config_s *config);

/**
 * @brief RLS算法更新一次
 * 
 * @param rls RLS实例指针
 * @param sample_vector 采样向量 [|ω|, τ^2]，2x1向量
 * @param actual_output 实际输出值(实测功率损耗)
 */
void RLSUpdate(RLSInstance *rls, float sample_vector[2], float actual_output);

/**
 * @brief 重置RLS算法
 * 
 * @param rls RLS实例指针
 */
void RLSReset(RLSInstance *rls);

/**
 * @brief 获取当前识别的参数
 * 
 * @param rls RLS实例指针
 * @param k1 输出k1参数指针
 * @param k2 输出k2参数指针
 */
void RLSGetParams(RLSInstance *rls, float *k1, float *k2);

#endif