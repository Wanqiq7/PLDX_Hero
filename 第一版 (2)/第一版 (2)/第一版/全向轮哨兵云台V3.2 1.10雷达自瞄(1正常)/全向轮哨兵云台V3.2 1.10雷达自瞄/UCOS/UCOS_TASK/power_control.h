#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "sys.h"

#define judge_buffer 60 //裁判系统缓冲能量上限60焦耳，暂时用宏定义表示
#define chassispowerlimit 70 //功率优先情况下的一级限定功率，单纯功率
#define pro_buffer 30 //裁判系统可用于调控进行功率限制的缓冲能量
#define Mecanum_PowerControl_PowerMax 4.0f
#define Scap_add_power 10.0f//使用超电时增加功率
#define RefereeSystem_GameStatus 1//1为比赛中，0为比赛未开始（要来自裁判系统的准确数据）
#define RefereeSystem_RemainTime 1//比赛时间（要来自裁判系统的准确数据）
#define Mecanum_PowerControl_T 3.0f
#define Mecanum_PowerControlGainCoefficientInitialValue		0.2f//功率控制增益系数初始值

typedef struct power_t{
	float Buffer_PowerRef;//功率增益;
	float Power_Limit;//功率上限;
	float motor_power[4];//四个电机的功率
	float motor_powerLimit[4];//四个电机的功率上限
	float scale;//速度缩放因子;
	float speed_magnitude;//速度向量模长，不考虑方向，只考虑大小的总平移速度大小
	float Max_Translation;//最大平移速度,要根据实际代码的Vx和Vy计算得到具体数据
	float current_power;//当前功率（计算值）
	float K;//系数,要实际测
	float K1;
	float K2;//系数,要实际测
	float PowerSum;//控制周期内的功率积分
	uint8_t count;//功率控制周期计数器
	float average_power;//一定时间窗口内的平均功率值
	float RefereeSystem_Energy;//裁判系统能量反馈
	float Energy_PowerLimit;//能量策略下的功率限制
	uint8_t cap_elc_ok;
}power_t;

typedef struct power_pid{
	float Kp;
	float Ki;
	float Kd;
	float error;
	float lasterror;
	float target;
	float target_cap;
	float target_nocap;
	float actual;
	float integral;
	float derivative;
	float out;
	uint8_t energy_control_enabled;//使能能量环PID控制的标志位
}power_pid;

extern power_t PC;
extern power_pid Energy_PID;
extern uint8_t chassis_gryo360_move;//小陀螺标志位;
extern uint8_t chassis_following_Gimbal;//底盘跟随云台标志位;
extern uint8_t chassis_following_old;
extern uint8_t chassis_sideway;//测试模式标志位
extern uint8_t pitch_up_flag;//上坡标志位;
extern float V_Gains;//速度增益系数

void Chassis_PowerControlParam_Init(void);
void Energy_PIDcontrol_loop(void);
void Chassis_PowerControlTask(float Vx,float Vy,float Vz,float simulated_energy);
void Chassis_PowerControlTest(float Vx,float Vy,float Vz,float Input_Energy);
//能量环控制使能函数
void Enable_EnergyLoopControl(uint8_t enable);
void Set_EnergyLoopTarget(float target);
void Set_EnergyLoopPID(float kp, float ki, float kd);

#endif

