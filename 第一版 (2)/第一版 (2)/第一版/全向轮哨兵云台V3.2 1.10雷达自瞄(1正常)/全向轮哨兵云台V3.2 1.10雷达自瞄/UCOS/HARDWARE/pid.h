#ifndef PID_H
#define PID_H

#include "Sys.h"

typedef struct PidTypeDef
	{
    char mode; //PID类型
									
    float Kp;  //PID 三个参数
    float Ki;
    float Kd;

		float max_erro; //最大误差
    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set; //设定值
    float fdb; //反馈值

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error; //误差项 0最新 1上一次 2上上次
	
	void ( *Param_Init)(struct PidTypeDef *pid,  //PID参数初始化
		char mode,
		float PID[3],
		float max_erro,
		float max_iout,
		float max_out			
	);
	float (*Cal_PID)(struct PidTypeDef *pid, const float ref, const float set);   //pid计算
		
}PidTypeDef;

typedef enum
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA, //增量式PID
}PID_MODE;

void pid_init(PidTypeDef* pid);
float low_pass_filter(float value);

#endif
