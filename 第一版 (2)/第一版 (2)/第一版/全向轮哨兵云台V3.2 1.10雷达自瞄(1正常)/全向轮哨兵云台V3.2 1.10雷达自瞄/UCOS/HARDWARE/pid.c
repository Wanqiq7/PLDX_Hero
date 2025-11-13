#include "pid.h"
#include "stdio.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

	/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(PidTypeDef *pid, char mode, float PID[3],float max_erro, float max_iout, float max_out)
{
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->mode = mode;
    pid->max_erro = max_erro;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

}
	
static float pid_caculate(PidTypeDef *pid, const float ref, const float set)
{  
		if (pid == NULL)
    {
        return 0;
    }
    if (pid->mode == PID_POSITION) 
    {		
				pid->error = set - ref;
				pid->Dbuf[0] = ref;
				pid->Iout = pid->Iout +  pid->error;
				LimitMax(pid->Iout, pid->max_iout);
				if(pid->error > pid->max_erro){pid->Iout = 0;}
				if(pid->error < -pid->max_erro){pid->Iout = 0;}			
				pid->Dbuf[2] = pid->Dbuf[0] - pid->Dbuf[1];
        pid->Pout = pid->Kp * pid->error;
				pid->Iout = pid->Ki * pid->Iout;				
				pid->Dout = pid->Kd * pid->Dbuf[2];
        pid->out = pid->Pout + pid->Iout - pid->Dout;
				pid->Dbuf[1] = ref;
				LimitMax(pid->out, pid->max_out);      
    }
    else if (pid->mode == PID_DELTA)
    {
				pid->Dbuf[0] = ref;
			  pid->Dbuf[2] = pid->Dbuf[0] - pid->Dbuf[1];
				pid->error = set - ref;
				pid->Dout = pid->Kd * pid->Dbuf[2];
        pid->Pout = pid->Kp * pid->error;
        pid->Iout = pid->Iout * pid->Ki + pid->Pout - pid->Dout;
				pid->out = pid->Iout;
        pid->Dbuf[1] = ref;        
    }
    return pid->out;
}
/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init(PidTypeDef* pid)
{
    pid->Param_Init = pid_param_init;
    pid->Cal_PID = pid_caculate;
}


float fc = 2.0f;     //截止频率
float Ts = 0.02f;    //采样周期
float pi = 3.14159f; //π
float alpha = 0;     //滤波系数

/************************ 滤波器初始化 alpha *****************************/

float low_pass_filter(float value)
{
  static float out_last = 0; //上一次滤波值
  float out;

	float b = 2.0f * (float)pi * fc * Ts;
  alpha = b / (b + 1);
	
	
  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
