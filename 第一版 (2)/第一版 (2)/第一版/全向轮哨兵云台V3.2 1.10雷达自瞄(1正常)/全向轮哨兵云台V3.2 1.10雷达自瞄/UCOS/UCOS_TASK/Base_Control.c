#include "Base_Control.h"
#include "AppTask.h"
#include "ChassisTask.h"
#include "GimbalTask.h"
#include "TIM7_FPS.H"
#include "can1.h"
#include "can2.h"
#include "usart.h"
#include "usart3_rc.h"

/**********************
云台任务V1.1

1.使用BM088I作为陀螺仪。
2.创建4个模式：
        （1）：云台无力。
        （2）：云台跟随底盘。
        （3）：底盘跟随云台。
        （4）：云台自动瞄准。
**********************/
double angle_v_as[3];
double PID_Yaw_I2;
double PID_Yaw_I, PID_YawW_I, PID_Yaw_D_1, PID_Yaw_D_2;
double PID_Pich_I, PID_Pich_D_1, PID_Pich_D_2;
double PID_Yaw_Out, PID_Pich_Out;
double M6020_X_V[2], M6020_X_V_err[2];
double M6020_J_V[2], M6020_J_V_err[2];
double M6020_OUT[2];
double YAW_posisition_err, YAW_posisition_err_D[2];
int kk;
double Yaw_Middl, Yaw_erro, Pich_erro;
double Poch_hignt_limit, Poch_down_limit;
double Yaw_target, Pich_target;
double target_yaw, target_pich;
double kkk;

float kp_X_com = 0.95, ki_X_com, kd_X_com = 0.6, Integral_bias_x; // x轴跟随
float err_X, last_err_X; // 当前误差
int Add_X;

float kp_Y_com = 0.72, ki_Y_com, kd_Y_com = 0.9, Integral_bias_y; // x轴跟随
float err_Y, last_err_Y; // 当前误差
int Add_Y;

int Pit_Add = 400;

extern int Data, cx, cy, cw, ch; // 串口接收的装甲板中心坐标值

int mid_X = 560; // 画面中心点		往右增大  77   80		//590
int mid_Y = 440; // 往上增大

int out_pitch;

unsigned char Auto_state = 0; // 自动模式状态标志 1:自动  0：手动
unsigned char Capture;        // 是否识别到目标
unsigned char
    Lose_Count; // 目标丢失时间计数器，此值在定时器中断（1s）内自增，在串口6中断内置零，当串口6未接收到数据不进中断，则此值一直自增，用于判断目标是否丢失
unsigned char com = 9;
unsigned char First_get;
extern short Position_Value_6020[2];

int Spin_Speed = 300;
float Yaw_in;

extern short gyro_0, gyro_1, gyro_2;
extern short accel_0, accel_1, accel_2;

unsigned char target_online;

unsigned char aim_auto = 0;

extern _Base_Control_Param Base_Control_Param;

char aim_test; // 自瞄测试

extern u16 Muzzle_heat0;      // 枪口热量
extern u16 Muzzle_heat1;      // 枪口热量
extern u16 Muzzle_heat_Limit; // 枪口热量上限
extern u16 Game_progress;     // 比赛阶段：
                          //	0：未开始
                          //	1：准备阶段
                          //	2：15秒自检
                          //	3：五秒倒计时
                          //	4：比赛中
                          //	5：比赛结算

extern u16 red_outpost_HP;  // 红方前哨站hp
extern u16 blue_outpost_HP; // 蓝方前哨站hp
extern u16 admit_num;       // 允许发弹量
extern u16 time;            // 当前比赛进程剩余时间

extern u16 Remian_hp; // 当前血量

extern char arrive;

int p_out;

int Open_MV_X() // openMV的x轴追踪
{
  err_X = cx - mid_X;       // 计算误差
  Integral_bias_x += err_X; // 误差累加积分
  p_out = (int)(kp_X_com * err_X);
  if (p_out >= 400) {
    p_out = 400;
  }

  if (p_out <= -400) {
    p_out = -400;
  }
  Add_X = p_out + ki_X_com * Integral_bias_x + kd_X_com * (err_X - last_err_X);
  last_err_X = err_X;
  return Add_X;
}

int Open_MV_Y() // openMV的Y轴追踪
{
  err_Y = cy - out_pitch;   // 计算误差
  Integral_bias_y += err_Y; // 误差累加积分

  Add_Y = kp_Y_com * err_Y + ki_Y_com * Integral_bias_y +
          kd_Y_com * (err_Y - last_err_Y);
  last_err_Y = err_Y;
  return Add_Y;
}

int pitch_min;
int pitch_max;
int pitch_add_max;

unsigned char init_sta; // pitch起始

/**********************************************************************************、
函数名：Motion_mode(void)
形式参数：无
作者：
时间：2022年2月6日
功能：控制云台世界角速度
**********************************************************************************/
void Motion_mode(void) {

  if (Auto_state == 1 &&
      Capture == 1) // 如果处于自动模式且找到目标, 进行视觉自瞄
  {
    if (cx != 0 || cy != 0) // 目标装甲板存在
    {
      Yaw_target = Open_MV_X() * -10;
      Pich_target = Open_MV_Y() * -10;
    }

    if (Lose_Count > 0 && Lose_Count <= 1) {
      Yaw_target = 0;
      Pich_target = 0;
    }
    Yaw_erro = Yaw_target - (double)gyro_souce[2] +
               7.6207; //   陀螺仪调试 3.7337 0.08-0.1 +右0.6207-0.6007
    Pich_erro = Pich_target - (double)gyro_souce[0] -
                3.321; // 陀螺仪调试 3.7307  0.2-0.3  +上0.2257-0.2307
    PID_Yaw_D_1 = gyro_souce[2];
    PID_Pich_D_1 = gyro_souce[0];
    PID_Yaw_I = PID_Yaw_I + Yaw_erro * 1.88;
    PID_Pich_I = PID_Pich_I + Pich_erro * 1.18;
    if (PID_Yaw_I > 19999) {
      PID_Yaw_I = 19999;
    }
    if (PID_Yaw_I < -19999) {
      PID_Yaw_I = -19999;
    }
    if (Yaw_erro > 9999) {
      PID_Yaw_I = 0;
    }
    if (Yaw_erro < -9999) {
      PID_Yaw_I = 0;
    }
    if (PID_Pich_I > 19999) {
      PID_Pich_I = 19999;
    }
    if (PID_Pich_I < -19999) {
      PID_Pich_I = -19999;
    }
    if (Pich_erro > 9999) {
      PID_Pich_I = 0;
    }
    if (Pich_erro < -9999) {
      PID_Pich_I = 0;
    }
    PID_Yaw_Out =
        Yaw_erro * 17 + PID_Yaw_I * 0.12 - (PID_Yaw_D_1 - PID_Yaw_D_2) * 2.5;
    // PID_Pich_Out = Pich_erro * 16 + PID_Pich_I*0.32 -
    // (PID_Pich_D_1-PID_Pich_D_2)*2.0;
    PID_Pich_Out = Pich_erro * 16 + PID_Pich_I * 0.32 -
                   (PID_Pich_D_1 - PID_Pich_D_2) * 2.0;
    PID_Pich_D_2 = gyro_souce[0];
    PID_Yaw_D_2 = gyro_souce[2];
    if (PID_Yaw_Out > 29999) {
      PID_Yaw_Out = 29999;
    }
    if (PID_Yaw_Out < -29999) {
      PID_Yaw_Out = -29999;
    }
    if (PID_Pich_Out > 29999) {
      PID_Pich_Out = 29999;
    }
    if (PID_Pich_Out < -29999) {
      PID_Pich_Out = -29999;
    }
    CAN2_Send_queue_add(0x1ff, 0x08, PID_Pich_Out, 0, 0, 0); //
    CAN1_Send_queue_add(0x2ff, 0x08, PID_Yaw_Out, 0, 0, 0);
  }

  if (Auto_state == 1 &&
      Capture == 0) // 如果为自动模式并丢失目标，则云台开始自主运动寻找目标
  {

    pitch_min = 3700; // pitch轴摆动范围
    pitch_max = 4500;
    pitch_add_max = 300;

    //		if(Position_Value_6020[0]>=6450){Pit_Add=-700;}
    ////pitch轴摆动范围 		if(Position_Value_6020[0]<=6100){Pit_Add= 700;}

    if (Position_Value_6020[0] >= pitch_max) {
      init_sta = 2;
    } else if (Position_Value_6020[0] <= pitch_min) {
      init_sta = 1;
    }

    if (init_sta == 1) {
      Pit_Add = pitch_add_max;
    }
    if (init_sta == 2) {
      Pit_Add = -pitch_add_max;
    }

    if (aim_test == 0) // 雷达控制时，雷达通过控制云台方向进行导航
    {
      Yaw_target = (-8) + (880 * speed_w); // yaw轴旋转
      Pich_target = 0;                     // Pit_Add;
    }

    if (aim_test == 1) // 自瞄测试时
    {
      Yaw_target = 0;        // yaw轴旋转
      Pich_target = Pit_Add; // Pit_Add;
    }

    Yaw_erro = Yaw_target - (double)gyro_souce[2] +
               7.6207; //   陀螺仪调试 3.7337 0.08-0.1 +右0.6207-0.6007
    Pich_erro = Pich_target - (double)gyro_souce[0] -
                3.321; // 陀螺仪调试 3.7307  0.2-0.3  +上0.2257-0.2307
    PID_Yaw_D_1 = gyro_souce[2];
    PID_Pich_D_1 = gyro_souce[0];
    PID_Yaw_I = PID_Yaw_I + Yaw_erro * 1.88;
    PID_Pich_I = PID_Pich_I + Pich_erro * 1.18;
    if (PID_Yaw_I > 19999) {
      PID_Yaw_I = 19999;
    }
    if (PID_Yaw_I < -19999) {
      PID_Yaw_I = -19999;
    }
    if (Yaw_erro > 9999) {
      PID_Yaw_I = 0;
    }
    if (Yaw_erro < -9999) {
      PID_Yaw_I = 0;
    }
    if (PID_Pich_I > 19999) {
      PID_Pich_I = 19999;
    }
    if (PID_Pich_I < -19999) {
      PID_Pich_I = -19999;
    }
    if (Pich_erro > 9999) {
      PID_Pich_I = 0;
    }
    if (Pich_erro < -9999) {
      PID_Pich_I = 0;
    }
    PID_Yaw_Out = Yaw_erro * 16 + PID_Yaw_I - (PID_Yaw_D_1 - PID_Yaw_D_2) * 1.5;
    PID_Pich_Out =
        Pich_erro * 16 + PID_Pich_I - (PID_Pich_D_1 - PID_Pich_D_2) * 1;
    PID_Pich_D_2 = gyro_souce[0];
    PID_Yaw_D_2 = gyro_souce[2];
    if (PID_Yaw_Out > 29999) {
      PID_Yaw_Out = 29999;
    }
    if (PID_Yaw_Out < -29999) {
      PID_Yaw_Out = -29999;
    }
    if (PID_Pich_Out > 29999) {
      PID_Pich_Out = 29999;
    }
    if (PID_Pich_Out < -29999) {
      PID_Pich_Out = -29999;
    }

    //		if(Game_progress!=4||aim_test!=1)
    ////比赛未开始，云台无力
    //		{
    //			PID_Pich_Out=0;
    //			PID_Yaw_Out=0;
    //		}
    CAN2_Send_queue_add(0x1ff, 0x08, PID_Pich_Out, 0, 0, 0); // 控制四个底盘电机
    CAN1_Send_queue_add(0x2ff, 0x08, PID_Yaw_Out, 0, 0, 0);
  }

  if (Auto_state == 0) // 如? ??侄??蛟铺ㄓ梢？仄骺刂?
  {
    Yaw_target = -(RC_Ctl.rc.ch0 - 1024) * 8;
    Pich_target = (RC_Ctl.rc.ch1 - 1024) * 8;
    Yaw_erro = Yaw_target - (double)(gyro_souce[2] + com) +
               7.6207; //   陀螺仪调试 3.7337 0.08-0.1 +右0.6207-0.6007
    Pich_erro = Pich_target - (double)gyro_souce[0] -
                3.321; // 陀螺仪调试 3.7307  0.2-0.3  +上0.2257-0.2307
    PID_Yaw_D_1 = (gyro_souce[2] + com);
    PID_Pich_D_1 = gyro_souce[0];
    PID_Yaw_I = PID_Yaw_I + Yaw_erro * 1.18;
    PID_Pich_I = PID_Pich_I + Pich_erro * 1.18;
    if (PID_Yaw_I > 19999) {
      PID_Yaw_I = 19999;
    }
    if (PID_Yaw_I < -19999) {
      PID_Yaw_I = -19999;
    }
    if (Yaw_erro > 9999) {
      PID_Yaw_I = 0;
    }
    if (Yaw_erro < -9999) {
      PID_Yaw_I = 0;
    }
    if (PID_Pich_I > 19999) {
      PID_Pich_I = 19999;
    }
    if (PID_Pich_I < -19999) {
      PID_Pich_I = -19999;
    }
    if (Pich_erro > 9999) {
      PID_Pich_I = 0;
    }
    if (Pich_erro < -9999) {
      PID_Pich_I = 0;
    }
    PID_Yaw_Out = Yaw_erro * 18 + PID_Yaw_I - (PID_Yaw_D_1 - PID_Yaw_D_2) * 1.2;
    PID_Pich_Out =
        Pich_erro * 18 + PID_Pich_I - (PID_Pich_D_1 - PID_Pich_D_2) * 0.2;
    PID_Pich_D_2 = gyro_souce[0];
    PID_Yaw_D_2 = (gyro_souce[2] + com);
    if (PID_Yaw_Out > 29999) {
      PID_Yaw_Out = 29999;
    }
    if (PID_Yaw_Out < -29999) {
      PID_Yaw_Out = -29999;
    }
    if (PID_Pich_Out > 29999) {
      PID_Pich_Out = 29999;
    }
    if (PID_Pich_Out < -29999) {
      PID_Pich_Out = -29999;
    }
    CAN2_Send_queue_add(0x1ff, 0x08, PID_Pich_Out, 0, 0, 0); // 控制四个底盘电机
    CAN1_Send_queue_add(0x2ff, 0x08, PID_Yaw_Out, 0, 0, 0);
  }
}

void Finding_Mode() {
  Yaw_target = 300;
  Pich_target = -Open_MV_Y() * 5;
}

/********/
// Func Name		:	Target_loss_judgment
// Performance	:	目标丢失判断
// Return			:
// Para				:
// Author			:
// Date				:
void Target_loss_judgment() {
  if (Auto_state == 1) // 如果为自动模式
  {
    if (Lose_Count < 3) // 如果计数器小于3且，则判断为找到目标，开始自瞄
    {
      Capture = 1; // 目标装甲板在线
    }

    if (Lose_Count >= 3) // 丢失目标3s后，开始自动搜索
    {
      Capture = 0; // 目标装甲板丢失
    }
  }
}

/********/
// Func Name		:	AutoAim_PitchAdjust
// Performance	:
// 根据openmv传回像素大小进行距离推断并调整自瞄时云台Pitch目标角度以调整弹道；缺点：数据不精确，灵敏度低
// （改进空间） Return			: Para				: Author
// : Date				:
void AutoAim_PitchAdjust() {
  unsigned char cw_con;
  int cw_av, cw_sum; // 用于计算像素大小
  cw_con++;
  cw_sum += cw;

  if (cw_con > 7) // 每接收七次openmv数据计算目标其像素大小平均值
  {
    cw_av = cw_sum / 7;
    cw_sum = 0;
    cw_con = 0;
  }
  // printf("%d\r",cw);

  if (cw <= 6)
    out_pitch = mid_Y + 5;
  else
    out_pitch = mid_Y;
}

/**********************************************************************************、
任务名：Gimbal_task(void *p_arg)
作者：
时间：2022年2月11日
功能：控制云台瞄准
模式一：运动模式
模式二：运动模式
模式三：运动模式
丢控保护：云台电机无力
任务频率：500FPS
**********************************************************************************/
void Gimbal_task(void *p_arg) {
  double last_world_angle;
  OS_ERR err;
  p_arg = p_arg;
  printf("云台任务 开始\r\n");

  while (1) {
    // printf("%d\r",Position_Value_6020[0]);

    AutoAim_PitchAdjust();  // 自瞄下的pitch目标值矫正
    Target_loss_judgment(); // 目标丢失判断

    if (RC_Ctl.rc.s2 == 3) // 模式三：运动模式
    {
      Motion_mode();
      last_world_angle = word_angle[2];
    } else if (RC_Ctl.rc.s2 == 1) // 模式二：运动模式
    {
      Motion_mode();
      last_world_angle = word_angle[2];
    } else if (RC_Ctl.rc.s2 == 2) // 模式一：运动模式
    {
      Motion_mode();
      last_world_angle = word_angle[2];
    } else // 模式四：丢控保护，云台无力
    {
      CAN1_Send_queue_add(0x1ff, 0x08, 0, 0, 0, 0); // 控制四个底盘电机
    }

    OSTimeDlyHMSM(0, 0, 0, 2, OS_OPT_TIME_HMSM_STRICT, &err); // 延时100ms
  }
}

/**********************************************************************************、
函数名：TV_test(void)
形式参数：无
作者：
时间：2022年2月16日
功能：自动瞄准模式
*********************************************************************************/
// void TV_test(void)
//{
//		Yaw_erro = -(RC_Ctl.rc.ch0-1024)*8 - gyro_souce[2] +
//(TV_target[0]-180)*28;//Yaw_erro = -(RC_Ctl.rc.ch0-1024)*8 - gyro_souce[2];
//	  Pich_erro = -(RC_Ctl.rc.ch1-1024)*8 - gyro_souce[0];
//		PID_Yaw_I = PID_Yaw_I + Yaw_erro;
//		PID_Pich_I = PID_Pich_I + Pich_erro*0.58;
//		PID_Yaw_Out = Yaw_erro*18 + PID_Yaw_I*0.88;
//		PID_Pich_Out = Pich_erro * 18 + PID_Pich_I;
//		if(PID_Yaw_I>9999){PID_Pich_Out = 9999;}
//		if(PID_Yaw_I<-9999){PID_Pich_Out = -9999;}
//		if(PID_Pich_I>9999){PID_Pich_Out = 9999;}
//		if(PID_Pich_I<-9999){PID_Pich_Out = -9999;}
//		if(PID_Yaw_Out>29999){PID_Yaw_Out = 29999;}
//		if(PID_Yaw_Out<-29999){PID_Yaw_Out = -29999;}
//		if(PID_Pich_Out>29999){PID_Pich_Out = 29999;}
//		if(PID_Pich_Out<-29999){PID_Pich_Out = -29999;}
//		CAN2_Send_queue_add(0x1ff,0x08,PID_Pich_Out,0,0,0);//控制四个底盘电机
//		CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);
// }
/**********************************************************************************、
函数名：world_angle_keep(double abbs)
形式参数：
1.视觉角度：0~418000，分别对应0~360度
作者：
时间：2022年2月14日
功能：控制云台角度稳定于某个世界角度
**********************************************************************************/
// double world_angle_target;
// double world_angle_erro;
// double world_angle_yaw_D1;
// double world_angle_yaw_D2;
// void world_angle_keep(double abbs)
//{
//		world_angle_target = abbs;
//			if(world_angle_target >= word_angle[2])
//		{
//			if((world_angle_target-word_angle[2]) >= 209000)
//			{
//				world_angle_erro = -word_angle[2]  +
//(world_angle_target - 418000);
//			}
//			else if((world_angle_target-word_angle[2]) < 209000)
//			{
//				world_angle_erro = (world_angle_target -
//word_angle[2]);
//			}
//		}
//		else if(world_angle_target < word_angle[2])
//		{
//			if((world_angle_target-word_angle[2]) >= -209000)
//			{
//				world_angle_erro =  -(word_angle[2] -
//world_angle_target);
//			}
//				if((world_angle_target-word_angle[2]) < -209000)
//			{
//				world_angle_erro =  (418000 - word_angle[2] +
//world_angle_target);
//			}
//		}
//	 if(world_angle_erro < -18000)
//		{
//			world_angle_erro = -18000;
//		}
//		if(world_angle_erro > 18000)
//		{
//			world_angle_erro = 18000;
//		}
//		world_angle_yaw_D1 = world_angle_erro;
//		Yaw_target = -(RC_Ctl.rc.ch0-1024)*1 +
//world_angle_erro*0.2033322 - (world_angle_yaw_D1 - world_angle_yaw_D2)*0.4;
//		Pich_target = (RC_Ctl.rc.ch1-1024)*8;
//		world_angle_yaw_D2 = world_angle_erro;
//		Yaw_erro = Yaw_target - (double)gyro_souce[2]+3.7337;
//		Pich_erro = Pich_target - (double)gyro_souce[0];
//		PID_Yaw_I = PID_Yaw_I + Yaw_erro*1.18;
//		PID_Pich_I = PID_Pich_I + Pich_erro*1.18;
//		if(PID_Yaw_I>9999){PID_Yaw_I = 9999;}
//		if(PID_Yaw_I<-9999){PID_Yaw_I = -9999;}
//		if(Yaw_erro>9999){PID_Yaw_I = 0;}
//		if(Yaw_erro<-9999){PID_Yaw_I = 0;}
//		PID_Yaw_D_1 = gyro_souce[2];
//		PID_Pich_D_1 = gyro_souce[0];
//		PID_Yaw_Out = Yaw_erro*15 + PID_Yaw_I -
//(PID_Yaw_D_1-PID_Yaw_D_2)*1.2; 		PID_Pich_Out = Pich_erro * 18 + PID_Pich_I -
//(PID_Pich_D_1-PID_Pich_D_2)*0.2; 		PID_Yaw_D_2 = gyro_souce[2]; 		PID_Pich_D_2 =
//gyro_souce[0]; 		if(PID_Yaw_Out>29999){PID_Yaw_Out = 29999;}
//		if(PID_Yaw_Out<-29999){PID_Yaw_Out = -29999;}
//		if(PID_Pich_Out>29999){PID_Pich_Out = 29999;}
//		if(PID_Pich_Out<-29999){PID_Pich_Out = -29999;}
//		//printf("%d,%d\r",cx,cy);
//		//printf("data1=%f,data2=%f,data3=%d,",PID_Yaw_Out,PID_Yaw_I2,gyro_souce[2]+4);
//		CAN2_Send_queue_add(0x1ff,0x08,PID_Pich_Out,0,0,0);//控制四个底盘电机
//		CAN1_Send_queue_add(0x2ff,0x08,PID_Yaw_Out,0,0,0);
// }//			Base_X_Speed(0.5);
//		}
//		if(Base_Control_Param.Count == 4280)  //50
//		{
//			Base_Control_Param.Count = 4280;
//			Base_Control_Param.Flag = 1;
//			Base_Stop();
//		}
//		if(Base_Control_Param.Flag == 1)
//		{
//			Gyro_360_motion(5000,1200);		//开启小陀螺
//		}
//
//	}
//		if(RC_Ctl.rc.s1 == 2)
//		{
//			Base_Control_Param.Count1++;
//			if(Base_Control_Param.Count1 >4000 &&
//Base_Control_Param.Count1 < 4700)   //350
//			{
//        Base_X_Speed(0);
//			 Base_Y_Speed(0.5);
//			}
//			if(Base_Control_Param.Count1 == 4700)
//			{
//		    Base_Control_Param.Count1 = 4700;
//				Base_Control_Param.Flag1 = 1;
//			  Base_Stop();
//			}
//       if(Base_Control_Param.Flag1 == 1)
//			{
//			  Gyro_360_motion(7500,1200);		//开启小陀螺

//			}
//		}
else {
  Base_Stop();
  Base_Control_Param.Count = 0;
  Base_Control_Param.Count1 = 0;
  Base_Control_Param.Flag = 0;
  Base_Control_Param.Flag1 = 0;
}
}

// printf("%d, %d\r",Game_progress,enforce_start);
OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err2); // 延时10ms
}
}
