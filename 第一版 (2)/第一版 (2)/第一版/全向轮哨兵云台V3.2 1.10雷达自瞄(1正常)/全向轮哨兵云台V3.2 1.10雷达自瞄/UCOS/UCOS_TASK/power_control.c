#include "power_control.h"
#include "Can_Task.h"
#include "can1.h"
#include "ChassisTask.h"

//可能要判断超电关与开
uint8_t pitch_up_flag=0;//上坡标志位;
float V_Gains=0;//速度增益系数
//后续还得加一个che电位器不为1024时候的标志位,那个变速小陀螺也会急剧消耗缓冲能量，增加超功率风险

static float Chassis_Energy=20000.0;//估计底盘能量
static float Last_RefereeSystem_Energy=20000.0;//上一次裁判系统剩余能量值，用于校准
float RefereeSystem_Energy=20000.0;
// static float RefereeSystem_Energy=0.0f;//裁判系统能量反馈，到时候换用具体能量值(来自裁判系统)
//还需要:超电能量(电容组能量)，裁判系统缓冲能量（实际值），裁判系统功率上限（实际值），比赛状态，剩余比赛时间，裁判系统能量反馈。
power_t PC;
power_pid Energy_PID;

void Chassis_PowerControlParam_Init(void)
{
	PC.count=0;
	chassis_gryo360_move=0;//小陀螺标志位在初始化的时候为0
//	chassis_following_old=0;
	chassis_following_Gimbal=0;//底盘跟随云台的标志位在初始化时置为0；
	chassis_sideway=0;
	pitch_up_flag=0;//坡上标志位，用于飞坡
	SCAP_Control(0,60,1,0);//开启超电
	PC.Max_Translation=4667.0f;
	V_Gains=1.0f;
  PC.PowerSum=0;
	
	//能量环PID初始化:
	Energy_PID.target_cap=1030.0f;//1030
	Energy_PID.target_nocap=40.0f;
	Energy_PID.Kp=1.2f;
	Energy_PID.Ki=0.01f;
	Energy_PID.Kd=0.3f;
	Energy_PID.error = 0.0f;
	Energy_PID.lasterror = 0.0f;
	Energy_PID.actual = 0.0f;
	Energy_PID.integral = 0.0f;
	Energy_PID.derivative = 0.0f;
	Energy_PID.energy_control_enabled = 1;//开启PID控制
}

//能量环PID控制函数:
void Energy_PIDcontrol_loop(void)
{
	if(Energy_PID.energy_control_enabled != 1)
			return;
	
	Energy_PID.target = PC.cap_elc_ok ? Energy_PID.target_cap : Energy_PID.target_nocap;
	
	// 当超电有电时:
	if(PC.cap_elc_ok == 1)
	{
			// 使用超级电容能量
			Energy_PID.actual = capEnergy / 255.0f * 2100.0f;
	}
	// 当超电没电时:
	else
	{
			// 使用裁判系统能量
			Energy_PID.actual = 60;
	}
	
	// 修改误差计算方向
	Energy_PID.error = Energy_PID.actual - Energy_PID.target;  // 实际值 - 目标值
	
	Energy_PID.integral += Energy_PID.error;
	if(Energy_PID.integral > 1000.0f)
			Energy_PID.integral = 1000.0f;
	if(Energy_PID.integral < -1000.0f)
			Energy_PID.integral = -1000.0f;
	
	Energy_PID.derivative = Energy_PID.error - Energy_PID.lasterror;
	Energy_PID.lasterror = Energy_PID.error;
	
	Energy_PID.out = Energy_PID.Kp * Energy_PID.error + 
									 Energy_PID.Ki * Energy_PID.integral + 
									 Energy_PID.Kd * Energy_PID.derivative;
	
	// 输出限幅
	if(Energy_PID.out > 30.0f) Energy_PID.out = 30.0f;
	if(Energy_PID.out < -30.0f) Energy_PID.out = -30.0f;
	
	// 应用能量环控制
	PC.Power_Limit += Energy_PID.out;
	
	// 确保功率限制在合理范围内
	if(PC.Power_Limit < 10.0f) PC.Power_Limit = 10.0f;
}
//功率上限处理:
void Chassis_PowerControlTest(float Vx,float Vy,float Vz,float Input_Energy)
{
    //由缓冲能量得到的功率控制的功率上限:
    PC.Buffer_PowerRef=1.0f/(60.0f-pro_buffer)*60;
    if(PC.Buffer_PowerRef>Mecanum_PowerControl_PowerMax)
    {
        PC.Buffer_PowerRef=Mecanum_PowerControl_PowerMax;
    }
    PC.Power_Limit=PC.Buffer_PowerRef*chassispowerlimit;

    PC.speed_magnitude=sqrt(Vx*Vx+Vy*Vy);
    PC.scale=PC.speed_magnitude/PC.Max_Translation;
		
		PC.cap_elc_ok=((capEnergy/255.0f)>0.5f);
		
		
    if(PC.scale>0.0f&&PC.scale<1.0f)
    {
			PC.Power_Limit*=PC.scale;//平移约束
    }
		
		Energy_PIDcontrol_loop();
		
    if(chassis_gryo360_move==1)
    {
			if(PC.Power_Limit>30.0f)
				PC.Power_Limit=30.0f;
        //PC.Power_Limit=PC.Buffer_PowerRef*chassispowerlimit;
//			PC.Power_Limit=50.0f;
    }
		if(chassis_following_Gimbal==1)
		{
			if(PC.Power_Limit>90.0f)
				PC.Power_Limit=90.0f;
//			PC.Power_Limit=40.0f;
		}
		if(chassis_sideway==1)
		{
			if(PC.Power_Limit>45.0f)
				PC.Power_Limit=45.0f;
//			PC.Power_Limit=40.0f;
		}

    #ifndef GO_Mode //解除功率上限，全速测试
    if(PC.Power_Limit>120.0f)
    {
        PC.Power_Limit=120.0f;
    }
    #endif
		
    if(PC.cap_elc_ok==1)
    {
        if(pitch_up_flag==1)//准备飞坡
        {
            PC.Power_Limit=100.0f;
            SCAP_Control(chassispowerlimit,60,1,0);
        }
        else//飞坡标志位置为0，默认平地模式
        {
            if(RefereeSystem_GameStatus==1&&RefereeSystem_RemainTime>0&&Chassis_Energy>0)
            {
                PC.Energy_PowerLimit=Chassis_Energy/1.0f/RefereeSystem_RemainTime;
                if(PC.Power_Limit>PC.Energy_PowerLimit)
                {
                    PC.Power_Limit=PC.Energy_PowerLimit;
                }
                //限制功率使用:
                SCAP_Control(PC.Power_Limit,60,1,0);
            }
        }
    }
    else//超电没电
    {
        if(Vx!=0||Vy!=0||Vz!=0)
        {
            SCAP_Control(PC.Power_Limit,60,1,0);
        }
        else
        {
            SCAP_Control(chassispowerlimit,60,1,0);
        }
        pitch_up_flag=0;
        if(RefereeSystem_GameStatus==1&&RefereeSystem_RemainTime>0&&Chassis_Energy>0)
        {
            PC.Energy_PowerLimit=Chassis_Energy/1.0f/RefereeSystem_RemainTime;
            if(PC.Power_Limit>PC.Energy_PowerLimit)
            {
                PC.Power_Limit=PC.Energy_PowerLimit;
            }
        }
    }

    //powercontrol：
		//当前功率模型让四轮功率不均等，没有根据实际情况进行功率调整导致底盘跟随云台时四轮功率不均等，待调整
    PC.current_power=1.732050807568877f*(Motor_Power[0]+Motor_Power[1]+Motor_Power[2]+Motor_Power[3]);//可以更换模型，采取更精确的模型
    PC.PowerSum+=PC.current_power;
    if(RefereeSystem_GameStatus==1)
    {
        Input_Energy-=PC.current_power*0.002f;
        Chassis_Energy=Input_Energy;
        if(Last_RefereeSystem_Energy!=RefereeSystem_Energy)
        {
            Chassis_Energy=RefereeSystem_Energy;
        }
        if(Chassis_Energy>RefereeSystem_Energy)
        {
            Chassis_Energy=RefereeSystem_Energy;
        }
        if(Chassis_Energy<0)
        {
            Chassis_Energy=0;
        }
        Last_RefereeSystem_Energy=RefereeSystem_Energy;
    }
    if (PC.count >= Mecanum_PowerControl_T)
    {
			PC.count = 0;
			PC.average_power = PC.PowerSum / Mecanum_PowerControl_T;
			PC.PowerSum = 0;
			//超级电容没电时的全局处理
			//更改为:仅当功率接近上限时才降低增益
			if (!PC.cap_elc_ok)
			{
				if (PC.average_power > PC.Power_Limit * 0.9f) // 阈值提高到95%
				{
					V_Gains -= 0.02f;  // 减小调整幅度
					if (V_Gains < 0.3f) // 提高最小增益限制
							V_Gains = 0.3f;
				}
				else if (PC.average_power < PC.Power_Limit * 0.7f)
				{
					// 当功率较低时允许缓慢恢复增益
					V_Gains += 0.01f;
					if (V_Gains > 1.0f)
							V_Gains = 1.0f;
				}
			}
		else
		{
			//小陀螺模式特殊处理
			if (chassis_gryo360_move == 1)
			{
				//超级电容有电
				if ((capEnergy / 255.0f) > 0.5f)
				{
					//小陀螺模式下保持速度增益有效
					if (PC.average_power < PC.Power_Limit - 5.0f)
					{
						//功率低于限制，缓慢加速
						V_Gains += 0.05f;
						if (V_Gains > 2.0f) V_Gains = 2.0f;
					}
					else if (PC.average_power > PC.Power_Limit + 5.0f)
					{
						//功率超过限制，减速
						if (60 < 60.0f - pro_buffer)
						{
								V_Gains -= 0.05f;
						}
						else
						{
								V_Gains -= 0.02f;
						}
						if (V_Gains < 0.2f) V_Gains = 0.2f;
					}
				}
				
				else
				{
					// 急速下降速度增益
					if (PC.average_power > PC.Power_Limit * 0.85f)
					{
						// 功率接近上限时急速下降
						V_Gains -= 0.3f; // 大幅降低增益
						if (V_Gains < 0.1f) V_Gains = 0.1f;
					}
					else if (PC.average_power > PC.Power_Limit)
					{
						// 功率超过上限时更大幅度下降
						V_Gains -= 0.5f;
						if (V_Gains < 0.05f) V_Gains = 0.05f;
					}
				}
			}
			//非小陀螺模式
			else
			{
				//原有增益调整逻辑
				if (PC.average_power < PC.Power_Limit / 2.0f)
				{
					V_Gains += 0.05f;
					if (V_Gains > 2) V_Gains = 2;
				}
				else if (PC.average_power < PC.Power_Limit - 5)
				{
					V_Gains += 0.02f;
					if (V_Gains > 2) V_Gains = 2;
				}
				else if (PC.average_power > PC.Power_Limit + 5)
				{
					if (60 < 60.0f - pro_buffer)
							V_Gains -= 0.05f;
					else
							V_Gains -= 0.02f;
					if (V_Gains < Mecanum_PowerControlGainCoefficientInitialValue / 2.0f)
							V_Gains = Mecanum_PowerControlGainCoefficientInitialValue / 2.0f;
				}
			}
		}
	}
    else if(Vx!=0||Vy!=0||chassis_gryo360_move==1||chassis_following_Gimbal==1)
    {
        PC.count++;
    }
    else
    {
        V_Gains=Mecanum_PowerControlGainCoefficientInitialValue;
    }
		
    if(chassis_gryo360_move==1&&(Vx!=0||Vy!=0))
    {
			  Vx*=1;//当前全局变量作为速度缩放的话很浪费功率，要改为数组对每个电机的功率进行处理
        Vy*=1;
        Vz*=1;
    }
		if (!PC.cap_elc_ok) //超级电容没电
    {
			//额外限制最大速度
			float max_speed = 800.0f;
			if (fabs(Vx) > max_speed) Vx = (Vx > 0) ? max_speed : -max_speed;
			if (fabs(Vy) > max_speed) Vy = (Vy > 0) ? max_speed : -max_speed;
			if (fabs(Vz) > max_speed) Vz = (Vz > 0) ? max_speed : -max_speed;
    }
    Vx*=V_Gains;
    Vy*=V_Gains;
    if(chassis_gryo360_move==1||chassis_following_Gimbal==1)
    {
			Vz*=V_Gains;
    }
}

//能量环控制使能函数
void Enable_EnergyLoopControl(uint8_t enable)
{
	Energy_PID.energy_control_enabled = enable;
	if (!enable) 
	{
		//禁用时重置积分项
		Energy_PID.integral=0;
		Energy_PID.lasterror=0;
	}
}

void Set_EnergyLoopTarget(float target)
{
	Energy_PID.target = target;
}

void Set_EnergyLoopPID(float kp, float ki, float kd)
{
	Energy_PID.Kp = kp;
	Energy_PID.Ki = ki;
	Energy_PID.Kd = kd;
}

