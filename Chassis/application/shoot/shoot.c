#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_lf, *friction_rf, *friction_lb, *friction_rb, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

// 摩擦轮电流前馈（克服静摩擦/负载），按轮分别注入
static float fric_ff_cur_lf = 0.0f;
static float fric_ff_cur_rf = 0.0f;
static float fric_ff_cur_lb = 0.0f;
static float fric_ff_cur_rb = 0.0f;

// 前馈方向常量：在初始化阶段根据电机反转标志决定（+1 或 -1）
static float fric_ff_dir_lf = 1.0f;
static float fric_ff_dir_rf = 1.0f;
static float fric_ff_dir_lb = 1.0f;
static float fric_ff_dir_rb = 1.0f;

// 占位：电流前馈幅值（CAN 指令单位），需联调后修正
#define FRICTION_CURRENT_FF_CMD 800.0f // TODO: 实测标定

void ShootInit() // 已适配四摩擦轮发射机构
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_feedforward_ptr = &fric_ff_cur_lf,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    // 左边摩擦轮
    friction_config.can_init_config.tx_id = 1,
    friction_lf = DJIMotorInit(&friction_config);
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_config.controller_param_init_config.current_feedforward_ptr = &fric_ff_cur_rf;
    friction_config.can_init_config.tx_id = 2;
    friction_rf = DJIMotorInit(&friction_config);
    // 第一排左右
    friction_config.can_init_config.can_handle = &hcan1;
    friction_config.can_init_config.tx_id = 8,
    friction_config.controller_param_init_config.current_feedforward_ptr = &fric_ff_cur_rb;
    friction_rb = DJIMotorInit(&friction_config);
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config.controller_param_init_config.current_feedforward_ptr = &fric_ff_cur_lb;
    friction_config.can_init_config.tx_id = 7;
    friction_lb = DJIMotorInit(&friction_config);

    // 基于电机反转标志，确定每个轮子的前馈方向常量
    fric_ff_dir_lf = (friction_lf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
    fric_ff_dir_rf = (friction_rf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
    fric_ff_dir_lb = (friction_lb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
    fric_ff_dir_rb = (friction_rb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 0, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 0, // 10
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_lf);
        DJIMotorStop(friction_lb);
        DJIMotorStop(friction_rf);
        DJIMotorStop(friction_rb);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_lf);
        DJIMotorEnable(friction_lb);
        DJIMotorEnable(friction_rf);
        DJIMotorEnable(friction_rb);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                               // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                        // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                        // 记录触发指令的时间
        dead_time = 150;                                                              // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                            // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                            // 记录触发指令的时间
        dead_time = 300;                                                                  // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 启动前馈：按方向给定恒定电流前馈，提升起转与稳速能力
        // 注意：方向由参考速度决定，若参考为0则不加前馈
        float dir_lf = 0.0f, dir_lb = 0.0f, dir_rf = 0.0f, dir_rb = 0.0f;
        // 根据目标弹速分档设置目标速度（保持原有逻辑），并给出方向
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_lf, 0);
            DJIMotorSetRef(friction_lb, 0);
            DJIMotorSetRef(friction_rb, 0);
            DJIMotorSetRef(friction_rf, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_lf, 0);
            DJIMotorSetRef(friction_lb, 0);
            DJIMotorSetRef(friction_rb, 0);
            DJIMotorSetRef(friction_rf, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_lf, 0);
            DJIMotorSetRef(friction_lb, 0);
            DJIMotorSetRef(friction_rb, 0);
            DJIMotorSetRef(friction_rf, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_lf, 3000);
            DJIMotorSetRef(friction_lb, 3000);
            DJIMotorSetRef(friction_rb, 3000);
            DJIMotorSetRef(friction_rf, 3000);
            dir_lf = dir_lb = dir_rf = dir_rb = 1.0f; // 统一正向
            break;
        }

        // 写入电流前馈（幅值固定，方向在初始化中由 motor_reverse_flag 预处理）
        fric_ff_cur_lf = fric_ff_dir_lf * FRICTION_CURRENT_FF_CMD;
        fric_ff_cur_lb = fric_ff_dir_lb * FRICTION_CURRENT_FF_CMD;
        fric_ff_cur_rf = fric_ff_dir_rf * FRICTION_CURRENT_FF_CMD;
        fric_ff_cur_rb = fric_ff_dir_rb * FRICTION_CURRENT_FF_CMD;
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_lf, 0);
        DJIMotorSetRef(friction_lb, 0);
        DJIMotorSetRef(friction_rb, 0);
        DJIMotorSetRef(friction_rf, 0);
        // 关闭时清零前馈
        fric_ff_cur_lf = fric_ff_cur_lb = fric_ff_cur_rf = fric_ff_cur_rb = 0.0f;
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}