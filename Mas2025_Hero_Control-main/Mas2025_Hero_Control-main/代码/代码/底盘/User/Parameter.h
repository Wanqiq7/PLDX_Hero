#ifndef __PARAMETER_H
#define __PARAMETER_H

/*=============================================结构参数=============================================*/
#define Yaw_GM6020PositionValue								5638//Yaw轴编码器值

#define M3508_ReductionRatio								268.0f/17.0f

#define GM6020_1Zero										7849//0x1EA9
#define GM6020_2Zero										371//0x173
#define GM6020_3Zero										7866//0x1EBA
#define GM6020_4Zero										5775//0x168F

/*=============================================麦轮参数=============================================*/
#define Mecanum_WheelRadius									7.0f//麦轮半径(单位cm)

#define Mecanum_rx											18.75f//底盘中心到轮子中心的距离的x轴分量(单位cm)
#define Mecanum_ry											18.0f//底盘中心到轮子中心的距离的y轴分量(单位cm)

#define Mecanum_GyroScopeAngularVelocity					5.0f//小陀螺角速度
/*=============================================底盘参数=============================================*/
#define Chassis_rx											0.1978f//底盘中心到轮子中心的距离的x轴分量(单位m)//395.6
#define Chassis_ry											0.1978f//底盘中心到轮子中心的距离的y轴分量(单位m)

#define Chassis_WheelRadius									0.06f//麦轮半径(单位m)

#define Chassis_GyroScopeAngularVelocity					5.0f//小陀螺角速度

#endif
