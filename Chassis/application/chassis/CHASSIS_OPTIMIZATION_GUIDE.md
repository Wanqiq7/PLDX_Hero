# Chassis.c 优化指南 - 参数映射说明

本文档详细说明了 `chassis.c` 优化后的参数配置变化，帮助您快速定位和调整参数。

---

## 📋 优化概述

### 主要改进
1. **参数结构化**：所有硬编码参数移至 `chassis_config` 结构体
2. **函数模块化**：大函数拆分为职责单一的小函数
3. **性能优化**：使用 `static const` 和 `inline` 优化性能
4. **代码可读性**：添加详细注释和步骤说明

---

## 🔧 参数映射表

### 1. 遥控器速度增益参数

| 原代码位置 | 原代码 | 新代码位置 | 新代码 | 说明 |
|-----------|--------|------------|--------|------|
| chassis.c:88-89 | `RC_CMD_MAX_LINEAR_SPEED = 2.0f` | chassis_config.rc.max_linear_speed | `2.0f` | m/s - 最大线速度 |
| chassis.c:89 | `RC_CMD_MAX_ANGULAR_SPEED = 2.5f` | chassis_config.rc.max_angular_speed | `2.5f` | rad/s - 最大角速度 |

**使用位置**：
```c
// 原代码：
chassis_cmd_recv.vx *= RC_CMD_MAX_LINEAR_SPEED;

// 新代码：
chassis_cmd_recv.vx *= chassis_config.rc.max_linear_speed;
```

---

### 2. 力控策略参数

| 原代码位置 | 原代码 | 新代码位置 | 新代码 | 说明 |
|-----------|--------|------------|--------|------|
| chassis.c:438 | `TORQUE_FEEDFORWARD_COEFF = 7.5f` | chassis_config.force.torque_feedforward_coeff | `7.5f` | N·m/(rad/s) - 扭矩前馈系数 |
| robot_def.h:117 | `FRICTION_THRESHOLD_OMEGA = 8.0f` | chassis_config.force.friction_threshold_omega | `8.0f` | rad/s - 摩擦补偿速度阈值 |
| robot_def.h:132 | `WHEEL_SPEED_FEEDBACK_COEFF = 0.08f` | chassis_config.force.wheel_speed_feedback_coeff | `0.08f` | A·s/rad - 轮速反馈系数 |
| chassis.c:528 | `OMEGA_ERROR_LPF_ALPHA = 0.85f` | chassis_config.force.omega_error_lpf_alpha | `0.85f` | 角速度误差滤波系数 |
| chassis.c:533 | `OMEGA_THRESHOLD = 15.0f` | chassis_config.force.omega_threshold | `15.0f` | rad/s - 过零保护角速度阈值 |

---

### 3. 运动学参数

| 原代码位置 | 原代码 | 新代码位置 | 新代码 | 说明 |
|-----------|--------|------------|--------|------|
| chassis.c:413 | `LPF_ALPHA = 0.85f` | chassis_config.kinematics.velocity_lpf_alpha | `0.85f` | 速度估算滤波系数 |
| chassis.c:85 | `SPEED_DEADBAND_THRESHOLD = 120.0f` | chassis_config.kinematics.speed_deadband | `120.0f` | deg/s - 速度死区阈值 |
| chassis.c:655 | `0.80f` (硬编码) | chassis_config.kinematics.follow_lpf_alpha | `0.80f` | 跟随模式滤波系数 |
| chassis.c:661 | `2.5f` (硬编码) | chassis_config.kinematics.rotate_speed | `2.5f` | rad/s - 小陀螺模式旋转速度 |

---

## 📝 配置结构体定义位置

### robot_def.h 中的定义 (第134-171行)
```c
typedef struct {
    float max_linear_speed;     // m/s - 最大线速度
    float max_angular_speed;    // rad/s - 最大角速度
} Chassis_RC_Config_t;

typedef struct {
    float torque_feedforward_coeff;     // N·m/(rad/s) - 扭矩前馈系数
    float friction_threshold_omega;     // rad/s - 摩擦补偿速度阈值
    float wheel_speed_feedback_coeff;   // A·s/rad - 轮速反馈系数
    float omega_error_lpf_alpha;        // 角速度误差滤波系数
    float omega_threshold;              // rad/s - 过零保护角速度阈值
} Chassis_Force_Control_Config_t;

typedef struct {
    float velocity_lpf_alpha;    // 速度估算滤波系数
    float speed_deadband;        // 速度死区阈值 (deg/s)
    float follow_lpf_alpha;      // 跟随模式滤波系数
    float rotate_speed;          // 小陀螺模式旋转速度 (rad/s)
} Chassis_Kinematics_Config_t;
```

### chassis.c 中的配置实例 (第88-107行)
```c
static const Chassis_Runtime_Config_t chassis_config = {
    .rc = {
        .max_linear_speed = 2.0f,      // m/s - 最大线速度
        .max_angular_speed = 2.5f      // rad/s - 最大角速度
    },
    .force = {
        .torque_feedforward_coeff = 7.5f,       // N·m/(rad/s) - 扭矩前馈系数
        .friction_threshold_omega = 8.0f,       // rad/s - 摩擦补偿速度阈值
        .wheel_speed_feedback_coeff = 0.08f,     // A·s/rad - 轮速反馈系数
        .omega_error_lpf_alpha = 0.85f,          // 角速度误差滤波系数
        .omega_threshold = 15.0f                 // rad/s - 过零保护角速度阈值
    },
    .kinematics = {
        .velocity_lpf_alpha = 0.85f,      // 速度估算滤波系数
        .speed_deadband = 120.0f,         // 速度死区阈值 (deg/s)
        .follow_lpf_alpha = 0.80f,        // 跟随模式滤波系数
        .rotate_speed = 2.5f              // 小陀螺模式旋转速度 (rad/s)
    }
};
```

---

## 🎯 调参快速指南

### 1. 调整遥控器响应速度
```c
// 在 chassis.c 第88-107行修改：
.rc = {
    .max_linear_speed = 3.0f,      // 增大提高线速度响应
    .max_angular_speed = 3.0f      // 增大提高角速度响应
}
```

### 2. 调整力控前馈强度
```c
// 在 chassis.c 第94-101行修改：
.force = {
    .torque_feedforward_coeff = 10.0f,    // 增大提高旋转响应
    // ...其他参数保持不变
}
```

### 3. 调整速度滤波平滑度
```c
// 在 chassis.c 第102-107行修改：
.kinematics = {
    .velocity_lpf_alpha = 0.90f,    // 增大（接近1）更平滑
    .follow_lpf_alpha = 0.85f,      // 增大（接近1）跟随更平滑
    // ...其他参数保持不变
}
```

### 4. 调整小陀螺模式转速
```c
// 在 chassis.c 第102-107行修改：
.kinematics = {
    .rotate_speed = 3.0f,            // 修改小陀螺转速 (rad/s)
    // ...其他参数保持不变
}
```

### 5. 调整摩擦补偿
```c
// 在 robot_def.h 第113-119行修改：
#define FRICTION_STATIC_CURRENT 0.6f      // 增大静摩擦补偿
#define FRICTION_DYNAMIC_CURRENT 0.4f     // 增大动摩擦补偿
#define FRICTION_THRESHOLD_OMEGA 10.0f   // 调整摩擦补偿生效速度阈值
```

---

## 🔄 函数调用变化

### 力控函数拆分
原函数：`ForceToCurrentConversion()` (50行)
拆分为：
- `CalculateSpeedFeedback()` - 速度反馈补偿
- `CalculateFrictionCompensation()` - 摩擦补偿（inline优化）
- 主函数更清晰，步骤化处理

### 速度控制优化
原函数：`VelocityToForceControl()`
新增：`CalculateTorqueFeedforward()` - 前馈补偿计算（inline优化）

### 速度估算优化
原函数：`EstimateChassisVelocity()`
- 使用预计算优化性能
- 添加详细算法说明
- 优化注释结构

---

## ⚡ 性能优化点

1. **预计算优化**：
   ```c
   static const float rotation_radius = sqrtf(...);  // 只计算一次
   const float inv_4 = 0.25f;                       // 避免除法
   ```

2. **内联函数**：
   ```c
   static inline float CalculateTorqueFeedforward(float target_wz);
   static inline float CalculateFrictionCompensation(float target_omega);
   ```

3. **静态变量**：
   ```c
   static float filtered_vx = 0.0f;  // 避免重复初始化
   ```

---

## 💡 调参建议

### 初学者调参顺序
1. 先调整 `max_linear_speed` 和 `max_angular_speed` 获得合适的响应速度
2. 再调整 `torque_feedforward_coeff` 改善旋转响应
3. 最后调整滤波系数 `*_lpf_alpha` 平滑运动

### 高级调参
1. 调整 `wheel_speed_feedback_coeff` 改善速度环稳定性
2. 调整 `omega_threshold` 优化过零行为
3. 调整摩擦补偿参数改善低速性能

### 注意事项
- 所有参数都在 `chassis_config` 结构体中集中管理
- 使用 `const` 修饰的参数不能在运行时修改
- 如需运行时调参，考虑移除 `const` 关键字
- 滤波系数范围：0.5-0.95，越接近1越平滑但响应越慢