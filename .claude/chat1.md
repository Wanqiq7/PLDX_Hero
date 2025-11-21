# 底盘功率控制调试记录

**时间**: 2025-11-19
**主题**: 底盘功率控制量纲修正、RLS参数拟合、功率预测调试

---

## 1. 功率控制量纲修正

### 问题发现
- 原代码混用转子角速度（`speed_aps * DEGREE_2_RAD`）和输出轴转矩常数（0.3 N·m/A）
- 导致功率计算被高估约19倍（减速比）

### 修改方案
**统一到输出轴侧（与力控底盘一致）**

#### 修改1: chassis.c:708-732 功率反馈计算
```c
// 修改前
float motor_speeds[4] = {
    motor_lf->measure.speed_aps * DEGREE_2_RAD,  // 转子角速度
    // ...
};

// 修改后
float motor_speeds[4] = {
    motor_lf->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL,  // 输出轴角速度
    // ...
};
```

#### 修改2: chassis.c:737-759 PowerMotorObj_t的target_av
```c
// 修改前
.target_av = target_wheel_omega[0],  // 转子角速度

// 修改后
.target_av = target_wheel_omega[0] / REDUCTION_RATIO_WHEEL,  // 输出轴角速度
```

### 验证结果
用户反馈："现象基本正常了"

---

## 2. RLS参数拟合流程

### 关键问题解答

**Q: RLS只是拟合工具，为什么不用Matlab？**
A: RLS有两种用法：
- **离线拟合**（一次性）：确实和Matlab无区别，Matlab甚至更好
- **在线自适应**（持续运行）⭐推荐：实时跟踪参数变化，自动适应环境
  - 电池电压下降 → k2变化
  - 电机温度上升 → k1增加10-30%
  - 场地地面变化 → k1变化
  - 电机磨损老化 → k1持续增加

**推荐做法**：
1. 阶段1：离线拟合初值（Matlab或RLS）
2. 阶段2：保持RLS开启，持续跟踪变化（lambda=0.995-0.999）

### RLS控制机制

**两个变量的关系**：
- `RLS_ENABLE`（power_controller.h:21）：编译时宏定义，初始化时被复制
- `controller_status.rls_enabled`：**运行时真正起作用的变量**

**运行时控制**：Ozone中直接修改 `controller_status.rls_enabled`
- = 1 → RLS工作
- = 0 → RLS停止

### k1/k2上下限设置

**位置**: `Chassis\modules\algorithm\controller.c:275-282`

```c
// k1限制
if (rls->params_vector[0] < 0.01f)  // 下限 0.01
    rls->params_vector[0] = 0.01f;
if (rls->params_vector[0] > 1.0f)   // 上限 1.0
    rls->params_vector[0] = 1.0f;

// k2限制
if (rls->params_vector[1] < 0.001f)  // 下限 0.001
    rls->params_vector[1] = 0.001f;
if (rls->params_vector[1] > 15.0f)   // 上限 15.0
    rls->params_vector[1] = 15.0f;
```

---

## 3. 功率预测vs实际功率

### 能量环PD vs 功率控制

**澄清误解**：能量环PD ≠ 功率限制机制

- **能量环PD作用**：根据能量缓冲/电容电压，动态调整功率上下限（保护能量缓冲）
- **功率限制机制**：基于 `predicted_power` 判断是否超限

### 超功率原因

**核心问题**：`predicted_power` ≠ `chassis_power`

原因：
1. k1、k2、k3参数不准确 → 预测功率偏小
2. 裁判系统测的是总功率（含主控板、IMU、CAN等非电机功耗5-15W）

### Ozone监控变量

**核心对比**：
- `controller_status.estimated_power` - 预测功率（代码计算）
- `referee_data.chassis_power` - 实际功率（裁判系统/电容测量）

**其他变量**：
- `power_k1`, `power_k2`, `power_k3` - 模型参数
- `power_limit_state.power_upper/lower` - 功率上下限
- `controller_status.max_power_limit` - 当前功率上限

---

## 4. 预测功率波形分析

### 观察到的问题

**波形特征**：
- 静态时：estimated_power ≈ 0W，chassis_power ≈ 5-10W
- 动态峰值1：estimated ≈ 30-40W，chassis ≈ 150W（误差75%）
- 动态峰值2：estimated ≈ 40-50W，chassis ≈ 180W（误差78%）

**诊断结果**：k1、k2参数远小于真实值（预测只有实际的1/4）

### 原因分析

1. **初始值太差**：k1_init、k2_init设置太小，RLS从错误起点开始
2. **上下限太严**：k1上限1.0、k2上限1.5，RLS被限幅卡住
3. **收敛条件不好**：机器人速度太慢、功率太小，RLS需要高速大功率数据

### 解决建议

1. 放宽k1/k2上限到10.0，让RLS充分收敛
2. 高速大范围移动机器人进行RLS拟合
3. 调整k3参数补偿其他模块功耗

---

## 5. 负功率预测问题

### 现象
estimated_power无法预测负功率（制动时chassis_power = -7.4W，但estimated ≈ 0W）

### 原因分析

**功率模型**：
```c
P = τ·ω + k1·|ω| + k2·τ² + k3/4
     ↑      ↑       ↑      ↑
   可负   永远≥0  永远≥0  永远≥0
```

即使 `τ·ω < 0`（制动），后三项损耗会"填平"负功率。

### 是否需要修复？

**不需要**，因为：
1. 功率控制只关心正功率（有负功率回收处理）
2. 负功率场景很少（只有急刹车）
3. 负功率时模型不准也不影响功率控制效果

---

## 6. 静态功耗变化问题（5W → 15W）

### 现象
- 刚开机静止：5W
- 行驶后停止：15W

### 根本原因：摩擦补偿持续输出

**代码位置**：`chassis.c:527-536`

```c
static inline float CalculateFrictionCompensation(float target_omega) {
  float normalized_speed = target_omega / friction_threshold_omega;
  float smooth_factor = tanhf(normalized_speed);

  return FRICTION_DYNAMIC_CURRENT +  // ← 0.5A 基础输出
         smooth_factor * (FRICTION_STATIC_CURRENT - FRICTION_DYNAMIC_CURRENT);
}
```

**停止时（target_omega = 0）**：
- smooth_factor = tanh(0) = 0
- friction_comp = **0.5A**（每个电机仍然输出）

### 功耗计算

**刚开机**（5W）：
- 主控板 + IMU + CAN ≈ 5W
- 电机：无补偿，电流 ≈ 0A

**行驶后停止**（15W）：
- 主控板等：5W
- 4个电机摩擦补偿：4 × 0.5A = 2A
  - 铜损：I²R ≈ 2W
  - 电调损耗：≈ 3W
  - 空载损耗：≈ 2W
- 云台/发射机构：≈ 3W
- **总计**：≈ 15W

### 解决方案

**方案1**（推荐）：修正k3参数
```c
#define k3_init 15.0f  // 改成实际测量值
```

**方案2**：优化摩擦补偿逻辑
```c
if (fabsf(target_omega) < 2.0f) {
    return 0.0f;  // 低速时完全禁用
}
```

---

## 7. 当前问题：右前轮不转

### 代码修改汇总

我修改过的位置：

1. **chassis.c:708-732**：motor_speeds计算（除以REDUCTION_RATIO_WHEEL）
2. **chassis.c:737-759**：PowerMotorObj_t的target_av（除以REDUCTION_RATIO_WHEEL）

### 可能原因

- 索引1（motor_rf）的数据计算错误
- 功率控制器对motor_objs[1]限幅过度

### 待调试

需要检查：
1. motor_objs[1]的pid_output、current_av、target_av值
2. PowerGetLimitedOutput对索引1的处理
3. limited_output[1]是否被过度限幅

---

## 附录：关键文件位置

- **功率控制器**：`Chassis/modules/power_controller/power_controller.c`
- **底盘控制**：`Chassis/application/chassis/chassis.c`
- **RLS算法**：`Chassis/modules/algorithm/controller.c`
- **参数定义**：`Chassis/application/robot_def.h`
- **功率控制配置**：`Chassis/modules/power_controller/power_controller.h`

---

## 重要参数

- **M3508转矩常数**：0.3 N·m/A（输出轴）
- **减速比**：REDUCTION_RATIO_WHEEL = 19.0f
- **摩擦补偿**：FRICTION_STATIC_CURRENT = 0.75f, FRICTION_DYNAMIC_CURRENT = 0.5f
- **功率模型**：P = τ·ω + k1·|ω| + k2·τ² + k3/4
