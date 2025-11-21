# 视觉模块架构重构记录

**日期**: 2025-11-20
**任务**: 视觉模块解耦重构，遵循项目三层架构规范

---

## 一、问题分析

### 1.1 用户反馈
> "Vision_Control_Mode_e为什么会存在于这里？现在的视觉模块的应用我还是觉得太冗杂了，解耦度不足"

### 1.2 架构问题清单

| 严重度 | 问题 | 说明 |
|--------|------|------|
| ⚠️⚠️⚠️ | 缺少独立视觉应用模块 | 视觉逻辑散落在 robot_cmd.c，违反三层架构 |
| ⚠️⚠️ | 模式枚举定义位置错误 | `Vision_Control_Mode_e` 应在 `robot_def.h` |
| ⚠️⚠️ | VisionControlSet() 职责过重 | 同时控制云台角度、模式、摩擦轮、拨盘 |
| ⚠️⚠️ | 耦合过紧 | 直接修改 gimbal/shoot 指令，未通过 message center |
| ⚠️ | 控制优先级隐式表达 | 调用顺序决定优先级，逻辑不透明 |

### 1.3 与项目标准对比

| 应用 | 独立目录 | Init/Task 函数 | 消息中心通信 |
|------|----------|----------------|--------------|
| 底盘 | `chassis/` | ✅ | ✅ |
| 云台 | `gimbal/` | ✅ | ✅ |
| 发射 | `shoot/` | ✅ | ✅ |
| **视觉(重构前)** | **无** | ❌ | ❌ |

---

## 二、重构方案

### 2.1 目标架构

```
Gimbal/application/
├── vision/           # 新增：视觉控制应用
│   ├── vision.c      # 视觉控制逻辑实现
│   └── vision.h      # 公开接口（Init/Task）
├── cmd/
│   └── robot_cmd.c   # 简化：只负责指令分发和数据融合
├── gimbal/
├── shoot/
└── robot_def.h       # 添加 vision_mode_e 和相关结构体
```

### 2.2 数据流设计

```
robot_cmd.c                 vision.c                    gimbal/shoot
    │                           │                           │
    │ Vision_Ctrl_Cmd_s         │                           │
    │ (vision_mode,             │                           │
    │  allow_auto_fire)         │                           │
    ├──────────────────────────►│                           │
    │                           │ ProcessAutoAim()          │
    │                           │ ProcessEnergyHit()        │
    │ Vision_Upload_Data_s      │                           │
    │ (yaw, pitch,              │                           │
    │  should_fire)             │                           │
    │◄──────────────────────────┤                           │
    │                           │                           │
    │ (数据融合后)               │                           │
    │ gimbal_cmd_send           │                           │
    │ shoot_cmd_send            │                           │
    └───────────────────────────┼──────────────────────────►│
```

---

## 三、具体修改

### 3.1 新增文件

#### `application/vision/vision.h`
```c
#ifndef VISION_APP_H
#define VISION_APP_H

void VisionAppInit(void);
void VisionAppTask(void);

#endif // VISION_APP_H
```

#### `application/vision/vision.c`
- 约145行代码
- 订阅 `vision_cmd`（来自 robot_cmd）
- 发布 `vision_data`（处理后的数据）
- 包含 `ProcessAutoAim()`、`ProcessEnergyHit()`、`ProcessManualAim()` 内部函数
- 在线状态检测和日志记录

### 3.2 修改 `robot_def.h`

**添加视觉控制模式枚举**：
```c
typedef enum {
  VISION_MODE_OFF = 0,    // 视觉关闭
  VISION_MODE_AUTO_AIM,   // 自动瞄准
  VISION_MODE_ENERGY_HIT, // 能量机关
  VISION_MODE_MANUAL_AIM, // 手动辅助瞄准
} vision_mode_e;
```

**添加视觉控制指令结构体**：
```c
typedef struct {
  vision_mode_e vision_mode;     // 视觉控制模式
  uint8_t allow_auto_fire;       // 允许自动射击
  float manual_yaw_offset;       // 手动微调yaw偏移量
  float manual_pitch_offset;     // 手动微调pitch偏移量
} Vision_Ctrl_Cmd_s;
```

**添加视觉上传数据结构体**：
```c
typedef struct {
  uint8_t vision_valid;   // 视觉数据有效标志
  uint8_t target_locked;  // 目标锁定标志
  float yaw;              // 目标yaw角度(弧度)
  float pitch;            // 目标pitch角度(角度)
  uint8_t should_fire;    // 建议射击标志
} Vision_Upload_Data_s;
```

### 3.3 简化 `robot_cmd.c`

**删除内容**（约120行）：
- `Vision_Control_Mode_e` 相关私有变量
- `VisionSetMode()` 函数
- `VisionGetMode()` 函数
- `VisionControlSet()` 函数（约90行）

**添加内容**：
```c
// 视觉控制消息(通过message center与vision应用通信)
static Publisher_t *vision_cmd_pub;           // 视觉控制指令发布者
static Subscriber_t *vision_data_sub;         // 视觉处理数据订阅者
static Vision_Ctrl_Cmd_s vision_cmd_send;     // 发送给视觉应用的控制指令
static Vision_Upload_Data_s vision_data_recv; // 从视觉应用接收的处理数据
```

**RobotCMDTask 中的数据融合逻辑**（约15行）：
```c
// 2️⃣ 视觉数据融合（当视觉有效时覆盖云台和发射控制）
if (vision_cmd_send.vision_mode != VISION_MODE_OFF && vision_data_recv.vision_valid) {
  gimbal_cmd_send.yaw = vision_data_recv.yaw;
  gimbal_cmd_send.pitch = vision_data_recv.pitch;
  LIMIT_PITCH_ANGLE(gimbal_cmd_send.pitch);

  if (vision_data_recv.should_fire) {
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.load_mode = LOAD_1_BULLET;
  }
}
```

### 3.4 修改 `robot_cmd.h`

删除所有视觉相关导出，只保留：
```c
void RobotCMDInit(void);
void RobotCMDTask(void);
```

### 3.5 修改 `robot.c`

```c
#include "vision.h"

void RobotInit() {
    // ...
    VisionAppInit();  // 视觉应用初始化(在cmd之前)
    RobotCMDInit();
    GimbalInit();
    ShootInit();
    // ...
}

void RobotTask() {
    VisionAppTask();  // 视觉数据处理(在cmd之前执行)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
    // ...
}
```

### 3.6 修改 `Makefile`

**C_SOURCES 添加**：
```makefile
application/vision/vision.c \
```

**C_INCLUDES 添加**：
```makefile
-Iapplication/vision \
```

---

## 四、编译结果

| 指标 | 重构前 | 重构后 | 变化 |
|------|--------|--------|------|
| FLASH | 87564 B | 87980 B | +416 B (+0.47%) |
| RAM | - | 77784 B | - |

编译成功 ✅

---

## 五、重构收益

| 方面 | 改进效果 |
|------|----------|
| **代码组织** | 视觉控制逻辑独立成模块，清晰可维护 |
| **架构一致性** | 遵循项目三层架构和消息中心设计 |
| **可扩展性** | 新增视觉模式只需修改 vision.c |
| **可测试性** | 视觉模块可独立测试 |
| **解耦程度** | robot_cmd 恢复为纯指令分发和数据融合 |
| **复用性** | Vision_Upload_Data_s 可被多个应用订阅 |

---

## 六、后续工作

1. **绑定控制接口**：将 `vision_cmd_send.vision_mode` 绑定到遥控器拨杆或键盘按键
2. **实现预留模式**：完善 `ProcessEnergyHit()` 和 `ProcessManualAim()` 逻辑
3. **参数调优**：根据实际测试调整视觉数据处理参数
4. **状态反馈**：添加 LED/UI 显示当前视觉模式

---

## 七、文件变更清单

### 新增
- `Gimbal/application/vision/vision.h`
- `Gimbal/application/vision/vision.c`

### 修改
- `Gimbal/application/robot_def.h`
- `Gimbal/application/cmd/robot_cmd.c`
- `Gimbal/application/cmd/robot_cmd.h`
- `Gimbal/application/robot.c`
- `Gimbal/Makefile`
