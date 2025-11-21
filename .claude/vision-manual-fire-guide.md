# 视觉锁定云台 + 鼠标手动发弹 实现指南

## 📋 背景说明

当前代码已实现**视觉驱动发射**模式：
- 视觉接管云台 yaw/pitch
- 视觉检测到目标锁定（READY_TO_FIRE）时自动触发射击
- `allow_auto_fire` 控制视觉是否有权自动射击

**目标实现**：
- 视觉只负责云台瞄准（yaw/pitch）
- 操作手通过鼠标/遥控器手动控制射击时机
- 视觉不参与射击决策

---

## 🎯 核心修改思路

### 当前逻辑（视觉驱动）
```
遥控器上档 → 视觉接管云台 → 视觉检测READY_TO_FIRE → 自动射击
```

### 目标逻辑（手动发弹）
```
遥控器上档 → 视觉接管云台 → 操作手手动触发 → 射击
```

---

## 🔧 具体修改步骤

### 步骤1：修改 VisionControlSet() - 只接管云台

**文件位置**：`Gimbal/application/cmd/robot_cmd.c` 第453-470行

**修改前**（当前代码）：
```c
static void VisionControlSet() {
  if (vision_cmd_send.vision_mode != VISION_MODE_OFF &&
      vision_data_recv.vision_valid) {
    // 接管云台
    gimbal_cmd_send.yaw = vision_data_recv.yaw;
    gimbal_cmd_send.pitch = vision_data_recv.pitch;
    LIMIT_PITCH_ANGLE(gimbal_cmd_send.pitch);

    // ⭐ 视觉驱动发射
    if (vision_data_recv.should_fire) {
      shoot_cmd_send.load_mode = LOAD_1_BULLET;
    } else {
      shoot_cmd_send.load_mode = LOAD_STOP;
    }
  }
}
```

**修改后**（手动发弹模式）：
```c
static void VisionControlSet() {
  if (vision_cmd_send.vision_mode != VISION_MODE_OFF &&
      vision_data_recv.vision_valid) {
    // 接管云台（仅控制瞄准）
    gimbal_cmd_send.yaw = vision_data_recv.yaw;
    gimbal_cmd_send.pitch = vision_data_recv.pitch;
    LIMIT_PITCH_ANGLE(gimbal_cmd_send.pitch);

    // ⭐ 删除发射控制部分，视觉不参与射击决策
    // 发射由鼠标/遥控器手动触发
  }
}
```

---

### 步骤2：在 MouseKeySet() 中添加手动发射逻辑

**文件位置**：`Gimbal/application/cmd/robot_cmd.c` 第324行附近

**在 MouseKeySet() 函数末尾添加**：

```c
/**
 * @brief 输入为键鼠时模式和控制量设置
 */
static void MouseKeySet() {
    // ... 现有的键鼠控制代码 ...

    // ======== 以下为新增部分 ========

    // 🎯 自瞄模式下的手动射击逻辑
    if (vision_cmd_send.vision_mode == VISION_MODE_AUTO_AIM) {
        // 检查前置条件：视觉在线 + 摩擦轮转速达标
        uint8_t vision_online = (vision_data_recv.vision_valid == 1);
        uint8_t friction_ready = (shoot_cmd_send.friction_mode == FRICTION_ON);

        // 可选：检查视觉是否锁定目标（更精确的射击时机）
        uint8_t target_locked = (vision_data_recv.target_locked == 1);

        if (vision_online && friction_ready) {
            // 鼠标左键单击 - 单发模式
            if (rc_data[TEMP].mouse.press_l) {
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
                shoot_cmd_send.shoot_rate = 1.0f; // 1Hz射频
            }
            // 鼠标左键长按 - 可选：连发模式
            else if (rc_data[TEMP].mouse.press_l &&
                     /* 长按判断逻辑 */ false) {
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            }
            else {
                shoot_cmd_send.load_mode = LOAD_STOP;
            }
        } else {
            // 前置条件未满足，停止射击
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    }
}
```

---

### 步骤3（可选）：在 RemoteControlSet() 中添加遥控器手动发射

**文件位置**：`Gimbal/application/cmd/robot_cmd.c` 第306-316行

**修改遥控器上档逻辑**：

```c
else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    // 上档：自瞄模式（手动发弹版本）
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.load_mode = LOAD_STOP; // 默认停止，等待手动触发
    vision_cmd_send.vision_mode = VISION_MODE_AUTO_AIM;
    vision_cmd_send.allow_auto_fire = 0; // ⭐ 禁用自动射击

    // 🎯 遥控器拨轮手动触发单发（可选）
    // 这里可以根据遥控器的拨轮或按键状态判断是否发射
    // 示例：检测遥控器某个通道的变化
    // if (rc_data[TEMP].rc.rocker_r1 > 600) {  // 假设右摇杆上推
    //     shoot_cmd_send.load_mode = LOAD_1_BULLET;
    // }
}
```

---

## 📊 对比总结

| 特性 | 视觉驱动发射（当前） | 手动发弹（目标） |
|------|-------------------|----------------|
| 云台控制 | 视觉接管 yaw/pitch | 视觉接管 yaw/pitch |
| 射击触发 | 视觉 READY_TO_FIRE 自动触发 | 鼠标/遥控器手动触发 |
| `allow_auto_fire` | 控制视觉是否自动射击 | 设为0，禁用自动射击 |
| `should_fire` | 驱动射击决策 | 不使用，可保留或删除 |
| 操作手控制 | 无法干预射击时机 | 完全控制射击时机 |

---

## ⚠️ 注意事项

### 1. 状态检查建议

手动发弹模式下，建议添加以下安全检查：

```c
// 检查摩擦轮是否达到目标转速（从 shoot_feed_sub 获取反馈）
uint8_t friction_speed_ok = (shoot_fetch_data.friction_speed_ready == 1);

// 检查视觉目标锁定状态（可选，提高命中率）
uint8_t target_locked = (vision_data_recv.target_locked == 1);

// 综合判断
if (鼠标左键 && vision_online && friction_speed_ok && target_locked) {
    shoot_cmd_send.load_mode = LOAD_1_BULLET;
}
```

### 2. 优先级控制

确保手动发射的优先级高于遥控器基础控制：

```c
// RobotCMDTask() 执行顺序
RemoteControlSet();     // 1️⃣ 低优先级：设置基础控制
VisionControlSet();     // 2️⃣ 中优先级：接管云台
MouseKeySet();          // 3️⃣ 高优先级：手动发射（可覆盖前两者）
EmergencyHandler();     // 4️⃣ 最高优先级：紧急停止
```

**当前代码的执行顺序**（第506-530行）：
```c
RemoteControlSet();
VisionControlSet();
EmergencyHandler();
```

**需要调整为**：
```c
RemoteControlSet();
VisionControlSet();
MouseKeySet();  // ⭐ 确保鼠标控制可以覆盖前面的设置
EmergencyHandler();
```

### 3. 遥控器模式兼容性

如果遥控器模式也需要手动发弹，需要在 `RemoteControlSet()` 中添加类似逻辑。

### 4. 视觉数据字段清理（可选）

如果确定不再使用视觉驱动发射，可以删除以下字段以简化代码：

**robot_def.h**:
```c
typedef struct {
    uint8_t vision_valid;
    uint8_t target_locked;
    float yaw;
    float pitch;
    // uint8_t should_fire;  ← 可以删除
} Vision_Upload_Data_s;

typedef struct {
    uint8_t vision_mode;
    // uint8_t allow_auto_fire;  ← 可以删除或设为0
    float manual_yaw_offset;
    float manual_pitch_offset;
} Vision_Ctrl_Cmd_s;
```

**vision.c**:
```c
// ProcessAutoAim() 中删除 should_fire 的处理
vision_upload_data.target_locked = 1;
// vision_upload_data.should_fire = ...;  ← 删除
```

---

## 🚀 快速实施清单

- [ ] 修改 `VisionControlSet()`：删除发射控制部分
- [ ] 在 `MouseKeySet()` 末尾添加手动发射逻辑
- [ ] 调整 `RobotCMDTask()` 执行顺序，确保鼠标控制在视觉控制之后
- [ ] 设置 `allow_auto_fire = 0`（禁用视觉自动射击）
- [ ] 可选：添加摩擦轮转速检查
- [ ] 可选：添加目标锁定状态检查
- [ ] 测试：视觉瞄准正常，鼠标点击触发射击

---

## 📝 相关文件

- `Gimbal/application/cmd/robot_cmd.c` - 主要修改文件
- `Gimbal/application/cmd/robot_cmd.h` - 函数声明
- `Gimbal/application/robot_def.h` - 数据结构定义
- `Gimbal/application/vision/vision.c` - 视觉处理逻辑（可选清理）

---

**生成时间**: 2025-01-21
**适用版本**: 当前 Gimbal 项目框架
**状态**: 待实施
