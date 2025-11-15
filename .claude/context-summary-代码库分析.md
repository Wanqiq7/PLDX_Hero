## 项目上下文摘要（代码库分析）

生成时间：2025-11-14 11:06:32

### 1. 项目基本信息
- **项目名称**: RoboMaster Hero Code
- **项目类型**: 嵌入式机器人控制系统（STM32）
- **目标平台**: DJI RoboMaster 比赛
- **主要语言**: C/C++
- **构建系统**: Makefile + CMake

### 2. 项目架构分析

#### 三层架构设计
1. **BSP层（板级支持包）**
   - 位置：`Chassis/bsp/`, `Gimbal/bsp/`
   - 功能：硬件抽象，STM32 HAL封装
   - 关键模块：CAN、UART、PWM、SPI、I2C、DWT

2. **模块层（设备和算法抽象）**
   - 位置：`Chassis/modules/`, `Gimbal/modules/`
   - 功能：设备驱动、算法实现
   - 关键模块：
     - 电机控制器：DJI、LK、HT、DM电机
     - 传感器：BMI088 IMU、ist8310磁力计
     - 算法：PID、LQR、EKF、卡尔曼滤波
     - 通信：CAN总线、消息中心

3. **应用层（机器人控制逻辑）**
   - 位置：`Chassis/application/`, `Gimbal/application/`
   - 功能：高级控制逻辑
   - 关键模块：底盘控制、云台控制、发射机构、指令处理

### 3. 关键技术特性

#### 控制系统
- **多环电机控制**：电流环（1kHz）→速度环→位置环
- **LQR力控**：底盘动力学控制，包含速度到力的转换
- **系统辨识**：基于RLS的电机参数自动整定
- **传感器融合**：四元数EKF处理IMU数据

#### 通信架构
- **Message Center**：发布-订阅模式解耦模块间通信
- **CAN总线协议**：1Mbps，8字节数据包
- **电机ID分配**：DJI电机使用0x200+motor_id

### 4. 开发环境配置

#### 硬件平台
- **MCU**: STM32F407IGH (ARM Cortex-M4, 168MHz)
- **工具链**: arm-none-eabi-gcc
- **调试器**: OpenOCD 或 J-Link
- **日志系统**: SEGGER RTT

#### 构建命令
```bash
# 使用Makefile
cd Chassis  # 或 Gimbal
make -j8

# 使用CMake（推荐）
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake -G Ninja ..
ninja
```

### 5. 项目配置

#### 开发板类型（robot_def.h）
- `ONE_BOARD`: 单板控制整个机器人
- `CHASSIS_BOARD`: 专用底盘控制器
- `GIMBAL_BOARD`: 专用云台控制器

#### 物理参数
- 轴距：0.56m
- 轮距：0.33m
- 车轮半径：0.077m
- 底盘质量：12.5kg
- 重心高度：0.132m

### 6. 关键文件和目录

```
Hero/
├── Chassis/          # 底盘控制代码
│   ├── bsp/         # BSP层
│   ├── modules/     # 模块层
│   ├── application/ # 应用层
│   └── cmake-build-debug/ # 构建目录
├── Gimbal/          # 云台控制代码（结构类似）
├── .claude/         # Claude工作文件
└── CLAUDE.md        # 开发准则（已修复重复）
```

### 7. 开发注意事项

1. **实时性要求**：
   - INStask: 1kHz
   - MotorTask: 200-1000Hz
   - 使用FreeRTOS管理任务

2. **调试工具**：
   - SEGGER RTT实时日志
   - DWT周期计数
   - Ozone调试器
   - CAN监控

3. **控制模式**：
   - 底盘：零力、旋转、不跟随、云台跟随
   - 云台：零力、自由、陀螺仪、系统辨识
   - 发射：单发、三发、连发

4. **安全特性**：
   - 电机温度监控
   - 电流限制
   - 急停功能

### 8. 项目独特性

- **专业级机器人控制系统**：面向竞技机器人的高性能控制
- **多层抽象设计**：清晰的软件架构，便于维护和扩展
- **先进控制算法**：LQR、系统辨识等现代控制理论应用
- **实时性能优化**：针对1kHz控制频率优化
- **模块化设计**：高内聚低耦合的模块结构