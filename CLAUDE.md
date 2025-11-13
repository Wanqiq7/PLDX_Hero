# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Build System
This project supports both Makefile and CMake build systems:

**Using Makefile (Traditional):**
```bash
# Build project
cd Chassis  # or cd Gimbal
make -j8

# Clean build
make clean

# Flash to hardware (OpenOCD)
make download_dap

# Flash with J-Link
make download_jlink
```

**Using CMake (Recommended):**
```bash
# Build with CMake and Ninja
cd Chassis  # or cd Gimbal
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake -G Ninja ..
ninja

# For development builds
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja ..
ninja
```

### Hardware Platform
- **Target MCU**: STM32F407IGH (ARM Cortex-M4, 168MHz)
- **Toolchain**: arm-none-eabi-gcc
- **Debugger**: OpenOCD or J-Link/JFlash
- **RTT Logging**: SEGGER RTT for real-time debugging

## Architecture Overview

This is a professional robotics control system for DJI RoboMaster competition with a **three-layer architecture**:

### Layer 1: BSP (Board Support Package)
**Location**: `Chassis/bsp/`, `Gimbal/bsp/`
Hardware abstraction layer providing STM32 HAL wrappers:
- `bsp_can.c/h` - CAN bus communication for motor control
- `bsp_usart.c/h` - UART for debugging and external comm
- `bsp_pwm.c/h` - PWM generation for servos
- `bsp_dwt.c/h` - Cycle counting for timing
- `bsp_spi.c/h`, `bsp_iic.c/h` - Sensor interfaces

### Layer 2: Modules
**Location**: `Chassis/modules/`, `Gimbal/modules/`
Device and algorithm abstraction:

**Motor Controllers**:
- `DJImotor/` - DJI smart motors (M3508, M2006, GM6020)
- `LKmotor/` - LK-TECH motors (LK9025)
- `HTmotor/` - HT04 motors
- `DMmotor/` - DM motors
- `step_motor/`, `servo_motor/` - Specialized motors

**Sensors & Processing**:
- `BMI088/` - 6-axis IMU sensor
- `ist8310/` - Magnetometer
- `imu/` - IMU processing with Kalman filtering
- `algorithm/` - Control algorithms (PID, LQR, EKF)

**Communication**:
- `can_comm/` - CAN bus management
- `message_center/` - Inter-process communication
- `remote/` - RC receiver interface
- `referee/` - Competition referee system

### Layer 3: Application
**Location**: `Chassis/application/`, `Gimbal/application/`
Robot-specific control logic:
- `chassis/chassis.c/h` - Chassis movement control
- `gimbal/gimbal.c/h` - Gimbal aiming and stabilization
- `shoot/shoot.c/h` - Shooting mechanism
- `cmd/robot_cmd.c/h` - Command processing
- `sysid/sysid_task.c/h` - System identification for tuning

## Control Systems Architecture

### Multi-Loop Motor Control
```c
typedef struct {
    PID_t current_PID;    // Current loop (1kHz)
    PID_t speed_PID;      // Velocity loop
    PID_t angle_PID;      // Position loop
    float pid_ref;        // Cascaded reference
} Motor_Controller_s;
```

### Advanced Control Algorithms
- **PID Controller**: Anti-windup, derivative filtering, output limiting
- **LQR Control**: Force control for chassis dynamics with velocity-to-force conversion
- **System Identification**: RLS-based motor parameter identification
- **Sensor Fusion**: Quaternion EKF for IMU processing

### Robot Configuration
Key physical parameters in `robot_def.h`:
```c
#define WHEEL_BASE 0.56f        // m (wheelbase)
#define TRACK_WIDTH 0.33f       // m (track width)
#define RADIUS_WHEEL 0.077f     // m (wheel radius)
#define CHASSIS_MASS 12.5f      // kg
#define CG_HEIGHT 0.132f        // m (center of gravity)
```

## Board Configuration

### Supported Board Types (define only one in `robot_def.h`):
- `ONE_BOARD` - Single STM32F407 controls entire robot
- `CHASSIS_BOARD` - Dedicated chassis controller
- `GIMBAL_BOARD` - Dedicated gimbal controller

### CAN Bus Protocol
- **Motor IDs**: DJI motors use 0x200 + motor_id, GM6020 uses 0x204 + motor_id
- **Message Structure**: 8-byte data packets at 1Mbps

## Development Workflow

### Code Standards
- **Naming**: PascalCase for functions, snake_case for variables
- **Comments**: Doxygen-style for public interfaces
- **Architecture**: Object-oriented C with struct-based instances
- **Safety**: Comprehensive error checking and bounds validation

### Debugging Tools
- **SEGGER RTT**: Real-time logging without interrupting execution
- **DWT Cycle Counting**: Precise timing measurements
- **Ozone Debugger**: Hardware debugging with variable visualization
- **CAN Monitoring**: Real-time bus traffic analysis

### Control Modes
**Chassis**: Zero-force, rotate (small gyro), no-follow, gimbal-follow
**Gimbal**: Zero-force, free mode, gyro mode, system identification
**Shooting**: Single-shot, burst (3 rounds), continuous fire

### Key Features
- **Force Control**: LQR-based chassis force control with friction compensation
- **System Identification**: Automated parameter tuning for motors
- **Multi-Sensor Fusion**: IMU integration with advanced filtering
- **Power Management**: Intelligent power control and monitoring
- **Safety Systems**: Motor temperature monitoring, current limiting, emergency stop

## Important Notes

- Always check board type definitions before flashing
- Power control module can be toggled via `POWER_CONTROLLER_ENABLE` macro
- Motor parameters are configurable in `robot_def.h`
- Use CMake builds for faster compilation during development
- RTT logging provides real-time debugging without JTAG dependency