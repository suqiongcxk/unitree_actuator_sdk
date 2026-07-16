# Unitree GO-M8010-6 电机控制库 (Orange Pi 5 Pro)

基于 Unitree 官方 SDK 封装的 GO-M8010-6 电机 RS-485 控制库，在 Orange Pi 5 Pro (RK3588) 上实现稳定双向通信。

## 硬件需求

| 组件 | 说明 |
|------|------|
| 开发板 | Orange Pi 5 Pro (RK3588)，运行 Linux 6.1+ |
| 电机 | Unitree GO-M8010-6 |
| RS-485 模块 | MAX485 / SP3485 等（DE+RE 短接，GPIO 控制方向） |
| 电源 | 24-48V DC 独立供电（勿从开发板取电） |

### 接线

```
Orange Pi 5 Pro          RS-485 模块           GO-M8010-6
─────────────────       ────────────          ────────────
UART4_M2 TX  (GPIO1_B2) → DI
UART4_M2 RX  (GPIO1_B3) ← RO
GPIO1_D7     (GPIO 63)  → DE + RE
GND                     → GND                  GND
                         A ─────────────────── RS-485+
                         B ─────────────────── RS-485-
                                               24-48V DC
```

> **详细接线文档：** [`motor_lib/使用注意事项.txt`](motor_lib/使用注意事项.txt)（含四路总线配置、引脚映射表、权限设置）

### 串口配置

需要在 `/boot/orangepiEnv.txt` 中启用 UART4_M2：

```
overlays=uart4-m2
```

重启后确认 `/dev/ttyS4` 存在：

```bash
ls -la /dev/ttyS4
```

## 编译

```bash
cd unitree_actuator_sdk
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

编译产出：

| 目标 | 说明 |
|------|------|
| `example_usage` | 单电机控制演示（阻尼/位置/速度/力矩模式） |
| `gpio_toggle_test` | GPIO 翻转测试（万用表验证 DE/RE 引脚） |
| `motor_test` | SDK 官方 Orange Pi 示例 |
| `example_multi_bus` | 多路并行总线演示 |

## 快速开始

### 1. 验证 GPIO（用万用表测 DE/RE 引脚电平）

```bash
sudo ./build/gpio_toggle_test
```

GPIO1_D7 应交替输出 3.3V / 0V，周期 1 秒。

### 2. 测试电机通信

```bash
sudo ./build/example_usage
```

成功时输出 `position` 和电机当前位置，且有计时信息：

```
tx=1us send=11us wait=50us rx=1us total=63us
position0.523du
```

## API 概览

### 单电机控制（MotorController）

```cpp
#include "motor_controller.h"

// GPIO 63 (GPIO1_D7), 串口 /dev/ttyS4, 电机 ID 0
MotorController motor(63, "/dev/ttyS4", 0);

// 位置控制: 目标 0.5 rad, kp=0.03, kd=0.01
motor.setPosition(0.5f, 0.03f, 0.01f);

// 速度控制: 3.14 rad/s
motor.setVelocity(3.14f, 0.02f);

// 阻尼模式: kd=-0.02
motor.setDamping(-0.02f);

// 力矩控制: 0.3 N·m
motor.setTorque(0.3f);

// 刹车
motor.brake();

// 读取状态（输出端量纲）
MotorState s = motor.getState();
// s.q=位置(rad), s.dq=速度(rad/s), s.tau=力矩(N·m), s.temp=温度(°C)
// s.correct=CRC校验通过, s.merror=错误码(0=正常)
```

### 多电机总线（MotorBus）

```cpp
#include "motor_controller.h"

MotorBus bus(63, "/dev/ttyS4");
bus.addMotor(0);
bus.addMotor(1);

bus.setPosition(0, 0.5f, 0.03f, 0.01f);  // 电机0 → 位置控制
bus.setVelocity(1, -3.14f, 0.01f);         // 电机1 → 速度控制
bus.sendRecv();  // 一次总线事务

MotorState s0 = bus.getState(0);
MotorState s1 = bus.getState(1);
```

### GPIO 工具

```cpp
#include "fast_gpio.h"

// chip=1, line=31 → GPIO1_D7
FastGPIO gpio(1, 31);
gpio.set(1);   // 高电平 (TX 模式)
gpio.set(0);   // 低电平 (RX 模式)
// 析构时自动释放 GPIO
```

## 关键实现细节

1. **GPIO 翻转速度：** 使用 Linux GPIO v2 ioctl (`/dev/gpiochip`)，翻转延迟 ~1-5µs（sysfs 需要 1-5ms）
2. **TX 完成检测：** 使用 `TIOCOUTQ` + `TIOCSERGETLSR (TEMT)` 轮询硬件状态寄存器，绕开内核 `tcdrain()` 在 RK3588 上的间歇性 11ms 阻塞问题
3. **CRC 校验：** 接收后需显式调用 `MotorData::extract_data()` 进行 CRC-CCITT 校验和字段解包
4. **减速比转换：** 所有 API 使用输出端量纲，库内部自动完成转子端 ↔ 输出端转换（减速比 6.33:1）

## 文件结构

```
unitree_actuator_sdk/
├── include/                  # SDK 头文件（已修改 SerialPort.h 添加 fd()）
│   ├── serialPort/           # 串口通信
│   ├── unitreeMotor/         # 电机协议定义
│   └── crc/                  # CRC-CCITT / CRC32
├── lib/                      # 预编译 SDK 库（.so）
├── motor_lib/                # 自定义电机控制库 ★
│   ├── motor_controller.h    # MotorController + MotorBus 类
│   ├── motor_controller.cpp  # 核心实现
│   ├── fast_gpio.cpp/h       # 高速 GPIO (ioctl ~1µs)
│   ├── example_usage.cpp     # 使用示例
│   ├── gpio_toggle_test.cpp  # GPIO 测试工具
│   └── 使用注意事项.txt       # 详细硬件文档（引脚映射、接线图）
├── overlays/                 # 设备树 overlay 参考（可选，实验性）
├── example/                  # SDK 官方示例
└── CMakeLists.txt            # 构建配置
```

## 许可

如有任何使用问题或需要协助，请联系 support@unitree.com。

原始 SDK 版权所有 © Unitree Robotics。
