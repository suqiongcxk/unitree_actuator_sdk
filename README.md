# Unitree GO-M8010-6 电机控制库 (Orange Pi 5 Pro)

基于 Unitree 官方 SDK 封装的 GO-M8010-6 电机 RS-485 控制库，支持 4 路独立 RS-485 总线并行控制。

## 硬件

| 组件 | 说明 |
|------|------|
| 开发板 | Orange Pi 5 Pro (RK3588), Linux 6.1+ |
| 电机 | Unitree GO-M8010-6 |
| RS-485 模块 | MAX485 / SP3485（DE+RE 短接，GPIO 控制方向） |
| 电源 | 24-48V DC 独立供电 |

## 启动配置

在 `/boot/orangepiEnv.txt` 中启用四路串口：

```
overlays=uart4-m2 uart6-m1 uart7-m2 uart0-m2
```

重启后确认：

```bash
ls -l /dev/ttyS0 /dev/ttyS4 /dev/ttyS6 /dev/ttyS7
```

## 四路总线引脚

| 总线 | 串口 | TX | RX | DE/RE | 全局GPIO |
|---|---|---|---|---|---|
| A | /dev/ttyS4 | GPIO1_B2 | GPIO1_B3 | GPIO1_D7 | 63 |
| B | /dev/ttyS6 | GPIO1_A0 | GPIO1_A1 | GPIO1_A7 | 39 |
| C | /dev/ttyS7 | GPIO1_B4 | GPIO1_B5 | GPIO1_A3 | 35 |
| D | /dev/ttyS0 | GPIO4_A4 | GPIO4_A3 | GPIO4_A5 | 133 |

> 详细引脚图、GPIO 芯片映射见 [`motor_lib/使用注意事项.txt`](motor_lib/使用注意事项.txt)

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
| `example_usage` | 单电机演示 |
| `example_multi_bus` | 四路并行总线演示 |
| `gpio_toggle_test` | GPIO 翻转测试 |

## 快速开始

```bash
# 1. GPIO 测试
sudo ./build/gpio_toggle_test

# 2. 单电机测试
sudo ./build/example_usage

# 3. 四路总线测试
sudo ./build/example_multi_bus
```

## API 概览

### 单电机（MotorController）

```cpp
MotorController motor(63, "/dev/ttyS4", 0);   // GPIO63, ttyS4, 电机ID=0

motor.setPosition(0.5, 0.03, 0.01);   // 位置控制
motor.setVelocity(3.14, 0.02);         // 速度控制
motor.setTorque(0.3);                  // 力矩控制
motor.brake();                         // 刹车

MotorState s = motor.getState();
// s.q, s.dq, s.tau, s.temp, s.correct, s.merror
```

### 单总线多电机（MotorBus）

```cpp
MotorBus bus(63, "/dev/ttyS4");
bus.addMotor(0);
bus.addMotor(1);
bus.setPosition(0, 0.5, 0.03, 0.01);
bus.setVelocity(1, -3.14, 0.01);
bus.sendRecv();
```

### 多路并行（MultiBusController）

```cpp
MultiBusController ctrl;
ctrl.addBus(1, 31, "/dev/ttyS4");   // 总线A: chip1:31
ctrl.addBus(1,  7, "/dev/ttyS6");   // 总线B: chip1:7
ctrl.addBus(1,  3, "/dev/ttyS7");   // 总线C: chip1:3
ctrl.addBus(4,  5, "/dev/ttyS0");   // 总线D: chip4:5

for (auto& bus : ctrl.buses()) {
    bus->addMotor(0);
    bus->addMotor(1);
    bus->addMotor(2);
}

ctrl.startAll(500);   // 500Hz 四路并行
// ... 下发指令 ...
ctrl.stopAll();
```

### GPIO

```cpp
FastGPIO gpio(1, 31);    // chip1, line31 → GPIO1_D7
gpio.set(1);              // 高电平
gpio.set(0);              // 低电平
```

## 实现细节

- **GPIO 翻转**：Linux GPIO v2 ioctl，延迟 ~1-5µs（sysfs 需 1-5ms）
- **TX 检测**：`TIOCOUTQ` + `TIOCSERGETLSR (TEMT)` 绕过 `tcdrain()` 在 RK3588 上的 11ms 阻塞
- **CRC**：接收后调用 `extract_data()` 进行 CRC-CCITT 校验
- **减速比**：API 使用输出端量纲，内部自动转换（6.33:1）

## 文件结构

```
├── include/               # SDK 头文件（串口、电机协议、CRC）
├── lib/                   # 预编译 .so
├── motor_lib/             # ★ 自定义控制库
│   ├── motor_controller.* # MotorController + MotorBus
│   ├── parallel_bus.*     # ParallelBus + MultiBusController
│   ├── fast_gpio.*        # 高速 GPIO
│   ├── example_usage.cpp
│   ├── example_multi_bus.cpp
│   └── 使用注意事项.txt    # 详细硬件文档
├── example/               # SDK 官方示例
└── CMakeLists.txt
```

## 许可

原始 SDK 版权所有 © Unitree Robotics。
