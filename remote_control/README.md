# 遥控接口

本目录集中管理RK3588的上层遥控入口。目前包括：

- `keyboard/`：PC键盘、SSH桥接和终端备用控制器；
- `network/`：RK3588调试热点的配置、开启和恢复脚本；
- `docs/KEYBOARD_REMOTE_CONTROL.md`：PC多键遥控说明；
- `docs/RK3588_WIFI_AP.md`：`Creeper_Dog`调试热点说明；
- `docs/SERIAL_PROTOCOL_V1.md`：ESP32-S3 UART协议；
- `docs/SERIAL_REMOTE_RECEIVER.md`：UART第一阶段独立握手诊断程序；
- `docs/SERIAL_VELOCITY_VALIDATOR.md`：使用真实速度管理器的无电机仲裁验证；
- `include/`、`src/`：UART协议编解码实现；
- `tests/`：键盘和串口协议纯软件测试。

键盘遥控已接入现有FIFO。ESP32-S3 UART链路已进入独立握手诊断阶段，尚未接入
机器人控制。

## ESP32-S3 串口遥控

目标链路：

```text
手机 ──Wi-Fi── ESP32-S3 ──3.3V TTL UART── RK3588 UART1_M1
                                            /dev/ttyS1
```

已确认硬件为自研 **ESP32-S3 PCB**。PCB UART直接连接Orange Pi 5 Pro排针上的
`UART1_M1`，不再使用CH340或USB转串口。

## 项目职责边界

- 本工程不编写手机端代码；
- 本工程不编写或维护ESP32-S3固件；
- 双方共同遵守冻结后的UART Protocol V1；
- 本工程只实现RK3588侧串口接收、协议解析、看门狗、控制权、安全接入与测试；
- ESP32-S3侧如何接收手机Wi-Fi命令属于另一工程，只需按UART协议输出和接收帧。

该链路只负责传递速度目标和安全指令。ESP32/串口线程不得直接控制电机，指令必须
进入现有 `VelocityCommandManager -> Safety Manager -> NN Policy`。

当前状态：已冻结 Protocol V1，并实现纯内存编解码器、流解析器和独立串口诊断程序；
尚未接入 `robot_control`，不会访问或驱动电机。

第一阶段独立诊断程序`serial_remote_receiver`已经实现。它可以打开`/dev/ttyS1`
完成HELLO/HEARTBEAT握手，并安全打印STOP、ESTOP和限幅后的CMD_VEL；它仍未接入
`robot_control`，不会访问或驱动电机。使用方法见`docs/SERIAL_REMOTE_RECEIVER.md`。

第二阶段无电机验证程序`serial_velocity_validator`已经实现。它复用实际
`VelocityCommandManager`，完成SERIAL/KEYBOARD控制权仲裁、单位转换、独立命令与
链路watchdog和逻辑ESTOP测试，但仍不打开电机串口或产生关节指令。当前正式
`robot_control`尚未接入UART；详见`docs/SERIAL_VELOCITY_VALIDATOR.md`。

## 硬件接线

```text
RK3588 GPIO1_B7（物理7脚，UART1_M1_RX）  <- ESP32-S3 TX
RK3588 GPIO1_B6（物理15脚，UART1_M1_TX） -> ESP32-S3 RX
RK3588 GND                                 -- ESP32-S3 GND
```

- 双方必须使用 **3.3V TTL**；不得接5V TTL或RS-232电平；
- TX/RX必须交叉，GND必须共地；
- ESP32-S3已有独立供电时，不连接排针3.3V/5V，避免重复供电；
- 只使用TX、RX、GND，不使用RTS/CTS硬件流控。

## 文档

- `docs/SERIAL_PROTOCOL_V1.md`：已冻结的串口帧、命令、状态和安全语义；
- `config/serial_remote.conf.example`：RK3588 端建议配置。
- `include/serial_protocol_v1.hpp`、`src/serial_protocol_v1.cpp`：纯协议实现；
- `tests/test_serial_protocol_v1.cpp`：拆包、粘包、噪声、CRC与序号测试。

纯软件测试：

```bash
cmake --build build --target test_serial_protocol_v1 -j2
./build/test_serial_protocol_v1
```

当前工程的四路电机UART和IMU均未使用GPIO1_B6/B7。2026-10-07已只读确认
`uart1-m1`启用且`/dev/ttyS1`存在。可用以下命令复核：

```bash
ls -l /dev/ttyS1
gpio readall
cat /boot/orangepiEnv.txt
```

物理7脚/15脚应为UART复用，且`/dev/ttyS1`存在。不要把`/dev/ttyS9`
误认为这组排针：`ttyS9` 是另一个控制器，不对应GPIO1_B6/B7。
