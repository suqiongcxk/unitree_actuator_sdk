# ESP32-S3 串口遥控

目标链路：

```text
手机 ──Wi-Fi── ESP32 ──3.3V TTL UART── CH340 ──USB── RK3588
                                      /dev/ttyUSB*
```

已确认硬件为自研 **ESP32-S3 PCB**，没有使用带USB转串口的开发板；PCB引出的
UART TX/RX连接外置CH340，再由CH340的USB连接RK3588。

## 项目职责边界

- 本工程不编写手机端代码；
- 本工程不编写或维护ESP32-S3固件；
- 双方共同遵守冻结后的UART Protocol V1；
- 本工程只实现RK3588侧串口接收、协议解析、看门狗、控制权、安全接入与测试；
- ESP32-S3侧如何接收手机Wi-Fi命令属于另一工程，只需按UART协议输出和接收帧。

该链路只负责传递速度目标和安全指令。ESP32/串口线程不得直接控制电机，指令必须
进入现有 `VelocityCommandManager -> Safety Manager -> NN Policy`。

当前状态：已冻结 Protocol V1，并实现纯内存编解码器和流解析器；尚未打开真实串口，
也未接入 `robot_control`，不会访问或驱动电机。

## 硬件注意事项

- 已确认当前CH340为 **3.3V TTL UART**，不能换成正负电压的RS-232转换器；
- `ESP32 TX -> CH340 RX`，`ESP32 RX <- CH340 TX`，并连接 GND；
- 不要换用5V TX的USB-UART直接连接ESP32-S3；更换模块后必须重新确认电平；
- 除非经过供电预算确认，不要同时用 CH340 VCC 和其他电源给 ESP32 供电；
- 不连接 CH340 的 DTR/RTS 到 ESP32 EN/BOOT，避免开串口时复位或进入下载模式；
- 当前自研PCB与CH340的3.3V逻辑电平条件已确认。

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

硬件接入后先执行只读识别：

```bash
lsusb
dmesg --follow
ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null
```

确认实际 VID/PID、设备节点和是否带唯一序列号后，再制定 udev 稳定设备名。不要在
未识别硬件前把 `/dev/ttyUSB0` 永久写死。
