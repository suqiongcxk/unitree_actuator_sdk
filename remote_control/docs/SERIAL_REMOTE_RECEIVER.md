# UART Protocol V1 第一阶段握手诊断程序

`serial_remote_receiver` 是独立诊断程序，只打开指定的TTL UART并验证Protocol V1。
它不链接`robot_control`、GPIO或电机库，不会向`VelocityCommandManager`提交速度。

## 功能边界

- `/dev/ttyS1`配置为115200、8N1、raw mode、无软硬件流控；
- HELLO生成非零`session_nonce`并返回HELLO_ACK；
- HEARTBEAT返回HEARTBEAT_ACK；
- RK3588发送方向独立维护sequence；
- 拒绝CRC、版本、长度、未知类型和旧sequence；
- 600ms没有有效帧时令会话超时，之后必须重新HELLO；
- STOP/ESTOP只记录，CMD_VEL只解析、限幅和打印；
- 永远不启动或调用机器人控制。

## 构建与无硬件测试

```bash
cd /home/orangepi/unitree_actuator_sdk
cmake -S . -B build
cmake --build build --target \
  serial_remote_receiver test_serial_protocol_v1 test_serial_remote_receiver -j2
./build/test_serial_protocol_v1
./build/test_serial_remote_receiver ./build/serial_remote_receiver
```

PTY测试不打开真实串口，覆盖HELLO/HEARTBEAT、独立发送序号、CRC错误、精确长度、
未知类型、旧序号、CMD_VEL限幅、STOP/ESTOP和600ms超时。

## 连接ESP32诊断

确保`robot_control`没有运行，且`/dev/ttyS1`没有被其他进程占用：

```bash
cd /home/orangepi/unitree_actuator_sdk
sudo ./build/serial_remote_receiver --device /dev/ttyS1
```

退出使用`Ctrl+C`。正常握手应看到`HELLO`、`HELLO_ACK`，之后周期打印
`HEARTBEAT -> HEARTBEAT_ACK`。日志中的`SAFE_DIAGNOSTIC_NO_ROBOT_CONTROL`和
`DIAGNOSTIC_ONLY_NOT_SUBMITTED`用于明确本程序不会控制机器人。
