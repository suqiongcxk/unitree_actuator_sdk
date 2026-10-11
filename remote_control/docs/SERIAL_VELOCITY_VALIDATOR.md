# UART速度控制无电机验证入口

`serial_velocity_validator`用于Protocol V1第二阶段验证。程序会打开`/dev/ttyS1`，
使用真实`VelocityCommandManager`完成三轴速度限幅、斜坡和watchdog，但不链接或启动
`robot_control`，不打开四路电机串口，也不会产生任何关节命令。

## 数据路径

```text
ESP32 Protocol V1 frame
  -> CRC/version/length/sequence/session检查
  -> control ownership仲裁
  -> mm/s、mrad/s转换为m/s、rad/s
  -> VelocityCommandManager::submit([vx, vy, wz])
  -> VelocityCommandManager::update()
  -> 只打印最终生效速度，不连接机器人
```

串口命令不会转换成键盘字符，也不会写键盘FIFO。键盘和串口共享的是三维速度接口及
同一个输入源仲裁语义，而不是按键事件。

## 控制权规则

- HELLO/HELLO_ACK只建立会话，不自动获得运动控制权；
- ESP32必须发送零载荷`CLAIM_CONTROL`；
- 仅当`CLAIM_ACK.result=OK`且`owner=SERIAL`后，CMD_VEL才会提交；
- KEYBOARD持有控制权时，串口CLAIM返回`BAD_STATE`；
- RELEASE_CONTROL归零并释放SERIAL；
- STOP只允许SERIAL owner使用，归零但保留SERIAL owner；
- 250ms没有新的CMD_VEL时目标归零，HEARTBEAT不会延长该时间；
- 600ms没有有效串口帧时归零并释放SERIAL owner；
- 串口链路超时不会清除或覆盖KEYBOARD owner；
- ESTOP锁定所有输入、清空owner并归零；
- ESTOP_CLEAR要求正确session nonce、目标和生效速度都为零、owner为NONE，并且
  RK3588本地操作者一次性明确授权。清除后仍保持零速度。

## 构建和自动测试

```bash
cd /home/orangepi/unitree_actuator_sdk
cmake -S . -B build
cmake --build build --target \
  serial_remote_receiver \
  serial_velocity_validator \
  test_serial_protocol_v1 \
  test_serial_remote_receiver \
  test_serial_velocity_control \
  test_serial_velocity_validator_pty \
  test_velocity_command_manager -j2

./build/test_serial_protocol_v1
./build/test_serial_remote_receiver ./build/serial_remote_receiver
./build/test_velocity_command_manager
./build/test_serial_velocity_control
./build/test_serial_velocity_validator_pty ./build/serial_velocity_validator
```

测试全部使用内存或PTY，不打开真实串口和电机。

## 实际ESP32无电机验证

先停止旧的`serial_remote_receiver`，确认没有运行`robot_control`：

```bash
cd /home/orangepi/unitree_actuator_sdk
sudo ./build/serial_velocity_validator --device /dev/ttyS1
```

当前ESP32如果只发送HELLO、HEARTBEAT和CMD_VEL，CMD_VEL会显示：

```text
decision=NOT_OWNER owner=NONE NO_ROBOT_OUTPUT
```

这是预期安全行为。ESP32需要补齐以下时序：

```text
HELLO
<- HELLO_ACK(session_nonce)
CLAIM_CONTROL
<- CLAIM_ACK(result=OK, owner=SERIAL)
CMD_VEL @ 20Hz
HEARTBEAT @ 5Hz
<- COMMAND_ACK / HEARTBEAT_ACK
STOP或RELEASE_CONTROL
```

收到合法CMD后，验证程序会分别打印`received_mks`、`accepted_target`和按实际斜坡
计算的`[EFFECTIVE] vx/vy/wz`，每行均带`NO_ROBOT_OUTPUT`。

诊断ESTOP_CLEAR时，必须先等待速度完全归零，再由RK3588本地终端输入：

```text
authorize-clear
```

该授权只允许下一次ESTOP_CLEAR尝试，错误nonce或条件不满足也会消耗授权。

## 尚未正式接入robot_control

当前模块已经实现可复用仲裁和watchdog，但`robot_control`仍沿用原键盘/FIFO入口，
尚未打开`/dev/ttyS1`。正式接入前还需要：

1. 将仲裁器置于RobotController生命周期内，并让键盘/FIFO和UART全部经过它；
2. 增加UART线程的启动、退出、断线重开和线程健康监控；
3. 将ESTOP接入统一硬件安全停机；当前全局急停为进程退出式锁存，没有运行中清除
   能力，因此实机ESTOP_CLEAR必须继续拒绝，或先设计人工重新使能流程；
4. 实现并验证10Hz STATUS回传以及真实机器人状态、owner和fault_bits映射；
5. 完成无电机实串口CLAIM/拒绝/超时/ESTOP测试后，再安排受保护的实机验收。
