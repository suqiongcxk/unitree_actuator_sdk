# ESP32-S3 ↔ RK3588 串口遥控协议 V1

状态：已冻结（2026-09-24，2026-09-25更新物理连接）；尚未接入实机。字段长度、
单位、字节序、消息编号和CRC规则不得单方面修改；不兼容变更必须升级协议版本。

硬件边界已确认：ESP32-S3使用自研PCB，通过3.3V TTL UART直接连接RK3588的
`UART1_M1` 排针，不再使用CH340或USB。本仓库不包含手机端或ESP32-S3固件，只
负责定义UART合同并实现RK3588接收端。传输介质变化不改变V1二进制帧格式。

## 1. 设计边界

串口层只负责：

- 帧同步、CRC、序号和版本检查；
- 控制权申请/释放；
- `vx / vy / yaw_rate` 速度命令；
- stop、锁存 estop、显式 estop_clear；
- 独立 cmd/heartbeat watchdog；
- ACK 和低频状态回传。

串口接收线程不得直接调用电机接口。有效速度必须提交到现有
`VelocityCommandManager`，再经过 Safety Manager 和 NN Policy。手机到 ESP32 的
Wi-Fi 协议及ESP32-S3实现属于另一工程；无论其内部形式如何，都必须重新编码成
本文定义的完整UART帧，不得把TCP/WebSocket分包直接透传成电机命令。

## 2. 物理层

```text
115200 baud, 8 data bits, no parity, 1 stop bit, no flow control
字节序：little-endian
电平：3.3V TTL
```

RK3588（Orange Pi 5 Pro）接线固定为：

```text
GPIO1_B7，物理7脚， UART1_M1_RX  <- ESP32-S3 TX
GPIO1_B6，物理15脚，UART1_M1_TX  -> ESP32-S3 RX
RK3588 GND                         -- ESP32-S3 GND
Linux设备节点：/dev/ttyS1
设备树overlay：uart1-m1
```

以上TX/RX方向以Orange Pi 5 Pro最新引脚规划图的信号标注为准，不根据设备树pin
数组的排列顺序推断方向。ESP32-S3侧具体UART控制器和GPIO号由其PCB定义，但TX/RX
必须交叉连接。禁止接入5V TTL或RS-232电平，也不要用RK3588排针给已有独立电源
的ESP32-S3重复供电。

115200 baud 对20Hz命令和10Hz状态有充足余量。禁止使用软件/硬件流控，以免悬空
RTS/CTS阻塞通信。

## 3. 帧格式

```text
Offset  Size  Field
0       2     SOF = AA 55
2       1     version = 01
3       1     message_type
4       2     payload_length，uint16 little-endian
6       4     sequence，uint32 little-endian
10      N     payload，最大48字节
10+N    2     CRC16，uint16 little-endian
```

CRC算法固定为 `CRC-16/CCITT-FALSE`：

```text
poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
```

CRC覆盖 `version` 到 payload 最后一个字节，不包含 `AA 55` 和 CRC自身。接收端必须：

1. 在任意字节流中搜索 `AA 55`；
2. 拒绝非V1版本或长度大于48的帧；
3. CRC失败时丢弃该帧并继续重新同步；
4. 不得因半帧、粘帧、噪声或未知消息类型阻塞控制线程；
5. 仅在完整且CRC正确后刷新相应计时器。

### CRC测试向量

以下是 `sequence=1, vx=+0.500m/s, vy=0, yaw=-0.500rad/s` 的 CMD_VEL：

```text
AA 55 01 10 06 00 01 00 00 00 F4 01 00 00 0C FE 34 D6
```

CRC输入为：

```text
01 10 06 00 01 00 00 00 F4 01 00 00 0C FE
```

期望CRC为 `0xD634`，线上little-endian字节为 `34 D6`。

## 4. ESP32 → RK3588 消息

| Type | 名称 | 长度 | Payload | 说明 |
|---:|---|---:|---|---|
| `0x01` | `HELLO` | 8 | `client_nonce:u32, capabilities:u32` | ESP32启动或串口重连后首先发送 |
| `0x02` | `CLAIM_CONTROL` | 0 | 空 | 申请串口速度控制权 |
| `0x03` | `RELEASE_CONTROL` | 0 | 空 | 平滑归零并释放控制权 |
| `0x10` | `CMD_VEL` | 6 | `vx_mm_s:i16, vy_mm_s:i16, yaw_mrad_s:i16` | 连续速度目标 |
| `0x11` | `STOP` | 0 | 空 | 平滑回零，保留控制权，不锁存急停 |
| `0x12` | `ESTOP` | 0 | 空 | 立即请求统一锁存安全停机 |
| `0x13` | `ESTOP_CLEAR` | 4 | `session_nonce:u32` | 仅满足清除条件时解除锁存 |
| `0x20` | `HEARTBEAT` | 4 | `esp_uptime_ms:u32` | 只刷新链路计时器 |

V1 对以上 payload 长度要求精确匹配。未知消息类型返回 `UNSUPPORTED`，且不得
刷新命令或心跳看门狗。

### CMD_VEL缩放

```text
vx_m_s       = vx_mm_s / 1000.0
vy_m_s       = vy_mm_s / 1000.0
yaw_rate_rad = yaw_mrad_s / 1000.0
```

使用整数而不是UART裸传float，避免NaN、ABI和字节序歧义。V1允许范围统一为
`[-1000,+1000]`，RK3588仍必须执行最终限幅和平滑；越界帧返回 `OUT_OF_RANGE`，
不得部分接受。

## 5. RK3588 → ESP32 消息

| Type | 名称 | 长度 | Payload | 说明 |
|---:|---|---:|---|---|
| `0x81` | `HELLO_ACK` | 8 | `session_nonce:u32, capabilities:u32` | 创建新会话，旧序号失效 |
| `0x82` | `CLAIM_ACK` | 6 | `request_seq:u32, result:u8, owner:u8` | 控制权结果 |
| `0x90` | `COMMAND_ACK` | 9 | `request_seq:u32, result:u8, fault_bits:u32` | 命令执行结果 |
| `0x91` | `STATUS` | 20 | 见下 | 建议10Hz主动发送 |
| `0xA0` | `HEARTBEAT_ACK` | 8 | `request_seq:u32, esp_uptime_ms:u32` | 用于链路和RTT诊断 |

STATUS V1 payload：

```text
rk_uptime_ms:u32
robot_state:u8
control_owner:u8
estop_latched:u8
reserved:u8
applied_vx_mm_s:i16
applied_vy_mm_s:i16
applied_yaw_mrad_s:i16
command_age_ms:u16
fault_bits:u32
```

result枚举：

```text
0 OK
1 BAD_STATE
2 NOT_OWNER
3 OUT_OF_RANGE
4 ESTOP_LATCHED
5 STALE_SEQUENCE
6 UNSUPPORTED
7 INTERNAL_ERROR
```

capabilities位定义：

```text
bit 0 CMD_VEL
bit 1 CONTROL_OWNERSHIP
bit 2 ESTOP
bit 3 BASIC_STATUS
bit 4 HEARTBEAT_RTT
bit 5..31 reserved（发送端置零、接收端忽略）
```

V1双方应声明 `0x0000001F`。

robot_state枚举：

```text
0 BOOTING
1 STANDBY
2 STANDING
3 ACTIVE
4 STOPPING
5 ESTOP
6 FAULT
```

control_owner枚举：

```text
0 NONE
1 KEYBOARD
2 SERIAL
```

fault_bits位定义：

```text
bit 0 SERIAL_LINK_LOST
bit 1 COMMAND_TIMEOUT
bit 2 ESTOP_LATCHED
bit 3 INTERNAL_FAULT
bit 4 MOTOR_FAULT
bit 5 IMU_INVALID
bit 6 STATE_ESTIMATE_INVALID
bit 7 CONTROL_BUSY
bit 8..31 reserved（发送端置零、接收端忽略）
```

## 6. 会话、序号和控制权

- ESP32每次启动生成随机 `client_nonce`；
- RK3588收到有效HELLO后生成新的 `session_nonce` 并清空该会话序号历史；
- 每个发送方向独立维护sequence，从1开始，`0xFFFFFFFF`后回到1，不使用0；
- 对已接收序号 `last`，当 `uint32_t(new_seq - last)` 位于
  `1..0x7FFFFFFF` 时视为新序号，否则视为重复或旧帧；
- 响应帧使用RK3588自身的sequence；ACK payload中的 `request_seq` 对应请求；
- 重复或倒序帧不得刷新cmd watchdog；
- 在CLAIM成功前，非零CMD_VEL返回NOT_OWNER；
- 同一时刻只允许一个控制源拥有运动控制权；串口不得与键盘命令叠加；
- SERIAL只能在当前没有其他owner时CLAIM，不能主动抢占KEYBOARD；控制源切换必须
  先由原owner归零并释放；
- RELEASE、串口断开或heartbeat超时后归零并释放串口控制权。

## 7. 安全时序

建议频率和超时：

```text
手机 -> ESP32 cmd_vel       20 Hz
ESP32 -> RK3588 CMD_VEL     20 Hz
ESP32 -> RK3588 HEARTBEAT    5 Hz
RK3588 -> ESP32 STATUS      10 Hz
ESP32手机命令超时          200 ms
RK3588 CMD_VEL超时         250 ms
RK3588 HEARTBEAT超时       600 ms
```

两个watchdog必须独立：HEARTBEAT持续存在不能延长旧的非零CMD_VEL。行为如下：

- 手机命令超时：ESP32立即发送STOP，不再重复旧速度；
- CMD_VEL超时：RK3588通过现有slew rate平滑归零；
- HEARTBEAT超时：归零、标记链路丢失并释放SERIAL控制权，保持站立控制；瞬时通信
  中断不直接让机器人进入阻尼下落；
- CRC错误或半帧：不更新任何命令时间戳；
- ESTOP或机器人内部硬故障：使用现有统一急停路径，12电机进入既定阻尼安全退出；
- ESTOP必须锁存，恢复通信不能自动清除；任何版本、长度和CRC均正确的V1 ESTOP
  都必须立即锁存，即使尚未CLAIM、序号重复或倒退，也不得因NOT_OWNER或
  STALE_SEQUENCE而忽略；UART物理链路视为受信任链路；
- ESTOP_CLEAR要求session_nonce正确、当前命令为零、状态允许且由用户明确操作；
- ESP32刚启动、重连或序号未同步时只能发送零速/握手，不能恢复上次速度。

## 8. RK3588实现约束

- 串口使用non-blocking/raw termios，解析器运行在线程或事件循环中；
- 解析线程只发布最新安全命令，不持有电机总线锁；
- 使用 `CLOCK_MONOTONIC` 计算所有超时；
- 串口读错误、设备不可用或 `EIO/ENODEV` 时立即令串口输入源失效；
- 自动重开串口时必须重新HELLO/CLAIM，不能保留旧会话或非零速度；
- 日志对CRC错误限频，计数器仍需保留；
- 设备名固定使用板载 `UART1_M1` 对应的 `/dev/ttyS1`；
- 先编写纯软件解析测试（拆包、粘包、噪声、CRC错误、seq回退、超时、estop），
  再做串口回环测试，最后才接 `robot_control`，整个前两阶段不驱动电机。

## 9. 部署期事项（不改变V1）

1. 用户启用 `uart1-m1` overlay并重启后，确认 `/dev/ttyS1` 存在且未被登录终端占用；
2. 完整机器人遥测如有需要，在V2中定义扩展消息；不得改变V1基础STATUS布局。

设备命名信息不改变V1线上帧格式。现在可以开始RK3588侧C++解析器实现；不需要
等待或编写ESP32-S3代码。
