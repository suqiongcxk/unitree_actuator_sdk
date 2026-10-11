# RK3588 四足机器人整机交接与维护手册

> 审计日期：2026-09-30  
> 主机：Orange Pi 5 Pro / RK3588 / `orangepi5pro`  
> 工程：`/home/orangepi/unitree_actuator_sdk`  
> 当前分支：`CXK`  
> 本文依据当前系统状态、源码、配置、协议和历史记录整理；没有启动或驱动任何电机。

## 1. 一页快速概览

这台 RK3588 是四足机器人的唯一实机运动主控。它读取 JY901S IMU，经四路半双工 RS-485 管理 12 台 Unitree GO-M8010-6 电机，在本机完成机械零点标定、状态估计、50 Hz ONNX 强化学习策略推理、关节目标转换、500 Hz 电机通信和统一安全停机。

当前主控制链路如下：

```text
键盘/FIFO速度命令 [vx,vy,yaw_rate]
  → 命令限幅、斜坡和超时
  → IMU与12关节反馈采集
  → Complementary状态估计、接触与腿式里程计
  → 48维observation
  → ONNX Runtime Actor
  → 12维raw action
  → q_des=default_pose+0.25×action
  → NN目标合法性检查
  → URDF坐标转换为电机坐标
  → 四路RS-485并行下发
  → 12关节反馈闭环
```

当前维护状态：

| 项目 | 状态 |
|---|---|
| 12 DOF和四路RS-485 | 已实现，有实机测试记录 |
| JY901S、Complementary状态估计 | 已实现，有实机测试记录 |
| Linear KF | 已实现且纯软件测试通过；不是默认后端 |
| 默认ONNX | V30/model_1299，文件与golden vectors本机复验通过 |
| 零命令站立 | 已有实机验收记录 |
| 键盘/FIFO遥控 | 已投入实机使用 |
| 长时间、多方向稳定行走 | 尚未完成最终验收；换向曾触发关节目标越界 |
| ESP32-S3 UART遥控 | 协议和纯解析器完成；真实串口及整机接入未完成 |
| 微信小程序/Robot AP | 已放弃；仓库文档和系统软件仍有残留 |

**接手后不要直接运行 `sudo ./build/robot_control`。** 先检查 Git、供电、悬吊、防夹、急停人员、12电机通信和所用模型身份，再按本文分级操作。

## 2. 整机硬件与软件架构

### 2.1 硬件关系

- RK3588：Linux主控、所有高级控制和安全状态的拥有者。
- 12×GO-M8010-6：每腿 hip/thigh/calf 三个关节。
- 4×RS-485收发链路：一条腿一路UART，并由GPIO控制DE/RE。
- JY901S：通过 `/dev/i2c-1` 提供四元数、加速度、角速度。
- ESP32-S3自研PCB：计划接手机遥控，通过UART1_M1向RK3588提交高级速度命令；当前未接入。
- 电机独立大功率电源：历史上供电不稳曾触发电机快闪和 `MError=5`。本地协议头把5列为保留值，后续现场判断为欠压；维护时必须同时保留“厂商协议未明确定义”和“现场供电诊断”的证据。

### 2.2 关键线程

`RobotController` 管理7条关键线程：

- IMU：200 Hz；
- 状态估计：50 Hz；
- ONNX/策略：50 Hz；
- 四路电机总线：每路目标500 Hz。

IMU→估计器、估计器→NN使用mutex保护的值快照；电机命令和反馈通过总线槽位做短锁拷贝。硬件I/O和ONNX推理不在共享锁中执行。周期基于 `CLOCK_MONOTONIC` 和绝对时间睡眠，同时记录最大gap、执行时间和deadline统计。

## 3. 当前RK3588系统环境

### 3.1 系统基线

| 项目 | 当前值 |
|---|---|
| 板卡/架构 | Orange Pi 5 Pro，RK3588，aarch64，8核（4×A55 + 4×A76） |
| 系统 | Orange Pi 1.0.6 Jammy / Ubuntu 22.04.5 LTS |
| 内核 | `6.1.43-rockchip-rk3588` |
| 时区 | Asia/Shanghai，NTP active |
| 内存 | 7.7 GiB；审计时约6.8 GiB available |
| 根分区 | 56 GiB ext4；审计时约45 GiB可用 |
| 存储 | `/dev/mmcblk1`；根目录和持久日志位于同一eMMC/SD存储 |
| 编译器 | GCC/G++ 11.4.0 |
| CMake/Make | 3.22.1 / 4.3 |
| Git/Python | 2.34.1 / 3.10.12 |
| Eigen | 3.4.0 |
| ONNX Runtime | 工程内置 `thirdparty/onnxruntime`，运行库1.18.0；模型交付记录的训练端版本为1.22.1 |

审计时CPU温度约29～31°C，系统刚启动约15分钟，无机器人控制进程运行。

### 3.2 用户与设备权限

`orangepi` 属于 `tty`、`dialout`、`sudo`、`input`、`netdev` 等组，因此串口通常可由普通用户打开；但它不在 `i2c` 组，`/dev/i2c-1` 属于 `root:i2c` 且权限0660，完整控制和IMU诊断当前通常需要 `sudo`。GPIO访问也按现有操作流程使用sudo。

当前设备节点：

```text
/dev/ttyS6  FL: 0,4,8
/dev/ttyS4  FR: 1,5,9
/dev/ttyS7  RL: 2,6,10
/dev/ttyS0  RR: 3,7,11
/dev/i2c-1  JY901S
```

`/dev/ttyS1` 当前不存在。`/boot/orangepiEnv.txt` 实际启用的是：

```text
uart4-m2 uart6-m1 uart7-m2 uart0-m2 i2c1-m4
```

因此 ESP32-S3 所需的 `uart1-m1` 还没有启用。不得为了测试遥控擅自修改overlay或重启。

### 3.3 网络与系统服务

当前 `wlan0=192.168.31.100/24`，使用名为 `Robotstar` 的普通 infrastructure Wi-Fi，默认网关为 `192.168.31.1`。另有 `SQ`、`ZZPdesktop` 和有线连接配置。审计未改变任何连接。

SSH监听所有接口的22端口；UFW当前inactive。系统还在所有接口监听：

- TCP 5555：root运行的 `adbd`；
- TCP/UDP 111：已启用的 `rpcbind`。

如果设备进入不可信网络，这两项和无防火墙状态属于安全风险，应在不影响既有调试方式的前提下单独评估，不要在运动调试当天临时改服务。

历史Robot AP残留：

- `hostapd`、`dnsmasq`软件包仍安装；
- `dnsmasq.service`开机启用但因53端口占用启动失败，是当前唯一failed unit；
- `hostapd.service`显示active(exited)，没有找到有效 `/etc/hostapd/hostapd.conf`；
- 当前没有热点接口，也没有hostapd/dnsmasq运行进程。

结论：Robot AP目前没有工作，但系统清理并不完整。未经授权不要自行disable、purge或切换网络。

## 4. 仓库目录和核心代码

### 4.1 Git状态

当前仓库：

```text
branch: CXK
origin: https://github.com/suqiongcxk/unitree_actuator_sdk
状态: 比 origin/CXK 领先4个提交
```

另有remote `wechat` 指向历史小程序独立仓库。工作区当前不是干净状态：

- 已修改：`remote_control/README.md`
- 已修改：`remote_control/config/serial_remote.conf.example`
- 已修改：`remote_control/docs/SERIAL_PROTOCOL_V1.md`
- 未跟踪：`robot_control/debug_logs/RK3588_HARDWARE_WORK_SUMMARY.md`
- 本次新增：本交接手册

`.git/REBASE_HEAD`可解析到旧提交，但没有 `.git/rebase-merge`、`.git/rebase-apply`，且 `git status`不显示rebase，因此当前**没有活动中的rebase**；它是Git保留的伪引用，不要仅因该文件存在就删除它。

### 4.2 目录说明

| 目录 | 用途 | 维护判断 |
|---|---|---|
| `robot_control/` | 整机主控、状态、策略、安全、遥控FIFO | 生产核心 |
| `motor_lib/` | M8010封装、四路总线、GPIO、标定、急停 | 生产核心 |
| `State_Estimation/` | JY901S I2C驱动 | 生产核心 |
| `remote_control/` | ESP32 UART V1协议、纯解析器和测试 | 开发中，未接整机 |
| `model_deliveries/` | 版本化模型、manifest、golden vectors和仿真证据 | 模型审计核心 |
| `robot_description/` | 20260714与20260906两版URDF和mesh | 新版20260906为当前基线 |
| `mujoco_sim/` | 独立、无硬件依赖的C++ MuJoCo接口 | 仿真/回归工具 |
| `robot_control/tests/` | 纯软件测试和硬件诊断混放 | 运行前必须看CMake/源码 |
| `robot_control/debug_logs/` | 阶段记录和部署审计 | 部分文档已过时 |
| `overlays/` | 实验性硬件RS-485 overlay | 当前未部署，不要使用 |
| `wechat_mini_program/` | 已放弃方案的计划文档 | 历史遗留 |
| `motor_tools/` | Unitree厂商工具的多架构版本 | 仅在明确流程中使用 |
| `build/` | 当前ARM64构建产物 | 可再生，不是源码 |
| `thirdparty/` | ONNX Runtime和pybind11 | 本机构建依赖 |

仓库外的 `/home/orangepi/test_serial_scan.py` 是旧JY901S串口扫描器，会依次打开并flush `/dev/ttyS0/S4/S6/S7`。这些端口现在属于电机总线，**绝不能在机器人程序运行时执行，也不建议作为现有I2C IMU的诊断工具。**

### 4.3 核心入口

- `robot_control/robot_control_main.cpp`：CLI、FIFO、健康输出、主循环和清理。
- `robot_control/robot_controller.cpp/.h`：硬件初始化、标定、线程和策略管线。
- `motor_lib/parallel_bus.cpp/.h`：四路500 Hz通信、CRC/ID检查、阻尼锁存。
- `motor_lib/ZeroPointCalibration.cpp`：机械限位零点、方向和站立前过渡。
- `robot_control/state_estimator.cpp`：默认Complementary前端和腿式里程计。
- `robot_control/linear_kf_*`：可选18状态Linear KF。
- `robot_control/nn_policy.cpp`：ONNX Runtime、平滑接管、验证包装。
- `robot_control/policy_observation.cpp`：48维输入唯一权威实现。
- `robot_control/velocity_command_manager.cpp`：三维速度命令限幅、hold和斜坡。
- `robot_control/runtime_safety.h`、`motion_safety.*`：运行故障锁存和运动指标。
- `remote_control/serial_protocol_v1.*`：UART V1纯协议，不打开串口。

## 5. 启动与控制数据流

正常程序启动顺序：

1. 校验电机ID 0～11完整且唯一；
2. 初始化 `/dev/i2c-1` 上的JY901S；
3. 执行陀螺静态校准；
4. 运行12关节机械限位标定；不足12/12直接退出；
5. 标定后并行预定位四个大腿；
6. 创建四路 `ParallelBus` 并启动500 Hz线程；
7. 平滑过渡到默认站姿；
8. 启动IMU、估计和NN线程；
9. 从默认站姿用2秒/100帧smoothstep混合到ONNX策略；
10. 接收主终端或FIFO速度命令；
11. 故障或人工停止后进入12电机阻尼并回收线程。

每个20 ms策略帧中，估计器先取得最新IMU和12关节快照，验证数据年龄和有限性，生成机体系速度、角速度、投影重力、关节状态；NN线程取得经过斜坡的速度命令，构造48维输入并推理。安全层验证候选 `q_des` 后，按motor ID转换回电机坐标并写入对应总线槽。四个500 Hz线程在两个策略帧之间重复发送最新目标并刷新反馈。

## 6. 电机、IMU和状态估计

### 6.1 电机/腿映射（以此表为准）

| 腿 | 串口 | DE/RE | hip/thigh/calf |
|---|---|---|---|
| FL | `/dev/ttyS6` | chip1:7 / GPIO39 | 0 / 4 / 8 |
| FR | `/dev/ttyS4` | chip1:31 / GPIO63 | 1 / 5 / 9 |
| RL | `/dev/ttyS7` | chip1:3 / GPIO35 | 2 / 6 / 10 |
| RR | `/dev/ttyS0` | chip4:5 / GPIO133 | 3 / 7 / 11 |

不同文档和测试程序对“总线A/B”的命名不一致：根README称A=FR、B=FL，而 `test_damping_packet_loss` 和运行时数组顺序把A=FL、B=FR。维护、日志和接线时必须使用“腿+串口+ID”，不要只写A/B。

另一个文档问题是 `motor_lib/使用注意事项.txt` 的某张表把UART4的RX/TX列名写反，且旧段落仍写 `i2c2-fast`；当前实际overlay、源码和设备节点明确是 `i2c1-m4`、`/dev/i2c-1`。

### 6.2 M8010通信

- 控制帧17 Byte，反馈帧16 Byte，CRC16；
- 每台电机为独立请求/响应事务；
- 只有完整帧、CRC正确、ID匹配才更新时间戳和状态；
- 短帧、timeout、CRC、wrong-ID分别计数；
- 坏帧保留最近有效数值，但本次事务标记无效；
- 最近有效反馈超过100 ms才触发状态无效，避免偶发丢包误停；
- 非零MError和90°C过温进入集中故障锁存；
- 安全退出覆盖旧命令为 `Kp=0,Kd=0.136,dq=0,tau=0`。

注意：M8010力矩语义存在文档冲突。厂商头文件注释写“实际关节输出扭矩”，但项目历史通过约10 kg静态重力闭合，采用“反馈进入足端力求解前乘6.333”的转子侧解释。该做法已用于当前接触估计且得到约99 N总支撑力，但在精密力控前仍应以厂商确认或力台标定为最终依据。

### 6.3 状态估计

默认是 `ComplementaryStateEstimator`，不是Kalman。主要功能：

- 四元数有限值、模长、归一化和符号连续；
- JY901S上电残余陀螺零偏校准与15 Hz低通；
- 投影重力，水平约 `[0,0,-1]`；
- 机械零点和方向映射后的12关节位置/速度/力矩；
- 足端运动学、关节力矩+雅可比的三维足端力估计；
- 接触置信度、腾空、滑移和冲击标记；
- 可信支撑足约束与IMU融合的机体线速度、机身高度。

Linear KF是可选18状态后端，通过 `--estimator kalman`选择；本机构建含Eigen，两个KF纯软件测试均通过。不要把它写成默认实机算法。

## 7. 强化学习模型部署

### 7.1 当前默认模型

```text
model_deliveries/model_0922_candidate/creeper_v30_model1299/
  creeper_model_v30_model1299_actor.onnx
  model_1299.pt
  policy_golden_vectors.json
  deployment_manifest.json
  simulation_report.md
```

本机复核：

```text
ONNX SHA256 = 280e3429e72573be96955a1341a9c00e492838db004b0b51c982279e97f45382
PT   SHA256 = 4285d2262ed122da182bba8e0790a8c477f11ad31940ad2fb1d502721da3a7cd
manifest引用文件 = 44/44存在且SHA256匹配
golden replay = 200帧PASS
最大ONNX差异 = 7.15e-7
最大q_des差异 = 2.38e-7
```

构建产物 `build/robot_control` 内嵌的默认绝对路径也是上述V30 ONNX。二进制时间晚于关键源码/CMake，说明当前构建与9月24日源码同步，但任何新改动后仍需重新构建。

### 7.2 模型接口

- 输入：`obs`，float32 `[1,48]`；
- 输出：`actions`，float32 `[1,12]`；
- 频率：50 Hz；
- default pose：`[0.1,-0.1,0.1,-0.1,0.8,0.8,1,1,-1.5,-1.5,-1.5,-1.5]`；
- action scale：0.25；
- `q_des=default_pose+0.25×raw_action`；
- 默认PD：`Kp=0.625,Kd=0.0125`。

48维顺序：机体系线速度3、角速度3、投影重力3、命令3、相对默认姿态的关节位置12、关节速度12、上一帧Actor原始action12。

当前没有额外observation normalization/clipping，也没有通用raw-action clip。jump-only、目标变化率、反馈速度和12关节聚合变化当前主要是监控项；NaN/Inf、关节越界和非法增益仍会阻止下发并停机。

交付manifest中的 `hardware_status=NOT_TESTED` 表示训练端交付时的状态。后续已有本机实机加载记录，但最近V30换向测试产生 `joint[9]=-0.779`，越过机械上限 `-0.838 rad`，触发 `NN_COMMAND_INVALID detail=9`。所以模型不能被描述为“全工况实机验收完成”。

## 8. 当前遥控方式和UART新方案

### 8.1 已使用：键盘/FIFO

主程序创建 `/tmp/creeper_velocity_command.fifo`。当前多键方案由Windows端`remote_control/keyboard/pc_keyboard_velocity_client.py`经SSH桥接发送完整三轴速度；`remote_control/keyboard/keyboard_velocity_console.py`仅作为SSH终端单键备用工具。两者只写高级速度，不接触电机。详细使用方法见`remote_control/docs/KEYBOARD_REMOTE_CONTROL.md`。

`VelocityCommandManager`范围为三轴 `[-1,1]`，默认输入有效期500 ms，单次hold最大5000 ms，斜率为 `0.5 m/s²、0.5 m/s²、1.0 rad/s²`。无人继续按键时不是保持旧速度，而是平滑回零。

### 8.2 待接入：ESP32-S3 UART1_M1

协议已冻结，纯内存编码、CRC、流解析和sequence测试已经实现并通过。设计链路：

```text
手机 → Wi-Fi → ESP32-S3 → 3.3V TTL UART → RK3588 /dev/ttyS1
```

引脚定义为GPIO1_B7/RX、GPIO1_B6/TX，115200-8-N-1。协议包括 `AA55`同步、版本、类型、长度、sequence、payload和CRC-16/CCITT-FALSE；定义HELLO、CLAIM/RELEASE、CMD_VEL、STOP、ESTOP、ESTOP_CLEAR、HEARTBEAT、ACK和STATUS。

尚未完成：启用uart1-m1、真实termios线程、会话nonce、控制权状态机、独立250 ms命令/600 ms心跳watchdog、状态回传、接入VelocityCommandManager和ESP32整机测试。

## 9. 安全机制

已经实现并有测试依据：

- 标定必须12/12成功，否则禁止控制；
- 标定、大腿预定位、3秒站立、2秒NN接管分阶段完成；
- IMU、电机、估计和策略输入做有限值与时间检查；
- 电机反馈新鲜度100 ms；
- IMU/EST/NN线程心跳阈值100 ms；
- MError非零、90°C、状态连续无效和状态流中断锁存首故障；
- NN检查NaN/Inf、机械角限位、Kp/Kd；危险候选不写总线；
- 验证失败不再跳回默认站姿，而是保持最后安全目标并请求阻尼；
- Ctrl+C、`s`、键盘X和内部故障最终走统一阻尼关闭；
- 关闭打印总线时序、首故障和12电机MError。

当前明确不是强制保护的项目：

- 单关节目标变化率5 rad/s：MONITOR_ONLY；
- 实测反馈速度3 rad/s：MONITOR_ONLY；
- 12关节单帧总变化0.5 rad：MONITOR_ONLY；
- jump-only超过0.1 rad：当前监控并继续接受。

不要以“模型在MuJoCo运行很久”为理由关闭机械角限位、CRC、MError或反馈年龄保护。

## 10. 标准操作流程

### 10.1 第一次登录：完全只读

```bash
cd /home/orangepi/unitree_actuator_sdk
git status --short --branch
ps -ef | grep -E 'robot_control|example_usage|keyboard_velocity' | grep -v grep
ls -l /dev/ttyS0 /dev/ttyS4 /dev/ttyS6 /dev/ttyS7 /dev/i2c-1
cat /boot/orangepiEnv.txt
df -h /
free -h
```

不要在控制程序运行时打开同一UART的测试工具。

### 10.2 标准构建

构建会修改 `build/`，应在确认源码分支和工作区后执行：

```bash
cd /home/orangepi/unitree_actuator_sdk
cmake -S . -B build
cmake --build build -j2
```

建议先用 `-j2`，便于在机器人上保留系统响应；确认资源充足后再增加并行度。

### 10.3 不访问硬件的回归测试

以下目标已确认是纯软件：

```bash
cd /home/orangepi/unitree_actuator_sdk

./build/test_double_buffer
./build/test_state_estimator
./build/test_legged_odometry
./build/test_foot_force_estimator
./build/test_policy_observation
./build/test_nn_validation
./build/test_velocity_command_manager
./build/test_runtime_safety
./build/test_motion_safety
./build/test_smooth_takeover
./build/test_linear_kf_estimator
./build/test_linear_kf_adapter
./build/test_serial_protocol_v1
./build/test_joint_mapping

./build/test_policy_golden_vectors \
  model_deliveries/model_0922_candidate/creeper_v30_model1299/policy_golden_vectors.json \
  model_deliveries/model_0922_candidate/creeper_v30_model1299/creeper_model_v30_model1299_actor.onnx
```

`test_projected_gravity` 和 `test_gyro_calibration` 会访问I2C实机，不属于纯软件测试。`test_joint_feedback*`、`test_damping_packet_loss`、`example_*`会访问电机。

### 10.4 12电机短时阻尼通信测试（会访问电机）

前提：机身可靠悬空、电机独立供电稳定、人员远离夹点、没有其他电机程序、现场人员明确同意。

```bash
cd /home/orangepi/unitree_actuator_sdk
sudo ./build/test_damping_packet_loss --duration 10 --hz 500 \
  2>/tmp/damping_comm_warnings.log \
  | tee /tmp/damping_packet_loss.txt
```

该程序发送 `Kp=0,Kd=0.02,dq=0,tau=0`，不发送位置目标，但关节仍可运动，不能省略悬吊。

### 10.5 正常标定、站立和ONNX

高风险实机操作，只能在吊带、可靠供电、急停人员和空旷环境下进行：

```bash
cd /home/orangepi/unitree_actuator_sdk
sudo ./build/robot_control \
  --log \
  --log-file /tmp/v30_robot_control.csv \
  2>&1 | tee -i /tmp/v30_robot_control.log
```

程序默认加载V30/model_1299。不要随意加 `--no-validate`。初次接手更适合先执行仅阻尼的 `--monitor-only`，但它仍会打开全部硬件并发送阻尼，不能当成普通软件测试：

```bash
sudo ./build/robot_control --monitor-only --log \
  --log-file /tmp/v30_monitor.csv \
  2>&1 | tee -i /tmp/v30_monitor.log
```

### 10.6 第二终端键盘控制

只有主程序已完成标定、站立和NN接管且FIFO存在后执行：

```bash
cd /home/orangepi/unitree_actuator_sdk
python3 remote_control/keyboard/keyboard_velocity_console.py \
  --vx 0.50 --vy 0.30 --yaw 0.50
```

松开按键依赖终端键盘重复和650 ms deadman。前后换向时当前软件不会强制“先到零并驻留”，最近事故表明换向过渡可能使模型输出越界；必须先松键等待 applied command 回零，再反向。

### 10.7 停止方式

- 平滑回零：键盘空格，或主终端输入 `z` 回车；
- 安全停止：键盘X、主终端 `s` 回车、Ctrl+C；
- 看到抖动、快速倾倒、滑移、异响、供电跌落、快闪或MError时立即停机并准备断电；
- 正常结束应看到IMU/EST/NN退出、四路总线回收、12电机阻尼命令完成和 `[Main] 程序退出`。

## 11. 常见故障排查

### 串口不存在

1. `cat /boot/orangepiEnv.txt`；
2. `dmesg | grep -E 'ttyS0|ttyS4|ttyS6|ttyS7'`；
3. 核对overlay，不要运行串口扫描器抢占总线；
4. `/dev/ttyS1`当前不存在是ESP32 UART尚未启用，不是电机故障。

### IMU初始化失败

1. 检查 `/dev/i2c-1` 和 `i2c1-m4`；
2. 检查权限，普通用户当前不在i2c组；
3. 检查JY901S供电、SDA/SCL和共地；
4. 不要用旧 `test_serial_scan.py` 查当前I2C IMU。

### 标定不足12/12

1. 立即停止，不允许绕过；
2. 从日志确认失败ID和对应腿/串口；
3. 先做该总线阻尼通信，不直接重复撞限位；
4. 检查供电、接插件、机械卡滞、CRC和MError；
5. 历史上ID2/6/10供电插接和ID3通信均出现过问题。

### CRC大量错误/反馈超时

1. 使用“腿+串口+ID”定位，忽略不统一的A/B字母；
2. 检查485 A/B、共地、DE/RE、终端和线束；
3. 看 `short/timeout/proto/crc/wrong_id/max_tx`，不要只看总丢包率；
4. 同腿三个ID一起陈旧通常是总线级问题；单ID异常更像电机/接头问题；
5. 不要让两个程序同时打开同一UART。

### `MOTOR_FEEDBACK_INVALID detail=N`

表示第N号反馈超过新鲜度/有效性门槛，不等同于MError=N。查看紧邻的 `MotorComm@fault`、age、fail和对应总线统计。

### `MOTOR_ERROR detail=N`或快闪

detail是电机ID。查看 `[MotorErrors@fault]` 中该ID的真实MError。历史MError=5与供电欠压相关；先断开运动测试，检查电源、接头、电缆压降和瞬态能力，重新上电清错后从阻尼通信开始，不要立刻行走。

### `NN_COMMAND_INVALID detail=N`

表示策略候选在第N关节出现NaN/Inf、机械越界、非法增益或组合验证失败。保留CSV，检查 `obs*`、`raw_action*`、`tgt*` 和 applied command。不要通过 `--no-validate`继续实机。

### FIFO不存在

先启动 `robot_control`。键盘脚本故意不创建FIFO，以免主程序不存在时留下普通文件。如果路径存在但不是FIFO，停止并人工确认，不要直接覆盖。

### ONNX或SHA不匹配

不要按文件名猜版本。核对sidecar、manifest和golden replay；失败则显式回滚到已知模型，不要重新导出后沿用旧SHA。

## 12. Git、模型、日志和配置维护

### 12.1 Git

当前 `CXK` 比远端领先4提交，先和项目负责人确认是否推送。不要直接运行根目录 `push.sh`：它会 `git add -A` 并固定 `git push origin main`，与当前CXK分支不匹配，可能提交无关文件或推错分支。

推荐人工流程：

```bash
git status --short --branch
git diff --check
git diff
git log --oneline --decorate -n 10
# 确认范围后使用 git add -- <明确文件>
# 确认当前分支后再 git push origin CXK
```

历史 `push_wechat_project.sh` 只服务已放弃的小程序双仓库流程，不应用于当前机器人/ESP32遥控开发。

### 12.2 模型更新

每个模型使用独立目录，至少保留：ONNX、ONNX SHA256、checkpoint、golden vectors、manifest和simulation report。更新步骤：

1. 校验全部SHA；
2. 检查 `[1,48]→[1,12]`、顺序、default pose、action scale和50 Hz；
3. C++ golden replay；
4. 仿真固定命令与换向序列；
5. `--monitor-only`；
6. 标定站立下 `--dry-run`；
7. 零命令真实接管；
8. 吊带下分级非零速度；
9. 失败时用 `--onnx <旧路径>`显式回滚。

### 12.3 日志

实时日志和CSV通常放 `/tmp`，重启后不保证保留。本次审计系统刚重启，未发现历史 `/tmp/*.log/*.csv`。关键验收后应将日志复制到按日期命名的持久目录，再在Markdown记录路径、命令、模型SHA、硬件状态和结论。不要提交巨型CSV到Git，除非仓库策略明确允许。

### 12.4 需要备份的最小集合

- 整个Git仓库及当前未推送提交；
- 当前工作区未提交的UART文档；
- `/boot/orangepiEnv.txt`；
- NetworkManager连接名称和非敏感参数，密码另用安全渠道保存；
- 当前模型交付目录和SHA；
- 新URDF `Creeper20260906`；
- 关键实机日志与测试结论；
- 若未来增加systemd/udev/UART服务，再单独备份对应unit和rules。

不要把 `/home/orangepi/.ssh` 私钥、Wi-Fi密码或Token提交进仓库。

## 13. 当前文档/代码不一致清单

1. 根README的A/B总线命名与运行时诊断数组不同；物理串口、腿和ID一致。
2. `motor_lib/使用注意事项.txt` 存在RX/TX列名和旧I2C2 overlay表述；实际为I2C1。
3. `robot_control/debug_logs/README.md` 的示例仍显式加载model_950；当前默认是V30/model_1299。
4. `STATE_ESTIMATION_PLAN.md`早期条目仍写hold最大2000 ms；当前代码和help为5000 ms。
5. `PROJECT_ISSUES.md`部分状态过时：四元数已在后续记录中验证，集中watchdog也已大幅完成；Issue-002还提到已删除的“三帧失败回StandingPolicy”旧逻辑。
6. 厂商M8010头文件的力矩注释与项目通过重量闭合得到的“乘6.333”解释冲突，精密力控前需再确认。
7. V30 manifest保留训练交付时 `hardware_status=NOT_TESTED`，但仓库外后续已有实机操作；两者时间口径不同。
8. UART直连方案的三份文档已在工作区修改但尚未提交；当前系统又尚未启用 `/dev/ttyS1`。
9. Robot AP代码路线已放弃，但 `wechat_mini_program/`、remote、push脚本及hostapd/dnsmasq系统软件仍存在。

## 14. 当前已知风险与后续待办

按优先级建议：

1. **P0：实机换向越界。** V30在后退回零再前进的过渡中，joint[9]目标越过机械上限。实现明确的“先到零—驻留—再反向”命令状态机，并在MuJoCo和真机复测。
2. **P0：仓库未同步。** CXK领先远端4提交且有未提交UART文档；先备份和明确提交范围，避免板卡损坏后丢失唯一版本。
3. **P1：供电完整性。** 历史快闪/MError=5与欠压有关。建立电源、接头、线径、压降和瞬态电流检查表。
4. **P1：UART遥控未完成。** `/dev/ttyS1`不存在；真实串口、ownership、watchdog、状态回传均待开发和分级测试。
5. **P1：安全阈值处于监控。** 目标速度、反馈速度、聚合变化目前不强制。需依据新模型实机分布和机构能力决定阈值。
6. **P1：系统网络暴露。** UFW inactive，adbd 5555和rpcbind 111监听所有接口；在不可信网络部署前需安全评估。
7. **P2：开机服务残留。** dnsmasq每次启动失败、hostapd残留；经授权后清理，不要在调试中途操作网络。
8. **P2：500 Hz热路径。** `PROJECT_ISSUES`记录每周期vector动态分配仍待优化；继续依据时序统计评估。
9. **P2：标定只测单侧限位。** 另一端由URDF行程推导，不能证明真实双侧机械行程。
10. **P2：文档清理。** 统一总线命名、模型示例、I2C描述和问题清单状态。

## 15. 新维护人员第一天检查清单

- [ ] 确认机器人、电机电源和吊带状态，确认当前没有控制进程。
- [ ] 阅读本文、`DATA_FLOW.md`、V30部署记录和UART V1协议。
- [ ] 执行只读系统/Git检查，不运行任何 `example_*`。
- [ ] 备份CXK领先的4个提交和未提交UART文档。
- [ ] 核对四路串口、I2C和overlay，不修改网络或设备树。
- [ ] 运行本文列出的纯软件测试和V30 golden replay。
- [ ] 确认ONNX/PT SHA和manifest完整性。
- [ ] 把FL/FR/RL/RR的串口、GPIO和ID表贴在设备旁。
- [ ] 与原负责人确认电源规格、急停方式、吊带流程和最近故障。
- [ ] 首次上电只做10秒阻尼通信，不直接标定/站立。
- [ ] 通过阻尼通信后再安排有监护人的标定和零命令站立。
- [ ] 在修复换向状态机前，不做前后直接反向测试。

## 16. 关键参数速查表

| 参数 | 值 |
|---|---|
| 主控 | RK3588 / Orange Pi 5 Pro / arm64 |
| 电机 | 12×GO-M8010-6，4路RS-485，每路3台 |
| IMU | JY901S，`/dev/i2c-1`，200 Hz |
| 状态估计 | 50 Hz，默认Complementary |
| 策略 | V30/model_1299，50 Hz |
| 网络接口 | `obs[1,48] float32 → actions[1,12] float32` |
| action scale | 0.25 |
| 正常PD | Kp=0.625，Kd=0.0125 |
| 退出阻尼 | Kp=0，Kd=0.136，约200 ms后回收线程 |
| 默认姿态 | hip `[0.1,-0.1,0.1,-0.1]`；thigh `[0.8,0.8,1,1]`；calf全为`-1.5` rad |
| 命令范围 | vx/vy/yaw均 `[-1,1]` |
| 命令斜率 | 0.5 / 0.5 m/s²；yaw 1.0 rad/s² |
| 命令超时/最大hold | 500 ms / 5000 ms |
| 电机反馈超时 | 100 ms |
| 上层线程心跳 | 100 ms |
| 过温保护 | 90°C |
| NN接管 | 2 s / 100帧 smoothstep |
| UART新方案 | UART1_M1 `/dev/ttyS1`，3.3V TTL，115200-8-N-1；当前未启用 |
| 当前SSH地址 | 审计时 `192.168.31.100`；网络变化后需重新确认 |

---

## 附录：本次审计实际验证结果

未启动电机、未运行 `robot_control`、未切换网络、未修改服务或overlay。执行了纯软件回归：double buffer、state estimator、legged odometry、foot force、policy observation、NN validation、velocity manager、runtime safety、motion safety、smooth takeover、Linear KF、UART protocol和joint mapping均PASS；V30 200帧golden replay PASS；manifest 44/44文件SHA匹配。

曾按名称把 `test_projected_gravity` 纳入测试列表，但检查后确认它是I2C实机诊断；普通用户因无I2C权限初始化失败并立即退出，未访问电机。后续维护人员应以CMake注释和源码判断测试是否接触硬件，而不能只看 `test_` 前缀。
