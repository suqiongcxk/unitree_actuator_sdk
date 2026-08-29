# 项目问题跟踪清单

> 用途：记录架构审阅和开发过程中发现的问题，后续逐项验证、修复并更新状态。  
> 建立日期：2026-08-09  
> 状态定义：`待确认`、`待修复`、`处理中`、`已解决`、`不处理`

## 问题汇总

| ID | 优先级 | 模块 | 问题 | 状态 |
|---|---|---|---|---|
| ISSUE-001 | P0 | 关节数据流 | 状态估计输出的关节排列与 NN 使用的 motor-ID 排列不一致 | 已解决 |
| ISSUE-002 | P0 | NN 安全验证 | `JOINT_LIMITS` 使用逐腿排列，但验证代码按数组下标/motor ID 访问 | 已解决 |
| ISSUE-003 | P0 | GPIO/RS-485 | `RobotController` 默认 GPIO chip/line 与硬件文档不一致 | 已解决 |
| ISSUE-004 | P0 | 并发通信 | `DoubleBuffer` 无法严格保证读指针使用期间对应槽不被覆盖 | 已解决 |
| ISSUE-005 | P1 | IMU | 四元数读取宽度与 Q30 缩放方式可能不匹配 | 待确认 |
| ISSUE-006 | P1 | 构建系统 | ONNX 头文件无条件包含，与 ONNX Runtime 可选编译设计冲突 | 待修复 |
| ISSUE-007 | P1 | 零点标定 | 当前“行程验证”由 URDF 行程反推，不能验证真实机械总行程 | 待确认 |
| ISSUE-008 | P1 | 实时控制 | 500 Hz 总线热路径每周期动态创建 vector | 待优化 |
| ISSUE-009 | P1 | 安全机制 | 缺少统一的通信、过温、CRC 和命令超时看门狗 | 处理中 |
| ISSUE-010 | P0 | 电机力矩 | SDK回传力矩的转子侧/关节侧语义曾被判断错误 | 已重新确认 |

## ISSUE-001：关节状态排列不一致

- 优先级：P0
- 位置：`robot_control/robot_controller.cpp:579-595`
- 现象：`estimationLoop()` 按总线顺序遍历电机，并通过递增的 `global_idx` 写入数组。
- 当前实际顺序：`[0,4,8,1,5,9,2,6,10,3,7,11]`。
- 其他模块约定：`EstimatedState`、默认站立姿态和 NN 输出均把数组下标当作 motor ID，即 `[0,1,...,11]`。
- 风险：NN 可能读取错位的关节位置、速度和力矩，进而向错误关节生成控制目标。
- 建议方向：直接使用 `mid` 写入 `joint_q[mid]` 等数组，或建立显式的 `PolicyIndex ↔ MotorID` 映射。
- 验证要求：增加 12 关节唯一标记的排列单元测试。
- 处理结果（2026-08-09）：状态估计改为按 `motor_id` 写入四组关节数组，并在初始化阶段检查 ID 0..11 是否完整、唯一且合法；`EstimatedState` 注释同步明确该约定。

## ISSUE-002：NN 关节限位表排列不一致

- 优先级：P0
- 位置：`robot_control/nn_validation.h:17-34`、`robot_control/nn_validation.cpp:35-55`
- 现象：`JOINT_LIMITS` 的内容按 Leg1、Leg2、Leg3、Leg4 排列，但 `validateJointLimits()` 用下标 `0..11` 直接访问。
- 风险：部分 thigh/calf 目标会使用其他关节类型的限位，可能误放行危险目标或错误拦截正常目标。
- 建议方向：将限位表严格改为 motor-ID/Z 字顺序，或按 motor ID 建立具名配置表。
- 验证要求：分别测试 12 个关节的上下限边界及越界值。
- 处理结果（2026-08-09）：限位表已改为 motor-ID 顺序；`margin` 改为向机械行程内部收缩；完整验证覆盖位置、NaN/Inf、单帧跳变以及 KP/KD 范围；连续 3 帧异常回退 StandingPolicy；ONNX 输出不足 12 维直接失败。
- 测试结果：新增纯软件目标 `test_nn_validation`，覆盖安全指令、Motor 4 排列、限位余量、NaN、过大 KP 和单帧跳变，当前全部通过。

## ISSUE-003：GPIO 编号体系混用

- 优先级：P0
- 位置：`robot_control/robot_controller.cpp:44-50`
- 现象：`RobotController::getDefaultConfig()` 使用 `(chip=0, line=133/39/35/63)`；`ParallelBus` 会将它们直接交给 GPIO v2 ioctl。
- 硬件文档映射：GPIO 133=`chip4:line5`，39=`chip1:line7`，35=`chip1:line3`，63=`chip1:line31`。
- 对比：同步 `MotorBus` 接收全局 GPIO 编号，并在内部通过 `/32`、`%32` 转换；两个 API 的参数语义不同。
- 风险：初始化失败、请求错误 GPIO line，或 RS-485 DE/RE 无法正确翻转。
- 建议方向：统一只使用 `gpio_chip + gpio_line`；若需要全局编号，提供明确命名的转换函数或工厂方法。
- 验证要求：逐路运行 GPIO 翻转测试并用示波器/逻辑分析仪确认 DE/RE 引脚。
- 处理结果（2026-08-09）：默认并行总线配置已改为 `chip4:5`、`chip1:7`、`chip1:3`、`chip1:31`；代码已编译，仍需硬件逐路验证。

## ISSUE-004：DoubleBuffer 读指针可能被生产者覆盖

- 优先级：P0
- 位置：`robot_control/shared_data.h:18-61`
- 现象：读者获得裸 `const T*` 后没有占用/释放协议；如果生产者在读者使用期间提交两次，就可能绕回并覆盖读者正在访问的槽。
- 附加问题：`last_read_seq_` 非原子且属于 buffer 对象，而监控 API、启动等待和内部线程可能形成多个读者，违反当前单读者假设。
- 风险：读取撕裂、未定义行为，以及状态估计或 NN 得到同一帧中混合的新旧字段。
- 建议方向：评估序列锁、三缓冲、读侧快照复制，或带读者占用标志的 SPSC 缓冲。
- 验证要求：ThreadSanitizer 压力测试，并模拟生产者远快于消费者的情况。
- 处理结果（2026-08-27）：保留`DoubleBuffer<T>`类名以减少接口改动，内部改为“单写者预备槽 + mutex保护的已发布快照”；读者在短临界区内复制值，并各自持有独立序列号，不再暴露可被覆盖的裸指针。
- 测试结果：30万帧、一快一慢两个并发读者压力测试通过；同一测试在ThreadSanitizer下通过，未报数据竞争或撕裂。

## ISSUE-005：JY901S 四元数格式待核对

- 优先级：P1
- 位置：`State_Estimation/jy901s.cpp:251-265`
- 现象：每个四元数分量读取 2 字节并按 `int16_t` 解析，但缩放因子使用 `1 / 2^30`（Q30）。
- 困惑：16 位原始值通常无法承载完整 Q30 分量，需要根据当前 JY901S 型号和寄存器手册确认实际格式。
- 风险：四元数幅值接近零或归一化错误，导致姿态及 NN 重力方向输入错误。
- 建议方向：以官方寄存器手册和实测单位四元数为准确定解析格式，并检查模长是否接近 1。
- 验证要求：静止平放时记录原始寄存器值、转换结果和四元数模长。

## ISSUE-006：ONNX 可选编译不完整

- 优先级：P1
- 位置：`robot_control/nn_policy.h:116`、根目录 `CMakeLists.txt` 的 ONNX 检测段
- 现象：CMake 在 ONNX Runtime 不存在时不定义 `ONNXRUNTIME_AVAILABLE`，但 `nn_policy.h` 仍无条件包含 `<onnxruntime_cxx_api.h>` 并声明 ONNX 类型成员。
- 风险：未安装 ONNX Runtime 时，`robot_control` 仍可能在编译阶段因缺少头文件失败，无法按设计回退到 `StandingPolicy`。
- 建议方向：用 `#ifdef ONNXRUNTIME_AVAILABLE` 包裹 include、类声明和实现，或拆分为独立 ONNX 源文件/target。
- 验证要求：分别在存在和不存在 ONNX Runtime 的环境执行干净构建。

## ISSUE-007：机械行程验证并未实际测量双侧限位

- 优先级：P1
- 位置：`motor_lib/ZeroPointCalibration.cpp:256-275`、`motor_lib/ZeroPointCalibration.cpp:495-549`
- 现象：标定只撞一侧机械限位，另一侧由 `zero_offset + motor_direction × urdf_limit` 推导，因此 `measured_range` 本质上等于配置的 `urdf_range`。
- 风险：机械装配行程异常、URDF 配置错误或另一侧限位偏移无法被当前验证发现。
- 建议方向：若确实需要验证机械总行程，应安全地测量双侧限位；否则将该步骤改名为“映射一致性检查”，避免产生错误安全保证。
- 验证要求：先确认是否允许每个关节双向撞限位，以及机械安全条件。

## ISSUE-008：500 Hz 总线热路径动态分配

- 优先级：P1
- 位置：`motor_lib/parallel_bus.cpp:207-228`
- 现象：每个控制周期创建 `sendVec` 和 `recvVec`，可能发生堆分配。
- 风险：引入不可预测的延迟和抖动，不利于实时控制。
- 建议方向：在 `start()` 前按电机数量预分配固定缓冲，控制循环中只覆盖已有对象。
- 验证要求：记录控制周期延迟分布、最大延迟和 deadline miss 次数。

## ISSUE-009：运行时安全保护尚未集中实现

- 优先级：P1
- 涉及模块：`parallel_bus.cpp`、`robot_controller.cpp`、`nn_validation.cpp`
- 现象：已有零散 CRC、错误码和 NN 输出验证，但缺少统一安全状态机。
- 建议至少覆盖：
  - 连续 CRC/通信失败阈值；
  - 反馈陈旧时间戳；
  - 电机过温、过流和编码器错误；
  - NN 指令超时/未更新；
  - 控制线程 deadline miss；
  - 异常时统一降级到阻尼或刹车模式。
- 建议方向：建立集中式 `SafetySupervisor`，定义故障等级、触发条件、锁存与恢复策略。
- 当前进展（2026-08-27）：已加入进程级急停、总线阻尼锁存、反馈CRC/ID检查、每电机反馈时间戳、100 ms新鲜度阈值、连续失败计数和NN状态流看门狗；新增首故障集中锁存与IMU/EST/NN 100 ms心跳看门狗。两轮实机基线显示单次总线循环可达约61 ms但未超过100 ms，因此周期超期只统计，心跳陈旧或连续状态异常才停机。
- 2026-08-27实机故障注入已验证IMU数据流中断在101 ms锁存并安全退出，ID0反馈超时在约140.17 ms锁存`MOTOR_FEEDBACK_INVALID detail=0`并安全退出；两次均完成12电机阻尼和线程回收。
- 已从本地GO-M8010-6协议头确认`temp`单位、90°C保护值及MError 1..4含义，并接入90°C/非零MError集中锁存；边界纯软件测试通过，不进行危险的真实过温/过流注入。剩余命令超时随Phase 4真实命令源一起实现。

## ISSUE-010：GO-M8010-6 力矩语义重新确认

- 优先级：P0
- 位置：`motor_controller.cpp`、`parallel_bus.cpp` 的命令和反馈换算。
- 重新核对：结合Unitree官方调试/服务代码和已知约10 kg整机的静态重力闭合，确认SDK回传`tau`为转子侧 N·m，进入足端雅可比力反解前需乘`6.333`。
- 处理结果（2026-08-27）：`EstimatedState::joint_torque`保留原始转子侧语义以兼容既有接口和日志，仅`FootForceEstimator`在动力学计算入口乘减速比；电机指令通道本轮未改动。
- 验证结果：4477帧离线数据全部求解有效，稳定段四足平均法向力总和约`98.65 N`，与整机重力一致；精密力控仍需力台标定。

## 后续更新约定

每次处理问题时，至少更新以下内容：

1. 汇总表中的状态；
2. 最终原因和采用的方案；
3. 修改文件及关键位置；
4. 已执行的测试和结果；
5. 若暂不修复，记录明确原因和风险接受结论。
