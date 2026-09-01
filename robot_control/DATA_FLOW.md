# 线程间数据通信详解

## 核心问题：传值(拷贝) vs 复用地址(共享)？

本文档详细描述每条数据通信路径的**内存归属**、**传递方式**、**线程安全机制**。

---

## 总览图

```
                    RobotController (owner of ALL memory)
                    ======================================
                    │                                     │
    ┌───────────────┤  imu_buffer_                        │
    │               │  est_buffer_                        │
    │               │  motor_ctrl_ (owns 4×ParallelBus)   │
    │               │                                     │
    ▼               ▼                                     ▼
┌──────────┐   ┌──────────────┐   ┌──────────────────────────────┐
│IMU Thread│   │  EST Thread  │   │  4× ParallelBus Threads      │
│ 200Hz    │   │  50Hz        │   │  500Hz each                  │
│          │   │              │   │                              │
│ 写入     │   │ 读取 写入    │   │  读取指令 写入反馈            │
│ ↓        │   │ ↑    ↓       │   │  ↑         ↓                 │
│ 预备/发布 │   │ 快照 预备/发布│   │  slots_    slots_            │
└────┬─────┘   └──┬───┬───────┘   └──┬───────────────────────────┘
     │            │   │              │
     │ 快照短锁   │   │ 快照短锁     │ mutex + copy
     │ 值拷贝     │   │ 值拷贝       │ 值拷贝
     │            │   │              │
     ▼            ▼   ▼              ▼
    独立副本      独立副本 写预备槽    读→拷贝→返回
    (可长期用)    (可长期用) (发布拷贝)  (值拷贝进出)
```

---

## 通信路径 #1: IMU Thread → State Estimation Thread

### 数据：`IMURawData` (角度、加速度、角速度、四元数、时间戳)

```
┌─────────────────────────────────────────────────────────────┐
│                      RobotController                        │
│                                                             │
│   imu_buffer_  (线程安全值快照，类名暂保留DoubleBuffer)     │
│   ┌──────────────────────────────────────────┐              │
│   │  writer_slot_ ←── 仅写线程访问           │              │
│   │  published_   ←── mutex保护              │              │
│   │  sequence_    ←── mutex保护              │              │
│   └──────────────────────────────────────────┘              │
│          ▲                     │                            │
│          │ 写入预备槽           │ 短锁值拷贝                  │
│          │                     ▼                            │
│   ┌──────┴──────┐    ┌─────────────────┐                   │
│   │ IMU Thread  │    │  EST Thread     │                   │
│   │             │    │                 │                   │
│   │ acquireWriteSlot()                │                    │
│   │   → T& 引用 │    │ tryRead(out,seq)│                   │
│   │   直接写入   │    │   → 独立副本  │                   │
│   │   预备槽字段 │    │   锁外读取    │                   │
│   │             │    │                 │                   │
│   │ commitWrite()│   │ 拷贝后立即解锁 │                   │
│   │   短锁发布  │    │ 使用本地数据   │                   │
│   └─────────────┘    └─────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

### 回答：**mutex保护的值快照**

- IMU线程直接填写仅由它访问的`writer_slot_`，这一步不加锁。
- `commitWrite()`在短临界区内将预备槽复制到`published_`并递增序列号。
- EST线程用`tryRead(out, reader_sequence)`在短临界区取得独立值副本，锁外运行估计。
- 每个读者拥有自己的`reader_sequence`，多个读者不会互相吞掉更新。
- **内存归属**: `imu_buffer_` 是 `RobotController` 的成员变量，生命周期由 `RobotController` 管理。

### 线程安全机制：**mutex + 值拷贝**
- 锁只覆盖一次结构体复制和序列号访问，不覆盖I2C、状态估计或NN推理。
- 该设计消除了旧双缓冲在生产者连续提交两帧后绕回覆盖读指针的竞态。

### 代码示意
```cpp
// === IMU Thread (写者) ===
IMURawData& data = imu_buffer_.acquireWriteSlot();  // 仅写线程访问
data.angles = ...;   // 直接写入buffer内的内存
data.acc    = ...;
imu_buffer_.commitWrite();  // 短锁发布一个值快照

// === EST Thread (读者) ===
IMURawData imu{};
IMUBuffer::Sequence cursor = 0;  // 每个读者各自持有
if (imu_buffer_.tryRead(imu, cursor)) {
    float ax = imu.acc.x;    // 锁外读取独立副本
    float gz = imu.gyro.z;
}
```

---

## 通信路径 #2: Motor Bus Threads → State Estimation Thread

### 数据：`MotorState` (关节位置、速度、力矩、温度、错误码)

```
┌─────────────────────────────────────────────────────────────┐
│                    MultiBusController                       │
│                                                             │
│  ParallelBus[0]              ParallelBus[3]                 │
│  ┌─────────────────┐        ┌─────────────────┐            │
│  │ slots_mtx_      │        │ slots_mtx_      │            │
│  │ slots_[0].data  │        │ slots_[0].data  │            │
│  │ slots_[1].data  │        │ slots_[1].data  │            │
│  │ slots_[2].data  │        │ slots_[2].data  │            │
│  └────────┬────────┘        └────────┬────────┘            │
│           │                          │                      │
│    getState(id):               getState(id):               │
│      lock(mtx)                   lock(mtx)                 │
│      data = slots_[i].data       data = slots_[i].data     │
│      unlock(mtx)                 unlock(mtx)               │
│      return COPY of data         return COPY of data       │
│           │                          │                      │
└───────────┼──────────────────────────┼──────────────────────┘
            │                          │
            ▼                          ▼
     ┌──────────────────────────────────────┐
     │         EST Thread (50Hz)            │
     │                                      │
     │  for b in 0..3:                      │
     │    for m in 0..2:                    │
     │      MotorState s =                 │
     │        bus(b).getState(m);  ← 值拷贝 │
     │      joint[gid] = s.q;              │
     └──────────────────────────────────────┘
```

### 回答：**值拷贝 (copy)**

- `ParallelBus::getState()` 内部：lock mutex → 从 `slots_[i].data` **拷贝** → unlock → return 拷贝
- EST 线程得到的是一个**独立的 MotorState 副本**，与总线线程的 `slots_` 内存完全分离
- 拷贝完成后立即解锁，互斥锁持有时间极短（~微秒级）

### 为什么用值拷贝而不是共享指针？
- MotorState 只有 ~24 字节，拷贝成本极低
- 锁持有时间短：拷贝 + 减速比转换，总共不超过几微秒
- 安全：返回后 EST 线程拥有独立数据，总线线程可以继续覆盖 `slots_` 互不干扰

### 线程安全机制：**互斥锁 (std::mutex)**
- `ParallelBus` 内部已有 `slots_mtx_`
- `getState()` 和 `setPosition()` 以及总线线程的 `controlLoop()` 都使用同一把锁

### 代码示意
```cpp
// ParallelBus::getState() 内部实现 (parallel_bus.cpp:124-143)
MotorState ParallelBus::getState(unsigned short motor_id) const {
    std::lock_guard<std::mutex> lock(slots_mtx_);  // 加锁
    MotorState s;                                    // 栈上分配
    for (const auto& slot : slots_) {
        if (slot.id != motor_id) continue;
        s.q  = slot.data.q  / gear_ratio_;           // 值拷贝 + 转换
        s.dq = slot.data.dq / gear_ratio_;
        // ...
        break;
    }
    return s;  // 解锁(lock_guard析构) + 返回副本
}
```

---

## 通信路径 #3: State Estimation Thread → NN Thread

### 数据：`EstimatedState`（本体姿态、线速度、角速度、12关节状态、接触标志）

```
┌─────────────────────────────────────────────────────────────┐
│                      RobotController                        │
│                                                             │
│   est_buffer_  (线程安全值快照，类名暂保留DoubleBuffer)     │
│   ┌──────────────────────────────────────────┐              │
│   │  writer_slot_ ←── 仅EST线程访问          │              │
│   │  published_   ←── mutex保护              │              │
│   │  sequence_    ←── mutex保护              │              │
│   └──────────────────────────────────────────┘              │
│          ▲                     │                            │
│          │ 写入预备槽           │ 短锁值拷贝                  │
│          │                     ▼                            │
│   ┌──────┴──────┐    ┌─────────────────┐                   │
│   │ EST Thread  │    │   NN Thread     │                   │
│   │             │    │                 │                   │
│   │ est = estimator.update(...)        │                   │
│   │   → EstimatedState (栈上临时对象)   │                   │
│   │                                     │                   │
│   │ buf.acquireWriteSlot()             │                   │
│   │   → 在预备槽构造状态             │                   │
│   │ buf.commitWrite()    EstimatedState e;                 │
│   │   → 短锁发布       buf.tryRead(e, cursor)             │
│   │                          → 独立副本                     │
│   │                          锁外推理                       │
│   └─────────────┘    └─────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

### 回答：**发布时拷贝，读取时再取得独立值副本**

- EST线程在仅写者可见的预备槽构造`EstimatedState`。
- `commitWrite()`在短锁内发布一份完整快照。
- NN线程通过`tryRead(est, cursor)`复制到自己的栈变量，推理期间不持锁。
- 启动等待或其他读者使用独立cursor，不会干扰NN线程。

### 线程安全机制：**mutex + 值拷贝**
- 与路径#1相同的快照机制；锁不覆盖估计算法和NN推理。

---

## 通信路径 #4: NN Thread → Motor Bus Threads (发送指令)

### 数据：电机指令（目标位置、KP、KD）

```
┌─────────────────────────────────────────────────────────────┐
│                      NN Thread (50Hz)                       │
│                                                             │
│  NNCommandSet cmds;                                         │
│  policy.infer(*est, cmds);  ← 计算出12个电机的目标值         │
│                                                             │
│  for each bus b in 0..3:                                    │
│    for each motor m in 0..2:                                │
│      bus(b).setPosition(m, q, kp, kd);  ←─── 值传递(函数参数)│
│                                                             │
└─────────────────────────┬───────────────────────────────────┘
                          │
             值作为函数参数传入 setPosition()
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    ParallelBus (Bus Thread 500Hz)           │
│                                                             │
│  void setPosition(id, q, kp, kd) {                          │
│      lock_guard lock(slots_mtx_);   ← 加锁                 │
│      for (auto& slot : slots_) {                            │
│          if (slot.id == id) {                               │
│              slot.cmd.q  = q;       ← 值拷贝到槽内          │
│              slot.cmd.kp = kp;      ←                      │
│              slot.cmd.kd = kd;      ←                      │
│              break;                                         │
│          }                                                  │
│      }                                                      │
│  }  ← 解锁(lock_guard析构)                                  │
│                                                             │
│  ── 异步 ──                                                 │
│                                                             │
│  controlLoop() {   ← 在独立线程中运行                        │
│      lock(slots_mtx_);                                      │
│      copy slot.cmd → sendVec;    ← 拷贝到本地，立即解锁      │
│      unlock(slots_mtx_);                                    │
│                                                             │
│      for each cmd in sendVec:                               │
│          gpio→set(1);  serial→send(...);                    │
│          gpio→set(0);  serial→recv(...);                    │
│                                                             │
│      lock(slots_mtx_);                                      │
│      slot.data = recv;            ← 写回反馈                │
│      unlock(slots_mtx_);                                    │
│  }                                                          │
└─────────────────────────────────────────────────────────────┘
```

### 回答：**值拷贝 (copy) —— 分两段**

**第一段: NN线程 → ParallelBus的slots_槽位 (值拷贝)**
- NN线程调用 `setPosition(m, q, kp, kd)`，参数 `q, kp, kd` 是 float 值
- `setPosition()` 内部加锁后将值**拷贝**到 `slots_[i].cmd` 结构体中
- 拷贝完成后解锁，NN线程可以继续做其他事情

**第二段: ParallelBus的slots_ → 总线线程本地sendVec (值拷贝)**
- 总线线程的 `controlLoop()` 每个周期开始时：加锁 → **拷贝** `slots_[i].cmd` 到本地 `sendVec` → 立即解锁
- 后续的硬件收发操作使用本地副本，不持锁
- 收发完成后：加锁 → 将结果**拷贝**回 `slots_[i].data` → 解锁

### 为什么设计成两段拷贝？
- **最小化锁持有时间**: 硬件收发（GPIO翻转、串口读写、TX完成等待）可能耗时几百微秒，不在锁内做
- **指令及时性**: NN线程随时可以写入新指令，总线线程下个周期自动取到最新值
- **无竞争**: 总线线程只在周期开始时拷贝指令，NN线程可以在任何时刻写入。

### 线程安全机制：**互斥锁 (std::mutex)**
- `ParallelBus` 内部的 `slots_mtx_` 保护所有对 `slots_` 的读写
- `setPosition()` 和 `controlLoop()` 竞争同一把锁，但在不同时间段使用

### 代码示意
```cpp
// NN线程侧 — 写入指令
bus.setPosition(0, 1.57f, 0.03f, 0.01f);  // 值传递，内部拷贝到slots_

// ParallelBus内部 — 总线线程侧 (parallel_bus.cpp:188-286)
void ParallelBus::controlLoop() {
    while (running_) {
        // 第1步: 拷贝指令出来（持锁）
        vector<MotorCmd> sendVec;
        {
            lock_guard<mutex> lock(slots_mtx_);
            for (auto& slot : slots_) {
                sendVec.push_back(slot.cmd);  // 值拷贝
            }
        }  // 解锁 — 锁持有 ~微秒

        // 第2步: 硬件收发（无锁，操作本地副本）
        for (auto& cmd : sendVec) {
            applyGearRatio(cmd);            // 无锁
            gpio_->set(1);                  // 无锁
            serial_->send(...);             // 无锁
            // ... 等待TX完成 ...
            gpio_->set(0);                  // 无锁
            serial_->recv(...);             // 无锁
        }

        // 第3步: 写回反馈（持锁）
        {
            lock_guard<mutex> lock(slots_mtx_);
            for (size_t i = 0; i < recvVec.size(); i++) {
                slots_[i].data = recvVec[i];  // 值拷贝
            }
        }  // 解锁

        clock_nanosleep(...);  // 等待下一个周期
    }
}
```

---

## 汇总表：每条路径的传递方式

| # | 数据 | 从 | 到 | 传递方式 | 拷贝次数 | 线程安全 |
|---|------|----|----|---------|---------|---------|
| 1 | IMURawData | IMU Thread | IMUBuffer | **发布快照** (writer_slot→published) | 1 | mutex短锁 |
| 2 | IMURawData | IMUBuffer | EST Thread | **值快照** (published→读者栈变量) | 1 | mutex短锁 |
| 3 | MotorState | Bus Threads | EST Thread | **值拷贝** (getState返回副本) | 1 | mutex |
| 4 | EstimatedState | EST Thread | EstimatedStateBuffer | **发布快照** (writer_slot→published) | 1 | mutex短锁 |
| 5 | EstimatedState | EstimatedStateBuffer | NN Thread | **值快照** (published→读者栈变量) | 1 | mutex短锁 |
| 6 | 电机指令 | NN Thread | Bus Threads | **值拷贝** (setPosition参数→slots_槽位) | 1 | mutex |
| 7 | 电机指令 | Bus slots_ | Bus 本地sendVec | **值拷贝** (slots_→sendVec) | 1 | mutex |
| 8 | 电机反馈 | Bus 本地recvVec | Bus slots_ | **值拷贝** (recvVec→slots_→getState返回) | 1 | mutex |

### 三维速度命令路径

`主终端或第二终端FIFO → VelocityCommandManager → NN Thread → PolicyObservationBuilder`：

- 输入为base frame的`[vx,vy,yaw_rate]`，训练范围均为`[-1,1]`。
- 第二终端通过命名FIFO发送文本；辅助监听线程只解析并提交命令，不访问串口、GPIO、IMU或电机。
- 提交时在短mutex临界区保存raw和limited；NN线程每个有效50 Hz周期取得applied值。
- applied经过斜率限制；输入超过看门狗期限后目标自动变为零并平滑回零。
- 只有applied进入Actor `observation[9..11]`；NaN/Inf永远不会被提交。
- `Ctrl+C`不经过命令斜坡，直接走全局急停和12电机阻尼锁存。
- FIFO中的`s`走同一个全局急停路径；原有7个实时线程仍为`1 IMU + 1 EST + 4 BUS + 1 NN`，额外线程仅属于非实时人机输入。
- `keyboard_velocity_console.py`通过同一FIFO提供`W/S/A/D/Q/E`实时键盘命令；脚本只使用不带`O_CREAT`的非阻塞写入，不会在主进程退出时遗留同名普通文件。
- 键盘控制台对`vx/vy/yaw`分别实施默认650 ms的死手超时：只有持续收到该轴按键或终端键盘重复事件时才保持非零，停止按键后该轴自动平滑回零。激活期间以100 ms周期刷新命令，不设显式`hold_ms`；控制台崩溃或断开后，主进程500 ms看门狗仍是第二层自动回零保护。
- `+/-`将前后和左右速度幅值同时增减`0.1 m/s`，`]/[`将偏航速度增减`0.1 rad/s`；所有按键只更改目标值，最终输入Actor的命令仍统一经过`VelocityCommandManager`的线速度`0.5 m/s²`和偏航`1.0 rad/s²`斜率限制，正反向切换不会单帧跳变。

### NN指令下发前的独立运动保护

`NN候选目标 + 上一帧已接受目标 + 关节反馈速度 → MotionSafety → commit/setPosition`：

- 保护位于NN推理之后、`commitAcceptedCommand()`和总线`setPosition()`之前；失败帧不会更新任何指令历史，也不会写入总线。
- 默认commissioning门限为目标变化率`2.5 rad/s`（50 Hz时`0.05 rad/帧`）、实测关节速度`3.0 rad/s`、单帧12关节绝对变化总和`0.25 rad`。
- 当超过`0.03 rad/帧`的关节多于3个时，也按多关节协同突变锁存停机。
- 该保护独立于`--no-validate`，不能通过关闭NN验证绕过；dry-run仍检查候选目标，但不因人工搬动机器人的反馈速度停机。
- 任一保护触发后保持总线已有的最后安全目标，由统一安全退出发送12电机`Kd=0.136`阻尼并回收线程。
- 上述门限来自当前零命令成功日志与`0.20 m/s`故障日志之间的实测间隔，仍需用新策略的多seed仿真关节速度分布复核。

### 关键结论

```
双层数据架构:
┌──────────────────────────────────────────────────┐
│  传感器数据流 (IMU → EST → NN)                     │
│  使用: SnapshotBuffer语义（类名暂为DoubleBuffer）   │
│  策略: writer槽 + mutex保护的发布快照 + 读者副本    │
│  拷贝: 发布1次、读取1次                             │
│  原因: 数据量小，优先消除悬空/覆盖指针和多读者竞态   │
├──────────────────────────────────────────────────┤
│  电机指令/反馈流 (NN → Bus / Bus → EST)            │
│  使用: mutex + 值拷贝                              │
│  策略: 每次调用都产生独立副本                       │
│  拷贝: 每段1次                                     │
│  原因: 数据量小(~24字节), 安全第一, 拷贝快         │
└──────────────────────────────────────────────────┘
```

---

## 内存归属一览

```
RobotController (主线程创建，生命周期覆盖所有子线程)
│
├── imu_buffer_          ← SnapshotBuffer语义<IMURawData>
│   ├── writer_slot_     ← 仅IMU写线程访问
│   └── published_       ← mutex保护的已发布快照
│
├── est_buffer_          ← SnapshotBuffer语义<EstimatedState>
│   ├── writer_slot_     ← 仅EST写线程访问
│   └── published_       ← mutex保护的已发布快照
│
├── imu_                 ← unique_ptr<JY901S>
│
└── motor_ctrl_          ← unique_ptr<MultiBusController>
    ├── ParallelBus[0]   ← unique_ptr, 拥有3个MotorSlot
    │   └── slots_[3]    ← 每个槽有 MotorCmd + MotorData
    ├── ParallelBus[1]
    │   └── slots_[3]
    ├── ParallelBus[2]
    │   └── slots_[3]
    └── ParallelBus[3]
        └── slots_[3]

线程只在各自拥有的预备槽或临时值副本上做耗时计算，不跨临界区持有共享指针。
RobotController析构时自动释放所有内存。
```
