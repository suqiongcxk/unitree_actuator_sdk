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
│ A面/B面  │   │ A面  A面/B面 │   │  slots_    slots_            │
└────┬─────┘   └──┬───┬───────┘   └──┬───────────────────────────┘
     │            │   │              │
     │ DoubleBuf  │   │ DoubleBuf    │ mutex + copy
     │ 指针读     │   │ 指针读       │ 值拷贝
     │            │   │              │
     ▼            ▼   ▼              ▼
    读B面        读B面 写A面         读→拷贝→返回
    (不拷贝)     (不拷贝) (拷贝写入)  (值拷贝进出)
```

---

## 通信路径 #1: IMU Thread → State Estimation Thread

### 数据：`IMURawData` (角度、加速度、角速度、四元数、时间戳)

```
┌─────────────────────────────────────────────────────────────┐
│                      RobotController                        │
│                                                             │
│   imu_buffer_  (DoubleBuffer<IMURawData>)                   │
│   ┌──────────────────────────────────────────┐              │
│   │  slot_[0]  ←── 写者当前槽                │              │
│   │  slot_[1]  ←── 读者当前槽                │              │
│   │  write_idx_ (atomic<int>)                │              │
│   │  seq_       (atomic<uint32_t>)           │              │
│   └──────────────────────────────────────────┘              │
│          ▲                     │                            │
│          │ 写入 (直接修改)      │ 读取 (const指针)            │
│          │                     ▼                            │
│   ┌──────┴──────┐    ┌─────────────────┐                   │
│   │ IMU Thread  │    │  EST Thread     │                   │
│   │             │    │                 │                   │
│   │ acquireWriteSlot()                │                    │
│   │   → T& 引用 │    │ tryAcquireRead()│                   │
│   │   直接写入   │    │   → const T*   │                   │
│   │   槽内字段   │    │   只读指针      │                   │
│   │             │    │                 │                   │
│   │ commitWrite()│   │ 不拷贝，直接读   │                   │
│   │   翻转idx   │    │ 槽内数据         │                   │
│   └─────────────┘    └─────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

### 回答：**复用同一个地址（共享内存 + 双缓冲）**

- 数据**不拷贝**。IMU线程获得`slot_[write_idx_]`的**引用**，直接写入槽内内存。
- 写完调用`commitWrite()`原子翻转索引，读者即可读取。
- 两个槽**永远不会同时被读写**：写者写A面时读者读B面，反之亦然。
- **内存归属**: `imu_buffer_` 是 `RobotController` 的成员变量，生命周期由 `RobotController` 管理。

### 线程安全机制：**无锁 (lock-free)**
- `write_idx_` 原子变量翻转，`seq_` 原子计数器递增
- `commitWrite()`使用`memory_order_release`确保写入先于索引翻转
- `tryAcquireRead()`使用`memory_order_acquire`确保读取在索引翻转之后

### 代码示意
```cpp
// === IMU Thread (写者) ===
IMURawData& data = imu_buffer_.acquireWriteSlot();  // 获得引用，不拷贝
data.angles = ...;   // 直接写入buffer内的内存
data.acc    = ...;
imu_buffer_.commitWrite();  // 原子翻转: 这个槽变成"可读"

// === EST Thread (读者) ===
const IMURawData* imu = imu_buffer_.tryAcquireRead();  // 获得指针，不拷贝
if (imu != nullptr) {
    float ax = imu->acc.x;   // 直接读取buffer内的内存
    float gz = imu->gyro.z;
}
// 读者读完不"释放"——双缓冲保证了写者不会覆盖正在被读的槽
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
│   est_buffer_  (DoubleBuffer<EstimatedState>)              │
│   ┌──────────────────────────────────────────┐              │
│   │  slot_[0]  ←── 写者当前槽                │              │
│   │  slot_[1]  ←── 读者当前槽                │              │
│   │  write_idx_ (atomic<int>)                │              │
│   │  seq_       (atomic<uint32_t>)           │              │
│   └──────────────────────────────────────────┘              │
│          ▲                     │                            │
│          │ 写入 (拷贝赋值)      │ 读取 (const指针)            │
│          │                     ▼                            │
│   ┌──────┴──────┐    ┌─────────────────┐                   │
│   │ EST Thread  │    │   NN Thread     │                   │
│   │             │    │                 │                   │
│   │ est = estimator.update(...)        │                   │
│   │   → EstimatedState (栈上临时对象)   │                   │
│   │                                     │                   │
│   │ buf.acquireWriteSlot()             │                   │
│   │   = est;  ← 拷贝赋值到buffer槽     │                   │
│   │ buf.commitWrite()    const EstState* e =                │
│   │                      buf.tryAcquireRead()               │
│   │                          → 只读指针                     │
│   │                          不拷贝                          │
│   └─────────────┘    └─────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

### 回答：**写入时拷贝，读取时复用地址**

- EST 线程先在栈上构造 `EstimatedState` 临时对象（包括从IMU buffer读指针、从getState()值拷贝的数据）
- `acquireWriteSlot()` 返回槽引用 → **拷贝赋值** `est` 到槽内
- `commitWrite()` 原子翻转
- NN线程 `tryAcquireRead()` 返回槽的 **const 指针 → 不拷贝，直接读**
- 双缓冲保证读写的槽不会冲突

### 线程安全机制：**无锁 (lock-free)**
- 与路径#1相同的双缓冲机制

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
| 1 | IMURawData | IMU Thread | IMUBuffer | **写引用** (直接写入buffer槽) | 0 | 双缓冲无锁 |
| 2 | IMURawData | IMUBuffer | EST Thread | **读指针** (const T*, 直接读buffer槽) | 0 | 双缓冲无锁 |
| 3 | MotorState | Bus Threads | EST Thread | **值拷贝** (getState返回副本) | 1 | mutex |
| 4 | EstimatedState | EST Thread | EstimatedStateBuffer | **值拷贝** (拷贝赋值到buffer槽) | 1 | 双缓冲无锁 |
| 5 | EstimatedState | EstimatedStateBuffer | NN Thread | **读指针** (const T*, 直接读buffer槽) | 0 | 双缓冲无锁 |
| 6 | 电机指令 | NN Thread | Bus Threads | **值拷贝** (setPosition参数→slots_槽位) | 1 | mutex |
| 7 | 电机指令 | Bus slots_ | Bus 本地sendVec | **值拷贝** (slots_→sendVec) | 1 | mutex |
| 8 | 电机反馈 | Bus 本地recvVec | Bus slots_ | **值拷贝** (recvVec→slots_→getState返回) | 1 | mutex |

### 关键结论

```
双层数据架构:
┌──────────────────────────────────────────────────┐
│  传感器数据流 (IMU → EST → NN)                     │
│  使用: DoubleBuffer<T>                            │
│  策略: 复用同一块内存 (写引用/读指针)               │
│  拷贝: 0次 (除了写入buffer时的1次结构体赋值)        │
│  原因: 高频数据(200Hz), 避免拷贝开销               │
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
├── imu_buffer_          ← DoubleBuffer<IMURawData>
│   ├── slots_[0]        ← 2个槽，约120字节
│   └── slots_[1]
│
├── est_buffer_          ← DoubleBuffer<EstimatedState>
│   ├── slots_[0]        ← 2个槽，约200字节
│   └── slots_[1]
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

所有线程不拥有数据内存，只持有引用/指针/临时副本。
RobotController析构时自动释放所有内存。
```
