#include "motor_controller.h"
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <algorithm>
#include <time.h>
#include <sys/ioctl.h>

// ═══════════════════════════════════════════════════════════════════════════════
// MotorController 实现
// ═══════════════════════════════════════════════════════════════════════════════

MotorController::MotorController(int gpio_pin, const std::string& serial_port,
                                 unsigned short motor_id)
    : gpio_pin_(gpio_pin)
    , gpio_(std::make_unique<FastGPIO>(gpio_pin_ / 32, gpio_pin_ % 32))
    , serial_port_(serial_port)
    , motor_id_(motor_id)
    , gear_ratio_(queryGearRatio(MotorType::GO_M8010_6))
    , connected_(false)
    , serial_(serial_port)   // SDK 内部配 4M 波特率, 勿再 open 第二 fd
{
    rx();   // 默认为接收模式（安全状态）

    // ── 初始化电机指令/数据结构体 ──
    cmd_.motorType = MotorType::GO_M8010_6;
    cmd_.id        = motor_id_;
    cmd_.mode      = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    cmd_.q         = 0.0f;
    cmd_.dq        = 0.0f;
    cmd_.kp        = 0.0f;
    cmd_.kd        = 0.0f;
    cmd_.tau       = 0.0f;

    data_.motorType = MotorType::GO_M8010_6;

    connected_ = true;
}

MotorController::~MotorController()
{
    connected_ = false;
    // FastGPIO 析构自动释放 GPIO 资源
}

// ── 私有辅助方法 ──

void MotorController::tx()
{
    // FastGPIO ioctl → ~1-5µs (vs sysfs 1-5ms)
    gpio_->set(1);
}

void MotorController::rx()
{
    gpio_->set(0);
}

/**
 * 将 cmd_ 中的输出端指令值转换为转子端（SDK 要求转子端输入）
 *
 * 减速器物理关系 (GR = 转子转速 / 输出转速):
 *   q_rotor  = q_output  * GR        (输出位置 × 减速比 = 转子位置)
 *   dq_rotor = dq_output * GR        (输出速度 × 减速比 = 转子速度)
 *   tau_rotor = tau_output / GR      (力矩被减速器放大，转子力矩 = 输出力矩 / 减速比)
 */
void MotorController::applyGearRatio()
{
    cmd_.q   *= gear_ratio_;
    cmd_.dq  *= gear_ratio_;
    if (cmd_.tau != 0.0f) {
        cmd_.tau /= gear_ratio_;
    }
}

// ── 公开 API ──

void MotorController::sendRecv()
{
    if (!connected_) return;

    // RS-485 半双工: 必须在 send/recv 之间翻转 GPIO 方向
    //
    // serial_.sendRecv() 内部是 write→recv 连续调用, GPIO 全程高电平,
    // 导致 recv 时收发器仍在发送模式, 永远收不到电机应答。
    // 拆分为 send + GPIO翻转 + recv。FastGPIO 翻转仅 ~1-5µs,
    // 而电机处理指令需 ~100-500µs, 翻转后完全来得及接收。

    // 1) 打包指令
    cmd_.modify_data(&cmd_);
    uint8_t* sendData = cmd_.get_motor_send_data();
    int sendLen = cmd_.hex_len;          // modify_data 设置: GO_M8010_6 = 17

    uint8_t* recvData = data_.get_motor_recv_data();

    // 2) 发送 → 等 TX 完成 → 翻 GPIO → 接收
    struct timespec t0, t1, t2, t3, t4;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    tx();
    clock_gettime(CLOCK_MONOTONIC, &t1);

    serial_.send(sendData, static_cast<size_t>(sendLen));
    clock_gettime(CLOCK_MONOTONIC, &t2);

    // ── TIOCOUTQ + TIOCSERGETLSR: 硬件级 TX 完成检测 ──
    //
    // tcdrain() 在 RK3588 内核 UART 驱动上间歇性阻塞 11-14ms (驱动用 ms 级
    // 轮询替代了硬件寄存器检查), 不可靠。
    //
    // 替代方案 (两步, 均为非阻塞 ioctl):
    //   (1) TIOCOUTQ:  等待内核 xmit 缓冲区清空 (数据推入硬件 FIFO)
    //   (2) TIOCSERGETLSR: 轮询 UART 硬件 TEMT 位 (TX FIFO + 移位寄存器全空)
    //       → 硬件传输真正完成的精确时刻
    //   → 翻转 GPIO 的时机精确到 ~1µs, 无内核调度参与
    {
        int fd = serial_.fd();

        // Step 1: 等内核缓冲清空 (通常 0-2 轮)
        int outq = 1, polls = 0;
        while (outq > 0 && polls < 10000) {
            if (ioctl(fd, TIOCOUTQ, &outq) < 0) break;
            polls++;
        }

        // Step 2: 轮询硬件 TX 状态 (TEMT=发送器完全空闲)
        // TIOCSERGETLSR → 读取 Line Status Register
        unsigned int lsr = 0;
        polls = 0;
        while (polls < 100000) {
            if (ioctl(fd, TIOCSERGETLSR, &lsr) < 0) break;
            if (lsr & TIOCSER_TEMT) break;  // TEMT=0x40: TX FIFO + 移位寄存器均空
            polls++;
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &t3);

    rx();
    clock_gettime(CLOCK_MONOTONIC, &t4);

    serial_.recv(recvData);
    data_.extract_data(&data_);  // 解包 raw bytes → 公有字段 + CRC 校验

    long tx_us    = (t1.tv_sec - t0.tv_sec) * 1000000L + (t1.tv_nsec - t0.tv_nsec) / 1000;
    long send_us  = (t2.tv_sec - t1.tv_sec) * 1000000L + (t2.tv_nsec - t1.tv_nsec) / 1000;
    long wait_us  = (t3.tv_sec - t2.tv_sec) * 1000000L + (t3.tv_nsec - t2.tv_nsec) / 1000;
    long rx_us    = (t4.tv_sec - t3.tv_sec) * 1000000L + (t4.tv_nsec - t3.tv_nsec) / 1000;
    std::cout << "tx=" << tx_us << "us send=" << send_us << "us wait=" << wait_us
              << "us rx=" << rx_us << "us total=" << (tx_us+send_us+wait_us+rx_us) << "us" << std::endl;



}

void MotorController::setPosition(float q, float kp, float kd)
{
    cmd_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_.q    = q;       // 暂存输出端值
    cmd_.kp   = kp;
    cmd_.kd   = kd;
    cmd_.dq   = 0.0f;
    cmd_.tau  = 0.0f;

    applyGearRatio();    // 输出端 → 转子端转换
    sendRecv();
}

void MotorController::setVelocity(float dq, float kd)
{
    cmd_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_.q    = 0.0f;
    cmd_.kp   = 0.0f;
    cmd_.kd   = kd;
    cmd_.dq   = dq;      // 暂存输出端值
    cmd_.tau  = 0.0f;

    applyGearRatio();
    sendRecv();
}
//设置阻尼模式
void MotorController::setdamping(float kd)
{
    cmd_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_.q    = 0.0f;
    cmd_.kp   = 0.0f;
    cmd_.kd   = kd  ;//设置kd系数，判断环境有关
    cmd_.dq   = 0.0f;
    cmd_.tau  = 0.0f;   

    applyGearRatio();
    sendRecv();

};

void MotorController::setTorque(float tau)
{
    cmd_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_.q    = 0.0f;
    cmd_.kp   = 0.0f;
    cmd_.kd   = 0.0f;
    cmd_.dq   = 0.0f;
    cmd_.tau  = tau;     // 暂存输出端值，applyGearRatio 中做除法

    applyGearRatio();
    sendRecv();
}

void MotorController::brake()
{
    cmd_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    cmd_.q    = 0.0f;
    cmd_.kp   = 0.0f;
    cmd_.kd   = 0.0f;
    cmd_.dq   = 0.0f;
    cmd_.tau  = 0.0f;

    // 刹车指令不需要减速比转换
    sendRecv();
}

MotorState MotorController::getState() const
{
    MotorState s;

    if (!connected_) return s;

    // ── 转子端 → 输出端转换（applyGearRatio 的逆过程） ──
    //
    // 注意: SDK 的 RIS_Fbk_t 结构体注释中 torque 写的是"关节输出扭矩(N·m)"，
    // 如果固件确实上报的是输出端力矩，把 TORQUE_IS_ROTOR 改成 false 即可。
    // 当前默认按转子端力矩处理，乘以减速比恢复输出端力矩。

    constexpr bool TORQUE_IS_ROTOR = true;   // 若固件上报输出端力矩则改为 false

    s.q       = data_.q  / gear_ratio_;      // 转子位置 → 输出位置
    s.dq      = data_.dq / gear_ratio_;      // 转子速度 → 输出速度
    s.tau     = TORQUE_IS_ROTOR ? (data_.tau * gear_ratio_) : data_.tau;
    s.temp    = data_.temp;
    s.merror  = data_.merror;
    s.correct = data_.correct;
    s.mode    = data_.mode;

    return s;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MotorBus 实现
// ═══════════════════════════════════════════════════════════════════════════════

MotorBus::MotorBus(int gpio_pin, const std::string& serial_port)
    : gpio_pin_(gpio_pin)
    , gpio_(std::make_unique<FastGPIO>(gpio_pin_ / 32, gpio_pin_ % 32))
    , serial_port_(serial_port)
    , gear_ratio_(queryGearRatio(MotorType::GO_M8010_6))
    , serial_(serial_port)
{
    rx();   // 默认为接收模式
}

MotorBus::~MotorBus()
{
    // FastGPIO 析构自动释放 GPIO 资源
}

MotorBus::MotorSlot* MotorBus::findSlot(unsigned short motor_id)
{
    for (auto& slot : slots_) {
        if (slot.id == motor_id) return &slot;
    }
    return nullptr;
}

bool MotorBus::addMotor(unsigned short motor_id)
{
    if (findSlot(motor_id)) return false;   // ID 已存在

    MotorSlot s;
    s.id             = motor_id;
    s.cmd.motorType  = MotorType::GO_M8010_6;
    s.cmd.id         = motor_id;
    s.cmd.mode       = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    s.cmd.q          = 0.0f;
    s.cmd.dq         = 0.0f;
    s.cmd.kp         = 0.0f;
    s.cmd.kd         = 0.0f;
    s.cmd.tau        = 0.0f;
    s.data.motorType = MotorType::GO_M8010_6;

    slots_.push_back(s);
    return true;
}

bool MotorBus::removeMotor(unsigned short motor_id)
{
    auto it = std::remove_if(slots_.begin(), slots_.end(),
                             [motor_id](const MotorSlot& s) { return s.id == motor_id; });
    if (it == slots_.end()) return false;
    slots_.erase(it, slots_.end());
    return true;
}

void MotorBus::tx() { gpio_->set(1); }
void MotorBus::rx() { gpio_->set(0); }

// ── 暂存指令（输出端量纲，直接在暂存时转换为转子端） ──

void MotorBus::setPosition(unsigned short motor_id, float q, float kp, float kd)
{
    auto* s = findSlot(motor_id);
    if (!s) return;

    s->cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    s->cmd.q    = q * gear_ratio_;     // 输出端位置 → 转子端位置
    s->cmd.kp   = kp;
    s->cmd.kd   = kd;
    s->cmd.dq   = 0.0f;
    s->cmd.tau  = 0.0f;
}

void MotorBus::setVelocity(unsigned short motor_id, float dq, float kd)
{
    auto* s = findSlot(motor_id);
    if (!s) return;

    s->cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    s->cmd.q    = 0.0f;
    s->cmd.kp   = 0.0f;
    s->cmd.kd   = kd;
    s->cmd.dq   = dq * gear_ratio_;     // 输出端速度 → 转子端速度
    s->cmd.tau  = 0.0f;
}

//设置阻尼模式
void MotorBus::setDamping(unsigned short motor_id , float kd)
{   
    auto* s = findSlot(motor_id);
    if (!s) return;
    s->cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    s->cmd.q    = 0.0f;
    s->cmd.kp   = 0.0f;
    s->cmd.kd   = kd  ;//设置kd系数，判断环境有关
    s->cmd.dq   = 0.0f;
    s->cmd.tau  = 0.0f;   
};

void MotorBus::setTorque(unsigned short motor_id, float tau)
{
    auto* s = findSlot(motor_id);
    if (!s) return;

    s->cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    s->cmd.q    = 0.0f;
    s->cmd.kp   = 0.0f;
    s->cmd.kd   = 0.0f;
    s->cmd.dq   = 0.0f;
    // 输出力矩 / 减速比 = 转子力矩
    s->cmd.tau  = (tau != 0.0f) ? (tau / gear_ratio_) : 0.0f;
}

void MotorBus::brake(unsigned short motor_id)
{
    auto* s = findSlot(motor_id);
    if (!s) return;

    s->cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    s->cmd.q    = 0.0f;
    s->cmd.kp   = 0.0f;
    s->cmd.kd   = 0.0f;
    s->cmd.dq   = 0.0f;
    s->cmd.tau  = 0.0f;
}

// ── 通信 ──

void MotorBus::sendRecv()
{
    if (slots_.empty()) return;

    // 组装发送向量
    std::vector<MotorCmd>  sendVec;
    std::vector<MotorData> recvVec;
    sendVec.reserve(slots_.size());
    recvVec.resize(slots_.size());

    for (auto& slot : slots_) {
        sendVec.push_back(slot.cmd);
    }

    // 一次 RS-485 事务：GPIO TX → 逐个轮询各电机 → GPIO RX
    tx();
    serial_.sendRecv(sendVec, recvVec);
    rx();

    // 将接收数据分发回各电机槽位
    for (size_t i = 0; i < recvVec.size() && i < slots_.size(); ++i) {
        slots_[i].data = recvVec[i];
    }
}

MotorState MotorBus::getState(unsigned short motor_id) const
{
    MotorState s;

    for (const auto& slot : slots_) {
        if (slot.id != motor_id) continue;

        constexpr bool TORQUE_IS_ROTOR = true;

        s.q       = slot.data.q  / gear_ratio_;
        s.dq      = slot.data.dq / gear_ratio_;
        s.tau     = TORQUE_IS_ROTOR ? (slot.data.tau * gear_ratio_) : slot.data.tau;
        s.temp    = slot.data.temp;
        s.merror  = slot.data.merror;
        s.correct = slot.data.correct;
        s.mode    = slot.data.mode;
        break;
    }

    return s;
}

std::vector<unsigned short> MotorBus::getMotorIds() const
{
    std::vector<unsigned short> ids;
    ids.reserve(slots_.size());
    for (const auto& s : slots_) {
        ids.push_back(s.id);
    }
    return ids;
}
