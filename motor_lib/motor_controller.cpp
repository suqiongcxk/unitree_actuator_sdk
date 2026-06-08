#include "motor_controller.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <algorithm>

// ═══════════════════════════════════════════════════════════════════════════════
// GPIO sysfs 辅助函数（RAII 风格 — 构造导出、析构释放）
// ═══════════════════════════════════════════════════════════════════════════════

/// 向 sysfs 导出 GPIO，使 /sys/class/gpio/gpioN/ 目录出现
static void gpio_export(int pin)
{
    std::ofstream e("/sys/class/gpio/export");
    if (!e.is_open()) {
        throw std::runtime_error("无法打开 /sys/class/gpio/export");
    }
    e << pin;
    e.close();
    // 等待 udev 创建 gpioN 目录并设置权限
    usleep(150000);
}

/// 设置 GPIO 方向: "out" 或 "in"
static void gpio_set_direction(int pin, const std::string& dir)
{
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream d(path);
    if (!d.is_open()) {
        throw std::runtime_error("无法打开 " + path + " —— GPIO 导出是否成功？");
    }
    d << dir;
    d.close();
}

/// 写 GPIO 值: 1=高电平 0=低电平
static void gpio_write(int pin, int value)
{
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream v(path);
    if (!v.is_open()) {
        throw std::runtime_error("无法打开 " + path);
    }
    v << value;
    v.close();
}

/// 从 sysfs 取消导出 GPIO（释放资源）
static void gpio_unexport(int pin)
{
    std::ofstream e("/sys/class/gpio/unexport");
    if (e.is_open()) {
        e << pin;
        e.close();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MotorController 实现
// ═══════════════════════════════════════════════════════════════════════════════

MotorController::MotorController(int gpio_pin, const std::string& serial_port,
                                 unsigned short motor_id)
    : gpio_pin_(gpio_pin)
    , serial_port_(serial_port)
    , motor_id_(motor_id)
    , gear_ratio_(queryGearRatio(MotorType::GO_M8010_6))
    , connected_(false)
    , serial_(serial_port)   // 使用默认参数: 4M 波特率, 16 字节接收, 非阻塞
{
    // ── 初始化 GPIO ──
    gpio_export(gpio_pin_);
    gpio_set_direction(gpio_pin_, "out");
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
    gpio_unexport(gpio_pin_);
}

// ── 私有辅助方法 ──

void MotorController::tx()
{
    // GPIO 高电平 → RS-485 芯片进入发送模式
    gpio_write(gpio_pin_, 1);
}

void MotorController::rx()
{
    // GPIO 低电平 → RS-485 芯片进入接收模式
    gpio_write(gpio_pin_, 0);
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

    // 一次完整的 RS-485 收发周期：
    //   [GPIO TX] → [发送 MotorCmd 17 字节] → [接收 MotorData 16 字节] → [GPIO RX]
    tx();
    serial_.sendRecv(&cmd_, &data_);   // SDK 内部完成 CRC 打包/校验和字段拆解
    rx();
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
    , serial_port_(serial_port)
    , gear_ratio_(queryGearRatio(MotorType::GO_M8010_6))
    , serial_(serial_port)
{
    gpio_export(gpio_pin_);
    gpio_set_direction(gpio_pin_, "out");
    rx();   // 默认为接收模式
}

MotorBus::~MotorBus()
{
    gpio_unexport(gpio_pin_);
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

void MotorBus::tx() { gpio_write(gpio_pin_, 1); }
void MotorBus::rx() { gpio_write(gpio_pin_, 0); }

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
    // SerialPort 内部会按顺序发送每个 MotorCmd，接收对应 MotorData
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
