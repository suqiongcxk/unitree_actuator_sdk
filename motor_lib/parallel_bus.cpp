#include "parallel_bus.h"
#include <algorithm>
#include <iostream>
#include <ctime>
#include <cstring>
#include <sys/ioctl.h>
#include "quiet_serial_recv.h"

namespace {
uint64_t monotonicNowNs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

void updateAtomicMax(std::atomic<uint64_t>& target, uint64_t value)
{
    uint64_t current = target.load(std::memory_order_relaxed);
    while (current < value
           && !target.compare_exchange_weak(current, value,
                                            std::memory_order_relaxed,
                                            std::memory_order_relaxed)) {
    }
}
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParallelBus 实现
// ═══════════════════════════════════════════════════════════════════════════════

ParallelBus::ParallelBus(int gpio_chip, int gpio_line, const std::string& serial_port)
    : gpio_chip_(gpio_chip)
    , gpio_line_(gpio_line)
    , serial_port_(serial_port)
    , gear_ratio_(queryGearRatio(MotorType::GO_M8010_6))
{
    gpio_   = std::make_unique<FastGPIO>(gpio_chip_, gpio_line_);
    serial_ = std::make_unique<SerialPort>(serial_port_);
}

ParallelBus::~ParallelBus()
{
    stop();
}

bool ParallelBus::addMotor(unsigned short motor_id)
{
    std::lock_guard<std::mutex> lock(slots_mtx_);

    // 检查是否已存在
    for (auto& s : slots_) {
        if (s.id == motor_id) return false;
    }

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

bool ParallelBus::removeMotor(unsigned short motor_id)
{
    std::lock_guard<std::mutex> lock(slots_mtx_);
    auto it = std::remove_if(slots_.begin(), slots_.end(),
                             [motor_id](const MotorSlot& s) { return s.id == motor_id; });
    if (it == slots_.end()) return false;
    slots_.erase(it, slots_.end());
    return true;
}

// ── 指令暂存（输出端量纲，在控制线程中做减速比转换） ──

void ParallelBus::setPosition(unsigned short motor_id, float q, float kp, float kd)
{
    if (emergency_latched_.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lock(slots_mtx_);
    if (emergency_latched_.load(std::memory_order_relaxed)) return;
    for (auto& s : slots_) {
        if (s.id != motor_id) continue;
        s.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        s.cmd.q    = q;       // 输出端值，控制线程中转换
        s.cmd.kp   = kp;
        s.cmd.kd   = kd;
        s.cmd.dq   = 0.0f;
        s.cmd.tau  = 0.0f;
        break;
    }
}

void ParallelBus::setVelocity(unsigned short motor_id, float dq, float kd)
{
    if (emergency_latched_.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lock(slots_mtx_);
    if (emergency_latched_.load(std::memory_order_relaxed)) return;
    for (auto& s : slots_) {
        if (s.id != motor_id) continue;
        s.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        s.cmd.q    = 0.0f;
        s.cmd.kp   = 0.0f;
        s.cmd.kd   = kd;
        s.cmd.dq   = dq;
        s.cmd.tau  = 0.0f;
        break;
    }
}

void ParallelBus::setTorque(unsigned short motor_id, float tau)
{
    if (emergency_latched_.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lock(slots_mtx_);
    if (emergency_latched_.load(std::memory_order_relaxed)) return;
    for (auto& s : slots_) {
        if (s.id != motor_id) continue;
        s.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        s.cmd.q    = 0.0f;
        s.cmd.kp   = 0.0f;
        s.cmd.kd   = 0.0f;
        s.cmd.dq   = 0.0f;
        s.cmd.tau  = tau;
        break;
    }
}

void ParallelBus::setDamping(unsigned short motor_id, float kd)
{
    if (emergency_latched_.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lock(slots_mtx_);
    if (emergency_latched_.load(std::memory_order_relaxed)) return;
    for (auto& s : slots_) {
        if (s.id != motor_id) continue;
        s.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        s.cmd.q = 0.0f;
        s.cmd.kp = 0.0f;
        s.cmd.kd = kd;
        s.cmd.dq = 0.0f;
        s.cmd.tau = 0.0f;
        break;
    }
}

void ParallelBus::enterEmergencyDamping(float kd)
{
    // 在同一临界区内先锁存再覆盖全部槽，杜绝旧控制指令重新写入。
    std::lock_guard<std::mutex> lock(slots_mtx_);
    emergency_kd_.store(kd, std::memory_order_relaxed);
    emergency_latched_.store(true, std::memory_order_release);
    for (auto& s : slots_) {
        s.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        s.cmd.q = 0.0f;
        s.cmd.kp = 0.0f;
        s.cmd.kd = kd;
        s.cmd.dq = 0.0f;
        s.cmd.tau = 0.0f;
    }
}

void ParallelBus::brake(unsigned short motor_id)
{
    if (emergency_latched_.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lock(slots_mtx_);
    if (emergency_latched_.load(std::memory_order_relaxed)) return;
    for (auto& s : slots_) {
        if (s.id != motor_id) continue;
        s.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        s.cmd.q    = 0.0f;
        s.cmd.kp   = 0.0f;
        s.cmd.kd   = 0.0f;
        s.cmd.dq   = 0.0f;
        s.cmd.tau  = 0.0f;
        break;
    }
}

MotorState ParallelBus::getState(unsigned short motor_id) const
{
    std::lock_guard<std::mutex> lock(slots_mtx_);

    MotorState s;
    for (const auto& slot : slots_) {
        if (slot.id != motor_id) continue;
        // 第一帧有效反馈到达前 MotorData 由厂商 SDK 持有，
        // 其数值字段未保证已初始化；此时只返回零值无效状态。
        if (slot.feedback_timestamp_ns != 0) {
            s.q       = slot.data.q  / gear_ratio_;
            s.dq      = slot.data.dq / gear_ratio_;
            s.tau     = slot.data.tau;  // 协议直接给出关节输出扭矩
            s.temp    = slot.data.temp;
            s.merror  = slot.data.merror;
            s.correct = slot.data.correct;
            s.mode    = slot.data.mode;
        }
        s.feedback_timestamp_ns = slot.feedback_timestamp_ns;
        s.consecutive_failures = slot.consecutive_failures;
        s.transaction_count = slot.transaction_count;
        s.success_count = slot.success_count;
        s.short_frame_count = slot.short_frame_count;
        s.protocol_failure_count = slot.protocol_failure_count;
        break;
    }
    return s;
}

std::vector<unsigned short> ParallelBus::getMotorIds() const
{
    std::lock_guard<std::mutex> lock(slots_mtx_);
    std::vector<unsigned short> ids;
    for (const auto& s : slots_) ids.push_back(s.id);
    return ids;
}

BusTimingStats ParallelBus::getTimingStats() const
{
    BusTimingStats stats;
    stats.loop_count = loop_count_.load(std::memory_order_relaxed);
    stats.max_loop_gap_ns = max_loop_gap_ns_.load(std::memory_order_relaxed);
    stats.max_cycle_duration_ns =
        max_cycle_duration_ns_.load(std::memory_order_relaxed);
    stats.gap_over_2ms = gap_over_2ms_.load(std::memory_order_relaxed);
    stats.gap_over_10ms = gap_over_10ms_.load(std::memory_order_relaxed);
    stats.gap_over_50ms = gap_over_50ms_.load(std::memory_order_relaxed);
    stats.gap_over_100ms = gap_over_100ms_.load(std::memory_order_relaxed);
    return stats;
}

// ── 减速比转换 ──

void ParallelBus::applyGearRatio(MotorCmd& cmd)
{
    cmd.q  *= gear_ratio_;
    cmd.dq *= gear_ratio_;
}

// ── 控制线程 ──

void ParallelBus::start(int hz)
{
    if (running_.load()) return;

    if (slots_.empty()) {
        std::cerr << "[ParallelBus] 警告: 总线上没有电机，拒绝启动\n";
        return;
    }

    target_hz_ = hz;
    previous_loop_start_ns_.store(0, std::memory_order_relaxed);
    loop_count_.store(0, std::memory_order_relaxed);
    max_loop_gap_ns_.store(0, std::memory_order_relaxed);
    max_cycle_duration_ns_.store(0, std::memory_order_relaxed);
    gap_over_2ms_.store(0, std::memory_order_relaxed);
    gap_over_10ms_.store(0, std::memory_order_relaxed);
    gap_over_50ms_.store(0, std::memory_order_relaxed);
    gap_over_100ms_.store(0, std::memory_order_relaxed);
    running_.store(true);
    thread_ = std::thread(&ParallelBus::controlLoop, this);
}

void ParallelBus::stop()
{
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void ParallelBus::controlLoop()
{
    const long period_ns = 1'000'000'000L / target_hz_;   // 周期纳秒数
    const int motor_count = static_cast<int>(slots_.size());

    // 用 MONOTONIC 时钟做绝对时间调度，避免累积漂移
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    long long cycle_count = 0;
    long long last_frequency_ns = 0;

    while (running_.load()) {
        const uint64_t cycle_start_ns = monotonicNowNs();
        const uint64_t previous_start_ns =
            previous_loop_start_ns_.exchange(cycle_start_ns,
                                             std::memory_order_relaxed);
        loop_count_.fetch_add(1, std::memory_order_relaxed);
        if (previous_start_ns > 0 && cycle_start_ns >= previous_start_ns) {
            const uint64_t gap_ns = cycle_start_ns - previous_start_ns;
            updateAtomicMax(max_loop_gap_ns_, gap_ns);
            if (gap_ns > 2'000'000ULL)
                gap_over_2ms_.fetch_add(1, std::memory_order_relaxed);
            if (gap_ns > 10'000'000ULL)
                gap_over_10ms_.fetch_add(1, std::memory_order_relaxed);
            if (gap_ns > 50'000'000ULL)
                gap_over_50ms_.fetch_add(1, std::memory_order_relaxed);
            if (gap_ns > 100'000'000ULL)
                gap_over_100ms_.fetch_add(1, std::memory_order_relaxed);
        }

        // ── 计算下一次唤醒时间 ──
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1'000'000'000L) {
            next.tv_nsec -= 1'000'000'000L;
            next.tv_sec++;
        }

        // ── 第一步: 从 slots_ 取出指令副本（持锁时间极短） ──
        std::vector<MotorCmd> sendVec;
        {
            std::lock_guard<std::mutex> lock(slots_mtx_);
            sendVec.reserve(slots_.size());
            for (auto& slot : slots_) {
                sendVec.push_back(slot.cmd);   // 拷贝指令
            }
        }

        // ── 第二步: 减速比转换（无锁，操作本地副本） ──
        for (auto& cmd : sendVec) {
            // 只在 FOC 模式下做转换，刹车指令不需要
            // （这里简单判断：q/dq/tau 有非零值就需要转换）
            applyGearRatio(cmd);
        }

        // ── 第三步: 逐电机独立 RS-485 事务 ──
        //   每台电机: tx→send→等待TX完成→rx→recv
        //   与 MotorController::sendRecv() 使用相同的硬件级 TX 完成检测
        std::vector<MotorData> recvVec(sendVec.size());
        // MotorData 默认构造函数不会初始化 motorType；必须在调用
        // get_motor_recv_data()/extract_data() 前明确指定协议类型，否则会
        // 选择未定义的接收缓冲区并造成短帧、CRC 错误。
        for (auto& data : recvVec) {
            data.motorType = MotorType::GO_M8010_6;
        }
        std::vector<size_t> recv_lengths(sendVec.size(), 0);
        for (size_t i = 0; i < sendVec.size(); ++i) {
            if (emergency_latched_.load(std::memory_order_acquire)) {
                // 即使本周期已取出旧指令，急停锁存后也在发送前强制改为阻尼。
                sendVec[i].mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                sendVec[i].q = 0.0f;
                sendVec[i].kp = 0.0f;
                sendVec[i].kd = emergency_kd_.load(std::memory_order_relaxed);
                sendVec[i].dq = 0.0f;
                sendVec[i].tau = 0.0f;
            }
            sendVec[i].modify_data(&sendVec[i]);
            uint8_t* sendData = sendVec[i].get_motor_send_data();
            int sendLen = sendVec[i].hex_len;
            uint8_t* recvData = recvVec[i].get_motor_recv_data();

            gpio_->set(1);   // TX 模式

            serial_->send(sendData, static_cast<size_t>(sendLen));

            // TIOCOUTQ + TIOCSERGETLSR: 硬件级 TX 完成检测
            {
                int fd = serial_->fd();
                int outq = 1, polls = 0;
                while (outq > 0 && polls < 10000) {
                    if (ioctl(fd, TIOCOUTQ, &outq) < 0) break;
                    polls++;
                }
                unsigned int lsr = 0;
                polls = 0;
                while (polls < 100000) {
                    if (ioctl(fd, TIOCSERGETLSR, &lsr) < 0) break;
                    if (lsr & TIOCSER_TEMT) break;
                    polls++;
                }
            }

            gpio_->set(0);   // RX 模式

            constexpr size_t kGoM8010RecvLength = 16;
            recv_lengths[i] = motor_io::quietSerialRecv(
                serial_->fd(), recvData, kGoM8010RecvLength);
            // 短帧禁止解包，也不能用未初始化字段覆盖上一帧有效状态。
            if (motor_io::hasValidGoM8010Crc(recvData, recv_lengths[i])) {
                recvVec[i].extract_data(&recvVec[i]);
            } else {
                recvVec[i].correct = false;
            }
        }

        // ── 第四步: 将接收数据写回 slots_ ──
        {
            std::lock_guard<std::mutex> lock(slots_mtx_);
            for (size_t i = 0; i < recvVec.size() && i < slots_.size(); ++i) {
                ++slots_[i].transaction_count;
                const bool full_frame = recv_lengths[i] == 16;
                const bool response_ok = full_frame && recvVec[i].correct
                                      && recvVec[i].motor_id == slots_[i].id;
                if (response_ok) {
                    slots_[i].data = recvVec[i];
                    struct timespec feedback_time;
                    clock_gettime(CLOCK_MONOTONIC, &feedback_time);
                    slots_[i].feedback_timestamp_ns =
                        static_cast<uint64_t>(feedback_time.tv_sec) * 1'000'000'000ULL
                      + static_cast<uint64_t>(feedback_time.tv_nsec);
                    slots_[i].consecutive_failures = 0;
                    ++slots_[i].success_count;
                } else {
                    ++slots_[i].consecutive_failures;
                    if (!full_frame) ++slots_[i].short_frame_count;
                    else ++slots_[i].protocol_failure_count;
                    // 保留最近一次有效数值和时间戳，但标记本帧无效。
                    slots_[i].data.correct = false;
                }
            }
        }

        // 包含指令快照、三台电机逐一收发以及反馈写回，用于区分
        // “通信/I/O耗时过长”和“线程未被调度”。
        const uint64_t cycle_end_ns = monotonicNowNs();
        if (cycle_end_ns >= cycle_start_ns) {
            updateAtomicMax(max_cycle_duration_ns_, cycle_end_ns - cycle_start_ns);
        }

        // ── 统计实际频率（每秒更新一次） ──
        cycle_count++;
        if (cycle_count % target_hz_ == 0) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long long now_ns = now.tv_sec * 1'000'000'000LL + now.tv_nsec;
            if (last_frequency_ns != 0) {
                float period = (now_ns - last_frequency_ns) / 1e9f;
                actual_hz_.store(target_hz_ / period);
            }
            last_frequency_ns = now_ns;
        }

        // ── 精准睡眠到下一次周期 ──
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MultiBusController 实现
// ═══════════════════════════════════════════════════════════════════════════════

MultiBusController::~MultiBusController()
{
    stopAll();
}

ParallelBus& MultiBusController::addBus(int gpio_chip, int gpio_line,
                                         const std::string& serial_port)
{
    auto bus = std::make_unique<ParallelBus>(gpio_chip, gpio_line, serial_port);
    ParallelBus& ref = *bus;
    buses_.push_back(std::move(bus));
    return ref;
}

void MultiBusController::startAll(int hz)
{
    for (auto& b : buses_) {
        b->start(hz);
    }
}

void MultiBusController::stopAll()
{
    for (auto& b : buses_) {
        b->stop();
    }
}

void MultiBusController::enterEmergencyDampingAll(float kd)
{
    for (auto& b : buses_) {
        b->enterEmergencyDamping(kd);
    }
}

ParallelBus& MultiBusController::bus(size_t index)
{
    return *buses_.at(index);
}

const ParallelBus& MultiBusController::bus(size_t index) const
{
    return *buses_.at(index);
}

std::vector<ParallelBus*> MultiBusController::buses()
{
    std::vector<ParallelBus*> ptrs;
    for (auto& b : buses_) ptrs.push_back(b.get());
    return ptrs;
}
