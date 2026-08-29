#ifndef __PARALLEL_BUS_H
#define __PARALLEL_BUS_H

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include "fast_gpio.h"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "motor_controller.h"   // 复用 MotorState

// 单路总线线程的只读时序快照。所有时间均基于 CLOCK_MONOTONIC。
struct BusTimingStats {
    uint64_t loop_count = 0;
    uint64_t max_loop_gap_ns = 0;        // 相邻两次控制循环开始时间的最大间隔
    uint64_t max_cycle_duration_ns = 0;  // 单次完整收发循环的最大执行时间
    uint64_t gap_over_2ms = 0;
    uint64_t gap_over_10ms = 0;
    uint64_t gap_over_50ms = 0;
    uint64_t gap_over_100ms = 0;
};

/**
 * @brief 单路 RS-485 并行总线（独立线程驱动）
 *
 * 每路总线上挂 2-4 个电机，运行在独立线程中，按设定频率循环收发。
 * 多路线程同时运行 → 真正的硬件并行。
 *
 * 使用示例:
 * @code
 *   ParallelBus bus(0, 63, "/dev/ttyS4");  // gpiochip0, line63, 串口 ttyS4
 *   bus.addMotor(0);
 *   bus.addMotor(1);
 *   bus.addMotor(2);
 *   bus.start(500);  // 500 Hz 控制频率
 *
 *   // 主线程异步下发指令
 *   bus.setPosition(0, 0.5, 0.03, 0.01);
 *   bus.setVelocity(1, 3.14, 0.02);
 *
 *   // 读取状态（线程安全）
 *   MotorState s = bus.getState(0);
 *
 *   bus.stop();
 * @endcode
 */
class ParallelBus {
public:
    /**
     * @param gpio_chip   GPIO 芯片编号 (0, 1, 2...)
     * @param gpio_line   GPIO 行偏移
     * @param serial_port 串口路径
     */
    ParallelBus(int gpio_chip, int gpio_line, const std::string& serial_port);
    ~ParallelBus();

    /// 向总线注册一台电机（必须在 start() 之前调用）
    bool addMotor(unsigned short motor_id);
    bool removeMotor(unsigned short motor_id);

    // ── 指令暂存（线程安全，start 之后也可以随时调用） ──

    void setPosition(unsigned short motor_id, float q, float kp, float kd);
    void setVelocity(unsigned short motor_id, float dq, float kd);
    void setTorque(unsigned short motor_id, float tau);
    void setDamping(unsigned short motor_id, float kd);
    void brake(unsigned short motor_id);

    /// 锁存急停阻尼；锁存后普通位置/速度/力矩指令均被拒绝。
    void enterEmergencyDamping(float kd = 0.02f);
    bool isEmergencyLatched() const { return emergency_latched_.load(); }

    // ── 状态读取（线程安全） ──

    MotorState getState(unsigned short motor_id) const;
    std::vector<unsigned short> getMotorIds() const;

    // ── 生命周期 ──

    /// 启动控制线程，hz 为每路总线期望控制频率
    void start(int hz = 500);
    /// 停止控制线程
    void stop();

    bool isRunning() const { return running_.load(); }

    // ── 属性 ──

    int getGpioChip() const { return gpio_chip_; }
    int getGpioLine() const { return gpio_line_; }
    const std::string& getPort() const { return serial_port_; }
    float getGearRatio() const { return gear_ratio_; }
    int getTargetHz() const { return target_hz_; }
    float getActualHz() const { return actual_hz_.load(); }
    BusTimingStats getTimingStats() const;

private:
    void controlLoop();   // 线程主循环

    // 在发送前将输出端指令转为转子端
    void applyGearRatio(MotorCmd& cmd);

    int gpio_chip_;
    int gpio_line_;
    std::string serial_port_;
    float gear_ratio_;

    std::unique_ptr<FastGPIO>    gpio_;
    std::unique_ptr<SerialPort>  serial_;

    struct MotorSlot {
        unsigned short id;
        MotorCmd cmd;
        MotorData data;
        uint64_t feedback_timestamp_ns = 0;
        uint32_t consecutive_failures = 0;
        uint64_t transaction_count = 0;
        uint64_t success_count = 0;
        uint64_t short_frame_count = 0;
        uint64_t protocol_failure_count = 0;
        uint64_t receive_timeout_count = 0;
        uint64_t crc_failure_count = 0;
        uint64_t wrong_id_count = 0;
        uint64_t max_transaction_duration_ns = 0;
    };
    std::vector<MotorSlot> slots_;
    mutable std::mutex slots_mtx_;   // 保护 slots_ 的读写

    std::thread  thread_;
    std::atomic<bool> running_{false};
    int target_hz_ = 500;
    std::atomic<float> actual_hz_{0.0f};
    std::atomic<bool> emergency_latched_{false};
    std::atomic<float> emergency_kd_{0.02f};

    // 诊断统计只由总线线程写入，其他线程通过原子快照读取，不进入 slots_mtx_。
    std::atomic<uint64_t> previous_loop_start_ns_{0};
    std::atomic<uint64_t> loop_count_{0};
    std::atomic<uint64_t> max_loop_gap_ns_{0};
    std::atomic<uint64_t> max_cycle_duration_ns_{0};
    std::atomic<uint64_t> gap_over_2ms_{0};
    std::atomic<uint64_t> gap_over_10ms_{0};
    std::atomic<uint64_t> gap_over_50ms_{0};
    std::atomic<uint64_t> gap_over_100ms_{0};
};

// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 多路总线控制器，管理 3-4 路独立的 RS-485 总线
 *
 * 每路总线独立线程驱动，多路同时运行实现硬件级并行。
 *
 * 使用示例:
 * @code
 *   MultiBusController controller;
 *
 *   // 配置 4 路总线，每路挂 3 个电机
 *   controller.addBus(0, 63, "/dev/ttyS4");   // bus 0 → motor 0,1,2
 *   controller.addBus(0, 120, "/dev/ttyS5");  // bus 1 → motor 3,4,5
 *   controller.addBus(0, 121, "/dev/ttyS6");  // bus 2 → motor 6,7,8
 *   controller.addBus(0, 122, "/dev/ttyS7");  // bus 3 → motor 9,10,11
 *
 *   for (auto& bus : controller.buses()) {
 *       bus->addMotor(0);
 *       bus->addMotor(1);
 *       bus->addMotor(2);
 *   }
 *
 *   controller.startAll(500);  // 4 路同时以 500Hz 运行
 *
 *   controller.bus(0).setPosition(0, 0.5, 0.03, 0.01);
 *   // ...
 *
 *   controller.stopAll();
 * @endcode
 */
class MultiBusController {
public:
    MultiBusController() = default;
    ~MultiBusController();

    /// 添加一路总线（必须在 startAll 之前调用）
    /// @return 返回新增的 ParallelBus 引用
    ParallelBus& addBus(int gpio_chip, int gpio_line, const std::string& serial_port);

    /// 启动所有总线线程
    void startAll(int hz = 500);

    /// 停止所有总线线程
    void stopAll();

    /// 所有总线锁存阻尼模式，后续普通控制指令不能覆盖。
    void enterEmergencyDampingAll(float kd = 0.02f);

    /// 按索引访问某路总线
    ParallelBus& bus(size_t index);
    const ParallelBus& bus(size_t index) const;

    /// 所有总线的引用
    std::vector<ParallelBus*> buses();
    size_t busCount() const { return buses_.size(); }

private:
    std::vector<std::unique_ptr<ParallelBus>> buses_;
};

#endif  // __PARALLEL_BUS_H
