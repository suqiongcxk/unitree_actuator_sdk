#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cstdint>
#include "fast_gpio.h"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

/**
 * @brief 电机状态（输出端，即经过减速器之后的关节端）
 *
 * 所有位置/速度/力矩均为输出端值，库内部自动完成转子端↔输出端转换。
 */
struct MotorState {
    float q       = 0.0f;   // 输出端位置 (rad)
    float dq      = 0.0f;   // 输出端速度 (rad/s)
    float tau     = 0.0f;   // 输出端力矩 (N·m)
    int   temp    = 0;      // 电机温度 (°C)
    int   merror  = 0;      // 错误码: 0=正常 1=过热 2=过流 3=过压 4=编码器故障
    bool  correct = false;  // CRC 校验通过标志
    unsigned char mode = 0; // 当前工作模式: 0=锁定 1=FOC闭环 2=编码器校准
    uint64_t feedback_timestamp_ns = 0; // 最近一次 CRC 正确反馈的 CLOCK_MONOTONIC 时间
    uint32_t consecutive_failures = 0;  // 连续 CRC/ID/接收失败次数
};

/**
 * @brief 单电机控制器，封装 GPIO RS-485 方向切换 + Unitree SDK 指令/数据
 *
 * 对外接口全部使用输出端量纲（位置、速度、力矩），内部通过 queryGearRatio()
 * 自动完成位置/速度的转子端↔输出端转换；GO-M8010-6 力矩协议本身使用输出端 N·m。
 *
 * 独立使用示例:
 * @code
 *   MotorController motor(63, "/dev/ttyS4", 0);
 *   motor.setPosition(1.57, 0.02, 0.01);   // 输出端 90°, kp=0.02, kd=0.01
 *   MotorState s = motor.getState();
 *   motor.brake();
 * @endcode
 */
class MotorController {
public:
    /**
     * @param gpio_pin    RS-485 方向控制 GPIO 编号（sysfs 编号）
     * @param serial_port 串口设备路径，如 "/dev/ttyS4"、"/dev/ttyUSB0"
     * @param motor_id    电机 CAN ID (0–14)
     */
    MotorController(int gpio_pin, const std::string& serial_port,
                    unsigned short motor_id = 0);
    ~MotorController();

    // ── 控制指令（参数均为输出端量纲） ──

    /// 位置控制：设定输出端目标位置 q (rad) 及 PD 增益
    /// kp: 刚度系数 0.0–1.0,  kd: 阻尼系数 0.0–1.0
    void setPosition(float q, float kp, float kd);

    /// 速度控制：设定输出端目标速度 dq (rad/s) 及阻尼
    /// kd: 阻尼系数 0.0–1.0
    void setVelocity(float dq, float kd);

    /// 力矩控制：设定输出端目标力矩 (N·m)
    void setTorque(float tau);

    //// 阻尼模式
    void setdamping(float kd);
    /// 刹车/锁定电机
    void brake();


    // ── 状态查询 ──

    /// 返回最近一次电机状态（输出端量纲）
    MotorState getState() const;

    /// 立即执行一次收发（通常由 setXxx / brake 自动调用，手动调用用于精确时序控制）
    void sendRecv();

    // ── 属性访问 ──

    int getGpioPin() const { return gpio_pin_; }
    const std::string& getPort() const { return serial_port_; }
    unsigned short getId() const { return motor_id_; }
    float getGearRatio() const { return gear_ratio_; }
    bool isConnected() const { return connected_; }

private:
    void tx();              // GPIO 拉高 → RS-485 发送模式 (~1µs, FastGPIO ioctl)
    void rx();              // GPIO 拉低 → RS-485 接收模式 (~1µs, FastGPIO ioctl)
    void applyGearRatio();  // 将 cmd_ 中的输出端值转为转子端值

    int gpio_pin_;                        // 保留: 全局 GPIO 编号 (用于 getGpioPin)
    std::unique_ptr<FastGPIO> gpio_;      // 高速 GPIO (Linux gpio-v2 ioctl, ~µs级)
    std::string serial_port_;
    unsigned short motor_id_;
    float gear_ratio_;
    bool connected_;

    SerialPort serial_;
    MotorCmd   cmd_;
    MotorData  data_;
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief 多电机总线控制器，多台电机共享一条 RS-485 总线
 *
 * 指令先"暂存"，然后由一次 sendRecv() 统一发出，保证多电机控制的时序一致性。
 *
 * 使用示例:
 * @code
 *   MotorBus bus(63, "/dev/ttyS4");
 *   bus.addMotor(0);
 *   bus.addMotor(1);
 *   bus.setPosition(0, 1.57, 0.02, 0.01);
 *   bus.setVelocity(1, -3.14, 0.01);
 *   bus.sendRecv();                          // 一次事务完成所有电机收发
 *   MotorState s0 = bus.getState(0);
 *   MotorState s1 = bus.getState(1);
 *   bus.brake(0);
 *   bus.brake(1);
 *   bus.sendRecv();
 * @endcode
 */
class MotorBus {
public:
    /**
     * @param gpio_pin    RS-485 方向控制 GPIO 编号
     * @param serial_port 串口设备路径
     */
    MotorBus(int gpio_pin, const std::string& serial_port);
    ~MotorBus();

    /// 向总线注册一台电机，id 重复时返回 false
    bool addMotor(unsigned short motor_id);

    /// 从总线移除一台电机
    bool removeMotor(unsigned short motor_id);

    // ── 暂存指令（输出端量纲，下次 sendRecv 时生效） ──

    void setPosition(unsigned short motor_id, float q, float kp, float kd);
    void setVelocity(unsigned short motor_id, float dq, float kd);
    void setTorque(unsigned short motor_id, float tau);
    void setDamping(unsigned short motor_id , float kd);
    void brake(unsigned short motor_id);

    // ── 通信 ──

    /// 发送所有暂存指令并更新各电机状态
    /// 力矩控制建议 ≥200 Hz，位置/速度控制建议 ≥100 Hz
    void sendRecv();

    /// 返回指定电机的最近一次状态（输出端量纲）
    MotorState getState(unsigned short motor_id) const;

    /// 返回总线上所有已注册的电机 ID
    std::vector<unsigned short> getMotorIds() const;

    /// 返回指定电机的原始接收报文指针和长度（十六进制打印用）
    const uint8_t* getRawRecvData(unsigned short motor_id, int& len);

    // ── 属性访问 ──

    int getGpioPin() const { return gpio_pin_; }
    const std::string& getPort() const { return serial_port_; }
    float getGearRatio() const { return gear_ratio_; }
    size_t motorCount() const { return slots_.size(); }

private:
    struct MotorSlot {
        unsigned short id;     // 电机 ID
        MotorCmd cmd;          // 暂存的发送指令
        MotorData data;        // 最近一次收到的反馈
    };

    MotorSlot* findSlot(unsigned short motor_id);

    void tx();
    void rx();

    int gpio_pin_;                        // 保留: 全局 GPIO 编号 (用于 getGpioPin)
    std::unique_ptr<FastGPIO> gpio_;      // 高速 GPIO (~µs级)
    std::string serial_port_;
    float gear_ratio_;

    SerialPort serial_;
    std::vector<MotorSlot> slots_;   // 按注册顺序存储
};

#endif  // __MOTOR_CONTROLLER_H
