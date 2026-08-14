#ifndef JY901S_H
#define JY901S_H

#include <cstdint>
#include <string>

// ═══════════════════════════════════════════════════════════════════════════════
//  JY901S IMU 传感器 — Linux I2C 驱动
// ═══════════════════════════════════════════════════════════════════════════════
//
//  接线: Orange Pi 5 Pro 40-pin 排针
//    pin 5  (GPIO1_D2, I2C1_M4_SCL) → JY901S SCL
//    pin 3  (GPIO1_D3, I2C1_M4_SDA) → JY901S SDA
//    pin 1  (3.3V)     → JY901S VCC
//    pin 9  (GND)      → JY901S GND
//
//  设备节点: /dev/i2c-1（需启用 overlay: i2c1-m4）
//  I2C 地址: 0x50

// ── I2C 地址 ─────────────────────────────────────────────────────────────────
#define JY901S_I2C_ADDR         0x50

// ── 寄存器地址 ───────────────────────────────────────────────────────────────
#define JY901S_REG_SAVE         0x00    // 保存配置（写入 0x0000）
#define JY901S_REG_RRATE        0x03    // 输出速率（0x0B=200Hz, 0x09=100Hz）
#define JY901S_REG_BAUD         0x04    // 串口波特率（0x07=230400）
#define JY901S_REG_AXIS6        0x24    // 6轴算法（0x01=六轴, 0x00=九轴）
#define JY901S_REG_BANDWIDTH    0x1F    // 陀螺仪带宽（0x00=256Hz）
#define JY901S_REG_KEY          0x69    // 解锁寄存器（写入 0xB588）

// 加速度寄存器（每轴 2 字节，共 6 字节）
#define JY901S_REG_ACC_X        0x34
#define JY901S_REG_ACC_Y        0x35
#define JY901S_REG_ACC_Z        0x36

// 角速度寄存器（每轴 2 字节，共 6 字节）
#define JY901S_REG_GYRO_X       0x37
#define JY901S_REG_GYRO_Y       0x38
#define JY901S_REG_GYRO_Z       0x39

// 欧拉角寄存器（每轴 2 字节，共 6 字节）
#define JY901S_REG_ROLL_L       0x3D
#define JY901S_REG_PITCH_L      0x3E   // 注意: JY901S 的 pitch/yaw 寄存器定义
#define JY901S_REG_YAW_L        0x3F   // 可能与常规顺序不同

// 四元数寄存器（每个分量 2 字节，共 8 字节）
#define JY901S_REG_Q0           0x51

// 四元数: 有符号 16-bit，实际值 = raw / 32768
#define JY901S_QUAT_SCALE       (1.0f / 32768.0f)

// ── 输出速率宏 ───────────────────────────────────────────────────────────────
#define JY901S_RRATE_200HZ      0x0B
#define JY901S_RRATE_100HZ      0x09
#define JY901S_RRATE_50HZ       0x08
#define JY901S_RRATE_20HZ       0x07
#define JY901S_RRATE_10HZ       0x06

// ── 转换系数 ─────────────────────────────────────────────────────────────────
// 角度: 0.005493 °/LSB → rad = * π/180
#define JY901S_ANGLE_SCALE      0.005493f       // 度/LSB
#define JY901S_ANGLE_TO_RAD     (0.005493f * 3.14159265f / 180.0f)  // 弧度/LSB

// 加速度: ±16g 量程, 16-bit → 1g = 2048 LSB → m/s²
#define JY901S_ACC_SCALE        (9.80665f / 2048.0f)   // m/s² per LSB

// 角速度: ±2000°/s 量程, 16-bit → 1°/s = 16.384 LSB → rad/s
#define JY901S_GYRO_SCALE       (2000.0f * 3.14159265f / 180.0f / 32768.0f)  // rad/s per LSB

// ── 错误码 ───────────────────────────────────────────────────────────────────
enum class JY901S_Status {
    OK = 0,
    ERROR_I2C_OPEN,       // 无法打开 I2C 设备
    ERROR_I2C_READ,        // I2C 读取失败
    ERROR_I2C_WRITE,       // I2C 写入失败
    ERROR_NOT_INITIALIZED, // 未初始化
    ERROR_UNLOCK_FAILED,   // 解锁失败
};

// ── 数据结构 ─────────────────────────────────────────────────────────────────

/// 三轴欧拉角
struct JY901S_AngleData {
    float roll;     // 横滚角 (度)
    float pitch;    // 俯仰角 (度)
    float yaw;      // 偏航角 (度)
};

/// 三轴加速度
struct JY901S_ACCData {
    float x;        // X 轴加速度 (m/s²)
    float y;        // Y 轴加速度 (m/s²)
    float z;        // Z 轴加速度 (m/s²)
};

/// 三轴角速度
struct JY901S_GyroData {
    float x;        // X 轴角速度 (rad/s)
    float y;        // Y 轴角速度 (rad/s)
    float z;        // Z 轴角速度 (rad/s)
};

/// 四元数
struct JY901S_Quaternion {
    float q0;       // w
    float q1;       // x
    float q2;       // y
    float q3;       // z
};

// ═══════════════════════════════════════════════════════════════════════════════
//  JY901S 类
// ═══════════════════════════════════════════════════════════════════════════════

class JY901S {
public:
    /**
     * @brief 构造函数
     * @param i2c_device  I2C 设备路径，如 "/dev/i2c-1"
     */
    explicit JY901S(const std::string& i2c_device = "/dev/i2c-1");
    ~JY901S();

    // ── 生命周期 ─────────────────────────────────────────────────────────

    /// 初始化：打开 I2C → 解锁 → 配置 200Hz/六轴/256Hz带宽 → 保存
    /// 采样率对 RL 控制很重要，默认 200Hz
    JY901S_Status init(int output_rate_hz = 200);

    /// 关闭 I2C 设备
    void close();

    /// 是否已初始化
    bool isInitialized() const { return fd_ >= 0; }

    // ── 传感器读取 ─────────────────────────────────────────────────────

    /// 读取三轴欧拉角 (Roll/Pitch/Yaw, 度)
    JY901S_Status readAngles(JY901S_AngleData& angles);

    /// 读取三轴加速度 (m/s²)
    JY901S_Status readAcceleration(JY901S_ACCData& acc);

    /// 读取三轴角速度 (rad/s)
    JY901S_Status readGyro(JY901S_GyroData& gyro);

    /// 读取四元数 (w, x, y, z)
    JY901S_Status readQuaternion(JY901S_Quaternion& quat);

    /// 一次读取全部常用数据（角度 + 加速度 + 角速度）
    /// 连续寄存器批量读取，效率高于分别调用
    JY901S_Status readAll(JY901S_AngleData& angles,
                          JY901S_ACCData& acc,
                          JY901S_GyroData& gyro);

    // ── 原始数据读取（调试用） ──────────────────────────────────────────

    /// 读取单个寄存器（8 位）
    JY901S_Status readReg8(uint8_t reg_addr, uint8_t& value);

    /// 读取单个寄存器（16 位，小端）
    JY901S_Status readReg16(uint8_t reg_addr, uint16_t& value);

    /// 从指定寄存器读取多个字节
    JY901S_Status readRegs(uint8_t reg_addr, uint8_t* buf, int len);

    /// 写入单个寄存器（8 位）
    JY901S_Status writeReg8(uint8_t reg_addr, uint8_t value);

    /// 写入单个寄存器（16 位，小端）
    JY901S_Status writeReg16(uint8_t reg_addr, uint16_t value);

    // ── 属性 ─────────────────────────────────────────────────────────────

    std::string getDevice() const { return i2c_device_; }

private:
    // I2C 组合读写（先写寄存器地址，再读数据）
    JY901S_Status i2cWriteThenRead(uint8_t reg_addr, uint8_t* buf, int len);

    // I2C 只写
    JY901S_Status i2cWrite(uint8_t reg_addr, const uint8_t* buf, int len);

    // 合并两个 int16_t（小端）→ float
    static float int16leToFloat(uint8_t lo, uint8_t hi, float scale);

    std::string i2c_device_;
    int fd_ = -1;
};

#endif  // JY901S_H
