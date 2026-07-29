/**
 * @file jy901s.cpp
 * @brief JY901S IMU 传感器 — Linux I2C 驱动实现
 *
 * 使用 Linux i2c-dev 接口（I2C_RDWR ioctl），替代 STM32 HAL 库。
 * 支持欧拉角、加速度、角速度、四元数的读取，以及传感器初始化配置。
 */

#include "jy901s.h"

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  辅助函数
// ═══════════════════════════════════════════════════════════════════════════════

float JY901S::int16leToFloat(uint8_t lo, uint8_t hi, float scale)
{
    int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    return static_cast<float>(raw) * scale;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════════

JY901S::JY901S(const std::string& i2c_device)
    : i2c_device_(i2c_device)
    , fd_(-1)
{
}

JY901S::~JY901S()
{
    close();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  I2C 底层操作
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 向指定寄存器写入数据
 *
 * 使用 I2C_RDWR ioctl 实现单次 I2C 写事务。
 */
JY901S_Status JY901S::i2cWrite(uint8_t reg_addr, const uint8_t* buf, int len)
{
    if (fd_ < 0)
        return JY901S_Status::ERROR_NOT_INITIALIZED;

    // 拼接 [寄存器地址 + 数据]
    uint8_t packet[len + 1];
    packet[0] = reg_addr;
    if (len > 0 && buf != nullptr)
        memcpy(packet + 1, buf, len);

    struct i2c_msg msg;
    msg.addr  = JY901S_I2C_ADDR;
    msg.flags = 0;              // 写
    msg.len   = static_cast<uint16_t>(len + 1);
    msg.buf   = packet;

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = &msg;
    ioctl_data.nmsgs = 1;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
        fprintf(stderr, "[JY901S] I2C write reg=0x%02X failed: %s\n",
                reg_addr, strerror(errno));
        return JY901S_Status::ERROR_I2C_WRITE;
    }
    return JY901S_Status::OK;
}

/**
 * @brief 先写寄存器地址，再读数据（I2C 组合事务，支持 repeated-start）
 *
 * 在一次 I2C 事务中完成"写寄存器地址 + 读数据"，
 * 两次消息之间使用 repeated-start 而非 stop-start。
 */
JY901S_Status JY901S::i2cWriteThenRead(uint8_t reg_addr, uint8_t* buf, int len)
{
    if (fd_ < 0)
        return JY901S_Status::ERROR_NOT_INITIALIZED;

    struct i2c_msg msgs[2];

    // 消息 1: 写寄存器地址（无 STOP，后面接 repeated-start）
    msgs[0].addr  = JY901S_I2C_ADDR;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg_addr;

    // 消息 2: 读数据
    msgs[1].addr  = JY901S_I2C_ADDR;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = static_cast<uint16_t>(len);
    msgs[1].buf   = buf;

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
        fprintf(stderr, "[JY901S] I2C read reg=0x%02X len=%d failed: %s\n",
                reg_addr, len, strerror(errno));
        return JY901S_Status::ERROR_I2C_READ;
    }
    return JY901S_Status::OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  初始化
// ═══════════════════════════════════════════════════════════════════════════════

JY901S_Status JY901S::init(int output_rate_hz)
{
    // ── 1. 打开 I2C 设备 ──────────────────────────────────────────────────
    fd_ = ::open(i2c_device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        fprintf(stderr, "[JY901S] Cannot open %s: %s\n",
                i2c_device_.c_str(), strerror(errno));
        return JY901S_Status::ERROR_I2C_OPEN;
    }

    printf("[JY901S] Opened %s, fd=%d\n", i2c_device_.c_str(), fd_);

    // ── 2. 解锁 ───────────────────────────────────────────────────────────
    // 向 KEY 寄存器 (0x69) 写入 0xB588
    // JY901S 手册要求：解锁后才能修改配置寄存器
    uint8_t unlock_buf[2] = {0x88, 0xB5};  // 小端: 低字节在前
    JY901S_Status st = i2cWrite(JY901S_REG_KEY, unlock_buf, 2);
    if (st != JY901S_Status::OK) {
        fprintf(stderr, "[JY901S] Unlock failed\n");
        close();
        return JY901S_Status::ERROR_UNLOCK_FAILED;
    }
    printf("[JY901S] Unlock OK\n");

    // 验证解锁
    uint16_t lock_val = 0;
    st = readReg16(JY901S_REG_KEY, lock_val);
    if (st == JY901S_Status::OK)
        printf("[JY901S] Lock status: 0x%04X (expected 0xB588)\n", lock_val);

    // ── 3. 配置输出速率 ───────────────────────────────────────────────────
    uint8_t rrate;
    switch (output_rate_hz) {
        case 200: rrate = JY901S_RRATE_200HZ; break;
        case 100: rrate = JY901S_RRATE_100HZ; break;
        case 50:  rrate = JY901S_RRATE_50HZ;  break;
        case 20:  rrate = JY901S_RRATE_20HZ;  break;
        case 10:  rrate = JY901S_RRATE_10HZ;  break;
        default:
            fprintf(stderr, "[JY901S] Unsupported rate %d Hz, using 200Hz\n",
                    output_rate_hz);
            rrate = JY901S_RRATE_200HZ;
    }
    writeReg16(JY901S_REG_RRATE, static_cast<uint16_t>(rrate));
    printf("[JY901S] Output rate set to %d Hz (0x%02X)\n", output_rate_hz, rrate);

    // ── 4. 配置六轴算法（高刷新率模式） ───────────────────────────────────
    writeReg16(JY901S_REG_AXIS6, 0x0001);
    printf("[JY901S] 6-axis algorithm enabled\n");

    // ── 5. 配置陀螺仪带宽 256Hz ───────────────────────────────────────────
    writeReg16(JY901S_REG_BANDWIDTH, 0x0000);
    printf("[JY901S] Gyro bandwidth set to 256Hz\n");

    // ── 6. 保存所有配置 ───────────────────────────────────────────────────
    st = writeReg16(JY901S_REG_SAVE, 0x0000);
    if (st == JY901S_Status::OK)
        printf("[JY901S] Config saved to flash\n");

    // ── 7. 验证配置 ───────────────────────────────────────────────────────
    uint8_t verify;
    readReg8(JY901S_REG_RRATE, verify);
    printf("[JY901S] Verify RRATE: 0x%02X\n", verify);
    readReg8(JY901S_REG_BANDWIDTH, verify);
    printf("[JY901S] Verify BANDWIDTH: 0x%02X\n", verify);
    readReg8(JY901S_REG_AXIS6, verify);
    printf("[JY901S] Verify AXIS6: 0x%02X\n", verify);

    printf("[JY901S] Init complete\n");
    return JY901S_Status::OK;
}

void JY901S::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  传感器数据读取
// ═══════════════════════════════════════════════════════════════════════════════

JY901S_Status JY901S::readAngles(JY901S_AngleData& angles)
{
    // 从 0x3D 连续读取 6 字节 (Roll_L/H, Pitch_L/H, Yaw_L/H)
    uint8_t buf[6] = {0};
    JY901S_Status st = i2cWriteThenRead(JY901S_REG_ROLL_L, buf, 6);
    if (st != JY901S_Status::OK) return st;

    angles.roll  = int16leToFloat(buf[0], buf[1], JY901S_ANGLE_SCALE);
    angles.pitch = int16leToFloat(buf[2], buf[3], JY901S_ANGLE_SCALE);
    angles.yaw   = int16leToFloat(buf[4], buf[5], JY901S_ANGLE_SCALE);

    return JY901S_Status::OK;
}

JY901S_Status JY901S::readAcceleration(JY901S_ACCData& acc)
{
    // 从 0x34 连续读取 6 字节 (AX_L/H, AY_L/H, AZ_L/H)
    uint8_t buf[6] = {0};
    JY901S_Status st = i2cWriteThenRead(JY901S_REG_ACC_X, buf, 6);
    if (st != JY901S_Status::OK) return st;

    acc.x = int16leToFloat(buf[0], buf[1], JY901S_ACC_SCALE);
    acc.y = int16leToFloat(buf[2], buf[3], JY901S_ACC_SCALE);
    acc.z = int16leToFloat(buf[4], buf[5], JY901S_ACC_SCALE);

    return JY901S_Status::OK;
}

JY901S_Status JY901S::readGyro(JY901S_GyroData& gyro)
{
    // 从 0x37 连续读取 6 字节 (GX_L/H, GY_L/H, GZ_L/H)
    uint8_t buf[6] = {0};
    JY901S_Status st = i2cWriteThenRead(JY901S_REG_GYRO_X, buf, 6);
    if (st != JY901S_Status::OK) return st;

    gyro.x = int16leToFloat(buf[0], buf[1], JY901S_GYRO_SCALE);
    gyro.y = int16leToFloat(buf[2], buf[3], JY901S_GYRO_SCALE);
    gyro.z = int16leToFloat(buf[4], buf[5], JY901S_GYRO_SCALE);

    return JY901S_Status::OK;
}

JY901S_Status JY901S::readQuaternion(JY901S_Quaternion& quat)
{
    // 从 0x51 连续读取 8 字节 (Q0_L/H ~ Q3_L/H)
    // 注意: JY901S 四元数量程是 Q30 格式: 实际值 = raw / 2^30
    uint8_t buf[8] = {0};
    JY901S_Status st = i2cWriteThenRead(JY901S_REG_Q0, buf, 8);
    if (st != JY901S_Status::OK) return st;

    constexpr float Q_SCALE = 1.0f / static_cast<float>(1 << 30);  // 1 / 2^30

    quat.q0 = int16leToFloat(buf[0], buf[1], Q_SCALE);  // w
    quat.q1 = int16leToFloat(buf[2], buf[3], Q_SCALE);  // x
    quat.q2 = int16leToFloat(buf[4], buf[5], Q_SCALE);  // y
    quat.q3 = int16leToFloat(buf[6], buf[7], Q_SCALE);  // z

    return JY901S_Status::OK;
}

JY901S_Status JY901S::readAll(JY901S_AngleData& angles,
                               JY901S_ACCData& acc,
                               JY901S_GyroData& gyro)
{
    // 从 0x34 到 0x3F 共 12 字节连续读取
    // 0x34-0x36: ACC X/Y/Z (6 bytes)
    // 0x37-0x39: GYRO X/Y/Z (6 bytes)
    // 0x3D-0x3F: ANGLE Roll/Pitch/Yaw (6 bytes)
    //
    // 注意：0x3A-0x3C 之间有 gap（可能是地磁数据），
    // 所以分两段读更可靠：
    //   段 1: 0x34~0x39 = ACC(6) + GYRO(6) = 12 bytes
    //   段 2: 0x3D~0x3F = Angle(6) = 6 bytes
    // 总共 2 次 I2C 事务，比 3 次单独读取更高效

    uint8_t buf1[12] = {0};
    JY901S_Status st = i2cWriteThenRead(JY901S_REG_ACC_X, buf1, 12);
    if (st != JY901S_Status::OK) return st;

    // ACC: buf1[0-5]
    acc.x = int16leToFloat(buf1[0], buf1[1], JY901S_ACC_SCALE);
    acc.y = int16leToFloat(buf1[2], buf1[3], JY901S_ACC_SCALE);
    acc.z = int16leToFloat(buf1[4], buf1[5], JY901S_ACC_SCALE);

    // GYRO: buf1[6-11]
    gyro.x = int16leToFloat(buf1[6],  buf1[7],  JY901S_GYRO_SCALE);
    gyro.y = int16leToFloat(buf1[8],  buf1[9],  JY901S_GYRO_SCALE);
    gyro.z = int16leToFloat(buf1[10], buf1[11], JY901S_GYRO_SCALE);

    // Angle: 单独读取 6 bytes from 0x3D
    uint8_t buf2[6] = {0};
    st = i2cWriteThenRead(JY901S_REG_ROLL_L, buf2, 6);
    if (st != JY901S_Status::OK) return st;

    angles.roll  = int16leToFloat(buf2[0], buf2[1], JY901S_ANGLE_SCALE);
    angles.pitch = int16leToFloat(buf2[2], buf2[3], JY901S_ANGLE_SCALE);
    angles.yaw   = int16leToFloat(buf2[4], buf2[5], JY901S_ANGLE_SCALE);

    return JY901S_Status::OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  底层寄存器读写
// ═══════════════════════════════════════════════════════════════════════════════

JY901S_Status JY901S::readReg8(uint8_t reg_addr, uint8_t& value)
{
    return i2cWriteThenRead(reg_addr, &value, 1);
}

JY901S_Status JY901S::readReg16(uint8_t reg_addr, uint16_t& value)
{
    uint8_t buf[2] = {0};
    JY901S_Status st = i2cWriteThenRead(reg_addr, buf, 2);
    if (st != JY901S_Status::OK) return st;

    value = static_cast<uint16_t>(buf[1]) << 8 | buf[0];  // 小端
    return JY901S_Status::OK;
}

JY901S_Status JY901S::readRegs(uint8_t reg_addr, uint8_t* buf, int len)
{
    return i2cWriteThenRead(reg_addr, buf, len);
}

JY901S_Status JY901S::writeReg8(uint8_t reg_addr, uint8_t value)
{
    return i2cWrite(reg_addr, &value, 1);
}

JY901S_Status JY901S::writeReg16(uint8_t reg_addr, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(value & 0xFF);         // 低字节
    buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);  // 高字节
    return i2cWrite(reg_addr, buf, 2);
}
