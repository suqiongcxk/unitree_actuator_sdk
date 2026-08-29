/**
 * @file ZeroPointCalibration.cpp
 *
 * Creeper 四足机器人 — 机械限位校准与上电启动控制
 *
 * 功能:
 *   1. 12 关节机械限位标定（基于状态监测的到位检测）
 *   2. 标定结果验证（实测行程 vs URDF 预期行程）
 *   3. 平滑过渡到默认站立姿态
 *   4. 完整启动状态机
 *
 * 12 电机映射 (Z 字排序):
 *   Hip:     0(FL), 1(FR), 2(RL), 3(RR)
 *   Thigh:   4(FL), 5(FR), 6(RL), 7(RR)
 *   Calf:    8(FL), 9(FR), 10(RL), 11(RR)
 *
 * 4 路 RS-485 总线:
 *   Leg1(FL): GPIO  39, /dev/ttyS6,  motors {0,  4,  8}
 *   Leg2(FR): GPIO  63, /dev/ttyS4,  motors {1,  5,  9}
 *   Leg3(RL): GPIO  35, /dev/ttyS7,  motors {2,  6, 10}
 *   Leg4(RR): GPIO 133, /dev/ttyS0,  motors {3,  7, 11}
 */

#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <cctype>
#include "motor_controller.h"
#include "ZeroPointCalibration.h"
#include "emergency_stop.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  全局数据
// ═══════════════════════════════════════════════════════════════════════════════

static volatile bool g_running = true;
static void sigint_handler(int) {
    g_running = false;
    requestEmergencyStop();
}

float ZERO_Position_MechLimitEnd[12]   = {0};  // 机械上限 (rad, 电机坐标系)
float ZERO_Position_MechLimitStart[12] = {0};  // 机械下限 (rad, 电机坐标系)
float motor_zero_offset[12]            = {0};  // URDF零点对应的电机位置
int   motor_direction_arr[12]          = {0};  // 电机→URDF方向系数
float URDF_Joint_zero_OFFSET[12]       = {0};  // URDF 零点偏置 (保留兼容)

/// 默认站立姿态 (Z 字排序)
const float default_joint_pos[12] = {
     0.1f, -0.1f,  0.1f, -0.1f,   // Hip:  FL, FR, RL, RR
     0.8f,  0.8f,  1.0f,  1.0f,   // Thigh
    -1.5f, -1.5f, -1.5f, -1.5f    // Calf
};

// ═══════════════════════════════════════════════════════════════════════════════
//  硬件配置
// ═══════════════════════════════════════════════════════════════════════════════

LEG_UART_SET Leg1_Uart;
LEG_UART_SET Leg2_Uart;
LEG_UART_SET Leg3_Uart;
LEG_UART_SET Leg4_Uart;
SINGLE_LEG_MOtor_SET Leg1_Motor;
SINGLE_LEG_MOtor_SET Leg2_Motor;
SINGLE_LEG_MOtor_SET Leg3_Motor;
SINGLE_LEG_MOtor_SET Leg4_Motor;

void LEG_UART_INIT(void)
{
    Leg1_Uart = {39,  "/dev/ttyS6"};
    Leg2_Uart = {63,  "/dev/ttyS4"};
    Leg3_Uart = {35,  "/dev/ttyS7"};
    Leg4_Uart = {133, "/dev/ttyS0"};
}

void LEG_MOTOR_INIT(void)
{
    LEG_UART_INIT();
    Leg1_Motor = {0, 4, 8,  Leg1_Uart};   // FL: hip=0, thigh=4, calf=8
    Leg2_Motor = {1, 5, 9,  Leg2_Uart};   // FR: hip=1, thigh=5, calf=9
    Leg3_Motor = {2, 6, 10, Leg3_Uart};   // RL: hip=2, thigh=6, calf=10
    Leg4_Motor = {3, 7, 11, Leg4_Uart};   // RR: hip=3, thigh=7, calf=11
}

// ═══════════════════════════════════════════════════════════════════════════════
//  腿部总线硬件表 (供 calibrateAllJoints 创建 MotorBus)
// ═══════════════════════════════════════════════════════════════════════════════

struct LegBusHardware {
    int gpio_pin;
    const char* serial_port;
};

static const LegBusHardware LEG_BUS_HW[4] = {
    {39,  "/dev/ttyS6"},   // Leg1 (FL)     B
    {63,  "/dev/ttyS4"},   // Leg2 (FR)     A
    {35,  "/dev/ttyS7"},   // Leg3 (RL)     C
    {133, "/dev/ttyS0"},   // Leg4 (RR)     D
};

// ═══════════════════════════════════════════════════════════════════════════════
//  标定配置表 (12 电机)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  坐标方向说明（已经实机标定和开环站立验证）:
//    motor_direction 定义 q_urdf = motor_direction * (q_motor - zero_offset)。
//    - Hip:   FL/FR 用 +1，RL/RR 用 -1
//    - Thigh: FL/RL 用 +1，FR/RR 用 -1
//    - Calf:  FL/RL 用 +1，FR/RR 用 -1
//  calib_velocity 只表示标定时撞向指定机械限位的电机速度方向，
//  与 motor_direction（电机坐标→URDF 坐标符号）不是同一概念。
//
//  hit_upper_first:
//    - true  = 先撞上限, 记录 MechLimitEnd,  推导 MechLimitStart
//    - false = 先撞下限, 记录 MechLimitStart, 推导 MechLimitEnd
//
//  calib_kd: 0.06-0.08 低阻尼确保安全接触机械限位
//  timeout_sec: 8 秒 (远大于预期运动时间, 防止卡死)

const JointCalibConfig* getCalibrationConfigs()
{
    static const JointCalibConfig configs[12] = {
        // ═══ Leg1: FL (GPIO 39, /dev/ttyS6) ═══
        {0,  0,  1.5f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, true,   +1},  // Hip  FL
        {4,  0,  1.5f, 0.08f, 8.0f, -1.5708f,  3.4907f,  5.0615f, true,   +1},  // Thigh FL
        {8,  0, -1.5f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false,  +1},  // Calf  FL

        // ═══ Leg2: FR (GPIO 63, /dev/ttyS4) ═══
        // 注意: FR Hip 速度方向为负 (镜像), 实机测试时如方向错误请调整 calib_velocity 符号
        {1,  1, -1.5f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, false,  +1},  // Hip  FR
        {5,  1, -1.5f, 0.08f, 8.0f, -1.5708f,  3.4907f,  5.0615f, true,   -1},  // Thigh FR
        {9,  1,  1.5f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false,  -1},  // Calf  FR

        // ═══ Leg3: RL (GPIO 35, /dev/ttyS7) ═══
        {2,  2, -1.5f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, true,   -1},  // Hip  RL
        {6,  2,  1.5f, 0.08f, 8.0f, -0.5236f,  4.5379f,  5.0615f, true,   +1},  // Thigh RL
        {10, 2, -1.5f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false,  +1},  // Calf  RL

        // ═══ Leg4: RR (GPIO 133, /dev/ttyS0) ═══
        // 注意: RR Hip 速度方向为负 (镜像), 实机测试时如方向错误请调整 calib_velocity 符号
        {3,  3,  1.5f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, false,  -1},  // Hip  RR
        {7,  3, -1.5f, 0.08f, 8.0f, -0.5236f,  4.5379f,  5.0615f, true,   -1},  // Thigh RR
        {11, 3,  1.5f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false,  -1},  // Calf  RR
    };
    return configs;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  calibrateSingleJoint — 单关节机械限位标定
// ═══════════════════════════════════════════════════════════════════════════════
//
//  检测逻辑:
//    1. 以低速低阻尼驱动电机向限位方向运动
//    2. 每 10ms 轮询: 读取位置和速度
//    3. 到位判定: |dq| < 0.05 rad/s 且连续 15 次 (150ms) 位置变化 < 0.001 rad
//    4. 超时保护: > timeout_sec
//    5. 通信失败保护: !state.correct 或 merror != 0
//
//  参数:
//    bus        - 该腿的 MotorBus 引用
//    cfg        - 标定配置
//    result     - [out] 标定结果

bool calibrateSingleJoint(MotorBus& bus,
                          const JointCalibConfig& cfg,
                          JointCalibResult& result)
{
    result = {};  // 清零
    result.success = false;
    if (isEmergencyStopRequested()) return false;

    // ── 1. 启动速度模式，向限位方向运动 ──
    bus.setVelocity(cfg.motor_id, cfg.calib_velocity, cfg.calib_kd);
    bus.sendRecv();

    std::cout << "    [Calib] Motor " << cfg.motor_id
              << " v=" << cfg.calib_velocity
              << " kd=" << cfg.calib_kd << std::flush;

    // ── 2. 轮询监测 ──
    auto t_start = std::chrono::steady_clock::now();
    float prev_q = 0.0f;
    bool  first_read = true;
    int   stall_count = 0;
    int   comm_fail_count = 0;   // 连续通信失败计数
    const int STALL_THRESHOLD   = 15;   // 连续 15 次 (150ms) 判定到位
    const int COMM_FAIL_MAX     = 10;   // 连续 10 次通信失败才退出
    const float STALL_VELOCITY  = 0.10f;   // rad/s
    const float STALL_DELTA_Q   = 0.005f;  // rad

    while (g_running && !isEmergencyStopRequested())
    {
        // ── 超时检查 ──
        auto now = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - t_start).count();
        if (elapsed_ms > cfg.timeout_sec * 1000)
        {
            std::cout << " TIMEOUT(" << elapsed_ms << "ms)" << std::endl;
            result.error_code = 2;
            result.calibration_time_ms = elapsed_ms;
            bus.brake(cfg.motor_id);
            bus.sendRecv();
            return false;
        }

        // ── 继续发送速度指令 (保持向限位运动) ──
        bus.setVelocity(cfg.motor_id, cfg.calib_velocity, cfg.calib_kd);
        bus.sendRecv();
        MotorState state = bus.getState(cfg.motor_id);

        // ── 通信检查 (连续失败 N 次才退出) ──
        if (!state.correct)
        {
            comm_fail_count++;
            if (comm_fail_count >= COMM_FAIL_MAX)
            {
                std::cout << " COMM_FAIL(x" << comm_fail_count << ")" << std::endl;
                result.error_code = 1;
                result.calibration_time_ms = elapsed_ms;
                bus.brake(cfg.motor_id);
                bus.sendRecv();
                return false;
            }
            usleep(10000);
            continue;   // 跳过本轮到位检测
        }
        else
        {
            comm_fail_count = 0;   // 通信恢复, 重置计数
        }

        // ── 电机故障检查 (连续 N 次才退出) ──
        if (state.merror != 0)
        {
            // merror 通常不会自己恢复, 出现即致命
            std::cout << " MERROR(" << state.merror << ")" << std::endl;
            result.error_code = 1;
            result.calibration_time_ms = elapsed_ms;
            bus.brake(cfg.motor_id);
            bus.sendRecv();
            return false;
        }

        // ── 到位判定: 速度接近零 AND 位置变化持续低于阈值 ──
        if (!first_read)
        {
            float delta_q = std::abs(state.q - prev_q);

            if (std::abs(state.dq) < STALL_VELOCITY && delta_q < STALL_DELTA_Q)
            {
                stall_count++;
                if (stall_count >= STALL_THRESHOLD)
                {
                    std::cout<< "get15"<<std::endl;
                    // 确认到达机械限位
                    result.final_velocity = state.dq;
                    result.calibration_time_ms = elapsed_ms;

                    // ── 计算 motor_zero_offset ──
                    // q_motor_hit = state.q  (电机坐标系)
                    // q_urdf_hit  = hit_upper_first ? urdf_upper : urdf_lower  (URDF)
                    // q_urdf = motor_dir * (q_motor - zero_offset)
                    // → zero_offset = q_motor_hit - motor_dir * q_urdf_hit
                    float q_motor_hit = state.q;
                    float q_urdf_hit  = cfg.hit_upper_first ? cfg.urdf_upper : cfg.urdf_lower;
                    float zero_off = q_motor_hit - cfg.motor_direction * q_urdf_hit;

                    motor_zero_offset[cfg.motor_id] = zero_off;
                    motor_direction_arr[cfg.motor_id] = cfg.motor_direction;

                    // 计算电机坐标系下的机械限位
                    float motor_at_urdf_lower = zero_off + cfg.motor_direction * cfg.urdf_lower;
                    float motor_at_urdf_upper = zero_off + cfg.motor_direction * cfg.urdf_upper;

                    result.mech_limit_start = std::min(motor_at_urdf_lower, motor_at_urdf_upper);
                    result.mech_limit_end   = std::max(motor_at_urdf_lower, motor_at_urdf_upper);
                    result.measured_range = result.mech_limit_end - result.mech_limit_start;
                    result.range_error    = std::abs(result.measured_range - cfg.urdf_range);
                    result.success = true;

                    std::cout << " OK q=" << std::fixed << std::setprecision(4)
                              << state.q << " (" << elapsed_ms << "ms)" << std::endl;
                    break;
                }
            }
            else
            {
                stall_count = 0;
            }
        }

        prev_q = state.q;
        first_read = false;
        usleep(10000);  // 10ms 轮询周期
    }

    if (!result.success)
    {
        // SIGINT 中断
        bus.brake(cfg.motor_id);
        bus.sendRecv();
        return false;
    }

    // ── 3. 标定完成 → 极低刚度位置保持 (防下垂, 比阻尼模式省电安全) ──
    //    后续 calibrateAllJoints 会对 Hip 立即覆盖为站立过渡;
    //    对 Thigh/Calf, 保持原地直到统一过渡阶段再逐步拉高刚度
    float q_urdf_hold = cfg.hit_upper_first ? cfg.urdf_upper : cfg.urdf_lower;
    float q_motor_hold = urdfToMotorPosition(cfg.motor_id, q_urdf_hold);
    bus.setPosition(cfg.motor_id, q_motor_hold, 0.02f, 0.01f);
    bus.sendRecv();

    // ── 4. 写入全局标定数组 ──
    ZERO_Position_MechLimitStart[cfg.motor_id] = result.mech_limit_start;
    ZERO_Position_MechLimitEnd[cfg.motor_id]   = result.mech_limit_end;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  calibrateAllJoints — 12 关节完整标定
// ═══════════════════════════════════════════════════════════════════════════════
//
//  标定顺序: 左侧 FL+RL 并行 → 右侧 FR+RR 并行
//  每腿内顺序: Calf → Thigh → [后腿安全回退] → Hip
//
//  返回值: 成功标定的关节数量 (0-12)

int calibrateAllJoints(JointCalibResult results[12], uint8_t leg_mask)
{
    const auto* configs = getCalibrationConfigs();
    std::atomic<int> success_count{0};

    static const char* LEG_NAME[4] = {"FL", "FR", "RL", "RR"};
    int selected_count = 0;
    for (int l = 0; l < 4; l++) if (leg_mask & (1 << l)) selected_count++;

    std::cout << "\n┌──────────────────────────────────────────┐" << std::endl;
    std::cout <<   "│  机械限位标定 — " << selected_count << " 条腿 (";
    bool first = true;
    for (int l = 0; l < 4; l++) {
        if (leg_mask & (1 << l)) {
            if (!first) std::cout << ",";
            std::cout << LEG_NAME[l];
            first = false;
        }
    }
    std::cout << ")                  │" << std::endl;
    std::cout <<   "└──────────────────────────────────────────┘" << std::endl;

    // 每个工作线程只操作自己的 MotorBus 和结果区间，不共享串口/GPIO。
    auto calibrateLeg = [&](int leg)
    {
        if (isEmergencyStopRequested() || !(leg_mask & (1 << leg))) return;
        try {
        std::cout << "\n── Leg" << (leg + 1) << " (" << LEG_NAME[leg]
                  << ", GPIO " << LEG_BUS_HW[leg].gpio_pin
                  << ", " << LEG_BUS_HW[leg].serial_port << ") ──"
                  << std::endl;

        // 为该腿创建临时 MotorBus (标定阶段 ParallelBus 尚未启动, 无 GPIO/串口冲突)
        MotorBus bus(LEG_BUS_HW[leg].gpio_pin, LEG_BUS_HW[leg].serial_port);

        // 注册该腿的 3 个电机
        for (int j = 0; j < 3; j++)
        {
            bus.addMotor(configs[leg * 3 + j].motor_id);
        }

        // ── 标定顺序: Calf → Thigh → [后腿大腿回退] → Hip ──
        // 先收拢远端, 后腿大腿需从限位退回安全角再动髋, 避免碰机身

        // 1) Calf (j=2)
        {
            int idx = leg * 3 + 2;
            if (calibrateSingleJoint(bus, configs[idx], results[idx]))
                success_count.fetch_add(1, std::memory_order_relaxed);
            else
                std::cerr << "    !! Motor " << configs[idx].motor_id
                          << " 标定失败, 继续..." << std::endl;
        }

        // 2) Thigh (j=1)
        {
            int idx = leg * 3 + 1;
            if (calibrateSingleJoint(bus, configs[idx], results[idx]))
                success_count.fetch_add(1, std::memory_order_relaxed);
            else
                std::cerr << "    !! Motor " << configs[idx].motor_id
                          << " 标定失败, 继续..." << std::endl;
        }

        // 3) [Leg3/RL & Leg4/RR] 大腿从限位退回安全角, 避免 Hip 校准时碰机身
        //    后腿大腿上限 ~4.54 rad (260°), 退 80° → ~3.14 rad
        const float REAR_THIGH_SAFE_BACKOFF_RAD = 1.3963f;  // 80°
        if ((leg == 2 || leg == 3) && results[leg * 3 + 1].success)
        {
            int thigh_idx = leg * 3 + 1;
            const auto& thigh_cfg = configs[thigh_idx];
            int thigh_motor = thigh_cfg.motor_id;

            // 当前位置: 校准后停在撞到的URDF限位处 (用转换函数, 不受 motor_direction 影响)
            float q_urdf_hit = thigh_cfg.hit_upper_first
                             ? thigh_cfg.urdf_upper : thigh_cfg.urdf_lower;
            float thigh_current_q = urdfToMotorPosition(thigh_motor, q_urdf_hit);

            // 目标: 从当前URDF位置退回 REAR_THIGH_SAFE_BACKOFF_RAD
            float q_urdf_safe = q_urdf_hit - REAR_THIGH_SAFE_BACKOFF_RAD;
            float thigh_safe_q = urdfToMotorPosition(thigh_motor, q_urdf_safe);

            std::cout << "    [Thigh] Motor " << thigh_motor
                      << " 退回安全角: " << std::fixed << std::setprecision(4)
                      << thigh_current_q << " → " << thigh_safe_q
                      << " (退 " << REAR_THIGH_SAFE_BACKOFF_RAD << " rad)" << std::endl;

            const int   THIGH_STEPS = 80;
            const float THIGH_TIME  = 0.8f;
            const float THIGH_KP    = 0.3f;
            const float THIGH_KD    = 0.02f;

            for (int step = 0; step <= THIGH_STEPS && !isEmergencyStopRequested(); step++)
            {
                float alpha = static_cast<float>(step) / THIGH_STEPS;
                float smooth_alpha = alpha * alpha * (3.0f - 2.0f * alpha);
                float q_cmd = thigh_current_q
                            + smooth_alpha * (thigh_safe_q - thigh_current_q);
                bus.setPosition(thigh_motor, q_cmd, THIGH_KP, THIGH_KD);
                bus.sendRecv();
                usleep(static_cast<int>(THIGH_TIME / THIGH_STEPS * 1e6));
            }
            std::cout << "    [Thigh] Motor " << thigh_motor << " 安全角到位" << std::endl;
        }

        // 4) Hip (j=0)
        {
            int idx = leg * 3 + 0;
            if (calibrateSingleJoint(bus, configs[idx], results[idx]))
                success_count.fetch_add(1, std::memory_order_relaxed);
            else
                std::cerr << "    !! Motor " << configs[idx].motor_id
                          << " 标定失败, 继续..." << std::endl;
        }

        // ── 髋关节定位到站立姿态 ──
        // Thigh/Calf 保持在阻尼模式 (无力下垂, 不受重力影响的 Hip 提前归位)
        int hip_idx = leg * 3 + 0;  // Hip 是每条腿配置表的第一个 (j=0)
        if (results[hip_idx].success)
        {
            const auto& hip_cfg = configs[hip_idx];
            int hip_motor_id = hip_cfg.motor_id;

            // 当前位置: 校准后停在撞到的URDF限位处 (用转换函数, 不受 motor_direction 影响)
            float q_urdf_hit = hip_cfg.hit_upper_first
                             ? hip_cfg.urdf_upper : hip_cfg.urdf_lower;
            float hip_current_q = urdfToMotorPosition(hip_motor_id, q_urdf_hit);

            // 目标: URDF 默认站立角度 → 电机输出端位置
            float hip_target_q = computeMotorTargetFromURDF(hip_motor_id,
                                  default_joint_pos[hip_motor_id]);

            const int   HIP_STEPS = 100;
            const float HIP_TIME  = 1.0f;
            const float HIP_KP    = 0.3f;
            const float HIP_KD    = 0.02f;

            std::cout << "    [Hip] Motor " << hip_motor_id
                      << " 定位站立: " << std::fixed << std::setprecision(4)
                      << hip_current_q << " → " << hip_target_q
                      << " (" << HIP_TIME << "s)" << std::endl;

            for (int step = 0; step <= HIP_STEPS && !isEmergencyStopRequested(); step++)
            {
                float alpha = static_cast<float>(step) / HIP_STEPS;
                float smooth_alpha = alpha * alpha * (3.0f - 2.0f * alpha);
                float q_cmd = hip_current_q
                            + smooth_alpha * (hip_target_q - hip_current_q);
                bus.setPosition(hip_motor_id, q_cmd, HIP_KP, HIP_KD);
                bus.sendRecv();
                usleep(static_cast<int>(HIP_TIME / HIP_STEPS * 1e6));
            }

            std::cout << "    [Hip] Motor " << hip_motor_id << " 到位" << std::endl;
        }

        // MotorBus 析构自动释放 GPIO 和串口
        } catch (const std::exception& e) {
            std::cerr << "[Calib] " << LEG_NAME[leg]
                      << " 标定线程异常: " << e.what() << std::endl;
            requestEmergencyStop();
        }
    };

    auto runPair = [&](int leg_a, int leg_b) {
        std::vector<std::thread> workers;
        if (leg_mask & (1 << leg_a)) workers.emplace_back(calibrateLeg, leg_a);
        if (leg_mask & (1 << leg_b)) workers.emplace_back(calibrateLeg, leg_b);
        for (auto& worker : workers) worker.join();
    };

    std::cout << "[Calib] 左侧 FL+RL 并行标定" << std::endl;
    runPair(0, 2);
    if (!isEmergencyStopRequested()) {
        std::cout << "[Calib] 右侧 FR+RR 并行标定" << std::endl;
        runPair(1, 3);
    }

    int final_success_count = success_count.load(std::memory_order_relaxed);
    std::cout << "\n[Calib] 标定完成: " << final_success_count
              << "/" << (selected_count * 3) << " 关节成功" << std::endl;
    return final_success_count;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  validateCalibrationResults — 标定结果验证
// ═══════════════════════════════════════════════════════════════════════════════
//
//  对比实测机械行程与 URDF 预期行程。
//  偏差超过 max_range_error 的关节将被标记为验证失败。

bool validateCalibrationResults(const JointCalibResult results[12],
                                const JointCalibConfig configs[12],
                                float max_range_error,
                                uint8_t leg_mask)
{
    bool all_valid = true;

    std::cout << "\n┌──────────────────────────────────────────┐" << std::endl;
    std::cout <<   "│  标定结果验证                             │" << std::endl;
    std::cout <<   "└──────────────────────────────────────────┘" << std::endl;

    std::cout << std::fixed << std::setprecision(4);

    for (int i = 0; i < 12; i++)
    {
        if (!(leg_mask & (1 << configs[i].leg_index))) continue;
        if (!results[i].success)
        {
            std::cerr << "  Motor " << std::setw(2) << configs[i].motor_id
                      << ": 标定失败 (error=" << results[i].error_code
                      << "), 跳过验证" << std::endl;
            all_valid = false;
            continue;
        }

        float expected_range = configs[i].urdf_range;
        float measured_range = results[i].measured_range;
        float error = std::abs(measured_range - expected_range);

        if (error > max_range_error)
        {
            std::cerr << "  Motor " << std::setw(2) << configs[i].motor_id
                      << ": ** 行程异常! **"
                      << " 实测=" << std::setw(8) << measured_range << " rad"
                      << " 预期=" << std::setw(8) << expected_range << " rad"
                      << " 偏差=" << std::setw(8) << error << " rad"
                      << " (上限=" << max_range_error << " rad)"
                      << std::endl;
            all_valid = false;
        }
        else
        {
            std::cout << "  Motor " << std::setw(2) << configs[i].motor_id
                      << ": OK  实测=" << std::setw(8) << measured_range << " rad"
                      << " (偏差 " << error << " rad)"
                      << "  [" << results[i].mech_limit_start
                      << ", " << results[i].mech_limit_end << "]"
                      << std::endl;
        }
    }

    if (all_valid)
        std::cout << "\n[Verify] ✓ 已选侧别的关节行程验证通过" << std::endl;
    else
        std::cerr << "\n[Verify] ✗ 部分关节行程验证失败!" << std::endl;

    return all_valid;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  computeMotorTargetFromURDF — URDF 关节角 → 电机输出端位置
// ═══════════════════════════════════════════════════════════════════════════════

float computeMotorTargetFromURDF(int motor_id, float urdf_target)
{
    // q_motor = motor_zero_offset + motor_direction * q_urdf
    return urdfToMotorPosition(motor_id, urdf_target);
}

bool enterDampingModeForAllMotors(float kd, int hold_ms)
{
    const auto* configs = getCalibrationConfigs();
    std::vector<std::unique_ptr<MotorBus>> buses;

    try {
        // 四路总线同时保持打开，使 12 个电机在保持窗口内持续收到阻尼帧。
        for (int leg = 0; leg < 4; ++leg) {
            auto bus = std::make_unique<MotorBus>(
                LEG_BUS_HW[leg].gpio_pin, LEG_BUS_HW[leg].serial_port);
            for (int j = 0; j < 3; ++j) {
                bus->addMotor(configs[leg * 3 + j].motor_id);
            }
            buses.push_back(std::move(bus));
        }

        const int period_us = 10000;  // 100 Hz 周期发送，避免只发一次即退出。
        const int rounds = std::max(1, hold_ms * 1000 / period_us);
        for (int round = 0; round < rounds; ++round) {
            for (int leg = 0; leg < 4; ++leg) {
                for (int j = 0; j < 3; ++j) {
                    buses[leg]->setDamping(configs[leg * 3 + j].motor_id, kd);
                }
                buses[leg]->sendRecv();
            }
            usleep(period_us);
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Safety] 全电机阻尼失败: " << e.what() << std::endl;
        return false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ZeroPointCalibration — 独立标定入口 (兼容旧接口)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  使用 MotorBus 进行完整的 12 关节标定 + 验证 + 站立过渡。
//  此函数供 example_usage 等独立测试程序调用。

void ZeroPointCalibration(void)
{
    resetEmergencyStop();
    g_running = true;
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    struct DampingOnExit {
        ~DampingOnExit() {
            std::cout << "\n[Calib] 全部电机进入阻尼模式..." << std::endl;
            enterDampingModeForAllMotors();
        }
    } damping_on_exit;

    std::cout << "╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Creeper 零位标定 — 独立模式                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;

    LEG_MOTOR_INIT();

    // ── 交互选择校准侧别 ──
    std::cout << "\n选择校准侧别:" << std::endl;
    std::cout << "  left  / 左侧 = FL + RL 并行" << std::endl;
    std::cout << "  right / 右侧 = FR + RR 并行" << std::endl;
    std::cout << "  all   / 全部 = 左侧完成后校准右侧" << std::endl;
    std::cout << "  请输入 left、right 或 all: " << std::flush;

    std::string input;
    std::getline(std::cin, input);

    std::transform(input.begin(), input.end(), input.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    uint8_t leg_mask = 0;
    if (input == "all" || input == "全部" || input.empty()) {
        leg_mask = 0x0F;
    } else if (input == "left" || input == "l" || input == "左" || input == "左侧") {
        leg_mask = 0x05;  // bit0=FL, bit2=RL
    } else if (input == "right" || input == "r" || input == "右" || input == "右侧") {
        leg_mask = 0x0A;  // bit1=FR, bit3=RR
    }
    if (leg_mask == 0) {
        std::cout << "未选择任何腿, 退出" << std::endl;
        return;
    }

    // ── 阶段 1: 关节标定 ──
    JointCalibResult results[12] = {};
    int expected_joints = 0;
    for (int l = 0; l < 4; l++) if (leg_mask & (1 << l)) expected_joints += 3;
    int ok_count = calibrateAllJoints(results, leg_mask);

    if (ok_count < expected_joints)
    {
        std::cerr << "\n[FATAL] 标定成功数不足 (" << ok_count << "/" << expected_joints << "), 退出" << std::endl;
        return;
    }

    // ── 阶段 2: 验证标定结果 ──
    const auto* configs = getCalibrationConfigs();
    bool verified = validateCalibrationResults(results, configs, 0.15f, leg_mask);

    if (!verified)
    {
        std::cerr << "[FATAL] 标定验证失败, 退出" << std::endl;
        return;
    }

    // 记录校准流程结束后的预计位置（下标严格等于 motor ID）。
    float expected_position[12] = {0};
    for (int i = 0; i < 12; ++i) {
        if (!results[i].success) continue;
        const auto& cfg = configs[i];
        int mid = cfg.motor_id;
        float q_urdf = cfg.hit_upper_first ? cfg.urdf_upper : cfg.urdf_lower;
        if (mid >= 0 && mid <= 3) {
            q_urdf = default_joint_pos[mid];  // Hip 已在标定线程内归位。
        } else if ((cfg.leg_index == 2 || cfg.leg_index == 3)
                   && mid >= 4 && mid <= 7 && cfg.hit_upper_first) {
            q_urdf -= 1.3963f;  // 后腿大腿已回退 80°。
        }
        expected_position[mid] = urdfToMotorPosition(mid, q_urdf);
    }

    // ── 阶段 3: 大腿预定位 (站立前将大腿调整到更安全的角度) ──
    //    各大腿均在 URDF 减小方向预定位；具体角度见下表，编码器方向自动转换。
    {
        struct ThighPrePos { int motor_id; float urdf_delta; };
        const ThighPrePos pre[4] = {
            {4, -1.1343f},  // FL Thigh: -65°
            {5, -1.1343f},  // FR Thigh: -65°
            {6, -0.698f},  // RL Thigh: -40°
            {7, -0.698f},  // RR Thigh: -40°
        };
        struct PreparedMove {
            bool active = false;
            int motor_id = -1;
            int leg = -1;
            float current_motor = 0.0f;
            float target_motor = 0.0f;
        };
        std::array<PreparedMove, 4> moves{};
        std::cout << "\n[PrePos] 四条大腿并行预定位 (URDF 减小方向)..."
                  << std::endl;

        // 先完成目标计算，再同时启动四路独立UART。
        for (int p = 0; p < 4; ++p) {
            if (isEmergencyStopRequested()) break;
            int mid = pre[p].motor_id;

            // configs/results 按腿排列，不可直接用 motor_id 作为数组下标。
            int cfg_idx = -1;
            for (int i = 0; i < 12; ++i) {
                if (configs[i].motor_id == mid) {
                    cfg_idx = i;
                    break;
                }
            }
            if (cfg_idx < 0 || !results[cfg_idx].success) continue;

            const auto& cfg = configs[cfg_idx];
            int leg = cfg.leg_index;

            // 当前 URDF 位置
            float q_urdf_hit = cfg.hit_upper_first ? cfg.urdf_upper : cfg.urdf_lower;
            float cur_urdf = q_urdf_hit;
            // 后腿大腿已在标定中退过 80°, 需要反映到当前URDF位置
            if ((leg == 2 || leg == 3) && mid == cfg.motor_id) {
                // 后腿大腿当前在 upper - 80°
                if (cfg.hit_upper_first) cur_urdf -= 1.3963f;
            }

            // 目标URDF位置 (限幅到URDF范围内留0.05rad余量)
            float tgt_urdf = cur_urdf + pre[p].urdf_delta;
            tgt_urdf = std::max(cfg.urdf_lower + 0.05f,
                       std::min(cfg.urdf_upper - 0.05f, tgt_urdf));

            // 转换到电机坐标
            float cur_motor  = urdfToMotorPosition(mid, cur_urdf);
            float tgt_motor  = urdfToMotorPosition(mid, tgt_urdf);
            float travel = std::abs(tgt_motor - cur_motor);

            std::cout << "  Thigh Motor " << mid
                      << ": URDF " << std::fixed << std::setprecision(3)
                      << cur_urdf << "→" << tgt_urdf
                      << " (Δ" << pre[p].urdf_delta << " rad, "
                      << travel << " rad motor)" << std::endl;

            moves[p] = {true, mid, leg, cur_motor, tgt_motor};
        }

        const int steps = 60;
        const float duration_sec = 0.8f;
        std::array<bool, 4> move_ok{{false, false, false, false}};
        std::vector<std::thread> workers;
        for (int p = 0; p < 4; ++p) {
            if (!moves[p].active) continue;
            workers.emplace_back([&, p]() {
                const auto& move = moves[p];
                try {
                    MotorBus bus(LEG_BUS_HW[move.leg].gpio_pin,
                                 LEG_BUS_HW[move.leg].serial_port);
                    if (!bus.addMotor(move.motor_id)) {
                        ::requestEmergencyStop();
                        return;
                    }
                    for (int step = 0;
                         step <= steps && !isEmergencyStopRequested(); ++step) {
                        const float alpha = static_cast<float>(step) / steps;
                        const float smooth = alpha * alpha * (3.0f - 2.0f * alpha);
                        const float command = move.current_motor
                                            + smooth * (move.target_motor
                                                      - move.current_motor);
                        bus.setPosition(move.motor_id, command, 0.15f, 0.01f);
                        bus.sendRecv();
                        usleep(static_cast<int>(duration_sec / steps * 1.0e6f));
                    }
                    move_ok[p] = !isEmergencyStopRequested();
                } catch (const std::exception& error) {
                    std::cerr << "[PrePos] Thigh " << move.motor_id
                              << " 并行预定位异常: " << error.what()
                              << std::endl;
                    ::requestEmergencyStop();
                }
            });
        }
        for (auto& worker : workers) worker.join();

        bool all_moves_ok = !isEmergencyStopRequested();
        for (int p = 0; p < 4; ++p) {
            if (!moves[p].active) continue;
            if (!move_ok[p]) {
                all_moves_ok = false;
                continue;
            }
            expected_position[moves[p].motor_id] = moves[p].target_motor;
        }
        if (!all_moves_ok) {
            std::cerr << "[PrePos] 大腿并行预定位未完成，终止站立过渡"
                      << std::endl;
            return;
        }
        std::cout << "[PrePos] 大腿预定位完成" << std::endl;
    }

    // ── 阶段 4: 从真实反馈位置平滑过渡到默认站立姿态 ──
    std::cout << "\n[Transition] 从当前反馈位置过渡到默认站立姿态..." << std::endl;

    float stand_start[12] = {0};
    float stand_target[12] = {0};
    float last_cmd[12] = {0};
    int comm_fail_count[12] = {0};
    std::vector<std::unique_ptr<MotorBus>> stand_buses(4);

    // 总线只创建一次；先发送预计当前位置的低刚度指令，并读取实际反馈作为插值起点。
    for (int leg = 0; leg < 4 && !isEmergencyStopRequested(); ++leg) {
        if (!(leg_mask & (1 << leg))) continue;
        stand_buses[leg] = std::make_unique<MotorBus>(
            LEG_BUS_HW[leg].gpio_pin, LEG_BUS_HW[leg].serial_port);
        for (int j = 0; j < 3; ++j) {
            int idx = leg * 3 + j;
            int mid = configs[idx].motor_id;
            stand_buses[leg]->addMotor(mid);
            stand_buses[leg]->setPosition(mid, expected_position[mid], 0.02f, 0.01f);
            stand_target[mid] = computeMotorTargetFromURDF(mid, default_joint_pos[mid]);
        }
        stand_buses[leg]->sendRecv();
        for (int j = 0; j < 3; ++j) {
            int idx = leg * 3 + j;
            int mid = configs[idx].motor_id;
            MotorState state = stand_buses[leg]->getState(mid);
            stand_start[mid] = state.correct ? state.q : expected_position[mid];
            last_cmd[mid] = stand_start[mid];
        }
    }

    const int TRANSITION_STEPS = 200;
    const float TRANSITION_TIME = 3.0f;
    const float STEP_DT = TRANSITION_TIME / TRANSITION_STEPS;
    const float KP_START = 0.02f;
    const float KP_END = 0.6f;
    const float KD = 0.0125f;
    const float MAX_STEP_DELTA = 0.02f;

    for (int step = 0; step <= TRANSITION_STEPS && g_running
                         && !isEmergencyStopRequested(); ++step) {
        float alpha = static_cast<float>(step) / TRANSITION_STEPS;
        float smooth_alpha = alpha * alpha * (3.0f - 2.0f * alpha);
        float kp = KP_START + smooth_alpha * (KP_END - KP_START);

        for (int leg = 0; leg < 4 && !isEmergencyStopRequested(); ++leg) {
            if (!stand_buses[leg]) continue;
            for (int j = 0; j < 3; ++j) {
                int idx = leg * 3 + j;
                int mid = configs[idx].motor_id;
                float desired = stand_start[mid]
                              + smooth_alpha * (stand_target[mid] - stand_start[mid]);
                float delta = desired - last_cmd[mid];
                delta = std::max(-MAX_STEP_DELTA, std::min(MAX_STEP_DELTA, delta));
                last_cmd[mid] += delta;
                stand_buses[leg]->setPosition(mid, last_cmd[mid], kp, KD);
            }
            stand_buses[leg]->sendRecv();

            // 通信连续失败或电机报错时停止站立，统一走安全阻尼退出。
            for (int j = 0; j < 3; ++j) {
                int idx = leg * 3 + j;
                int mid = configs[idx].motor_id;
                MotorState state = stand_buses[leg]->getState(mid);
                comm_fail_count[mid] = state.correct ? 0 : comm_fail_count[mid] + 1;
                if (state.merror != 0 || comm_fail_count[mid] >= 10) {
                    std::cerr << "[Transition] Motor " << mid
                              << " 反馈异常, merror=" << state.merror
                              << ", comm_fail=" << comm_fail_count[mid] << std::endl;
                    requestEmergencyStop();
                    break;
                }
            }
        }
        usleep(static_cast<int>(STEP_DT * 1e6));
    }

    if (g_running && !isEmergencyStopRequested())
    {
        std::cout << "[Transition] ✓ 到达站立姿态" << std::endl;
        std::cout << "\n[Calib] 标定完成, 机器人处于站立姿态" << std::endl;
        std::cout << "  按 Ctrl+C 退出" << std::endl;

        // 以 100 Hz 保持最终站立指令，避免只发送一次后依赖电机内部保持状态。
        while (g_running && !isEmergencyStopRequested())
        {
            for (int leg = 0; leg < 4 && !isEmergencyStopRequested(); ++leg) {
                if (!stand_buses[leg]) continue;
                for (int j = 0; j < 3; ++j) {
                    int idx = leg * 3 + j;
                    int mid = configs[idx].motor_id;
                    stand_buses[leg]->setPosition(mid, stand_target[mid], KP_END, KD);
                }
                stand_buses[leg]->sendRecv();
            }
            usleep(10000);
        }
    }

    std::cout << "[Calib] 已退出" << std::endl;
}
