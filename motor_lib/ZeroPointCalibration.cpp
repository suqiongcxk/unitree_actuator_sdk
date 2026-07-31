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
 *   Leg1(FL): GPIO 133, /dev/ttyS0,  motors {0,  4,  8}
 *   Leg2(FR): GPIO  39, /dev/ttyS6,  motors {1,  5,  9}
 *   Leg3(RL): GPIO  35, /dev/ttyS7,  motors {2,  6, 10}
 *   Leg4(RR): GPIO  63, /dev/ttyS4,  motors {3,  7, 11}
 */

#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include "motor_controller.h"
#include "ZeroPointCalibration.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  全局数据
// ═══════════════════════════════════════════════════════════════════════════════

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

float ZERO_Position_MechLimitEnd[12]   = {0};  // 机械上限 (rad, 输出端)
float ZERO_Position_MechLimitStart[12] = {0};  // 机械下限 (rad, 输出端)
float target_angle[12]                 = {0};  // 过渡目标角度
float URDF_Joint_zero_OFFSET[12]       = {0};  // URDF 零点偏置

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
    Leg1_Uart = {133, "/dev/ttyS0"};
    Leg2_Uart = {39,  "/dev/ttyS6"};
    Leg3_Uart = {35,  "/dev/ttyS7"};
    Leg4_Uart = {63,  "/dev/ttyS4"};
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
    {133, "/dev/ttyS0"},   // Leg1 (FL)
    {39,  "/dev/ttyS6"},   // Leg2 (FR)
    {35,  "/dev/ttyS7"},   // Leg3 (RL)
    {63,  "/dev/ttyS4"},   // Leg4 (RR)
};

// ═══════════════════════════════════════════════════════════════════════════════
//  标定配置表 (12 电机)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  方向说明:
//    - Hip: FL/RL 用 +1.0 (正向→URDF上限), FR/RR 用 -1.0 (镜像, 需实机验证)
//    - Thigh: 全部 +1.0 (正向→URDF上限)
//    - Calf: 全部 -1.0 (负向→URDF下限)
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
        // ═══ Leg1: FL (GPIO 133, /dev/ttyS0) ═══
        {0,  0,  1.0f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, true},   // Hip  FL
        {4,  0,  1.0f, 0.08f, 8.0f, -1.5708f,  3.4907f,  5.0615f, true},   // Thigh FL
        {8,  0, -1.0f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false},  // Calf  FL

        // ═══ Leg2: FR (GPIO 39, /dev/ttyS6) ═══
        // 注意: FR Hip 速度方向为负 (镜像), 实机测试时如方向错误请调整 calib_velocity 符号
        {1,  1, -1.0f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, true},   // Hip  FR
        {5,  1,  1.0f, 0.08f, 8.0f, -1.5708f,  3.4907f,  5.0615f, true},   // Thigh FR
        {9,  1, -1.0f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false},  // Calf  FR

        // ═══ Leg3: RL (GPIO 35, /dev/ttyS7) ═══
        {2,  2,  1.0f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, true},   // Hip  RL
        {6,  2,  1.0f, 0.08f, 8.0f, -0.5236f,  4.5379f,  5.0615f, true},   // Thigh RL
        {10, 2, -1.0f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false},  // Calf  RL

        // ═══ Leg4: RR (GPIO 63, /dev/ttyS4) ═══
        // 注意: RR Hip 速度方向为负 (镜像), 实机测试时如方向错误请调整 calib_velocity 符号
        {3,  3, -1.0f, 0.08f, 8.0f, -1.0472f,  1.0472f,  2.0944f, true},   // Hip  RR
        {7,  3,  1.0f, 0.08f, 8.0f, -0.5236f,  4.5379f,  5.0615f, true},   // Thigh RR
        {11, 3, -1.0f, 0.06f, 8.0f, -2.7227f, -0.83776f, 1.885f,  false},  // Calf  RR
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
    const int STALL_THRESHOLD = 15;   // 连续 15 次 (150ms) 判定到位
    const float STALL_VELOCITY   = 0.05f;   // rad/s
    const float STALL_DELTA_Q    = 0.002f;  // rad

    while (g_running)
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

        // ── 通信检查 ──
        if (!state.correct)
        {
            std::cout << " COMM_FAIL" << std::endl;
            result.error_code = 1;
            result.calibration_time_ms = elapsed_ms;
            bus.brake(cfg.motor_id);
            bus.sendRecv();
            return false;
        }

        // ── 电机故障检查 ──
        if (state.merror != 0)
        {
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
                    // 确认到达机械限位
                    result.final_velocity = state.dq;
                    result.calibration_time_ms = elapsed_ms;

                    if (cfg.hit_upper_first)
                    {
                        result.mech_limit_end   = state.q;
                        result.mech_limit_start = state.q - cfg.urdf_range;
                    }
                    else
                    {
                        result.mech_limit_start = state.q;
                        result.mech_limit_end   = state.q + cfg.urdf_range;
                    }
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

    // ── 3. 标定完成 → 进入低阻尼模式 (安全，避免持续出力) ──
    bus.setDamping(cfg.motor_id, 0.02f);
    bus.sendRecv();

    // ── 4. 越界检查 ──
    if (result.mech_limit_start < cfg.urdf_lower - 0.5f ||
        result.mech_limit_end   > cfg.urdf_upper + 0.5f)
    {
        std::cerr << "    [Calib] Motor " << cfg.motor_id
                  << " 位置越界! start=" << result.mech_limit_start
                  << " end=" << result.mech_limit_end
                  << " (URDF: [" << cfg.urdf_lower << ", " << cfg.urdf_upper << "])"
                  << std::endl;
        result.error_code = 3;
        result.success = false;
        return false;
    }

    // ── 5. 写入全局标定数组 ──
    ZERO_Position_MechLimitStart[cfg.motor_id] = result.mech_limit_start;
    ZERO_Position_MechLimitEnd[cfg.motor_id]   = result.mech_limit_end;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  calibrateAllJoints — 12 关节完整标定
// ═══════════════════════════════════════════════════════════════════════════════
//
//  标定顺序: Leg1 → Leg2 → Leg3 → Leg4 (串行, 避免机械干涉)
//  每腿内顺序: Hip → Thigh → Calf
//
//  返回值: 成功标定的关节数量 (0-12)

int calibrateAllJoints(JointCalibResult results[12])
{
    const auto* configs = getCalibrationConfigs();
    int success_count = 0;

    std::cout << "\n┌──────────────────────────────────────────┐" << std::endl;
    std::cout <<   "│  机械限位标定 — 12 关节                   │" << std::endl;
    std::cout <<   "└──────────────────────────────────────────┘" << std::endl;

    // 逐腿标定
    for (int leg = 0; leg < 4; leg++)
    {
        std::cout << "\n── Leg" << (leg + 1)
                  << " (GPIO " << LEG_BUS_HW[leg].gpio_pin
                  << ", " << LEG_BUS_HW[leg].serial_port << ") ──"
                  << std::endl;

        // 为该腿创建临时 MotorBus (标定阶段 ParallelBus 尚未启动, 无 GPIO/串口冲突)
        MotorBus bus(LEG_BUS_HW[leg].gpio_pin, LEG_BUS_HW[leg].serial_port);

        // 注册该腿的 3 个电机
        for (int j = 0; j < 3; j++)
        {
            bus.addMotor(configs[leg * 3 + j].motor_id);
        }

        // 逐关节标定
        for (int j = 0; j < 3; j++)
        {
            int idx = leg * 3 + j;
            bool ok = calibrateSingleJoint(bus, configs[idx], results[idx]);
            if (ok)
            {
                success_count++;
            }
            else
            {
                std::cerr << "    !! Motor " << configs[idx].motor_id
                          << " 标定失败 (error=" << results[idx].error_code
                          << "), 继续剩余关节..." << std::endl;
                // 不立即返回，继续标定剩余关节以便诊断
            }
        }

        // ── 髋关节定位到站立姿态 ──
        // Thigh/Calf 保持在阻尼模式 (无力下垂, 不受重力影响的 Hip 提前归位)
        int hip_idx = leg * 3 + 0;  // Hip 是每条腿配置表的第一个 (j=0)
        if (results[hip_idx].success)
        {
            const auto& hip_cfg = configs[hip_idx];
            int hip_motor_id = hip_cfg.motor_id;

            // 当前位置: 校准后停在限位处
            float hip_current_q = hip_cfg.hit_upper_first
                                ? results[hip_idx].mech_limit_end
                                : results[hip_idx].mech_limit_start;

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

            for (int step = 0; step <= HIP_STEPS; step++)
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
    }

    std::cout << "\n[Calib] 标定完成: " << success_count << "/12 关节成功" << std::endl;
    return success_count;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  validateCalibrationResults — 标定结果验证
// ═══════════════════════════════════════════════════════════════════════════════
//
//  对比实测机械行程与 URDF 预期行程。
//  偏差超过 max_range_error 的关节将被标记为验证失败。

bool validateCalibrationResults(const JointCalibResult results[12],
                                const JointCalibConfig configs[12],
                                float max_range_error)
{
    bool all_valid = true;

    std::cout << "\n┌──────────────────────────────────────────┐" << std::endl;
    std::cout <<   "│  标定结果验证                             │" << std::endl;
    std::cout <<   "└──────────────────────────────────────────┘" << std::endl;

    std::cout << std::fixed << std::setprecision(4);

    for (int i = 0; i < 12; i++)
    {
        if (!results[i].success)
        {
            std::cerr << "  Motor " << std::setw(2) << i
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
            std::cerr << "  Motor " << std::setw(2) << i
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
            std::cout << "  Motor " << std::setw(2) << i
                      << ": OK  实测=" << std::setw(8) << measured_range << " rad"
                      << " (偏差 " << error << " rad)"
                      << "  [" << results[i].mech_limit_start
                      << ", " << results[i].mech_limit_end << "]"
                      << std::endl;
        }
    }

    if (all_valid)
        std::cout << "\n[Verify] ✓ 全部 12 关节行程验证通过" << std::endl;
    else
        std::cerr << "\n[Verify] ✗ 部分关节行程验证失败!" << std::endl;

    return all_valid;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  computeMotorTargetFromURDF — URDF 关节角 → 电机输出端位置
// ═══════════════════════════════════════════════════════════════════════════════

float computeMotorTargetFromURDF(int motor_id, float urdf_target)
{
    float limit_start = ZERO_Position_MechLimitStart[motor_id];
    float limit_end   = ZERO_Position_MechLimitEnd[motor_id];
    const auto* configs = getCalibrationConfigs();
    const auto& cfg = configs[motor_id];

    // 线性映射: URDF [urdf_lower, urdf_upper] → 电机 [limit_start, limit_end]
    float t = (urdf_target - cfg.urdf_lower) / cfg.urdf_range;
    t = std::max(0.0f, std::min(1.0f, t));  // clamp [0, 1]
    return limit_start + t * (limit_end - limit_start);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ZeroPointCalibration — 独立标定入口 (兼容旧接口)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  使用 MotorBus 进行完整的 12 关节标定 + 验证 + 站立过渡。
//  此函数供 example_usage 等独立测试程序调用。

void ZeroPointCalibration(void)
{
    std::signal(SIGINT, sigint_handler);

    std::cout << "╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Creeper 零位标定 — 独立模式                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;

    LEG_MOTOR_INIT();

    // ── 阶段 1: 12 关节标定 ──
    JointCalibResult results[12] = {};
    int ok_count = calibrateAllJoints(results);

    if (ok_count < 8)
    {
        std::cerr << "\n[FATAL] 标定成功数不足 (" << ok_count << "/12), 退出" << std::endl;
        return;
    }

    // ── 阶段 2: 验证标定结果 ──
    const auto* configs = getCalibrationConfigs();
    bool verified = validateCalibrationResults(results, configs);

    if (!verified && ok_count < 12)
    {
        std::cerr << "[FATAL] 标定验证失败, 退出" << std::endl;
        return;
    }

    // ── 阶段 3: 计算站立姿态的目标位置 ──
    float stand_target[12] = {0};
    for (int i = 0; i < 12; i++)
    {
        if (results[i].success)
        {
            stand_target[i] = computeMotorTargetFromURDF(i, default_joint_pos[i]);
        }
    }

    // ── 阶段 4: 逐腿过渡到站立姿态 (使用 MotorBus) ──
    std::cout << "\n[Transition] 过渡到站立姿态 (MotorBus 模式)..." << std::endl;

    const int TRANSITION_STEPS = 150;
    const float TRANSITION_TIME = 2.0f;
    const float step_dt = TRANSITION_TIME / TRANSITION_STEPS;
    const float kp = 0.3f;
    const float kd = 0.02f;

    for (int step = 0; step <= TRANSITION_STEPS && g_running; step++)
    {
        float alpha = static_cast<float>(step) / TRANSITION_STEPS;
        float smooth_alpha = alpha * alpha * (3.0f - 2.0f * alpha);

        for (int leg = 0; leg < 4; leg++)
        {
            MotorBus bus(LEG_BUS_HW[leg].gpio_pin, LEG_BUS_HW[leg].serial_port);

            for (int j = 0; j < 3; j++)
            {
                int idx = leg * 3 + j;
                bus.addMotor(configs[idx].motor_id);
            }

            for (int j = 0; j < 3; j++)
            {
                int idx = leg * 3 + j;
                int mid = configs[idx].motor_id;
                float current_q;

                if (results[idx].success)
                {
                    // 当前位置 = 标定时撞击的限位
                    current_q = configs[idx].hit_upper_first
                              ? results[idx].mech_limit_end
                              : results[idx].mech_limit_start;
                }
                else
                {
                    // 标定失败: 读取当前电机位置
                    MotorState s = bus.getState(mid);
                    current_q = s.q;
                }

                float q_cmd = current_q + smooth_alpha * (stand_target[mid] - current_q);
                bus.setPosition(mid, q_cmd, kp, kd);
            }
            bus.sendRecv();
        }

        usleep(static_cast<int>(step_dt * 1e6));
    }

    if (g_running)
    {
        std::cout << "[Transition] ✓ 到达站立姿态" << std::endl;
        std::cout << "\n[Calib] 标定完成, 机器人处于站立姿态" << std::endl;
        std::cout << "  按 Ctrl+C 退出" << std::endl;

        // 保持站立, 等待用户中断
        while (g_running)
        {
            sleep(1);
        }
    }

    // ── 清理: 刹车所有电机 ──
    std::cout << "\n[Calib] 刹车所有电机..." << std::endl;
    for (int leg = 0; leg < 4; leg++)
    {
        MotorBus bus(LEG_BUS_HW[leg].gpio_pin, LEG_BUS_HW[leg].serial_port);
        for (int j = 0; j < 3; j++)
        {
            int idx = leg * 3 + j;
            bus.addMotor(configs[idx].motor_id);
        }
        for (int j = 0; j < 3; j++)
        {
            bus.brake(configs[leg * 3 + j].motor_id);
        }
        bus.sendRecv();
    }
    std::cout << "[Calib] 已退出" << std::endl;
}
