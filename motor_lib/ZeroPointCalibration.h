#ifndef __ZERO_POINT_CALIBRATION_H
#define __ZERO_POINT_CALIBRATION_H

#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include <cstdint>

// ═══════════════════════════════════════════════════════════════════════════════
//  URDF 关节软件限位 (rad, 输出端)
// ═══════════════════════════════════════════════════════════════════════════════

// ── Hip 关节 (0,1,2,3): [-60°, 60°] ──
#define Joint0_Begin   -1.0472
#define Joint0_End      1.0472
#define Joint1_Begin   -1.0472
#define Joint1_End      1.0472
#define Joint2_Begin   -1.0472
#define Joint2_End      1.0472
#define Joint3_Begin   -1.0472
#define Joint3_End      1.0472

// ── Thigh 前腿 (4,5): [-90°, 200°] ──
#define Joint4_Begin   -1.5708
#define Joint4_End      3.4907
#define Joint5_Begin   -1.5708
#define Joint5_End      3.4907

// ── Thigh 后腿 (6,7): [-30°, 260°] ──
#define Joint6_Begin   -0.5236
#define Joint6_End      4.5379
#define Joint7_Begin   -0.5236
#define Joint7_End      4.5379

// ── Calf (8,9,10,11): [-156°, -48°] ──
#define Joint8_Begin   -2.7227
#define Joint8_End     -0.83776
#define Joint9_Begin   -2.7227
#define Joint9_End     -0.83776
#define Joint10_Begin  -2.7227
#define Joint10_End    -0.83776
#define Joint11_Begin  -2.7227
#define Joint11_End    -0.83776

// ═══════════════════════════════════════════════════════════════════════════════
//  硬件配置结构体
// ═══════════════════════════════════════════════════════════════════════════════

struct LEG_UART_SET
{
    int GPIO_PIN;
    const char* SERIAL_PORT;
};

struct SINGLE_LEG_MOtor_SET
{
    int HIP_Motor;
    int thigh_Motor;
    int lower_leg_Motor;
    LEG_UART_SET Leg_UART;
};

extern LEG_UART_SET Leg1_Uart;
extern LEG_UART_SET Leg2_Uart;
extern LEG_UART_SET Leg3_Uart;
extern LEG_UART_SET Leg4_Uart;
extern SINGLE_LEG_MOtor_SET Leg1_Motor;
extern SINGLE_LEG_MOtor_SET Leg2_Motor;
extern SINGLE_LEG_MOtor_SET Leg3_Motor;
extern SINGLE_LEG_MOtor_SET Leg4_Motor;

// ═══════════════════════════════════════════════════════════════════════════════
//  标定配置 — 每个关节的标定参数
// ═══════════════════════════════════════════════════════════════════════════════

struct JointCalibConfig
{
    unsigned short motor_id;        // 电机 ID (0-11)
    int    leg_index;               // 腿部索引 (0=FL, 1=FR, 2=RL, 3=RR)
    float  calib_velocity;          // 标定运动速度 (rad/s, 输出端, 正/负决定方向)
    float  calib_kd;                // 标定阻尼系数 (低刚度, 0.0-1.0)
    float  timeout_sec;             // 最大标定时间 (秒)
    float  urdf_lower;              // URDF 下限 (rad)
    float  urdf_upper;              // URDF 上限 (rad)
    float  urdf_range;              // URDF 行程 = upper - lower (rad)
    bool   hit_upper_first;         // true=先撞上限(MechLimitEnd), false=先撞下限(MechLimitStart)
};

// ═══════════════════════════════════════════════════════════════════════════════
//  标定结果 — 单关节标定返回值
// ═══════════════════════════════════════════════════════════════════════════════

struct JointCalibResult
{
    bool   success;                 // 标定是否成功
    int    error_code;              // 0=成功, 1=通信失败, 2=超时, 3=越界
    float  mech_limit_start;        // 实测机械下限 (rad, 输出端)
    float  mech_limit_end;          // 实测机械上限 (rad, 输出端)
    float  measured_range;          // 实测行程 (rad)
    float  range_error;             // 与 URDF 行程的偏差 (rad)
    float  final_velocity;          // 检测到位时的速度 (rad/s)
    int    calibration_time_ms;     // 标定耗时 (毫秒)
};

// ═══════════════════════════════════════════════════════════════════════════════
//  启动阶段状态机
// ═══════════════════════════════════════════════════════════════════════════════

enum class StartupPhase
{
    INIT_COMM,          // 初始化通信
    CALIBRATING,        // 12 关节机械限位校准
    VERIFY_RESULTS,     // 校准结果检查
    TRANSITION_STAND,   // 缓慢过渡到站立姿态
    HOLD_STANDING,      // 稳定保持
    NN_ACTIVE,          // 开启神经网络控制
    FAULT               // 故障状态
};

// ═══════════════════════════════════════════════════════════════════════════════
//  全局标定数据 (12 电机)
// ═══════════════════════════════════════════════════════════════════════════════

extern float ZERO_Position_MechLimitEnd[12];    // 机械上限 (rad, 输出端)
extern float ZERO_Position_MechLimitStart[12];  // 机械下限 (rad, 输出端)
extern float target_angle[12];                  // 过渡目标角度
extern float URDF_Joint_zero_OFFSET[12];        // URDF 零点偏置

/// 默认站立姿态 (Z 字排序: Hip[0-3], Thigh[4-7], Calf[8-11])
extern const float default_joint_pos[12];

// ═══════════════════════════════════════════════════════════════════════════════
//  MotorBus 前向声明
// ═══════════════════════════════════════════════════════════════════════════════

class MotorBus;

// ═══════════════════════════════════════════════════════════════════════════════
//  核心标定函数
// ═══════════════════════════════════════════════════════════════════════════════

/// 获取 12 电机标定配置表
const JointCalibConfig* getCalibrationConfigs();

/// 单关节标定：驱动电机撞向机械限位，使用状态监测判定到位
/// @param bus         该腿的 MotorBus 引用
/// @param cfg         标定参数
/// @param result      [out] 标定结果
/// @return true=标定成功
bool calibrateSingleJoint(MotorBus& bus,
                          const JointCalibConfig& cfg,
                          JointCalibResult& result);

/// 12 关节完整标定（逐腿创建临时 MotorBus 实例）
/// @param results  [out] 12 个关节的标定结果 (索引 = motor_id)
/// @return 成功标定的关节数量 (0-12)
int calibrateAllJoints(JointCalibResult results[12]);

/// 验证标定结果：对比实测行程与 URDF 预期行程
/// @param results          12 个标定结果
/// @param configs          12 个标定配置
/// @param max_range_error  允许的最大行程偏差 (rad, 默认 0.15)
/// @return true=全部通过验证
bool validateCalibrationResults(const JointCalibResult results[12],
                                const JointCalibConfig configs[12],
                                float max_range_error = 0.15f);

/// URDF 关节角 → 电机输出端位置的映射
/// @param motor_id    电机 ID (0-11)
/// @param urdf_target URDF 坐标系下的目标角度 (rad)
/// @return 对应的电机输出端位置 (rad)
float computeMotorTargetFromURDF(int motor_id, float urdf_target);

// ═══════════════════════════════════════════════════════════════════════════════
//  腿部初始化函数 (保持兼容)
// ═══════════════════════════════════════════════════════════════════════════════

void ZeroPointCalibration(void);
void LEG_MOTOR_INIT(void);

#endif  // __ZERO_POINT_CALIBRATION_H
