#ifndef __ROBOT_CONTROL_NN_VALIDATION_H
#define __ROBOT_CONTROL_NN_VALIDATION_H

#include "shared_data.h"
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <algorithm>

// ═══════════════════════════════════════════════════════════════════════════════
//  NN 验证 — 确保神经网络输出在物理合理范围内
// ═══════════════════════════════════════════════════════════════════════════════

// ── 关节角度限制 (rad, 输出端) ──
// 对应 ZeroPointCalibration.h 中的定义
constexpr float JOINT_LIMITS[12][2] = {
    // Leg1: hip=0, thigh=4, lower_leg=8
    {-1.0472f,  1.0472f},   // Joint 0  (hip)
    {-1.5708f,  3.4907f},   // Joint 4  (thigh)
    {-2.7227f, -0.83776f},  // Joint 8  (lower_leg)
    // Leg2: hip=1, thigh=5, lower_leg=9
    {-1.0472f,  1.0472f},   // Joint 1  (hip)
    {-1.5708f,  3.4907f},   // Joint 5  (thigh)
    {-2.7227f, -0.83776f},  // Joint 9  (lower_leg)
    // Leg3: hip=2, thigh=6, lower_leg=10
    {-1.0472f,  1.0472f},   // Joint 2  (hip)
    {-0.5236f,  4.5379f},   // Joint 6  (thigh)
    {-2.7227f, -0.83776f},  // Joint 10 (lower_leg)
    // Leg4: hip=3, thigh=7, lower_leg=11
    {-1.0472f,  1.0472f},   // Joint 3  (hip)
    {-0.5236f,  4.5379f},   // Joint 7  (thigh)
    {-2.7227f, -0.83776f},  // Joint 11 (lower_leg)
};

// ── 验证结果 ─────────────────────────────────────────────────────────────────

struct ValidationResult {
    bool passed = true;

    // 越界统计
    int   out_of_bounds_count = 0;
    int   nan_count           = 0;
    int   inf_count           = 0;

    // 平滑度
    float max_jump_rad        = 0.0f;   // 相邻帧最大跳跃 (rad)
    bool  jump_exceeded       = false;

    // 详情 (只记录首次违规)
    int   first_bad_joint     = -1;
    float first_bad_value     = 0.0f;
    float first_bad_limit_lo  = 0.0f;
    float first_bad_limit_hi  = 0.0f;

    std::string summary() const;
};

// ── 验证函数 ─────────────────────────────────────────────────────────────────

/// 检查所有关节目标值是否在机械限位内
/// @param targets  12 关节目标位置 (rad, 输出端)
/// @param margin   允许超出限位的裕度 (rad)
/// @return         验证结果
ValidationResult validateJointLimits(const float targets[12], float margin = 0.05f);

/// 检查是否有 NaN 或 Inf
ValidationResult validateFinite(const float targets[12]);

/// 检查相邻帧输出是否平滑 (无突变跳变)
/// @param current  当前帧目标
/// @param previous 上一帧目标
/// @param max_delta 单帧最大允许变化 (rad)
ValidationResult validateSmoothness(const float current[12], const float previous[12],
                                     float max_delta = 0.5f);

/// 组合验证: 限位 + 有限值 + 平滑度
ValidationResult validateAll(const float targets[12],
                              const float previous[12],
                              float joint_margin     = 0.05f,
                              float max_jump         = 0.5f);

// ═══════════════════════════════════════════════════════════════════════════════
//  NNInferenceLogger — 记录每次推理的输入/输出到 CSV 文件
// ═══════════════════════════════════════════════════════════════════════════════

class NNInferenceLogger {
public:
    /// @param filepath  输出 CSV 文件路径
    /// @param max_lines 最大记录行数 (超过后循环覆盖)
    explicit NNInferenceLogger(const std::string& filepath, size_t max_lines = 10000);
    ~NNInferenceLogger();

    /// 记录一次推理
    /// @param input_state  输入: 估计状态
    /// @param output_cmds  输出: 电机指令
    /// @param valid        推理是否成功
    /// @param latency_us   推理耗时 (微秒)
    void log(const EstimatedState& input_state,
             const NNCommandSet& output_cmds,
             bool valid, int latency_us);

    /// 是否正在记录
    bool isEnabled() const { return enabled_; }
    void setEnabled(bool enabled) { enabled_ = enabled; }

    /// 获取已记录行数
    size_t lineCount() const { return line_count_; }

private:
    void writeHeader();

    std::ofstream file_;
    std::string   filepath_;
    size_t        max_lines_;
    size_t        line_count_ = 0;
    bool          enabled_ = true;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  命令行标志 — 控制 NN 验证行为
// ═══════════════════════════════════════════════════════════════════════════════

struct NNControlFlags {
    bool dry_run    = false;   // 干运行: NN 计算但不发送指令到电机
    bool log_io     = false;   // 记录每帧输入/输出到 CSV
    bool validate   = true;    // 运行时验证 (限位 + NaN)
    bool compare    = false;   // 同时运行 standing policy, 对比输出差异

    std::string log_filepath = "/tmp/nn_inference_log.csv";
};

#endif  // __ROBOT_CONTROL_NN_VALIDATION_H
