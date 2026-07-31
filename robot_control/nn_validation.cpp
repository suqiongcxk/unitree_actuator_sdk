#include "nn_validation.h"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <iostream>

// ═══════════════════════════════════════════════════════════════════════════════
//  ValidationResult::summary()
// ═══════════════════════════════════════════════════════════════════════════════

std::string ValidationResult::summary() const
{
    std::ostringstream oss;
    oss << "Validation: " << (passed ? "PASS" : "FAIL");

    if (nan_count > 0)    oss << " | NaN:" << nan_count;
    if (inf_count > 0)    oss << " | Inf:" << inf_count;
    if (out_of_bounds_count > 0) {
        oss << " | OOB:" << out_of_bounds_count
            << " (joint[" << first_bad_joint << "]="
            << std::fixed << std::setprecision(3) << first_bad_value
            << " limit=[" << first_bad_limit_lo << "," << first_bad_limit_hi << "])";
    }
    if (jump_exceeded)    oss << " | JUMP:" << std::fixed << std::setprecision(3) << max_jump_rad << "rad";

    if (passed) oss << " | all good";
    return oss.str();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  validateJointLimits()
// ═══════════════════════════════════════════════════════════════════════════════

ValidationResult validateJointLimits(const float targets[12], float margin)
{
    ValidationResult r;

    for (int i = 0; i < 12; ++i) {
        float lo = JOINT_LIMITS[i][0] - margin;
        float hi = JOINT_LIMITS[i][1] + margin;

        if (targets[i] < lo || targets[i] > hi) {
            r.out_of_bounds_count++;
            if (r.first_bad_joint < 0) {
                r.first_bad_joint    = i;
                r.first_bad_value    = targets[i];
                r.first_bad_limit_lo = JOINT_LIMITS[i][0];
                r.first_bad_limit_hi = JOINT_LIMITS[i][1];
            }
        }
    }

    if (r.out_of_bounds_count > 0) r.passed = false;
    return r;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  validateFinite()
// ═══════════════════════════════════════════════════════════════════════════════

ValidationResult validateFinite(const float targets[12])
{
    ValidationResult r;

    for (int i = 0; i < 12; ++i) {
        if (std::isnan(targets[i])) {
            r.nan_count++;
            if (r.first_bad_joint < 0) {
                r.first_bad_joint = i;
                r.first_bad_value = targets[i];
            }
        }
        if (std::isinf(targets[i])) {
            r.inf_count++;
            if (r.first_bad_joint < 0) {
                r.first_bad_joint = i;
                r.first_bad_value = targets[i];
            }
        }
    }

    if (r.nan_count > 0 || r.inf_count > 0) r.passed = false;
    return r;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  validateSmoothness()
// ═══════════════════════════════════════════════════════════════════════════════

ValidationResult validateSmoothness(const float current[12], const float previous[12],
                                     float max_delta)
{
    ValidationResult r;

    for (int i = 0; i < 12; ++i) {
        float jump = std::abs(current[i] - previous[i]);
        if (jump > r.max_jump_rad) r.max_jump_rad = jump;
        if (jump > max_delta) {
            r.jump_exceeded = true;
            if (r.first_bad_joint < 0) {
                r.first_bad_joint = i;
                r.first_bad_value = jump;
            }
        }
    }

    if (r.jump_exceeded) r.passed = false;
    return r;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  validateAll()
// ═══════════════════════════════════════════════════════════════════════════════

ValidationResult validateAll(const float targets[12],
                              const float previous[12],
                              float joint_margin,
                              float max_jump)
{
    ValidationResult r_finite  = validateFinite(targets);
    ValidationResult r_bounds  = validateJointLimits(targets, joint_margin);
    ValidationResult r_smooth  = validateSmoothness(targets, previous, max_jump);

    ValidationResult combined;
    combined.passed             = r_finite.passed && r_bounds.passed && r_smooth.passed;
    combined.nan_count          = r_finite.nan_count;
    combined.inf_count          = r_finite.inf_count;
    combined.out_of_bounds_count = r_bounds.out_of_bounds_count;
    combined.max_jump_rad       = r_smooth.max_jump_rad;
    combined.jump_exceeded      = r_smooth.jump_exceeded;

    // 保留第一个错误详情
    if (!r_finite.passed) {
        combined.first_bad_joint    = r_finite.first_bad_joint;
        combined.first_bad_value    = r_finite.first_bad_value;
    } else if (!r_bounds.passed) {
        combined.first_bad_joint    = r_bounds.first_bad_joint;
        combined.first_bad_value    = r_bounds.first_bad_value;
        combined.first_bad_limit_lo = r_bounds.first_bad_limit_lo;
        combined.first_bad_limit_hi = r_bounds.first_bad_limit_hi;
    } else if (!r_smooth.passed) {
        combined.first_bad_joint    = r_smooth.first_bad_joint;
        combined.first_bad_value    = r_smooth.first_bad_value;
    }

    return combined;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  NNInferenceLogger
// ═══════════════════════════════════════════════════════════════════════════════

NNInferenceLogger::NNInferenceLogger(const std::string& filepath, size_t max_lines)
    : filepath_(filepath), max_lines_(max_lines)
{
    file_.open(filepath_, std::ios::out | std::ios::trunc);
    if (file_.is_open()) {
        writeHeader();
        std::cout << "[NN Logger] 记录到 " << filepath_ << std::endl;
    } else {
        std::cerr << "[NN Logger] 无法打开文件: " << filepath_ << std::endl;
    }
}

NNInferenceLogger::~NNInferenceLogger()
{
    if (file_.is_open()) {
        file_.close();
        std::cout << "[NN Logger] 已关闭, 共 " << line_count_ << " 行" << std::endl;
    }
}

void NNInferenceLogger::writeHeader()
{
    file_ << "timestamp_ns,frame,valid,latency_us";
    // 输入: 机体状态
    file_ << ",pos_x,pos_y,pos_z,ori_w,ori_x,ori_y,ori_z";
    file_ << ",lv_x,lv_y,lv_z,av_x,av_y,av_z";
    // 输入: 12 关节
    for (int i = 0; i < 12; ++i) {
        file_ << ",j" << i << "_pos,j" << i << "_vel,j" << i << "_tau";
    }
    // 输出: 12 关节目标
    for (int i = 0; i < 12; ++i) {
        file_ << ",tgt" << i;
    }
    file_ << "\n";
}

void NNInferenceLogger::log(const EstimatedState& input_state,
                             const NNCommandSet& output_cmds,
                             bool valid, int latency_us)
{
    if (!enabled_ || !file_.is_open()) return;

    // 循环覆盖
    if (line_count_ >= max_lines_) {
        file_.close();
        file_.open(filepath_, std::ios::out | std::ios::trunc);
        writeHeader();
        line_count_ = 0;
    }

    file_ << input_state.timestamp_ns << ","
          << line_count_ << ","
          << (valid ? 1 : 0) << ","
          << latency_us;

    // 机体状态
    for (int i = 0; i < 3; ++i)  file_ << "," << input_state.position[i];
    for (int i = 0; i < 4; ++i)  file_ << "," << input_state.orientation[i];
    for (int i = 0; i < 3; ++i)  file_ << "," << input_state.linear_velocity[i];
    for (int i = 0; i < 3; ++i)  file_ << "," << input_state.angular_velocity[i];

    // 12 关节输入
    for (int i = 0; i < 12; ++i) {
        file_ << "," << input_state.joint_position[i]
              << "," << input_state.joint_velocity[i]
              << "," << input_state.joint_torque[i];
    }

    // 12 关节输出目标
    for (int i = 0; i < 12; ++i) {
        file_ << "," << output_cmds.joint_position_target[i];
    }

    file_ << "\n";
    line_count_++;
}
