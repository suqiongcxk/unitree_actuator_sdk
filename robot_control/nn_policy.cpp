#include "nn_policy.h"
#include <cstring>
#include <cmath>
#include <iomanip>
#include <chrono>

// ═══════════════════════════════════════════════════════════════════════════════
//  StandingPolicy
// ═══════════════════════════════════════════════════════════════════════════════

StandingPolicy::StandingPolicy(const float* default_pose_12, float kp, float kd)
{
    std::memcpy(standing_cmd_.joint_position_target, default_pose_12, 12 * sizeof(float));
    for (int i = 0; i < 12; ++i) {
        standing_cmd_.kp[i] = kp;
        standing_cmd_.kd[i] = kd;
    }
    standing_cmd_.valid = true;
}

bool StandingPolicy::infer(const EstimatedState& est, NNCommandSet& cmds)
{
    (void)est;
    cmds = standing_cmd_;
    return cmds.valid;
}

void StandingPolicy::commitAcceptedCommand(const NNCommandSet&)
{}

// ═══════════════════════════════════════════════════════════════════════════════
//  ValidatingPolicy — 包装实际策略，输出前验证
// ═══════════════════════════════════════════════════════════════════════════════

ValidatingPolicy::ValidatingPolicy(std::unique_ptr<NNPolicy> inner,
                                     std::unique_ptr<NNPolicy> fallback,
                                     FallbackMode mode)
    : inner_(std::move(inner))
    , fallback_(std::move(fallback))
    , mode_(mode)
{}

bool ValidatingPolicy::infer(const EstimatedState& est, NNCommandSet& cmds)
{
    total_count_++;

    // ── 1. 运行内部策略 ──
    NNCommandSet raw_cmds;
    bool raw_ok = inner_->infer(est, raw_cmds);

    if (!raw_ok || !raw_cmds.valid) {
        std::cerr << "[ValidatingPolicy] 内部策略返回无效" << std::endl;
        fail_count_++;
        consecutive_fail_count_++;
        // 回退
        if (fallback_ && mode_ == FallbackMode::STANDING) {
            return fallback_->infer(est, cmds);
        }
        if (mode_ == FallbackMode::PREV_FRAME && consecutive_fail_count_ >= 3 && fallback_) {
            return fallback_->infer(est, cmds);
        }
        if (has_prev_ && mode_ == FallbackMode::PREV_FRAME) {
            cmds = prev_cmds_;
            return true;
        }
        return false;
    }

    // ── 2. 运行验证 ──
    // 首帧必须与当前真实关节位置比较，不能用本帧自身绕过跳变检查。
    const float* prev = has_prev_ ? prev_cmds_.joint_position_target : est.joint_position;
    last_result_ = validateCommandSet(raw_cmds, prev);

    if (!last_result_.passed) {
        fail_count_++;
        consecutive_fail_count_++;
        std::cerr << "[ValidatingPolicy] " << last_result_.summary() << std::endl;

        switch (mode_) {
        case FallbackMode::STANDING:
            if (fallback_) {
                std::cout << "  → 回退到 " << fallback_->name() << std::endl;
                return fallback_->infer(est, cmds);
            }
            return false;

        case FallbackMode::PREV_FRAME:
            if (consecutive_fail_count_ >= 3 && fallback_) {
                std::cout << "  → 连续异常，回退到 " << fallback_->name() << std::endl;
                return fallback_->infer(est, cmds);
            }
            if (has_prev_) {
                std::cout << "  → 维持上帧指令" << std::endl;
                cmds = prev_cmds_;
                return true;
            }
            return false;

        case FallbackMode::NONE:
            // 仍然使用 raw_cmds，但已打印警告
            break;
        }
    }

    // 缓存只能在控制器确认最终命令后更新，不能在这里提前写入。
    consecutive_fail_count_ = 0;
    cmds = raw_cmds;
    return true;
}

void ValidatingPolicy::commitAcceptedCommand(const NNCommandSet& cmds)
{
    prev_cmds_ = cmds;
    has_prev_ = true;
    inner_->commitAcceptedCommand(cmds);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ComparingPolicy — 对比两个策略
// ═══════════════════════════════════════════════════════════════════════════════

ComparingPolicy::ComparingPolicy(std::unique_ptr<NNPolicy> primary,
                                   std::unique_ptr<NNPolicy> baseline)
    : primary_(std::move(primary))
    , baseline_(std::move(baseline))
{}

bool ComparingPolicy::infer(const EstimatedState& est, NNCommandSet& cmds)
{
    // ── 主策略推理 (实际控制) ──
    bool ok = primary_->infer(est, cmds);

    // ── 基线策略推理 (仅对比) ──
    NNCommandSet baseline_cmds;
    if (baseline_->infer(est, baseline_cmds) && baseline_cmds.valid) {
        // 计算最大关节差异
        float max_diff = 0.0f;
        for (int i = 0; i < 12; ++i) {
            float diff = std::abs(cmds.joint_position_target[i]
                                  - baseline_cmds.joint_position_target[i]);
            if (diff > max_diff) max_diff = diff;
        }
        last_max_diff_ = max_diff;

        // 每 50 帧 (1秒@50Hz) 打印一次差异
        static int compare_cycle = 0;
        if (++compare_cycle % 50 == 0) {
            std::cout << "[Compare] " << primary_->name() << " vs "
                      << baseline_->name()
                      << " | max_diff=" << std::fixed << std::setprecision(4)
                      << last_max_diff_ << " rad" << std::endl;
        }
    }

    return ok;
}

void ComparingPolicy::commitAcceptedCommand(const NNCommandSet& cmds)
{
    primary_->commitAcceptedCommand(cmds);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ONNXPolicy
// ═══════════════════════════════════════════════════════════════════════════════

ONNXPolicy::ONNXPolicy(const std::string& model_path,
                       const float* default_pose_12,
                       float action_scale,
                       float kp, float kd)
    : model_path_(model_path)
    , action_scale_(action_scale)
    , kp_(kp)
    , kd_(kd)
    , observation_builder_(default_pose_12, action_scale)
{
    std::memcpy(default_pose_, default_pose_12, 12 * sizeof(float));
}

bool ONNXPolicy::initialize()
{
    try {
        Ort::SessionOptions session_opts;
        session_opts.SetIntraOpNumThreads(1);       // 单线程推理 (NN线程独占)
        session_opts.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_ALL);

        session_ = std::make_unique<Ort::Session>(env_,
            model_path_.c_str(), session_opts);

        // ── 提取输入信息 ──
        size_t num_inputs = session_->GetInputCount();
        input_names_.resize(num_inputs);
        for (size_t i = 0; i < num_inputs; ++i) {
            auto name_ptr = session_->GetInputNameAllocated(i, allocator_);
            input_names_[i] = name_ptr.release();  // 需要手动管理内存
            Ort::TypeInfo type_info = session_->GetInputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            input_shape_ = tensor_info.GetShape();
        }

        // ── 提取输出信息 ──
        size_t num_outputs = session_->GetOutputCount();
        output_names_.resize(num_outputs);
        for (size_t i = 0; i < num_outputs; ++i) {
            auto name_ptr = session_->GetOutputNameAllocated(i, allocator_);
            output_names_[i] = name_ptr.release();
            Ort::TypeInfo type_info = session_->GetOutputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            output_shape_ = tensor_info.GetShape();
        }

        printModelInfo();

        if (!std::isfinite(action_scale_) || std::abs(action_scale_) < 1e-8f
                || input_names_.size() != 1 || output_names_.size() != 1
                || input_shape_.size() != 2 || output_shape_.size() != 2
                || (input_shape_[0] != 1 && input_shape_[0] != -1)
                || (output_shape_[0] != 1 && output_shape_[0] != -1)
                || input_shape_[1] !=
                    static_cast<int64_t>(PolicyObservationBuilder::kObservationSize)
                || output_shape_[1] <
                    static_cast<int64_t>(PolicyObservationBuilder::kActionSize)) {
            std::cerr << "[ONNXPolicy] 不支持的配置：action_scale 必须非零，"
                      << "模型要求单输入 [1/-1,48]、单输出 [1/-1,>=12]"
                      << std::endl;
            session_.reset();
            return false;
        }
        return true;

    } catch (const Ort::Exception& e) {
        std::cerr << "[ONNXPolicy] 初始化失败: " << e.what() << std::endl;
        return false;
    }
}

void ONNXPolicy::printModelInfo() const
{
    std::cout << "═══════════════════════════════════════════" << std::endl;
    std::cout << "  ONNX Model: " << model_path_ << std::endl;
    std::cout << "  Inputs:  " << input_names_.size() << std::endl;
    for (size_t i = 0; i < input_names_.size(); ++i) {
        std::cout << "    [" << i << "] " << input_names_[i]
                  << "  shape=[";
        for (size_t j = 0; j < input_shape_.size(); ++j) {
            if (j > 0) std::cout << ",";
            std::cout << input_shape_[j];
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "  Outputs: " << output_names_.size() << std::endl;
    for (size_t i = 0; i < output_names_.size(); ++i) {
        std::cout << "    [" << i << "] " << output_names_[i]
                  << "  shape=[";
        for (size_t j = 0; j < output_shape_.size(); ++j) {
            if (j > 0) std::cout << ",";
            std::cout << output_shape_[j];
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "═══════════════════════════════════════════" << std::endl;
}

int ONNXPolicy::inputCount() const { return static_cast<int>(input_names_.size()); }
int ONNXPolicy::outputCount() const { return static_cast<int>(output_names_.size()); }

bool ONNXPolicy::infer(const EstimatedState& est, NNCommandSet& cmds)
{
    if (!session_) return false;

    try {
        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);

        // ── 构建观测向量 (48维, 具体格式取决于训练时的obs定义) ──
        //
        // 常见的 48 维观测拆分 (请根据实际训练配置调整!):
        //   [0..2]   机身角速度 gyro (3)
        //   [3..5]   机身欧拉角 rpy (3)  或 重力方向 (3)
        //   [6..8]   命令速度 vx,vy,vyaw (3)
        //   [9..20]  关节位置 (12)
        //   [21..32] 关节速度 (12)
        //   [33..44] 上帧关节动作 (12)
        //   [45..47] 或其它 (3)
        //
        // 这里按通用格式填充 — 你需要根据训练代码确认实际顺序!

        const auto& obs = observation_builder_.build(est);

        // ── 推理 ──
        std::vector<int64_t> input_shape_final = {1, 48};
        auto input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, const_cast<float*>(obs.data()), obs.size(),
            input_shape_final.data(), input_shape_final.size());

        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr},
            input_names_.data(), &input_tensor, 1,
            output_names_.data(), output_names_.size());

        // ── 提取模型输出, 施加 delta 公式 ──
        //   target[i] = default_pose[i] + action_scale * model_output[i]
        float* raw_out = output_tensors[0].GetTensorMutableData<float>();
        size_t out_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

        if (out_size < 12) {
            std::cerr << "[ONNXPolicy] 输出元素不足: " << out_size << " < 12" << std::endl;
            return false;
        }

        for (size_t i = 0; i < std::min<size_t>(out_size, 12); ++i) {
            cmds.joint_position_target[i] = default_pose_[i] + action_scale_ * raw_out[i];
        }
        for (int i = 0; i < 12; ++i) {
            cmds.kp[i] = kp_;
            cmds.kd[i] = kd_;
        }
        cmds.valid = true;

        return true;

    } catch (const Ort::Exception& e) {
        std::cerr << "[ONNXPolicy] 推理错误: " << e.what() << std::endl;
        return false;
    }
}

void ONNXPolicy::commitAcceptedCommand(const NNCommandSet& cmds)
{
    if (!observation_builder_.commitAcceptedCommand(cmds)) {
        std::cerr << "[ONNXPolicy] 拒绝提交无效的上一帧动作" << std::endl;
    }
}
