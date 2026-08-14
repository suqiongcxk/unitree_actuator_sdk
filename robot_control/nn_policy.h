#ifndef __ROBOT_CONTROL_NN_POLICY_H
#define __ROBOT_CONTROL_NN_POLICY_H

#include "shared_data.h"
#include "nn_validation.h"
#include "policy_observation.h"
#include <memory>
#include <iostream>

// ═══════════════════════════════════════════════════════════════════════════════
//  NNPolicy — 神经网络策略抽象接口
// ═══════════════════════════════════════════════════════════════════════════════

class NNPolicy {
public:
    virtual ~NNPolicy() = default;

    virtual bool infer(const EstimatedState& est, NNCommandSet& cmds) = 0;

    /// 控制器确认最终命令后调用；被验证拒绝的原始输出不得进入动作历史。
    virtual void commitAcceptedCommand(const NNCommandSet&) {}

    /// 返回策略名称 (用于日志)
    virtual const char* name() const = 0;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  StandingPolicy — 恒定站立姿态 (基线参考)
// ═══════════════════════════════════════════════════════════════════════════════

class StandingPolicy : public NNPolicy {
public:
    StandingPolicy(const float* default_pose_12, float kp, float kd);
    bool infer(const EstimatedState& est, NNCommandSet& cmds) override;
    void commitAcceptedCommand(const NNCommandSet& cmds) override;
    const char* name() const override { return "StandingPolicy"; }

private:
    NNCommandSet standing_cmd_;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ValidatingPolicy — 包装器: 在任意策略外层加验证
// ═══════════════════════════════════════════════════════════════════════════════
//
//  将实际策略的输出通过 validateAll() 检查。
//  如果验证失败，可选择:
//    - 回退到 StandingPolicy (安全模式)
//    - 丢弃本帧指令，维持上帧 (保守模式)
//    - 仅打印警告 (调试模式)

class ValidatingPolicy : public NNPolicy {
public:
    enum class FallbackMode {
        NONE,           // 只打印警告，仍使用 NN 输出
        PREV_FRAME,     // 丢弃本帧，维持上帧指令
        STANDING,       // 回退到站立姿态
    };

    ValidatingPolicy(std::unique_ptr<NNPolicy> inner,
                     std::unique_ptr<NNPolicy> fallback = nullptr,
                     FallbackMode mode = FallbackMode::PREV_FRAME);

    bool infer(const EstimatedState& est, NNCommandSet& cmds) override;
    void commitAcceptedCommand(const NNCommandSet& cmds) override;
    const char* name() const override { return inner_->name(); }

    /// 获取上一帧的验证结果
    const ValidationResult& lastResult() const { return last_result_; }
    /// 累计统计
    int totalInferences() const { return total_count_; }
    int totalFailures()   const { return fail_count_; }

private:
    std::unique_ptr<NNPolicy> inner_;
    std::unique_ptr<NNPolicy> fallback_;
    FallbackMode              mode_;
    ValidationResult          last_result_;
    int                       total_count_ = 0;
    int                       fail_count_  = 0;
    int                       consecutive_fail_count_ = 0;
    NNCommandSet              prev_cmds_;   // 上一帧有效指令
    bool                      has_prev_ = false;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ComparingPolicy — 对比两个策略的输出差异 (调试验证用)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  同时运行 primary (如 ONNX) 和 baseline (如 StandingPolicy)，
//  记录两者输出的差异。primary 的输出发送到电机，
//  baseline 仅用于对比，不影响控制。

class ComparingPolicy : public NNPolicy {
public:
    ComparingPolicy(std::unique_ptr<NNPolicy> primary,
                    std::unique_ptr<NNPolicy> baseline);

    bool infer(const EstimatedState& est, NNCommandSet& cmds) override;
    void commitAcceptedCommand(const NNCommandSet& cmds) override;
    const char* name() const override { return primary_->name(); }

    /// 获取上帧两个策略的最大输出差异 (rad)
    float lastMaxDiff() const { return last_max_diff_; }

private:
    std::unique_ptr<NNPolicy> primary_;
    std::unique_ptr<NNPolicy> baseline_;
    float last_max_diff_ = 0.0f;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ONNXPolicy — ONNX Runtime 推理 (需要 libonnxruntime)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  编译要求: 安装 onnxruntime
//    sudo apt install libonnxruntime-dev
//  或在 CMakeLists.txt 中指定 onnxruntime 路径
//
//  使用:
//    auto policy = std::make_unique<ONNXPolicy>("model.onnx");
//    policy->initialize();  // 验证输入/输出 tensor 形状

#include <onnxruntime_cxx_api.h>

class ONNXPolicy : public NNPolicy {
public:
    /// @param model_path     ONNX 模型文件路径
    /// @param default_pose   12 关节默认位置 (rad, 输出端, 与训练时一致)
    /// @param action_scale  模型输出的缩放因子 (训练用0.25)
    /// @param kp, kd         默认 PD 增益
    ONNXPolicy(const std::string& model_path,
               const float* default_pose_12,
               float action_scale,
               float kp, float kd);

    /// 初始化 ONNX Runtime 会话，验证 I/O 形状
    bool initialize();

    bool infer(const EstimatedState& est, NNCommandSet& cmds) override;
    void commitAcceptedCommand(const NNCommandSet& cmds) override;
    const char* name() const override { return "ONNXPolicy"; }

    // ── 模型信息 (初始化后可用) ──

    const std::string& modelPath() const { return model_path_; }
    bool isInitialized() const { return session_ != nullptr; }

    int inputCount()  const;
    int outputCount() const;

    const std::array<float, PolicyObservationBuilder::kObservationSize>&
    lastObservation() const { return observation_builder_.lastObservation(); }
    const std::array<float, PolicyObservationBuilder::kActionSize>&
    previousAction() const { return observation_builder_.previousAction(); }

    /// 打印模型输入/输出 tensor 信息
    void printModelInfo() const;

private:
    std::string model_path_;
    float default_pose_[12];
    float action_scale_;
    float kp_, kd_;
    PolicyObservationBuilder observation_builder_;

    Ort::Env    env_{ORT_LOGGING_LEVEL_WARNING, "ONNXPolicy"};
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    // 缓存 tensor 信息以加速推理
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<int64_t>     input_shape_;
    std::vector<int64_t>     output_shape_;
};

#endif  // __ROBOT_CONTROL_NN_POLICY_H
