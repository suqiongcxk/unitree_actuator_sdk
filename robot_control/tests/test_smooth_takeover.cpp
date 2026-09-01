#include "nn_policy.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>

namespace {

class ConstantActorPolicy final : public NNPolicy {
public:
    explicit ConstantActorPolicy(const NNCommandSet& command) : command_(command) {}

    bool infer(const EstimatedState&, NNCommandSet& cmds) override
    {
        cmds = command_;
        return true;
    }

    const char* name() const override { return "ConstantActorPolicy"; }

private:
    NNCommandSet command_;
};

bool expect(bool condition, const char* message)
{
    if (!condition) std::cerr << "[FAIL] " << message << std::endl;
    return condition;
}

NNCommandSet makeStandingCommand()
{
    constexpr float pose[12] = {
        0.1f, -0.1f, 0.1f, -0.1f,
        0.8f,  0.8f, 1.0f,  1.0f,
       -1.5f, -1.5f, -1.5f, -1.5f,
    };
    NNCommandSet command;
    for (int i = 0; i < 12; ++i) {
        command.joint_position_target[i] = pose[i];
        command.kp[i] = 0.625f;
        command.kd[i] = 0.0125f;
    }
    command.valid = true;
    return command;
}

NNCommandSet makeActorCommand(const NNCommandSet& standing)
{
    // 最大偏差接近实机预演观察到的 0.24 rad，用来复现原始首帧跳变。
    constexpr float delta[12] = {
         0.16f, -0.15f, 0.18f, -0.15f,
         0.08f,  0.24f, 0.02f,  0.10f,
        -0.18f,  0.22f, 0.21f,  0.23f,
    };
    NNCommandSet command = standing;
    for (int i = 0; i < 12; ++i)
        command.joint_position_target[i] += delta[i];
    return command;
}

}  // namespace

int main()
{
    bool ok = true;
    const NNCommandSet standing = makeStandingCommand();
    const NNCommandSet actor = makeActorCommand(standing);
    EstimatedState state;

    // 安全层没有接受候选帧时，混合进度不得偷偷前进。
    SmoothTakeoverPolicy rejected(
        std::make_unique<ConstantActorPolicy>(actor), standing, 100);
    NNCommandSet rejected_candidate;
    ok &= expect(rejected.infer(state, rejected_candidate),
                 "平滑策略应产生候选指令");
    rejected.commitAcceptedCommand(standing);
    ok &= expect(rejected.acceptedBlendFrames() == 0,
                 "回退到旧站姿时不得推进接管进度");

    auto smooth_owner = std::make_unique<SmoothTakeoverPolicy>(
        std::make_unique<ConstantActorPolicy>(actor), standing, 100);
    SmoothTakeoverPolicy* smooth = smooth_owner.get();
    ValidatingPolicy validating(
        std::move(smooth_owner), nullptr,
        ValidatingPolicy::FallbackMode::PREV_FRAME);

    // 对应 RobotController：验证历史从开环站立的锁存目标开始。
    validating.commitAcceptedCommand(standing);
    NNCommandSet previous = standing;
    float observed_max_jump = 0.0f;

    for (int frame = 0; frame < 100; ++frame) {
        NNCommandSet output;
        ok &= expect(validating.infer(state, output),
                     "100 帧接管期间推理不应失败");
        ok &= expect(validating.lastResult().passed,
                     "平滑后的每一帧都应通过安全验证");

        for (int i = 0; i < 12; ++i) {
            observed_max_jump = std::max(
                observed_max_jump,
                std::abs(output.joint_position_target[i]
                       - previous.joint_position_target[i]));
        }
        validating.commitAcceptedCommand(output);
        previous = output;
    }

    ok &= expect(smooth->takeoverComplete(), "100 个接受帧后应完成接管");
    ok &= expect(observed_max_jump < 0.01f,
                 "2 秒 smoothstep 接管的单帧位置变化应远小于 0.1 rad 门限");
    for (int i = 0; i < 12; ++i) {
        ok &= expect(std::abs(previous.joint_position_target[i]
                           - actor.joint_position_target[i]) < 1.0e-6f,
                     "接管结束目标应等于 Actor 目标");
    }

    // 单关节跳变当前仅监测，不得请求停机或丢弃Actor目标。
    NNCommandSet monitored_jump = standing;
    monitored_jump.joint_position_target[7] += 0.11f;
    ValidatingPolicy jump_monitor(
        std::make_unique<ConstantActorPolicy>(monitored_jump));
    jump_monitor.commitAcceptedCommand(standing);
    NNCommandSet monitored_output;
    ok &= expect(jump_monitor.infer(state, monitored_output),
                 "监测模式应接受跳变候选目标");
    ok &= expect(!jump_monitor.requiresSafetyStop(),
                 "监测模式不得因单关节跳变停机");
    ok &= expect(std::abs(monitored_output.joint_position_target[7]
                       - monitored_jump.joint_position_target[7]) < 1.0e-6f,
                 "监测模式应保留Actor候选目标");

    // P0回归：真正越过机械限位的Actor目标不得切换到StandingPolicy。第一帧失败
    // 就必须保持最后接受指令并请求控制器进入统一阻尼停机。
    NNCommandSet unsafe = standing;
    unsafe.joint_position_target[7] = JOINT_LIMITS[7][1] + 0.1f;
    NNCommandSet distinct_fallback = standing;
    distinct_fallback.joint_position_target[7] -= 0.4f;
    ValidatingPolicy fail_safe(
        std::make_unique<ConstantActorPolicy>(unsafe),
        std::make_unique<ConstantActorPolicy>(distinct_fallback),
        // 即使遗留配置仍传STANDING，也绝不允许切换站姿。
        ValidatingPolicy::FallbackMode::STANDING);
    fail_safe.commitAcceptedCommand(standing);
    NNCommandSet rejected_output;
    ok &= expect(fail_safe.infer(state, rejected_output),
                 "危险候选应返回最后安全指令供停机前保持");
    ok &= expect(fail_safe.requiresSafetyStop(),
                 "首个危险候选必须立即请求安全停机");
    ok &= expect(fail_safe.safetyStopDetail() == 7,
                 "安全停机应报告首个跳变关节");
    for (int i = 0; i < 12; ++i) {
        ok &= expect(std::abs(rejected_output.joint_position_target[i]
                           - standing.joint_position_target[i]) < 1.0e-6f,
                     "危险候选不得跳到Standing fallback或写入Actor目标");
    }

    if (!ok) return 1;
    std::cout << "[PASS] smooth NN takeover tests, max_jump="
              << observed_max_jump << " rad" << std::endl;
    return 0;
}
