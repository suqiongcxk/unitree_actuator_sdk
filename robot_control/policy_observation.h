#ifndef __ROBOT_CONTROL_POLICY_OBSERVATION_H
#define __ROBOT_CONTROL_POLICY_OBSERVATION_H

#include "shared_data.h"

#include <array>
#include <cstddef>

class PolicyObservationBuilder {
public:
    static constexpr std::size_t kObservationSize = 48;
    static constexpr std::size_t kActionSize = 12;

    PolicyObservationBuilder(const float* default_pose_12, float action_scale);

    /// 按当前约定构建 48 维观测。首帧 previous_action 为全零。
    const std::array<float, kObservationSize>& build(const EstimatedState& est);

    /// 仅提交最终通过验证、准备下发（或 dry-run 接受）的命令。
    bool commitAcceptedCommand(const NNCommandSet& cmds);

    const std::array<float, kActionSize>& previousAction() const {
        return previous_action_;
    }
    const std::array<float, kObservationSize>& lastObservation() const {
        return observation_;
    }

private:
    std::array<float, kActionSize> default_pose_{};
    std::array<float, kActionSize> previous_action_{};
    std::array<float, kObservationSize> observation_{};
    float action_scale_ = 0.0f;
};

#endif
