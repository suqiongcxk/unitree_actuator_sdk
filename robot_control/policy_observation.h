#ifndef __ROBOT_CONTROL_POLICY_OBSERVATION_H
#define __ROBOT_CONTROL_POLICY_OBSERVATION_H

#include "shared_data.h"

#include <array>
#include <cstddef>

class PolicyObservationBuilder {
public:
    static constexpr std::size_t kObservationSize = 48;
    static constexpr std::size_t kActionSize = 12;

    explicit PolicyObservationBuilder(const float* default_pose_12);

    /// 按当前约定构建 48 维观测。首帧 previous_action 为全零。
    const std::array<float, kObservationSize>& build(const EstimatedState& est);

    /// 保存 Actor 本帧原始输出，与安全层最终接受命令完全分离。
    bool commitRawAction(const float* raw_action_12);
    bool setVelocityCommand(const std::array<float, 3>& command);

    const std::array<float, kActionSize>& previousAction() const {
        return previous_raw_action_;
    }
    const std::array<float, kObservationSize>& lastObservation() const {
        return observation_;
    }

private:
    std::array<float, kActionSize> default_pose_{};
    std::array<float, kActionSize> previous_raw_action_{};
    std::array<float, kObservationSize> observation_{};
    std::array<float, 3> velocity_command_{};
};

#endif
