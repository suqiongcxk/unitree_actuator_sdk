#include "policy_observation.h"

#include <cmath>
#include <cstring>

PolicyObservationBuilder::PolicyObservationBuilder(
    const float* default_pose_12, float action_scale)
    : action_scale_(action_scale)
{
    std::memcpy(default_pose_.data(), default_pose_12,
                kActionSize * sizeof(float));
    previous_action_.fill(0.0f);
    observation_.fill(0.0f);
}

const std::array<float, PolicyObservationBuilder::kObservationSize>&
PolicyObservationBuilder::build(const EstimatedState& est)
{
    observation_.fill(0.0f);

    for (int i = 0; i < 3; ++i) {
        observation_[i] = est.angular_velocity[i];
        observation_[3 + i] = est.projected_gravity[i];
    }

    // [6..8] 速度命令当前固定为零；接入遥控命令时必须与训练缩放一致。
    for (int i = 0; i < 12; ++i) {
        observation_[9 + i] = est.joint_position[i];
        observation_[21 + i] = est.joint_velocity[i];
        observation_[33 + i] = previous_action_[i];
    }
    // [45..47] 仍为预留零值，等待训练端定义确认。
    return observation_;
}

bool PolicyObservationBuilder::commitAcceptedCommand(const NNCommandSet& cmds)
{
    if (!cmds.valid || !std::isfinite(action_scale_)
                    || std::abs(action_scale_) < 1e-8f) {
        return false;
    }

    std::array<float, kActionSize> next_action{};
    for (int i = 0; i < 12; ++i) {
        if (!std::isfinite(cmds.joint_position_target[i])) return false;
        next_action[i] = (cmds.joint_position_target[i] - default_pose_[i])
                       / action_scale_;
        if (!std::isfinite(next_action[i])) return false;
    }
    previous_action_ = next_action;
    return true;
}
