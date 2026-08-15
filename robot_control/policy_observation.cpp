#include "policy_observation.h"

#include <cmath>
#include <cstring>

PolicyObservationBuilder::PolicyObservationBuilder(
    const float* default_pose_12)
{
    std::memcpy(default_pose_.data(), default_pose_12,
                kActionSize * sizeof(float));
    previous_raw_action_.fill(0.0f);
    observation_.fill(0.0f);
}

const std::array<float, PolicyObservationBuilder::kObservationSize>&
PolicyObservationBuilder::build(const EstimatedState& est)
{
    observation_.fill(0.0f);

    for (int i = 0; i < 3; ++i) {
        observation_[i] = est.body_linear_velocity[i];
        observation_[3 + i] = est.angular_velocity[i];
        observation_[6 + i] = est.projected_gravity[i];
        observation_[9 + i] = velocity_command_[i];
    }

    for (int i = 0; i < 12; ++i) {
        observation_[12 + i] = est.joint_position[i] - default_pose_[i];
        observation_[24 + i] = est.joint_velocity[i]; // 训练 default joint velocity = 0
        observation_[36 + i] = previous_raw_action_[i];
    }
    return observation_;
}

bool PolicyObservationBuilder::commitRawAction(const float* raw_action_12)
{
    std::array<float, kActionSize> next_action{};
    for (int i = 0; i < 12; ++i) {
        if (!raw_action_12 || !std::isfinite(raw_action_12[i])) return false;
        next_action[i] = raw_action_12[i];
    }
    previous_raw_action_ = next_action;
    return true;
}

bool PolicyObservationBuilder::setVelocityCommand(const std::array<float, 3>& command)
{
    for (float value : command) if (!std::isfinite(value)) return false;
    velocity_command_ = command;
    return true;
}
