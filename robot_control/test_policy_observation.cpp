#include "policy_observation.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {
bool expect(bool condition, const char* message)
{
    if (!condition) std::cerr << "[FAIL] " << message << std::endl;
    return condition;
}
}

int main()
{
    bool ok = true;
    float default_pose[12];
    for (int i = 0; i < 12; ++i) default_pose[i] = 0.1f * i;
    PolicyObservationBuilder builder(default_pose, 0.25f);

    EstimatedState est;
    for (int i = 0; i < 3; ++i) {
        est.angular_velocity[i] = 10.0f + i;
        est.projected_gravity[i] = 20.0f + i;
    }
    for (int i = 0; i < 12; ++i) {
        est.joint_position[i] = 100.0f + i;
        est.joint_velocity[i] = 200.0f + i;
    }

    const auto& first = builder.build(est);
    bool layout_ok = true;
    for (int i = 0; i < 3; ++i) {
        layout_ok = layout_ok && first[i] == est.angular_velocity[i]
                              && first[3 + i] == est.projected_gravity[i]
                              && first[6 + i] == 0.0f
                              && first[45 + i] == 0.0f;
    }
    for (int i = 0; i < 12; ++i) {
        layout_ok = layout_ok && first[9 + i] == est.joint_position[i]
                              && first[21 + i] == est.joint_velocity[i]
                              && first[33 + i] == 0.0f;
    }
    ok &= expect(layout_ok, "首帧 48 维布局及零动作历史必须正确");

    NNCommandSet accepted;
    accepted.valid = true;
    for (int i = 0; i < 12; ++i)
        accepted.joint_position_target[i] = default_pose[i] + 0.25f * (i - 5.0f);
    ok &= expect(builder.commitAcceptedCommand(accepted), "有效最终命令应可提交");

    const auto& second = builder.build(est);
    bool action_ok = true;
    for (int i = 0; i < 12; ++i)
        action_ok = action_ok && std::abs(second[33 + i] - (i - 5.0f)) < 1e-6f;
    ok &= expect(action_ok, "上一帧 action 必须由最终目标反算并写入 [33..44]");

    NNCommandSet rejected = accepted;
    rejected.joint_position_target[3] = std::numeric_limits<float>::quiet_NaN();
    ok &= expect(!builder.commitAcceptedCommand(rejected), "NaN 命令不得污染动作历史");
    const auto& after_reject = builder.build(est);
    ok &= expect(std::abs(after_reject[36] - (-2.0f)) < 1e-6f,
                 "拒绝提交后必须保留上一帧有效 action");

    if (!ok) return 1;
    std::cout << "[PASS] Policy observation and previous-action tests" << std::endl;
    return 0;
}
