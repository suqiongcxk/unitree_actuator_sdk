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
    PolicyObservationBuilder builder(default_pose);

    EstimatedState est;
    for (int i = 0; i < 3; ++i) {
        est.body_linear_velocity[i] = 10.0f + i;
        est.angular_velocity[i] = 20.0f + i;
        est.projected_gravity[i] = 30.0f + i;
    }
    for (int i = 0; i < 12; ++i) {
        est.joint_position[i] = default_pose[i] + 0.01f * (i + 1);
        est.joint_velocity[i] = 200.0f + i;
    }
    const std::array<float, 3> command{{0.4f, -0.2f, 0.6f}};
    ok &= expect(builder.setVelocityCommand(command), "有效三维速度命令应可设置");

    const auto& first = builder.build(est);
    bool layout_ok = true;
    for (int i = 0; i < 3; ++i) {
        layout_ok = layout_ok && first[i] == est.body_linear_velocity[i]
                              && first[3 + i] == est.angular_velocity[i]
                              && first[6 + i] == est.projected_gravity[i]
                              && first[9 + i] == command[i];
    }
    for (int i = 0; i < 12; ++i) {
        layout_ok = layout_ok
            && std::abs(first[12 + i] - 0.01f * (i + 1)) < 1e-6f
            && first[24 + i] == est.joint_velocity[i]
            && first[36 + i] == 0.0f;
    }
    ok &= expect(layout_ok, "首帧 48 维布局及零动作历史必须正确");

    std::array<float, 12> raw_action{};
    for (int i = 0; i < 12; ++i) raw_action[i] = i - 5.0f;
    ok &= expect(builder.commitRawAction(raw_action.data()), "有效 Actor 原始输出应可提交");

    const auto& second = builder.build(est);
    bool action_ok = true;
    for (int i = 0; i < 12; ++i)
        action_ok = action_ok && std::abs(second[36 + i] - raw_action[i]) < 1e-6f;
    ok &= expect(action_ok, "上一帧 Actor 原始 action 必须写入 [36..47]");

    std::array<float, 12> rejected = raw_action;
    rejected[3] = std::numeric_limits<float>::quiet_NaN();
    ok &= expect(!builder.commitRawAction(rejected.data()), "NaN raw action 不得污染动作历史");
    const auto& after_reject = builder.build(est);
    ok &= expect(std::abs(after_reject[39] - raw_action[3]) < 1e-6f,
                 "拒绝 raw action 后必须保留上一帧 Actor 输出");

    auto invalid_command = command;
    invalid_command[1] = std::numeric_limits<float>::infinity();
    ok &= expect(!builder.setVelocityCommand(invalid_command),
                 "NaN/Inf 速度命令必须被拒绝");
    const auto& after_bad_command = builder.build(est);
    ok &= expect(after_bad_command[10] == command[1],
                 "拒绝速度命令后必须保留上一帧有效命令");

    // body_height/position 不属于训练端的 48 维 Actor observation。
    const auto observation_before_height_change = builder.build(est);
    est.body_height += 0.02f;
    est.position[2] += 0.02f;
    const auto observation_after_height_change = builder.build(est);
    bool height_isolation_ok = true;
    for (int i = 0; i < 48; ++i) {
        height_isolation_ok = height_isolation_ok
            && observation_before_height_change[i] == observation_after_height_change[i];
    }
    ok &= expect(height_isolation_ok,
                 "高度定义修正不得改变 48 维 Actor observation");

    if (!ok) return 1;
    std::cout << "[PASS] Policy observation and previous-action tests" << std::endl;
    return 0;
}
