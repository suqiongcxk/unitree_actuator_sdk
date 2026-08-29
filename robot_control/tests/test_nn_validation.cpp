#include "nn_validation.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

NNCommandSet makeSafeCommand()
{
    NNCommandSet cmds;
    for (int i = 0; i < 12; ++i) {
        cmds.joint_position_target[i] = 0.5f * (JOINT_LIMITS[i][0] + JOINT_LIMITS[i][1]);
        cmds.kp[i] = 0.3f;
        cmds.kd[i] = 0.02f;
    }
    cmds.valid = true;
    return cmds;
}

bool expect(bool condition, const char* message)
{
    if (!condition) std::cerr << "[FAIL] " << message << std::endl;
    return condition;
}

}  // namespace

int main()
{
    bool ok = true;
    NNCommandSet safe = makeSafeCommand();
    float previous[12];
    for (int i = 0; i < 12; ++i) previous[i] = safe.joint_position_target[i];

    ok &= expect(validateCommandSet(safe, previous).passed,
                 "安全指令应通过");

    NNCommandSet wrong_order = safe;
    wrong_order.joint_position_target[4] = -2.0f;
    ok &= expect(!validateCommandSet(wrong_order, previous).passed,
                 "Motor 4 不得误用 Calf 限位");

    NNCommandSet margin = safe;
    margin.joint_position_target[0] = JOINT_LIMITS[0][1] - 0.01f;
    ok &= expect(!validateCommandSet(margin, previous, 0.05f, 10.0f).passed,
                 "机械限位内侧安全余量应生效");

    NNCommandSet nan_cmd = safe;
    nan_cmd.joint_position_target[8] = std::numeric_limits<float>::quiet_NaN();
    ok &= expect(!validateCommandSet(nan_cmd, previous).passed,
                 "NaN 位置必须失败");

    NNCommandSet gain = safe;
    gain.kp[3] = 0.9f;
    ok &= expect(!validateCommandSet(gain, previous).passed,
                 "过大 KP 必须失败");

    NNCommandSet jump = safe;
    jump.joint_position_target[6] += 0.11f;
    ok &= expect(!validateCommandSet(jump, previous).passed,
                 "单帧跳变必须失败");

    if (!ok) return 1;
    std::cout << "[PASS] NN safety validation tests" << std::endl;
    return 0;
}
