#include "foot_force_estimator.h"
#include "leg_kinematics.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {
bool expect(bool condition, const char* message) {
    if (!condition) std::cerr << "[FAIL] " << message << std::endl;
    return condition;
}

float distance3(const float lhs[3], const float rhs[3]) {
    float sum = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        const float delta = lhs[axis] - rhs[axis];
        sum += delta * delta;
    }
    return std::sqrt(sum);
}
}  // namespace

int main() {
    bool ok = true;
    CreeperLegKinematics kinematics;
    FootForceEstimator estimator;
    float q[12] = {0.0f};
    float dq[12] = {0.0f};
    for (int leg = 0; leg < 4; ++leg) {
        q[leg] = 0.0f;
        q[4 + leg] = 0.8f;
        q[8 + leg] = -1.5f;
    }

    const float expected_force[3] = {8.0f, -3.0f, 45.0f};
    for (int leg = 0; leg < 4; ++leg) {
        const auto result = kinematics.compute(leg, q, dq);
        ok &= expect(result.valid, "默认姿态运动学必须有效");
        float torque[3] = {0.0f};
        for (int joint = 0; joint < 3; ++joint)
            for (int axis = 0; axis < 3; ++axis)
                torque[joint] -= result.jacobian[axis][joint]
                               * expected_force[axis];
        const auto estimate = estimator.estimate(result.jacobian, torque);
        ok &= expect(estimate.valid, "非奇异姿态足端力估计必须有效");
        ok &= expect(distance3(estimate.force_body, expected_force) < 0.02f,
                     "DLS必须恢复已知三维地面反力");
        ok &= expect(std::abs(estimate.normal_force - expected_force[2]) < 0.02f,
                     "法向支撑力必须取base系+Z分量");
    }

    float singular_jacobian[3][3] = {{0.0f}};
    float torque[3] = {1.0f, 2.0f, 3.0f};
    const auto singular = estimator.estimate(singular_jacobian, torque);
    ok &= expect(!singular.valid, "奇异雅可比不得产生有效足端力");

    float invalid_jacobian[3][3] = {{0.0f}};
    invalid_jacobian[0][0] = std::numeric_limits<float>::quiet_NaN();
    const auto invalid = estimator.estimate(invalid_jacobian, torque);
    ok &= expect(!invalid.valid, "非有限输入必须失效");

    if (!ok) return 1;
    std::cout << "[PASS] foot force estimator tests" << std::endl;
    return 0;
}
