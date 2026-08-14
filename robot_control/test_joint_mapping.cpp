#include "ZeroPointCalibration.h"

#include <cmath>
#include <iostream>

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
    bool seen[12] = {false};
    const JointCalibConfig* configs = getCalibrationConfigs();

    for (int leg = 0; leg < 4; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            const JointCalibConfig& cfg = configs[leg * 3 + joint];
            const int expected_id = leg + joint * 4;  // Z字: hip, thigh, calf
            ok &= expect(cfg.motor_id == expected_id,
                         "标定表必须按每腿 hip/thigh/calf 映射到 Z 字 motor ID");
            ok &= expect(cfg.motor_id < 12 && !seen[cfg.motor_id],
                         "motor ID 必须在 0..11 内且唯一");
            seen[cfg.motor_id] = true;

            motor_direction_arr[cfg.motor_id] = cfg.motor_direction;
            motor_zero_offset[cfg.motor_id] = 10.0f + cfg.motor_id;
        }
    }

    for (int id = 0; id < 12; ++id) {
        ok &= expect(seen[id], "motor ID 0..11 必须完整");
        const float q_urdf = -0.55f + 0.1f * id;
        const float q_motor = urdfToMotorPosition(id, q_urdf);
        ok &= expect(std::abs(motorToUrdfPosition(id, q_motor) - q_urdf) < 1e-6f,
                     "位置 URDF→motor→URDF 必须可逆");

        const float dq_urdf = -1.2f + 0.2f * id;
        const float dq_motor = urdfToMotorVelocity(id, dq_urdf);
        ok &= expect(std::abs(motorToUrdfVelocity(id, dq_motor) - dq_urdf) < 1e-6f,
                     "速度方向映射必须与位置一致且可逆");

        const float tau_urdf = -3.0f + 0.5f * id;
        const float tau_motor = urdfToMotorTorque(id, tau_urdf);
        ok &= expect(std::abs(motorToUrdfTorque(id, tau_motor) - tau_urdf) < 1e-6f,
                     "力矩方向映射必须与位置/速度一致且可逆");
    }

    if (!ok) return 1;
    std::cout << "[PASS] Joint ID, position, velocity and torque mapping tests" << std::endl;
    return 0;
}
