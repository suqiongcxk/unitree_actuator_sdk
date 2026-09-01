#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "creeper_sim/joint_mapping.h"

namespace creeper_sim {
enum class StateMode { GroundTruth, SensorEmulation };
struct RobotState {
 std::array<double,3> body_linear_velocity{}, body_angular_velocity{}, projected_gravity{}, specific_force{};
 std::array<double,4> orientation{{1,0,0,0}};
 std::array<double,12> joint_position{}, joint_velocity{}, joint_torque{};
 std::array<bool,4> contact{};
 double base_height=0; double time=0; bool valid=false;
};
struct ControlCommand { std::array<double,3> velocity{}; };
struct SafetyStatus { bool fault_latched=false; std::string reason; std::uint64_t torque_saturations=0; double max_target_jump=0; };
struct SimulationConfig {
 double physics_dt=.005; int decimation=4; double kp=25, kd=.5, action_scale=.25;
 double torque_limit=23.7, velocity_limit=30.1, takeover_seconds=1.0;
};
}
