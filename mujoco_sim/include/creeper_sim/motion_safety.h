#pragma once
#include <array>
#include <string>
#include "creeper_sim/simulation_types.h"
namespace creeper_sim {
bool finiteState(const RobotState&, std::string& reason);
bool validateTarget(const std::array<double,12>& target, const RobotState&, const std::array<double,12>& lower, const std::array<double,12>& upper, std::string& reason);
}
