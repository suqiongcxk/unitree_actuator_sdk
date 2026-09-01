#pragma once
#include <array>
#include "creeper_sim/simulation_types.h"
namespace creeper_sim {
class PolicyObservation {
public:
 using Observation=std::array<float,48>; using Action=std::array<float,12>;
 const Observation& build(const RobotState&, const ControlCommand&);
 bool commitRawAction(const Action&);
 const Observation& last() const { return observation_; }
 const Action& previousRawAction() const { return previous_; }
private: Observation observation_{}; Action previous_{};
};
}
