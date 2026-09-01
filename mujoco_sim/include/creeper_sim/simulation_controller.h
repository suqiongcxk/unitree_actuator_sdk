#pragma once
#include <atomic>
#include <fstream>
#include "creeper_sim/mujoco_backend.h"
#include "creeper_sim/onnx_policy.h"
#include "creeper_sim/policy_observation.h"
namespace creeper_sim {
class SimulationController {
public:
 explicit SimulationController(SimulationConfig={});
 bool initialize(const std::string& model,const std::string& keyframe,const std::string& onnx,std::string& error);
 bool run(double duration, StateMode mode, const ControlCommand&, bool realtime, const std::string& log, std::string& error);
 void requestStop(){stop_.store(true);} const SafetyStatus& safety() const{return safety_;}
 MujocoBackend& backend(){return backend_;}
private:
 SimulationConfig config_; MujocoBackend backend_; PolicyObservation observation_; OnnxPolicy policy_;
 std::array<double,12> target_=kDefaultPose; SafetyStatus safety_; std::atomic<bool> stop_{false};
};
}
