#include "creeper_sim/simulation_controller.h"
#include "creeper_sim/motion_safety.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <thread>
namespace creeper_sim {
SimulationController::SimulationController(SimulationConfig c):config_(c){}
bool SimulationController::initialize(const std::string&m,const std::string&k,const std::string&o,std::string&e){if(!backend_.load(m,e)||!backend_.reset(k,e))return false;if(std::abs(backend_.timestep()-config_.physics_dt)>1e-12){e="MJCF timestep does not match physics_dt";return false;}target_=kDefaultPose;if(!o.empty()&&!policy_.load(o,e))return false;return true;}
bool SimulationController::run(double duration,StateMode mode,const ControlCommand&command,bool realtime,const std::string&log,std::string&e){
 std::ofstream csv;if(!log.empty()){csv.open(log);if(!csv){e="cannot open log";return false;}csv<<"time,base_z,policy_tick,base_contact,torque_saturations,max_target_jump\n";}
 const auto wall0=std::chrono::steady_clock::now();std::uint64_t step=0;const int takeover_steps=std::max(1,int(config_.takeover_seconds/(config_.physics_dt*config_.decimation)));
 while(!stop_.load()&&backend_.time()<duration){RobotState s=backend_.readState(mode);if(!finiteState(s,e)){safety_.fault_latched=true;safety_.reason=e;break;}
  const bool policy_tick=(step%config_.decimation)==0;if(policy_tick&&policy_.loaded()){
   auto&obs=observation_.build(s,command);std::array<float,12>a{};if(!policy_.infer(obs,a,e)||!observation_.commitRawAction(a)){safety_.fault_latched=true;safety_.reason=e.empty()?"invalid raw action":e;break;}
   const double alpha=std::min(1.0,double(step/config_.decimation+1)/takeover_steps);for(int i=0;i<12;++i){double candidate=kDefaultPose[i]+config_.action_scale*a[i];double blended=kDefaultPose[i]+alpha*(candidate-kDefaultPose[i]);safety_.max_target_jump=std::max(safety_.max_target_jump,std::abs(blended-target_[i]));target_[i]=blended;}
  }
  if(!validateTarget(target_,s,backend_.lowerLimits(),backend_.upperLimits(),e)){safety_.fault_latched=true;safety_.reason=e;break;}
  std::array<double,12>tau{};for(int i=0;i<12;++i){double raw=config_.kp*(target_[i]-s.joint_position[i])-config_.kd*s.joint_velocity[i];tau[i]=std::clamp(raw,-config_.torque_limit,config_.torque_limit);if(tau[i]!=raw)++safety_.torque_saturations;}backend_.applyTorques(tau);backend_.step();
  if(backend_.baseContact()){safety_.fault_latched=true;safety_.reason="base contact";break;}if(csv&&step%config_.decimation==0)csv<<std::setprecision(10)<<backend_.time()<<','<<s.base_height<<','<<1<<','<<0<<','<<safety_.torque_saturations<<','<<safety_.max_target_jump<<'\n';
  ++step;if(realtime){auto due=wall0+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(backend_.time()));std::this_thread::sleep_until(due);}
 }
 if(safety_.fault_latched){e=safety_.reason;return false;}return true;
}
}
