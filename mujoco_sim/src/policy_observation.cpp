#include "creeper_sim/policy_observation.h"
#include "creeper_sim/joint_mapping.h"
#include <cmath>
namespace creeper_sim {
const PolicyObservation::Observation& PolicyObservation::build(const RobotState& s,const ControlCommand& c){
 observation_.fill(0);
 for(int i=0;i<3;++i){observation_[i]=s.body_linear_velocity[i];observation_[3+i]=s.body_angular_velocity[i];observation_[6+i]=s.projected_gravity[i];observation_[9+i]=c.velocity[i];}
 for(int i=0;i<12;++i){observation_[12+i]=s.joint_position[i]-kDefaultPose[i];observation_[24+i]=s.joint_velocity[i];observation_[36+i]=previous_[i];}
 return observation_;
}
bool PolicyObservation::commitRawAction(const Action& a){for(float v:a)if(!std::isfinite(v))return false;previous_=a;return true;}
}
