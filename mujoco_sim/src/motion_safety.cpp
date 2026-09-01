#include "creeper_sim/motion_safety.h"
#include <cmath>
namespace creeper_sim {
bool finiteState(const RobotState&s,std::string&r){
 auto ck=[&](const auto&a,const char*n){for(auto v:a)if(!std::isfinite(static_cast<double>(v))){r=std::string("non-finite ")+n;return false;}return true;};
 return s.valid&&ck(s.body_linear_velocity,"linear velocity")&&ck(s.body_angular_velocity,"angular velocity")&&ck(s.projected_gravity,"gravity")&&ck(s.joint_position,"joint position")&&ck(s.joint_velocity,"joint velocity");
}
bool validateTarget(const std::array<double,12>&t,const RobotState&s,const std::array<double,12>&lo,const std::array<double,12>&hi,std::string&r){
 for(int i=0;i<12;++i){if(!std::isfinite(t[i])){r="non-finite target";return false;}if(t[i]<lo[i]||t[i]>hi[i]){r="target outside joint limit: "+std::to_string(i);return false;}if(std::abs(s.joint_velocity[i])>30.1){r="joint velocity limit: "+std::to_string(i);return false;}}return true;
}
}
