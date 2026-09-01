#pragma once
#include <array>
#include <string_view>

namespace creeper_sim {
constexpr std::size_t kJointCount = 12;
inline constexpr std::array<std::string_view, kJointCount> kJointNames{{
 "FL_hip_joint","FR_hip_joint","RL_hip_joint","RR_hip_joint",
 "FL_thigh_joint","FR_thigh_joint","RL_thigh_joint","RR_thigh_joint",
 "FL_calf_joint","FR_calf_joint","RL_calf_joint","RR_calf_joint"}};
inline constexpr std::array<std::string_view, kJointCount> kActuatorNames{{
 "FL_hip_actuator","FR_hip_actuator","RL_hip_actuator","RR_hip_actuator",
 "FL_thigh_actuator","FR_thigh_actuator","RL_thigh_actuator","RR_thigh_actuator",
 "FL_calf_actuator","FR_calf_actuator","RL_calf_actuator","RR_calf_actuator"}};
inline constexpr std::array<double, kJointCount> kDefaultPose{{
 .1,-.1,.1,-.1,.8,.8,1.,1.,-1.5,-1.5,-1.5,-1.5}};
inline std::array<double,kJointCount> rawActionToTarget(const std::array<float,kJointCount>& action,double scale=.25){std::array<double,kJointCount> out{};for(std::size_t i=0;i<kJointCount;++i)out[i]=kDefaultPose[i]+scale*action[i];return out;}
}
