#pragma once
#include <array>
#include <memory>
#include <string>
#include "creeper_sim/simulation_types.h"
struct mjModel_; struct mjData_;
namespace creeper_sim {
class MujocoBackend {
public:
 MujocoBackend(); ~MujocoBackend();
 bool load(const std::string&, std::string&); bool reset(const std::string&, std::string&);
 RobotState readState(StateMode) const;
 void applyTorques(const std::array<double,12>&); void step();
 const std::array<double,12>& lowerLimits() const{return lower_;}
 const std::array<double,12>& upperLimits() const{return upper_;}
 double timestep() const; bool baseContact() const; double time() const;
 const std::array<int,12>& qposAddresses() const{return qpos_adr_;}
 const std::array<int,12>& dofAddresses() const{return dof_adr_;}
private:
 mjModel_* model_=nullptr; mjData_* data_=nullptr;
 std::array<int,12> joint_id_{}, actuator_id_{}, qpos_adr_{}, dof_adr_{}; int base_body_id_=-1;
 std::array<int,12> joint_pos_sensor_{}, joint_vel_sensor_{}, force_sensor_{};
 std::array<int,4> foot_touch_sensor_{}; int imu_quat_=-1, imu_gyro_=-1, imu_accel_=-1;
 std::array<double,12> lower_{}, upper_{};
 mutable std::array<double,3> estimated_world_velocity_{}; mutable double estimator_time_=-1;
};
}
