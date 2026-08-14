#ifndef ROBOT_CONTROL_LEGGED_ODOMETRY_H
#define ROBOT_CONTROL_LEGGED_ODOMETRY_H

#include "leg_kinematics.h"

struct LeggedOdometryConfig {
    float contact_torque_on = 1.0f;       // N·m，待实机标定
    float contact_torque_off = 0.6f;      // 迟滞下限
    float contact_height_band = 0.035f;   // 仅用于高度置信度，不否决高低地形接触
    float max_contact_foot_speed = 2.0f;  // 相对机体，m/s
    float slip_residual_threshold = 0.35f;// 支撑腿速度约束不一致，m/s
    int contact_on_frames = 3;
    int contact_off_frames = 2;
    float velocity_correction_gain = 0.35f;
    float velocity_decay_no_contact = 0.995f;
};

struct LeggedOdometryOutput {
    float foot_position[4][3] = {{0}};
    float foot_velocity[4][3] = {{0}};
    bool contact[4] = {false};
    float contact_confidence[4] = {0};
    float linear_velocity_world[3] = {0};
    float body_height = 0.0f;
    float velocity_confidence = 0.0f;
    bool slipping = false;
    bool airborne = true;
    bool impact = false;
    bool valid = false;
};

class LeggedOdometry {
public:
    explicit LeggedOdometry(const LeggedOdometryConfig& config = {});
    LeggedOdometryOutput update(const float q[12], const float dq[12],
        const float tau[12], const float quaternion_body_to_world[4],
        const float angular_velocity_body[3], const float acceleration_body[3],
        float dt_sec);

private:
    enum class ContactPhase { AIR, TOUCHDOWN, STANCE };
    LeggedOdometryConfig config_;
    CreeperLegKinematics kinematics_;
    ContactPhase contact_phase_[4] = {
        ContactPhase::AIR, ContactPhase::AIR,
        ContactPhase::AIR, ContactPhase::AIR};
    int on_count_[4] = {0};
    int off_count_[4] = {0};
    float velocity_world_[3] = {0};
    float previous_vertical_acceleration_ = 0.0f;
    bool has_previous_acceleration_ = false;
};

#endif
