#ifndef ROBOT_CONTROL_LEGGED_ODOMETRY_H
#define ROBOT_CONTROL_LEGGED_ODOMETRY_H

#include "leg_kinematics.h"
#include "foot_force_estimator.h"

struct LeggedOdometryConfig {
    // 4477帧实机日志离线标定值。优先使用地面对足端的+Z法向力，
    // 避免切向力或关节构型变化被简单力矩范数误认为触地。
    float feedback_torque_to_joint = 6.333f; // GO-M8010-6转子→关节减速比
    float contact_normal_force_on = 7.60f;   // N，连续满足后判定触地
    float contact_normal_force_off = 3.80f;  // N，连续低于后判定离地
    // 2026-08-27 二次实机标定：完全离地稳定值约 0.09~0.18 N·m、
    // 运动瞬态最高约 0.23 N·m；仅在足端力求解无效时作为安全回退。
    float contact_torque_on = 0.28f;       // 转子N·m，回退触地阈值
    float contact_torque_off = 0.20f;      // 转子N·m，回退离地阈值
    float contact_height_band = 0.035f;   // 仅用于高度置信度，不否决高低地形接触
    float foot_radius = 0.020f;            // URDF 足球碰撞球半径，m
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
    float foot_force_body[4][3] = {{0}};  // 地面对足端的力，base系，N
    float normal_force[4] = {0};          // max(0, Fz)，N
    float force_residual[4] = {0};        // ||J^T F + tau||，N·m
    bool foot_force_valid[4] = {false};
    bool contact_used_force[4] = {false}; // false表示本帧退回力矩范数
    bool contact[4] = {false};
    float contact_confidence[4] = {0};
    float linear_velocity_world[3] = {0};
    float body_height = 0.0f;             // base 原点到足底接触平面的高度，m
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
    FootForceEstimator force_estimator_;
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
