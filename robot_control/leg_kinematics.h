#ifndef ROBOT_CONTROL_LEG_KINEMATICS_H
#define ROBOT_CONTROL_LEG_KINEMATICS_H

// Creeper20260714 URDF 的四腿运动学。腿序固定为 FL, FR, RL, RR，
// 输入关节数组则使用控制程序的 motor-ID 排列。
class CreeperLegKinematics {
public:
    static constexpr int kLegCount = 4;

    struct Result {
        float foot_position[3] = {0};       // 足端相对 base，base 坐标系，m
        float foot_velocity[3] = {0};       // 关节运动产生的相对速度，m/s
        float jacobian[3][3] = {{0}};       // [vx,vy,vz] / [hip,thigh,calf]
        bool valid = false;
    };

    Result compute(int leg, const float joint_position[12],
                   const float joint_velocity[12]) const;
};

#endif
