#ifndef ROBOT_CONTROL_ROBOT_MODEL_PARAMETERS_H
#define ROBOT_CONTROL_ROBOT_MODEL_PARAMETERS_H

// Creeper20260906.urdf 的受控端共享模型常量。
// 坐标系：+X向前、+Y向左、+Z向上；长度m，质量kg。
namespace creeper_model {

inline constexpr const char* kUrdfVersion = "Creeper20260906";

inline constexpr double kHipOriginX = 0.1366;
inline constexpr double kHipOriginY = 0.0650;
inline constexpr double kHipToThighX = 0.06825;
inline constexpr double kHipToThighY = 0.0186;
inline constexpr double kThighToCalfY = 0.0572;
inline constexpr double kThighToCalfZ = -0.2100;
inline constexpr double kCalfToFootZ = -0.2080;
inline constexpr double kFootRadius = 0.0200;

// 当前实机估计器不做刚体动力学预测；这些质量参数用于
// 版本对齐、仿真与后续动力学模块，不直接改变现有电机指令。
inline constexpr double kBaseMassKg = 5.08740163863498;
inline constexpr double kHipMassKg = 0.60205;
inline constexpr double kFrontThighMassKg = 0.87467;
inline constexpr double kRearThighMassKg = 0.87728;
inline constexpr double kCalfMassKg = 0.23193;
inline constexpr double kFootMassKg = 0.001;
inline constexpr double kImuMassKg = 0.001;
inline constexpr double kTotalUrdfMassKg =
    kBaseMassKg + 4.0 * kHipMassKg
    + 2.0 * kFrontThighMassKg + 2.0 * kRearThighMassKg
    + 4.0 * kCalfMassKg + 4.0 * kFootMassKg + kImuMassKg;

}  // namespace creeper_model

#endif  // ROBOT_CONTROL_ROBOT_MODEL_PARAMETERS_H
