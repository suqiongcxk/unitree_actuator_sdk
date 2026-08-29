#ifndef ROBOT_CONTROL_FOOT_FORCE_ESTIMATOR_H
#define ROBOT_CONTROL_FOOT_FORCE_ESTIMATOR_H

struct FootForceEstimatorConfig {
    // DLS: F = -(J*J^T + lambda^2 I)^-1 * J*tau。
    float damping = 1.0e-3f;                  // m
    float min_normalized_determinant = 1.0e-3f;
    float max_force_norm = 500.0f;            // N
    float max_torque_residual = 0.10f;        // N·m
};

struct FootForceEstimate {
    // 地面对足端的外力，base坐标系：+X前、+Y左、+Z上，单位N。
    float force_body[3] = {0.0f, 0.0f, 0.0f};
    float normal_force = 0.0f;                // max(0, Fz)，N
    float force_norm = 0.0f;                  // N
    float torque_residual = 0.0f;             // ||J^T F + tau||，N·m
    float normalized_determinant = 0.0f;      // 0..1，越小越接近奇异
    bool valid = false;
};

class FootForceEstimator {
public:
    explicit FootForceEstimator(const FootForceEstimatorConfig& config = {});

    /// 由URDF关节力矩估算地面对足端的三维力。
    FootForceEstimate estimate(const float jacobian[3][3],
                               const float joint_torque[3]) const;

private:
    FootForceEstimatorConfig config_;
};

#endif
