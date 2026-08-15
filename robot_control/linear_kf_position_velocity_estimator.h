#ifndef LINEAR_KF_POSITION_VELOCITY_ESTIMATOR_H
#define LINEAR_KF_POSITION_VELOCITY_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

namespace creeper {

// 独立 Creeper20260714 腿部运动学。坐标：X前、Y左、Z上；单位 m/rad/s。
class EigenLegKinematics {
public:
    struct Result {
        Eigen::Vector3d position_body = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity_body = Eigen::Vector3d::Zero();
        Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
        bool valid = false;
    };
    Result compute(int leg, const Eigen::Matrix<double,12,1>& q,
                   const Eigen::Matrix<double,12,1>& dq) const;
};

class LinearKFPositionVelocityEstimator {
public:
    static constexpr int kStateDim = 18;
    static constexpr int kLegCount = 4;
    using StateVector = Eigen::Matrix<double,kStateDim,1>;
    using Covariance = Eigen::Matrix<double,kStateDim,kStateDim>;

    struct Config {
        double gravity = 9.80665;
        double position_process_noise = 2e-5;
        double velocity_process_noise = 2e-3;
        double foot_process_noise = 1e-4;
        double foot_position_noise = 2e-4;
        double zero_velocity_noise = 3e-3;
        double low_confidence_multiplier = 100.0;
        double trusted_contact_threshold = 0.5;
        double slip_velocity_innovation = 0.45; // m/s
        double min_dt = 1e-4;
        double max_dt = 0.1;
    };

    struct Input {
        // JY901S 约定：body→world，排列 w,x,y,z，必须可归一化。
        Eigen::Quaterniond orientation_body_to_world = Eigen::Quaterniond::Identity();
        // IMU 比力（含静止时 +g），body frame，m/s²。
        Eigen::Vector3d specific_force_body = Eigen::Vector3d::Zero();
        // motor-ID 排列：Hip[0..3], Thigh[4..7], Calf[8..11]。
        Eigen::Matrix<double,12,1> joint_position = Eigen::Matrix<double,12,1>::Zero();
        Eigen::Matrix<double,12,1> joint_velocity = Eigen::Matrix<double,12,1>::Zero();
        std::array<double,4> contact_confidence{{0,0,0,0}};
        double dt = 0.0;
    };

    struct Output {
        Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d body_velocity = Eigen::Vector3d::Zero();
        Covariance covariance = Covariance::Identity();
        std::array<bool,4> slip_suspected{{false,false,false,false}};
        bool valid = false;
    };

    LinearKFPositionVelocityEstimator();
    explicit LinearKFPositionVelocityEstimator(const Config& config);
    void reset(const Eigen::Vector3d& initial_position_world = Eigen::Vector3d::Zero());
    Output update(const Input& input);
    const StateVector& state() const noexcept { return x_; }
    const Covariance& covariance() const noexcept { return P_; }

private:
    bool validateInput(const Input& input) const;
    Eigen::Vector3d estimateAngularVelocityBody(
        const Eigen::Quaterniond& current, double dt) const;

    Config config_;
    EigenLegKinematics kinematics_;
    StateVector x_ = StateVector::Zero();
    Covariance P_ = Covariance::Identity();
    Eigen::Quaterniond previous_orientation_ = Eigen::Quaterniond::Identity();
    bool initialized_ = false;
    bool has_previous_orientation_ = false;
};

} // namespace creeper
#endif
