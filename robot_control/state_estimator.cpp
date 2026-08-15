#include "state_estimator.h"
#include <cmath>
#include <cstring>

namespace {
constexpr float kGravity = 9.80665f;
constexpr float kTwoPi = 6.2831853071795864769f;
}

PassthroughEstimator::PassthroughEstimator(uint64_t imu_timeout_ns)
    : PassthroughEstimator(imu_timeout_ns, GyroEstimatorConfig{})
{}

PassthroughEstimator::PassthroughEstimator(
    uint64_t imu_timeout_ns, const GyroEstimatorConfig& gyro_config)
    : imu_timeout_ns_(imu_timeout_ns), gyro_config_(gyro_config)
{}

void PassthroughEstimator::resetGyroCalibration()
{
    gyro_cal_samples_ = 0;
    std::memset(gyro_cal_mean_, 0, sizeof(gyro_cal_mean_));
    std::memset(gyro_cal_m2_, 0, sizeof(gyro_cal_m2_));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PassthroughEstimator — PLACEHOLDER
// ═══════════════════════════════════════════════════════════════════════════════
//
//  不做任何滤波，仅将输入数据直通拷贝到 EstimatedState。
//  后续替换为真正的 Kalman / Complementary Filter。

EstimatedState PassthroughEstimator::update(
    const IMURawData& imu,
    const float* joint_position,
    const float* joint_velocity,
    const float* joint_torque,
    const int*   joint_error,
    const bool*  joint_valid,
    const uint64_t* joint_age_ns,
    const uint32_t* joint_failure_count,
    uint64_t now_ns)
{
    EstimatedState est;

    est.timestamp_ns = now_ns;

    // 即使姿态无效，仍复制关节反馈供诊断使用，但禁止状态进入 NN。
    std::memcpy(est.joint_position, joint_position, 12 * sizeof(float));
    std::memcpy(est.joint_velocity, joint_velocity, 12 * sizeof(float));
    std::memcpy(est.joint_torque,   joint_torque,   12 * sizeof(float));
    std::memcpy(est.joint_error,    joint_error,    12 * sizeof(int));
    std::memcpy(est.joint_valid,    joint_valid,    12 * sizeof(bool));
    std::memcpy(est.joint_age_ns, joint_age_ns, 12 * sizeof(uint64_t));
    std::memcpy(est.joint_failure_count, joint_failure_count,
                12 * sizeof(uint32_t));
    for (int i = 0; i < 12; ++i)
        if (joint_age_ns[i] > est.max_joint_age_ns) est.max_joint_age_ns = joint_age_ns[i];

    auto fail = [&](EstimateStatus status) -> EstimatedState {
        est.status_code = static_cast<int>(status);
        est.consecutive_invalid_count = ++consecutive_invalid_count_;
        return est;
    };

    est.joint_feedback_valid = true;
    constexpr uint64_t motor_timeout_ns = 100'000'000ULL;
    bool motor_timed_out = false;
    for (int i = 0; i < 12; ++i) {
        const bool finite = std::isfinite(joint_position[i])
                         && std::isfinite(joint_velocity[i])
                         && std::isfinite(joint_torque[i]);
        if (!joint_valid[i] || joint_error[i] != 0 || !finite) {
            est.joint_valid[i] = false;
            est.joint_feedback_valid = false;
        }
        if (joint_age_ns[i] > motor_timeout_ns) {
            est.joint_valid[i] = false;
            est.joint_feedback_valid = false;
            motor_timed_out = true;
        }
    }

    if (!imu.valid) {
        return fail(EstimateStatus::IMU_READ_FAILED);
    }
    if (imu.timestamp_ns == 0 || imu.timestamp_ns > now_ns) {
        return fail(EstimateStatus::IMU_TIMESTAMP_INVALID);
    }

    est.imu_age_ns = now_ns - imu.timestamp_ns;
    if (est.imu_age_ns > imu_timeout_ns_) {
        return fail(EstimateStatus::IMU_TIMEOUT);
    }

    float q[4] = {imu.quat.q0, imu.quat.q1, imu.quat.q2, imu.quat.q3};
    for (float component : q) {
        if (!std::isfinite(component)) {
            return fail(EstimateStatus::QUATERNION_NONFINITE);
        }
    }

    const float norm_sq = q[0] * q[0] + q[1] * q[1]
                        + q[2] * q[2] + q[3] * q[3];
    const float norm = std::sqrt(norm_sq);
    est.quaternion_raw_norm = norm;
    // 合法单位四元数应接近 1；留出传输量化余量，同时拒绝零值和严重损坏数据。
    if (!std::isfinite(norm) || norm < 0.5f || norm > 1.5f) {
        return fail(EstimateStatus::QUATERNION_NORM_INVALID);
    }

    for (float& component : q) component /= norm;

    // q 与 -q 表示同一旋转；保持相邻帧同半球，避免观测发生无意义跳变。
    if (has_previous_quaternion_) {
        const float dot = q[0] * previous_quaternion_[0]
                        + q[1] * previous_quaternion_[1]
                        + q[2] * previous_quaternion_[2]
                        + q[3] * previous_quaternion_[3];
        if (dot < 0.0f) {
            for (float& component : q) component = -component;
        }
    }
    std::memcpy(previous_quaternion_, q, sizeof(q));
    has_previous_quaternion_ = true;

    // 四元数校验成功后立即保存；陀螺仪校准期间也可用于诊断姿态。
    est.orientation[0] = q[0];  // w
    est.orientation[1] = q[1];  // x
    est.orientation[2] = q[2];  // y
    est.orientation[3] = q[3];  // z
    est.orientation_valid = true;

    // JY901S 四元数按 body→world 使用；用 R^T 将世界系重力 [0,0,-1]
    // 旋转到机体系。水平时结果应为 [0,0,-1]，且不受纯 Yaw 影响。
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    est.projected_gravity[0] = -2.0f * (qx * qz - qw * qy);
    est.projected_gravity[1] = -2.0f * (qw * qx + qy * qz);
    est.projected_gravity[2] = -(qw * qw - qx * qx - qy * qy + qz * qz);
    const float gravity_norm = std::sqrt(
        est.projected_gravity[0] * est.projected_gravity[0]
      + est.projected_gravity[1] * est.projected_gravity[1]
      + est.projected_gravity[2] * est.projected_gravity[2]);
    if (!std::isfinite(gravity_norm) || gravity_norm < 0.99f || gravity_norm > 1.01f) {
        return fail(EstimateStatus::QUATERNION_NORM_INVALID);
    }
    for (float& component : est.projected_gravity) component /= gravity_norm;
    est.projected_gravity_valid = true;

    const float raw_gyro[3] = {imu.gyro.x, imu.gyro.y, imu.gyro.z};
    for (float component : raw_gyro) {
        if (!std::isfinite(component)) {
            return fail(EstimateStatus::GYRO_NONFINITE);
        }
        if (std::abs(component) > gyro_config_.max_abs_gyro) {
            return fail(EstimateStatus::GYRO_RANGE_INVALID);
        }
    }

    if (has_previous_raw_gyro_) {
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(raw_gyro[axis] - previous_raw_gyro_[axis])
                    > gyro_config_.max_gyro_jump) {
                return fail(EstimateStatus::GYRO_JUMP_INVALID);
            }
        }
    }
    std::memcpy(previous_raw_gyro_, raw_gyro, sizeof(raw_gyro));
    has_previous_raw_gyro_ = true;

    // JY901S 已做内部温漂补偿；这里只估计上电后的残余常值零偏。
    if (!gyro_calibrated_ && gyro_config_.calibration_samples > 0) {
        const float gyro_norm = std::sqrt(raw_gyro[0] * raw_gyro[0]
                                        + raw_gyro[1] * raw_gyro[1]
                                        + raw_gyro[2] * raw_gyro[2]);
        const float acc_norm = std::sqrt(imu.acc.x * imu.acc.x
                                       + imu.acc.y * imu.acc.y
                                       + imu.acc.z * imu.acc.z);
        const bool stationary = std::isfinite(acc_norm)
            && gyro_norm <= gyro_config_.stationary_gyro_threshold
            && std::abs(acc_norm - kGravity) <= gyro_config_.stationary_acc_tolerance;

        if (!stationary) {
            resetGyroCalibration();
        } else {
            ++gyro_cal_samples_;
            for (int axis = 0; axis < 3; ++axis) {
                const float delta = raw_gyro[axis] - gyro_cal_mean_[axis];
                gyro_cal_mean_[axis] += delta / static_cast<float>(gyro_cal_samples_);
                const float delta2 = raw_gyro[axis] - gyro_cal_mean_[axis];
                gyro_cal_m2_[axis] += delta * delta2;
            }

            if (gyro_cal_samples_ >= gyro_config_.calibration_samples) {
                bool stable = true;
                for (int axis = 0; axis < 3; ++axis) {
                    const float variance = gyro_cal_samples_ > 1
                        ? gyro_cal_m2_[axis] / static_cast<float>(gyro_cal_samples_ - 1)
                        : 0.0f;
                    stable = stable && std::sqrt(variance) <= gyro_config_.max_calibration_stddev;
                }
                if (stable) {
                    std::memcpy(gyro_bias_, gyro_cal_mean_, sizeof(gyro_bias_));
                    gyro_calibrated_ = true;
                    has_filtered_gyro_ = false;
                } else {
                    resetGyroCalibration();
                }
            }
        }
    } else if (gyro_config_.calibration_samples <= 0) {
        gyro_calibrated_ = true;
    }

    std::memcpy(est.gyro_bias, gyro_bias_, sizeof(gyro_bias_));
    est.gyro_calibrated = gyro_calibrated_;
    if (!gyro_calibrated_) {
        est.orientation_valid = true;
        return fail(EstimateStatus::GYRO_CALIBRATING);
    }

    const float corrected[3] = {
        raw_gyro[0] - gyro_bias_[0],
        raw_gyro[1] - gyro_bias_[1],
        raw_gyro[2] - gyro_bias_[2]
    };

    float dt = 1.0f / 50.0f;
    if (previous_gyro_timestamp_ns_ > 0 && imu.timestamp_ns > previous_gyro_timestamp_ns_)
        dt = static_cast<float>(imu.timestamp_ns - previous_gyro_timestamp_ns_) * 1e-9f;
    previous_gyro_timestamp_ns_ = imu.timestamp_ns;
    if (dt < 0.001f || dt > 0.1f) dt = 1.0f / 50.0f;

    const float alpha = 1.0f - std::exp(-kTwoPi * gyro_config_.lowpass_cutoff_hz * dt);
    for (int axis = 0; axis < 3; ++axis) {
        if (!has_filtered_gyro_) filtered_gyro_[axis] = corrected[axis];
        else filtered_gyro_[axis] += alpha * (corrected[axis] - filtered_gyro_[axis]);
    }
    has_filtered_gyro_ = true;

    // ── 机体状态 ──
    // x/y 无全局位置参考；z 在后面由支撑足高度约束给出。
    est.position[0] = 0.0f;
    est.position[1] = 0.0f;
    est.position[2] = 0.0f;

    // 角速度: JY901S 内部处理后，再减残余零偏并进行低延迟低通。
    est.angular_velocity[0] = filtered_gyro_[0];
    est.angular_velocity[1] = filtered_gyro_[1];
    est.angular_velocity[2] = filtered_gyro_[2];

    est.gyro_valid = true;
    if (motor_timed_out)
        return fail(EstimateStatus::MOTOR_FEEDBACK_TIMEOUT);
    if (!est.joint_feedback_valid) {
        return fail(EstimateStatus::JOINT_FEEDBACK_INVALID);
    }

    if (previous_estimation_timestamp_ns_ > now_ns) {
        previous_estimation_timestamp_ns_ = now_ns;
        return fail(EstimateStatus::ESTIMATION_TIMESTAMP_INVALID);
    }
    if (previous_estimation_timestamp_ns_ == 0) {
        est.dt_sec = 1.0f / 50.0f;
    } else {
        est.dt_sec = static_cast<float>(now_ns - previous_estimation_timestamp_ns_) * 1e-9f;
        if (est.dt_sec < 0.001f || est.dt_sec > 0.1f) {
            previous_estimation_timestamp_ns_ = now_ns;
            return fail(EstimateStatus::ESTIMATION_DT_INVALID);
        }
    }
    previous_estimation_timestamp_ns_ = now_ns;

    // Step 7：用 URDF 腿部运动学、电机力矩和 IMU 融合接触/线速度。
    // 无可靠支撑足时仍输出积分值，但置信度为 0，不伪装高可信状态。
    const float acc_body[3] = {imu.acc.x, imu.acc.y, imu.acc.z};
    const auto leg = legged_odometry_.update(
        est.joint_position, est.joint_velocity, est.joint_torque,
        est.orientation, est.angular_velocity, acc_body, est.dt_sec);
    if (leg.valid) {
        std::memcpy(est.foot_position, leg.foot_position, sizeof(est.foot_position));
        std::memcpy(est.foot_velocity, leg.foot_velocity, sizeof(est.foot_velocity));
        std::memcpy(est.contact, leg.contact, sizeof(est.contact));
        std::memcpy(est.contact_confidence, leg.contact_confidence,
                    sizeof(est.contact_confidence));
        std::memcpy(est.linear_velocity, leg.linear_velocity_world,
                    sizeof(est.linear_velocity));
        // world→body，使用归一化后的 body→world 四元数旋转矩阵转置。
        est.body_linear_velocity[0] =
            (1.0f-2.0f*(qy*qy+qz*qz))*est.linear_velocity[0]
          + 2.0f*(qx*qy+qw*qz)*est.linear_velocity[1]
          + 2.0f*(qx*qz-qw*qy)*est.linear_velocity[2];
        est.body_linear_velocity[1] =
            2.0f*(qx*qy-qw*qz)*est.linear_velocity[0]
          + (1.0f-2.0f*(qx*qx+qz*qz))*est.linear_velocity[1]
          + 2.0f*(qy*qz+qw*qx)*est.linear_velocity[2];
        est.body_linear_velocity[2] =
            2.0f*(qx*qz+qw*qy)*est.linear_velocity[0]
          + 2.0f*(qy*qz-qw*qx)*est.linear_velocity[1]
          + (1.0f-2.0f*(qx*qx+qy*qy))*est.linear_velocity[2];
        est.body_height = leg.body_height;
        est.position[2] = leg.body_height;
        est.linear_velocity_confidence = leg.velocity_confidence;
        est.leg_odometry_valid = true;
        est.airborne = leg.airborne;
        est.slipping = leg.slipping;
        est.landing_impact = leg.impact;
    }

    consecutive_invalid_count_ = 0;
    est.consecutive_invalid_count = 0;
    est.valid = true;
    est.status_code = static_cast<int>(EstimateStatus::OK);
    return est;
}
