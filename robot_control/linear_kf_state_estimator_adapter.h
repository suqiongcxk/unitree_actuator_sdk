#ifndef LINEAR_KF_STATE_ESTIMATOR_ADAPTER_H
#define LINEAR_KF_STATE_ESTIMATOR_ADAPTER_H

#include "state_estimator.h"

#ifdef LINEAR_KF_AVAILABLE
#include "linear_kf_position_velocity_estimator.h"

// 将独立 Eigen Linear KF 接入现有 StateEstimator 数据流。
// 前端继续负责 Step 1~5 校验、陀螺仪滤波和 Step 7 接触置信度。
class LinearKFStateEstimatorAdapter final : public StateEstimator {
public:
    EstimatedState update(const IMURawData& imu,const float* q,const float* dq,
        const float* tau,const int* error,const bool* joint_valid,
        const uint64_t* age,const uint32_t* failures,uint64_t now_ns) override;
private:
    ComplementaryStateEstimator frontend_;
    creeper::LinearKFPositionVelocityEstimator kf_;
};
#endif
#endif
