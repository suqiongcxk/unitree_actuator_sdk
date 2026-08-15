#include "linear_kf_state_estimator_adapter.h"

#ifdef LINEAR_KF_AVAILABLE
#include <cstring>

EstimatedState LinearKFStateEstimatorAdapter::update(
    const IMURawData& imu,const float* q,const float* dq,const float* tau,
    const int* error,const bool* joint_valid,const uint64_t* age,
    const uint32_t* failures,uint64_t now_ns) {
    EstimatedState est=frontend_.update(
        imu,q,dq,tau,error,joint_valid,age,failures,now_ns);
    est.estimator_backend=1;
    if(!est.valid)return est;

    creeper::LinearKFPositionVelocityEstimator::Input in;
    in.orientation_body_to_world=Eigen::Quaterniond(
        est.orientation[0],est.orientation[1],est.orientation[2],est.orientation[3]);
    in.specific_force_body=Eigen::Vector3d(imu.acc.x,imu.acc.y,imu.acc.z);
    for(int i=0;i<12;++i){in.joint_position[i]=est.joint_position[i];in.joint_velocity[i]=est.joint_velocity[i];}
    for(int i=0;i<4;++i)in.contact_confidence[i]=est.contact_confidence[i];
    in.dt=est.dt_sec;
    const auto out=kf_.update(in);
    if(!out.valid){
        est.valid=false;est.status_code=static_cast<int>(EstimateStatus::LINEAR_KF_INVALID);
        return est;
    }
    for(int i=0;i<3;++i){
        est.position[i]=static_cast<float>(out.position_world[i]);
        est.linear_velocity[i]=static_cast<float>(out.world_velocity[i]);
        est.body_linear_velocity[i]=static_cast<float>(out.body_velocity[i]);
    }
    for(int r=0;r<18;++r)for(int c=0;c<18;++c)
        est.state_covariance[r][c]=static_cast<float>(out.covariance(r,c));
    est.covariance_valid=true;
    for(int i=0;i<4;++i)est.slipping=est.slipping||out.slip_suspected[i];
    return est;
}
#endif
