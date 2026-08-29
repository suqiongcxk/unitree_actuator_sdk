#include "linear_kf_state_estimator_adapter.h"

#include <cmath>
#include <iostream>

int main(){
#ifndef LINEAR_KF_AVAILABLE
    return 0;
#else
    LinearKFStateEstimatorAdapter estimator;IMURawData imu{};
    imu.quat={1,0,0,0};imu.acc={0,0,9.80665f};imu.gyro={0,0,0};imu.valid=true;
    float q[12]={0},dq[12]={0},tau[12];int error[12]={0};bool valid[12];
    uint64_t age[12]={0};uint32_t failures[12]={0};
    for(int l=0;l<4;++l){q[l]=0;q[4+l]=.8f;q[8+l]=-1.5f;tau[l]=tau[4+l]=tau[8+l]=2;}
    for(bool&v:valid)v=true;EstimatedState state;
    uint64_t now=1'000'000'000ULL;
    for(int i=0;i<70;++i){imu.timestamp_ns=now;state=estimator.update(imu,q,dq,tau,error,valid,age,failures,now);now+=20'000'000ULL;}
    bool covariance_finite=true;for(int r=0;r<18;++r)for(int c=0;c<18;++c)covariance_finite&=std::isfinite(state.state_covariance[r][c]);
    if(!state.valid||state.estimator_backend!=1||!state.covariance_valid||!covariance_finite){
        std::cerr<<"[FAIL] Linear KF adapter did not publish valid KF state/covariance\n";return 1;
    }
    std::cout<<"[PASS] Linear KF StateEstimator adapter test\n";return 0;
#endif
}
