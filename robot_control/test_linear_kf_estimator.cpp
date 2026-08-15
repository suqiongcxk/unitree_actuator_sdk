#include "linear_kf_position_velocity_estimator.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>

using creeper::EigenLegKinematics;
using creeper::LinearKFPositionVelocityEstimator;
namespace {
bool check(bool c,const char*m){if(!c)std::cerr<<"[FAIL] "<<m<<'\n';return c;}
LinearKFPositionVelocityEstimator::Input nominal(){
    LinearKFPositionVelocityEstimator::Input in;in.dt=.01;in.specific_force_body={0,0,9.80665};
    for(int l=0;l<4;++l){in.joint_position[l]=0;in.joint_position[4+l]=.8;in.joint_position[8+l]=-1.5;in.contact_confidence[l]=1;}
    return in;
}
void setFootRelativeVelocity(LinearKFPositionVelocityEstimator::Input&in,int leg,const Eigen::Vector3d&v){
    EigenLegKinematics kin;auto k=kin.compute(leg,in.joint_position,in.joint_velocity);
    Eigen::Vector3d qd=k.jacobian.fullPivLu().solve(v);in.joint_velocity[leg]=qd[0];in.joint_velocity[4+leg]=qd[1];in.joint_velocity[8+leg]=qd[2];
}
}
int main(){bool ok=true;
    {LinearKFPositionVelocityEstimator kf;auto in=nominal();LinearKFPositionVelocityEstimator::Output o;
     for(int n=0;n<300;++n)o=kf.update(in);ok&=check(o.valid&&o.world_velocity.norm()<.015,"静止站立速度应收敛到零");ok&=check(o.position_world.z()>.1&&o.position_world.z()<.6,"静止初始化高度应合理");}
    {LinearKFPositionVelocityEstimator kf;auto in=nominal();for(double&c:in.contact_confidence)c=0;LinearKFPositionVelocityEstimator::Output o;
     in.specific_force_body.x()=1;for(int n=0;n<100;++n)o=kf.update(in);in.specific_force_body.x()=0;double x0=o.position_world.x();for(int n=0;n<100;++n)o=kf.update(in);
     ok&=check(o.world_velocity.x()>.75&&o.world_velocity.x()<1.15,"腾空预测应保留约 1m/s 匀速");ok&=check(o.position_world.x()-x0>.7,"匀速阶段位置应沿 +X 增长");}
    {LinearKFPositionVelocityEstimator kf;auto in=nominal();for(double&c:in.contact_confidence)c=0;in.contact_confidence[0]=1;setFootRelativeVelocity(in,0,{-0.2,0,0});LinearKFPositionVelocityEstimator::Output o;
     for(int n=0;n<100;++n)o=kf.update(in);ok&=check(o.world_velocity.x()>.12&&o.world_velocity.x()<.27,"单脚支撑应反推出 +X 基座速度");}
    {LinearKFPositionVelocityEstimator kf;auto in=nominal();for(double&c:in.contact_confidence)c=0;in.contact_confidence[0]=in.contact_confidence[3]=1;setFootRelativeVelocity(in,0,{-0.15,0,0});setFootRelativeVelocity(in,3,{-0.15,0,0});LinearKFPositionVelocityEstimator::Output o;
     for(int n=0;n<100;++n)o=kf.update(in);ok&=check(o.world_velocity.x()>.09&&o.world_velocity.x()<.22,"对角支撑应得到一致速度约束");}
    {LinearKFPositionVelocityEstimator kf;auto in=nominal();LinearKFPositionVelocityEstimator::Output before,after;
     for(int n=0;n<30;++n)before=kf.update(in);double p0=before.covariance.trace();for(double&c:in.contact_confidence)c=0;for(int n=0;n<30;++n)after=kf.update(in);
     ok&=check(after.valid&&after.covariance.trace()>p0,"腾空时协方差应增长");}
    {LinearKFPositionVelocityEstimator kf;auto in=nominal();setFootRelativeVelocity(in,0,{-1.2,0,0});LinearKFPositionVelocityEstimator::Output o;
     for(int n=0;n<60;++n)o=kf.update(in);ok&=check(o.slip_suspected[0],"异常足端速度创新应标记打滑");ok&=check(std::abs(o.world_velocity.x())<.25,"打滑脚协方差放大后不应拖动多数支撑约束");}
    if(!ok)return 1;std::cout<<"[PASS] Independent 18-state Linear KF tests\n";return 0;
}
