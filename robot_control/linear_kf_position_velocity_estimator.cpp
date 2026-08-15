#include "linear_kf_position_velocity_estimator.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

namespace creeper {
namespace {
Eigen::Matrix3d rx(double q){double c=std::cos(q),s=std::sin(q);Eigen::Matrix3d R;R<<1,0,0,0,c,-s,0,s,c;return R;}
Eigen::Matrix3d ry(double q){double c=std::cos(q),s=std::sin(q);Eigen::Matrix3d R;R<<c,0,s,0,1,0,-s,0,c;return R;}
bool finite(const Eigen::MatrixXd& v){return v.array().isFinite().all();}
double clamp01(double x){return std::max(0.0,std::min(1.0,x));}
}

EigenLegKinematics::Result EigenLegKinematics::compute(
    int leg,const Eigen::Matrix<double,12,1>& q,const Eigen::Matrix<double,12,1>& dq) const {
    Result o;if(leg<0||leg>=4||!finite(q)||!finite(dq))return o;
    int id[3]={leg,4+leg,8+leg};double front=leg<2?1.0:-1.0,left=(leg==0||leg==2)?1.0:-1.0;
    Eigen::Vector3d h(front*.1366,left*.065,0),ht(front*.05825,left*.0186,0),tc(0,left*.0572,-.210),cf(0,0,-.208);
    Eigen::Matrix3d Rh=rx(q[id[0]]),Rt=ry(q[id[1]]),Rc=ry(q[id[2]]);
    Eigen::Vector3d p0=h,p1=p0+Rh*ht,p2=p1+Rh*Rt*tc,p3=p2+Rh*Rt*Rc*cf;
    Eigen::Vector3d axes[3]={Eigen::Vector3d::UnitX(),Rh*Eigen::Vector3d::UnitY(),Rh*Rt*Eigen::Vector3d::UnitY()};
    Eigen::Vector3d origins[3]={p0,p1,p2};
    for(int j=0;j<3;++j)o.jacobian.col(j)=axes[j].cross(p3-origins[j]);
    Eigen::Vector3d qd(dq[id[0]],dq[id[1]],dq[id[2]]);
    o.position_body=p3;o.velocity_body=o.jacobian*qd;o.valid=finite(o.position_body)&&finite(o.velocity_body);return o;
}

LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator()
    : LinearKFPositionVelocityEstimator(Config{}) {}
LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator(const Config& c):config_(c){reset();}
void LinearKFPositionVelocityEstimator::reset(const Eigen::Vector3d& p){
    x_.setZero();x_.segment<3>(0)=p;P_.setIdentity();P_*=1.0;initialized_=false;has_previous_orientation_=false;
}
bool LinearKFPositionVelocityEstimator::validateInput(const Input& in) const {
    double n=in.orientation_body_to_world.norm();if(!std::isfinite(n)||n<.5||n>1.5)return false;
    if(!finite(in.specific_force_body)||!finite(in.joint_position)||!finite(in.joint_velocity)||!std::isfinite(in.dt)||in.dt<config_.min_dt||in.dt>config_.max_dt)return false;
    for(double c:in.contact_confidence)if(!std::isfinite(c))return false;return true;
}
Eigen::Vector3d LinearKFPositionVelocityEstimator::estimateAngularVelocityBody(const Eigen::Quaterniond& qc,double dt) const {
    if(!has_previous_orientation_)return Eigen::Vector3d::Zero();
    Eigen::Quaterniond dq=previous_orientation_.conjugate()*qc;if(dq.w()<0)dq.coeffs()*=-1;
    Eigen::AngleAxisd aa(dq.normalized());if(!std::isfinite(aa.angle())||aa.angle()<1e-9)return Eigen::Vector3d::Zero();
    return aa.axis()*(aa.angle()/dt);
}

LinearKFPositionVelocityEstimator::Output LinearKFPositionVelocityEstimator::update(const Input& in){
    Output out;if(!validateInput(in))return out;Eigen::Quaterniond quat=in.orientation_body_to_world.normalized();
    Eigen::Matrix3d Rbw=quat.toRotationMatrix();Eigen::Vector3d omega=estimateAngularVelocityBody(quat,in.dt);
    std::array<EigenLegKinematics::Result,4> legs;for(int i=0;i<4;++i){legs[i]=kinematics_.compute(i,in.joint_position,in.joint_velocity);if(!legs[i].valid)return out;}
    if(!initialized_){
        double zsum=0,wsum=0;for(int i=0;i<4;++i){double w=clamp01(in.contact_confidence[i]);zsum+=w*(Rbw*legs[i].position_body).z();wsum+=w;}
        if(wsum>0)x_.z()=-zsum/wsum;
        for(int i=0;i<4;++i)x_.segment<3>(6+3*i)=x_.head<3>()+Rbw*legs[i].position_body;
        initialized_=true;
    }

    Eigen::Matrix<double,18,18>A=Eigen::Matrix<double,18,18>::Identity();A.block<3,3>(0,3)=in.dt*Eigen::Matrix3d::Identity();
    Eigen::Vector3d aw=Rbw*in.specific_force_body+Eigen::Vector3d(0,0,-config_.gravity);
    x_.head<3>()+=x_.segment<3>(3)*in.dt+.5*aw*in.dt*in.dt;x_.segment<3>(3)+=aw*in.dt;
    Eigen::Matrix<double,18,18>Q=Eigen::Matrix<double,18,18>::Zero();
    Q.block<3,3>(0,0).diagonal().setConstant(config_.position_process_noise*in.dt);
    Q.block<3,3>(3,3).diagonal().setConstant(config_.velocity_process_noise*in.dt);
    for(int i=0;i<4;++i){double c=clamp01(in.contact_confidence[i]);double scale=1+(1-c)*config_.low_confidence_multiplier;Q.block<3,3>(6+3*i,6+3*i).diagonal().setConstant(config_.foot_process_noise*in.dt*scale);}
    P_=A*P_*A.transpose()+Q;

    constexpr int M=24;Eigen::Matrix<double,M,18>H=Eigen::Matrix<double,M,18>::Zero();Eigen::Matrix<double,M,1>z;Eigen::Matrix<double,M,M>Rm=Eigen::Matrix<double,M,M>::Zero();
    for(int i=0;i<4;++i){int rp=3*i,rv=12+3*i;Eigen::Vector3d relw=Rbw*legs[i].position_body;
        H.block<3,3>(rp,0)=-Eigen::Matrix3d::Identity();H.block<3,3>(rp,6+3*i)=Eigen::Matrix3d::Identity();z.segment<3>(rp)=relw;
        H.block<3,3>(rv,3)=Eigen::Matrix3d::Identity();
        Eigen::Vector3d velocity_candidate=-Rbw*(omega.cross(legs[i].position_body)+legs[i].velocity_body);
        double c=clamp01(in.contact_confidence[i]);bool trusted=c>=config_.trusted_contact_threshold;
        Eigen::Vector3d vel_innovation=velocity_candidate-x_.segment<3>(3);bool slip=trusted&&vel_innovation.norm()>config_.slip_velocity_innovation;out.slip_suspected[i]=slip;
        Eigen::Vector3d predicted_relative=x_.segment<3>(6+3*i)-x_.head<3>();
        if(slip) z.segment<3>(rp)=predicted_relative;
        else z.segment<3>(rp)=(1.0-c)*predicted_relative+c*relw;
        // 低接触置信度时，观测同时向预测速度退化，避免腾空腿的
        // “伪零速”在长时间内缓慢拖停 IMU 积分；打滑腿则完全使用预测值。
        if(slip) z.segment<3>(rv)=x_.segment<3>(3);
        else z.segment<3>(rv)=(1.0-c)*x_.segment<3>(3)+c*velocity_candidate;
        double scale=(!trusted||slip)?config_.low_confidence_multiplier:1.0;
        Rm.block<3,3>(rp,rp).diagonal().setConstant(config_.foot_position_noise*scale);
        Rm.block<3,3>(rv,rv).diagonal().setConstant(config_.zero_velocity_noise*scale);
    }
    Eigen::Matrix<double,M,M>S=H*P_*H.transpose()+Rm;Eigen::LDLT<Eigen::Matrix<double,M,M>>ldlt(S);if(ldlt.info()!=Eigen::Success)return out;
    Eigen::Matrix<double,18,M>K=P_*H.transpose()*ldlt.solve(Eigen::Matrix<double,M,M>::Identity());x_+=K*(z-H*x_);
    Eigen::Matrix<double,18,18>I=Eigen::Matrix<double,18,18>::Identity(),IKH=I-K*H;P_=IKH*P_*IKH.transpose()+K*Rm*K.transpose();P_=(P_+P_.transpose())*.5;
    previous_orientation_=quat;has_previous_orientation_=true;
    out.position_world=x_.head<3>();out.world_velocity=x_.segment<3>(3);out.body_velocity=Rbw.transpose()*out.world_velocity;out.covariance=P_;out.valid=finite(x_)&&finite(P_);return out;
}
} // namespace creeper
