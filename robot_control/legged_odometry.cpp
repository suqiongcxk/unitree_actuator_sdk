#include "legged_odometry.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kGravity = 9.80665f;
float norm3(const float v[3]) { return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
bool finite3(const float v[3]) { return std::isfinite(v[0])&&std::isfinite(v[1])&&std::isfinite(v[2]); }
void cross3(const float a[3], const float b[3], float o[3]) {
    o[0]=a[1]*b[2]-a[2]*b[1]; o[1]=a[2]*b[0]-a[0]*b[2]; o[2]=a[0]*b[1]-a[1]*b[0];
}
void rotateBodyToWorld(const float q[4], const float v[3], float o[3]) {
    const float w=q[0],x=q[1],y=q[2],z=q[3];
    o[0]=(1-2*(y*y+z*z))*v[0]+2*(x*y-z*w)*v[1]+2*(x*z+y*w)*v[2];
    o[1]=2*(x*y+z*w)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-x*w)*v[2];
    o[2]=2*(x*z-y*w)*v[0]+2*(y*z+x*w)*v[1]+(1-2*(x*x+y*y))*v[2];
}
}

LeggedOdometry::LeggedOdometry(const LeggedOdometryConfig& config) : config_(config) {}

LeggedOdometryOutput LeggedOdometry::update(const float q[12], const float dq[12],
    const float tau[12], const float quat[4], const float omega[3], const float acc_body[3],
    float dt)
{
    LeggedOdometryOutput out;
    if (!q||!dq||!tau||!quat||!omega||!acc_body || !std::isfinite(dt)
        || dt<0.001f || dt>0.1f || !finite3(omega) || !finite3(acc_body)) return out;
    float min_z=1e9f;
    for (int leg=0;leg<4;++leg) {
        const auto k=kinematics_.compute(leg,q,dq);
        if (!k.valid) return out;
        for(int a=0;a<3;++a){out.foot_position[leg][a]=k.foot_position[a];out.foot_velocity[leg][a]=k.foot_velocity[a];}
        min_z=std::min(min_z,k.foot_position[2]);
    }

    for (int leg=0;leg<4;++leg) {
        const int ids[3]={leg,4+leg,8+leg};
        float torque_sq=0;
        for(int j=0;j<3;++j){if(!std::isfinite(tau[ids[j]]))return out; torque_sq+=tau[ids[j]]*tau[ids[j]];}
        const float torque=std::sqrt(torque_sq);
        const float speed=norm3(out.foot_velocity[leg]);
        const bool touchdown_evidence=torque>=config_.contact_torque_on
            && speed<=config_.max_contact_foot_speed;
        const bool liftoff_evidence=torque<config_.contact_torque_off;

        // 每只脚独立的接触状态机。足端高度不是硬门槛：高台/石块上的
        // 支撑足只要有持续承载证据，仍可进入 STANCE。
        switch(contact_phase_[leg]) {
        case ContactPhase::AIR:
            off_count_[leg]=0;
            if(touchdown_evidence){contact_phase_[leg]=ContactPhase::TOUCHDOWN;on_count_[leg]=1;}
            break;
        case ContactPhase::TOUCHDOWN:
            if(touchdown_evidence){
                if(++on_count_[leg]>=config_.contact_on_frames)contact_phase_[leg]=ContactPhase::STANCE;
            } else { contact_phase_[leg]=ContactPhase::AIR;on_count_[leg]=0; }
            break;
        case ContactPhase::STANCE:
            on_count_[leg]=0;
            if(liftoff_evidence){
                if(++off_count_[leg]>=config_.contact_off_frames){contact_phase_[leg]=ContactPhase::AIR;off_count_[leg]=0;}
            } else off_count_[leg]=0;
            break;
        }
        // on_frames=1 时首帧就允许进入 STANCE，便于诊断/单元测试。
        if(contact_phase_[leg]==ContactPhase::TOUCHDOWN && config_.contact_on_frames<=1)
            contact_phase_[leg]=ContactPhase::STANCE;
        const bool in_contact=contact_phase_[leg]==ContactPhase::STANCE;
        out.contact[leg]=in_contact;
        const float torque_span=std::max(0.01f,config_.contact_torque_on-config_.contact_torque_off);
        const float torque_conf=std::max(0.0f,std::min(1.0f,(torque-config_.contact_torque_off)/torque_span));
        // 高度只占较小权重：地形越高置信度可略降，但不会被误判为离地。
        const float excess_height=std::max(0.0f,out.foot_position[leg][2]-min_z);
        const float height_conf=std::max(0.0f,1.0f-excess_height/std::max(0.001f,4.0f*config_.contact_height_band));
        out.contact_confidence[leg]=in_contact?(0.8f*torque_conf+0.2f*height_conf):0.0f;
    }

    // JY901S 加速度是比力：世界系线加速度 = R*a_body + [0,0,-g]。
    float specific_world[3]; rotateBodyToWorld(quat,acc_body,specific_world);
    float linear_acc[3]={specific_world[0],specific_world[1],specific_world[2]-kGravity};
    for(int a=0;a<3;++a) velocity_world_[a]+=linear_acc[a]*dt;

    float candidates[4][3]={{0}}; int count=0;
    for(int leg=0;leg<4;++leg) if(out.contact[leg]) {
        float rotational[3]; cross3(omega,out.foot_position[leg],rotational);
        float body_candidate[3];
        for(int a=0;a<3;++a)body_candidate[a]=-(rotational[a]+out.foot_velocity[leg][a]);
        rotateBodyToWorld(quat,body_candidate,candidates[count]); ++count;
    }
    if(count>0){
        float mean[3]={0};
        for(int i=0;i<count;++i)for(int a=0;a<3;++a)mean[a]+=candidates[i][a]/count;
        float max_residual=0;
        for(int i=0;i<count;++i){float d[3];for(int a=0;a<3;++a)d[a]=candidates[i][a]-mean[a];max_residual=std::max(max_residual,norm3(d));}
        out.slipping=count>=2 && max_residual>config_.slip_residual_threshold;
        const float gain=out.slipping?0.05f:config_.velocity_correction_gain;
        for(int a=0;a<3;++a)velocity_world_[a]+=(mean[a]-velocity_world_[a])*gain;
        out.velocity_confidence=(out.slipping?0.2f:1.0f)*std::min(1.0f,count/2.0f);

        float height_sum=0; int height_count=0;
        for(int leg=0;leg<4;++leg)if(out.contact[leg]){
            float pw[3];rotateBodyToWorld(quat,out.foot_position[leg],pw);
            // foot_position 是 URDF 足球心；地面接触点还要沿世界 Z
            // 向下一个球半径，故 base 原点高度需要加 foot_radius。
            height_sum+=-pw[2]+config_.foot_radius;++height_count;
        }
        out.body_height=height_count?height_sum/height_count:0.0f;
    } else {
        for(float& v:velocity_world_)v*=config_.velocity_decay_no_contact;
    }
    out.airborne=count==0;
    out.impact=has_previous_acceleration_
        && std::abs(linear_acc[2]-previous_vertical_acceleration_)>15.0f && count>0;
    previous_vertical_acceleration_=linear_acc[2];has_previous_acceleration_=true;
    for(int a=0;a<3;++a)out.linear_velocity_world[a]=velocity_world_[a];
    out.valid=finite3(out.linear_velocity_world)&&std::isfinite(out.body_height);
    return out;
}
