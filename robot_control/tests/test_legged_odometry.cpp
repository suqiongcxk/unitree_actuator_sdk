#include "leg_kinematics.h"
#include "legged_odometry.h"

#include <cmath>
#include <iostream>

namespace {
bool expect(bool c,const char* m){if(!c)std::cerr<<"[FAIL] "<<m<<std::endl;return c;}
float distance3(const float a[3],const float b[3]){
    float s=0;for(int i=0;i<3;++i){float d=a[i]-b[i];s+=d*d;}return std::sqrt(s);
}
bool solve3(float a[3][3],const float b[3],float x[3]){
    float m[3][4];for(int r=0;r<3;++r){for(int c=0;c<3;++c)m[r][c]=a[r][c];m[r][3]=b[r];}
    for(int c=0;c<3;++c){int p=c;for(int r=c+1;r<3;++r)if(std::abs(m[r][c])>std::abs(m[p][c]))p=r;
        if(std::abs(m[p][c])<1e-5f)return false;for(int k=c;k<4;++k)std::swap(m[c][k],m[p][k]);
        float d=m[c][c];for(int k=c;k<4;++k)m[c][k]/=d;
        for(int r=0;r<3;++r)if(r!=c){float f=m[r][c];for(int k=c;k<4;++k)m[r][k]-=f*m[c][k];}}
    for(int i=0;i<3;++i)x[i]=m[i][3];return true;
}
void setGroundForceTorques(CreeperLegKinematics& kin,const float q[12],
                           float normal_force,float tau[12],
                           float gear_ratio=6.333f){
    for(int leg=0;leg<4;++leg){
        float zero_dq[12]={0};const auto k=kin.compute(leg,q,zero_dq);
        const int ids[3]={leg,4+leg,8+leg};
        for(int joint=0;joint<3;++joint)
            // 静力平衡约定：tau + J^T F_ground = 0。
            tau[ids[joint]]=-k.jacobian[2][joint]*normal_force/gear_ratio;
    }
}
}

int main(){
    bool ok=true; CreeperLegKinematics kin;
    float q[12]={0},dq[12]={0},tau[12]={0};
    for(int l=0;l<4;++l){q[l]=0.0f;q[4+l]=0.8f;q[8+l]=-1.5f;}
    setGroundForceTorques(kin,q,20.0f,tau);

    // 雅可比必须与正运动学有限差分一致。
    auto base=kin.compute(0,q,dq); ok&=expect(base.valid,"FL 正运动学应有效");
    const float eps=1e-4f;
    for(int j=0;j<3;++j){int id[3]={0,4,8};q[id[j]]+=eps;auto moved=kin.compute(0,q,dq);q[id[j]]-=eps;
        for(int a=0;a<3;++a){float fd=(moved.foot_position[a]-base.foot_position[a])/eps;
            ok&=expect(std::abs(fd-base.jacobian[a][j])<8e-4f,"雅可比应与有限差分一致");}}

    LeggedOdometryConfig cfg;cfg.contact_on_frames=1;cfg.contact_off_frames=1;cfg.velocity_correction_gain=1.0f;
    LeggedOdometry odom(cfg);const float quat[4]={1,0,0,0},omega[3]={0,0,0},acc[3]={0,0,9.80665f};
    auto standing=odom.update(q,dq,tau,quat,omega,acc,0.02f);
    ok&=expect(standing.valid&&!standing.airborne,"静止承载时应检测到支撑足");
    ok&=expect(std::abs(standing.linear_velocity_world[0])<1e-5f&&
        std::abs(standing.linear_velocity_world[1])<1e-5f&&std::abs(standing.linear_velocity_world[2])<1e-5f,
        "静止站立速度应接近零");
    ok&=expect(standing.body_height>0.1f&&standing.body_height<0.6f,"站立高度应处于合理范围");

    // body_height 的语义是 base 原点到足底接触平面，而不是到足球心。
    float center_height=0.0f;
    for(int leg=0;leg<4;++leg)center_height+=-standing.foot_position[leg][2]/4.0f;
    ok&=expect(std::abs(standing.body_height-(center_height+cfg.foot_radius))<1e-6f,
        "站立高度必须包含 URDF 足球半径");

    // 足球半径只修正高度语义，不得改变接触、足端位置或速度估计。
    LeggedOdometryConfig zero_radius_cfg=cfg;zero_radius_cfg.foot_radius=0.0f;
    LeggedOdometry zero_radius_odom(zero_radius_cfg);
    auto zero_radius=zero_radius_odom.update(q,dq,tau,quat,omega,acc,0.02f);
    ok&=expect(std::abs(standing.body_height-zero_radius.body_height-cfg.foot_radius)<1e-6f,
        "足球半径应只给 body_height 增加 0.02 m");
    for(int leg=0;leg<4;++leg){
        ok&=expect(standing.contact[leg]==zero_radius.contact[leg],
            "足球半径不得改变接触状态");
        ok&=expect(distance3(standing.foot_position[leg],zero_radius.foot_position[leg])<1e-7f,
            "足球半径不得改变足端运动学位置");
    }
    ok&=expect(distance3(standing.linear_velocity_world,zero_radius.linear_velocity_world)<1e-7f,
        "足球半径不得改变速度估计");

    // 崎岖地面：一只脚明显高于最低脚时，持续承载不得被高度硬门槛否决。
    float rough_q[12];for(int i=0;i<12;++i)rough_q[i]=q[i];rough_q[4]=0.15f;
    float rough_tau[12]={0};setGroundForceTorques(kin,rough_q,20.0f,rough_tau);
    LeggedOdometry rough_odom(cfg);
    auto rough=rough_odom.update(rough_q,dq,rough_tau,quat,omega,acc,0.02f);
    float min_foot_z=rough.foot_position[0][2],max_foot_z=min_foot_z;
    for(int l=1;l<4;++l){min_foot_z=std::min(min_foot_z,rough.foot_position[l][2]);max_foot_z=std::max(max_foot_z,rough.foot_position[l][2]);}
    ok&=expect(max_foot_z-min_foot_z>cfg.contact_height_band,"测试姿态应构造超过高度带的落差");
    ok&=expect(rough.contact[0],"高地形上的承载足不得被高度条件否决");

    // 为每条支撑腿求 J*dq=[-0.2,0,0]，零足端世界速度约束应反推机体 +X 移动。
    for(int l=0;l<4;++l){auto k=kin.compute(l,q,dq);float rhs[3]={-0.2f,0,0},sol[3];
        ok&=expect(solve3(k.jacobian,rhs,sol),"默认姿态雅可比不应奇异");
        dq[l]=sol[0];dq[4+l]=sol[1];dq[8+l]=sol[2];}
    auto moving=odom.update(q,dq,tau,quat,omega,acc,0.02f);
    ok&=expect(moving.linear_velocity_world[0]>0.18f&&moving.linear_velocity_world[0]<0.22f,
        "支撑足约束应正确反推 +X 机体速度");
    ok&=expect(!moving.slipping&&moving.velocity_confidence>0.9f,"一致的多足约束应为高置信度");

    // 仅 FL 给出与其他三足显著不一致的速度约束，必须判定打滑并降低置信度。
    {
        auto k=kin.compute(0,q,dq);float rhs[3]={-1.0f,0,0},sol[3];
        ok&=expect(solve3(k.jacobian,rhs,sol),"FL 打滑测试姿态雅可比不应奇异");
        dq[0]=sol[0];dq[4]=sol[1];dq[8]=sol[2];
    }
    auto slipping=odom.update(q,dq,tau,quat,omega,acc,0.02f);
    ok&=expect(slipping.slipping,"单腿速度约束与其他支撑腿显著不一致时应判定打滑");
    ok&=expect(slipping.velocity_confidence>0.19f&&slipping.velocity_confidence<0.21f,
        "打滑时速度置信度应降为 0.2");

    // 失去所有力矩支撑证据时必须标记腾空，速度置信度归零。
    for(float& t:tau)t=0;auto airborne=odom.update(q,dq,tau,quat,omega,acc,0.02f);
    ok&=expect(airborne.airborne&&airborne.velocity_confidence==0.0f,"无支撑证据时应降为腾空/低置信度");

    // 4477帧离线标定回归：用法向支撑力完成触地→离地→重新触地。
    LeggedOdometry hysteresis_odom;
    float loaded_tau[12]={0}, unloaded_tau[12]={0};
    setGroundForceTorques(kin,q,10.0f,loaded_tau);
    setGroundForceTorques(kin,q,1.2f,unloaded_tau);
    LeggedOdometryOutput hysteresis_state;
    for(int frame=0;frame<3;++frame)
        hysteresis_state=hysteresis_odom.update(q,dq,loaded_tau,quat,omega,acc,0.02f);
    ok&=expect(!hysteresis_state.airborne&&hysteresis_state.contact[0]
        &&hysteresis_state.contact[1]&&hysteresis_state.contact[2]&&hysteresis_state.contact[3],
        "低载荷持续三帧后四足必须判定触地");

    // 已触地后法向力落在迟滞区间内仍保持接触，覆盖不平地面轻载腿。
    float light_contact_tau[12]={0};
    setGroundForceTorques(kin,q,5.0f,light_contact_tau);
    for(int frame=0;frame<10;++frame)
        hysteresis_state=hysteresis_odom.update(q,dq,light_contact_tau,quat,omega,acc,0.02f);
    ok&=expect(!hysteresis_state.airborne&&hysteresis_state.contact[0]
        &&hysteresis_state.contact[1]&&hysteresis_state.contact[2]&&hysteresis_state.contact[3],
        "不平地面轻载支撑足不得被长期误判离地");

    for(int frame=0;frame<2;++frame)
        hysteresis_state=hysteresis_odom.update(q,dq,unloaded_tau,quat,omega,acc,0.02f);
    ok&=expect(hysteresis_state.airborne,"低力矩持续两帧后必须判定完全离地");
    for(int frame=0;frame<3;++frame)
        hysteresis_state=hysteresis_odom.update(q,dq,loaded_tau,quat,omega,acc,0.02f);
    ok&=expect(!hysteresis_state.airborne&&hysteresis_state.contact[0]
        &&hysteresis_state.contact[1]&&hysteresis_state.contact[2]&&hysteresis_state.contact[3],
        "重新承载三帧后四足必须恢复触地");

    for(int leg=0;leg<4;++leg){
        ok&=expect(hysteresis_state.foot_force_valid[leg],"默认姿态足端力求解应有效");
        ok&=expect(hysteresis_state.contact_used_force[leg],"接触检测应优先使用Fz");
        ok&=expect(std::abs(hysteresis_state.normal_force[leg]-10.0f)<0.05f,
            "估算Fz应恢复构造的法向力");
    }

    if(!ok)return 1;std::cout<<"[PASS] Step 7 leg kinematics and odometry tests"<<std::endl;return 0;
}
