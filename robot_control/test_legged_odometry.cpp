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
}

int main(){
    bool ok=true; CreeperLegKinematics kin;
    float q[12]={0},dq[12]={0},tau[12];
    for(int l=0;l<4;++l){q[l]=0.0f;q[4+l]=0.8f;q[8+l]=-1.5f; tau[l]=tau[4+l]=tau[8+l]=2.0f;}

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

    // 崎岖地面：一只脚明显高于最低脚时，持续承载不得被高度硬门槛否决。
    float rough_q[12];for(int i=0;i<12;++i)rough_q[i]=q[i];rough_q[4]=0.15f;
    LeggedOdometry rough_odom(cfg);
    auto rough=rough_odom.update(rough_q,dq,tau,quat,omega,acc,0.02f);
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

    // 失去所有力矩支撑证据时必须标记腾空，速度置信度归零。
    for(float& t:tau)t=0;auto airborne=odom.update(q,dq,tau,quat,omega,acc,0.02f);
    ok&=expect(airborne.airborne&&airborne.velocity_confidence==0.0f,"无支撑证据时应降为腾空/低置信度");

    if(!ok)return 1;std::cout<<"[PASS] Step 7 leg kinematics and odometry tests"<<std::endl;return 0;
}
