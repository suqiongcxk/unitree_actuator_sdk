#include "leg_kinematics.h"

#include <cmath>

namespace {
struct Vec3 { float x, y, z; };

Vec3 add(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vec3 mul(Vec3 a, float s) { return {a.x*s, a.y*s, a.z*s}; }
Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}

Vec3 rx(Vec3 v, float q) {
    const float c=std::cos(q), s=std::sin(q);
    return {v.x, c*v.y-s*v.z, s*v.y+c*v.z};
}

Vec3 ry(Vec3 v, float q) {
    const float c=std::cos(q), s=std::sin(q);
    return {c*v.x+s*v.z, v.y, -s*v.x+c*v.z};
}

bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
}

CreeperLegKinematics::Result CreeperLegKinematics::compute(
    int leg, const float q[12], const float dq[12]) const
{
    Result out;
    if (leg < 0 || leg >= kLegCount || !q || !dq) return out;
    const int ids[3] = {leg, 4 + leg, 8 + leg};
    for (int id : ids)
        if (!std::isfinite(q[id]) || !std::isfinite(dq[id])) return out;

    const float front = leg < 2 ? 1.0f : -1.0f;
    const float left = (leg == 0 || leg == 2) ? 1.0f : -1.0f;
    // 以 URDF joint origin 为唯一几何来源，不使用 STL 外观尺寸。
    const Vec3 hip_o{front*0.1366f, left*0.0650f, 0.0f};
    const Vec3 hip_to_thigh{front*0.05825f, left*0.0186f, 0.0f};
    const Vec3 thigh_to_calf{0.0f, left*0.0572f, -0.2100f};
    const Vec3 calf_to_foot{0.0f, 0.0f, -0.2080f};

    const float qh=q[ids[0]], qt=q[ids[1]], qc=q[ids[2]];
    const Vec3 p_hip=hip_o;
    const Vec3 p_thigh=add(p_hip, rx(hip_to_thigh, qh));
    const Vec3 p_calf=add(p_thigh, rx(ry(thigh_to_calf, qt), qh));
    const Vec3 p_foot=add(p_calf, rx(ry(ry(calf_to_foot, qc), qt), qh));

    const Vec3 axis_hip{1,0,0};
    const Vec3 axis_thigh=rx({0,1,0}, qh);
    const Vec3 axis_calf=rx(ry({0,1,0}, qt), qh);
    const Vec3 cols[3] = {
        cross(axis_hip, sub(p_foot,p_hip)),
        cross(axis_thigh, sub(p_foot,p_thigh)),
        cross(axis_calf, sub(p_foot,p_calf))
    };
    Vec3 velocity{0,0,0};
    for (int j=0;j<3;++j) {
        out.jacobian[0][j]=cols[j].x;
        out.jacobian[1][j]=cols[j].y;
        out.jacobian[2][j]=cols[j].z;
        velocity=add(velocity,mul(cols[j],dq[ids[j]]));
    }
    out.foot_position[0]=p_foot.x; out.foot_position[1]=p_foot.y; out.foot_position[2]=p_foot.z;
    out.foot_velocity[0]=velocity.x; out.foot_velocity[1]=velocity.y; out.foot_velocity[2]=velocity.z;
    out.valid=finite(p_foot)&&finite(velocity);
    return out;
}
