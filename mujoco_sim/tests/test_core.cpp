#include "creeper_sim/joint_mapping.h"
#include "creeper_sim/mujoco_backend.h"
#include "creeper_sim/onnx_policy.h"
#include "creeper_sim/policy_observation.h"
#include <cmath>
#include <iostream>
#include <set>
using namespace creeper_sim;
namespace {int failures=0;void check(bool ok,const char*msg){if(!ok){++failures;std::cerr<<"FAIL: "<<msg<<'\n';}}bool near(double a,double b,double e=1e-6){return std::abs(a-b)<e;}}
int main(int argc,char**argv){if(argc!=2){std::cerr<<"model path required\n";return 2;}std::string error;MujocoBackend b;check(b.load(argv[1],error),error.c_str());if(failures)return 1;check(b.reset("standing",error),error.c_str());
 std::set<int>qa,da;for(int x:b.qposAddresses())qa.insert(x);for(int x:b.dofAddresses())da.insert(x);check(qa.size()==12&&da.size()==12,"joint addresses must be unique");check(*qa.begin()>=7,"floating base must precede hinge qpos");check(*da.begin()>=6,"floating base must precede hinge qvel");
 RobotState gt=b.readState(StateMode::GroundTruth),se=b.readState(StateMode::SensorEmulation);check(gt.valid&&se.valid,"both state modes valid");for(int i=0;i<12;++i){check(near(gt.joint_position[i],kDefaultPose[i]),"standing keyframe/default pose mismatch");check(kDefaultPose[i]>=b.lowerLimits()[i]&&kDefaultPose[i]<=b.upperLimits()[i],"default outside limit");}
 for(int i=0;i<3;++i)check(near(gt.projected_gravity[i],i==2?-1:0,1e-7),"ground-truth projected gravity");for(int i=0;i<3;++i)check(near(se.projected_gravity[i],gt.projected_gravity[i],1e-7),"sensor quaternion convention");for(double w:se.body_angular_velocity)check(near(w,0,1e-7),"static gyro must be zero");check(near(se.specific_force[0],0,1e-6)&&near(se.specific_force[1],0,1e-6)&&near(se.specific_force[2],9.80665,1e-5),"static accelerometer must report +g on body Z");
 PolicyObservation p;ControlCommand c;c.velocity={.05,-.02,.1};auto first=p.build(gt,c);check(first.size()==48,"observation size");for(int i=0;i<12;++i)check(first[36+i]==0,"first previous action zero");for(int i=0;i<3;++i)check(near(first[9+i],c.velocity[i]),"command indices");PolicyObservation::Action a{};for(int i=0;i<12;++i)a[i]=float(i)-5.5f;check(p.commitRawAction(a),"commit raw action");auto second=p.build(gt,c);auto qdes=rawActionToTarget(a);for(int i=0;i<12;++i){check(second[36+i]==a[i],"previous raw action timing");check(near(qdes[i],kDefaultPose[i]+.25*a[i]),"action target formula");}
 check(near(b.timestep(),.005),"physics dt");check(4*b.timestep()==.02,"50Hz/200Hz decimation");std::array<double,12>zero{};b.applyTorques(zero);b.step();check(near(b.time(),.005,1e-10),"one physics step");
 std::cout<<(failures?"FAIL":"PASS")<<" checks failures="<<failures<<" onnx_runtime="<<OnnxPolicy::runtimeAvailable()<<'\n';return failures?1:0;}
