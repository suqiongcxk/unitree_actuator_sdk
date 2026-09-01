#include "creeper_sim/simulation_controller.h"
#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <stdexcept>
using namespace creeper_sim;
namespace {std::atomic<bool> interrupted{false};SimulationController* active=nullptr;void signalHandler(int){interrupted=true;if(active)active->requestStop();}void usage(){std::cout<<"mujoco_robot_control --model FILE [--onnx FILE] [--mode ground-truth|sensor-emulation] [--headless] [--realtime] [--duration SEC] [--command VX VY YAW] [--log FILE] [--keyframe NAME] [--no-policy]\n";}}
int main(int argc,char**argv){std::string model,onnx,log,key="standing";StateMode mode=StateMode::GroundTruth;ControlCommand cmd;double duration=10;bool realtime=false,no_policy=false;
 try{for(int i=1;i<argc;++i){std::string a=argv[i];auto next=[&](){if(++i>=argc)throw std::runtime_error("missing value after "+a);return std::string(argv[i]);};if(a=="--model")model=next();else if(a=="--onnx")onnx=next();else if(a=="--mode"){auto v=next();if(v=="ground-truth")mode=StateMode::GroundTruth;else if(v=="sensor-emulation")mode=StateMode::SensorEmulation;else throw std::runtime_error("invalid mode");}else if(a=="--duration")duration=std::stod(next());else if(a=="--command")for(double&v:cmd.velocity)v=std::stod(next());else if(a=="--log")log=next();else if(a=="--keyframe")key=next();else if(a=="--realtime")realtime=true;else if(a=="--headless"){}else if(a=="--no-policy")no_policy=true;else if(a=="--help"){usage();return 0;}else throw std::runtime_error("unknown option: "+a);}if(model.empty())throw std::runtime_error("--model is required");if(duration<=0)throw std::runtime_error("duration must be positive");if(no_policy)onnx.clear();
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';usage();return 2;}
 std::signal(SIGINT,signalHandler);SimulationController c;active=&c;std::string error;if(!c.initialize(model,key,onnx,error)){std::cerr<<"initialization failed: "<<error<<'\n';return 1;}
 if(!c.run(duration,mode,cmd,realtime,log,error)){std::cerr<<"result=FAIL reason="<<error<<'\n';return 1;}
 active=nullptr;std::cout<<"result=PASS time="<<c.backend().time()<<" interrupted="<<interrupted<<" torque_saturations="<<c.safety().torque_saturations<<" max_target_jump="<<c.safety().max_target_jump<<'\n';return 0;}
