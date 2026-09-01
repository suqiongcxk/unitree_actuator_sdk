#include "creeper_sim/onnx_policy.h"
#include <algorithm>
#if CREEPER_HAS_ONNX
#include <onnxruntime_cxx_api.h>
#include <cstring>
#endif
namespace creeper_sim {
struct OnnxPolicy::Impl {
#if CREEPER_HAS_ONNX
 Ort::Env env{ORT_LOGGING_LEVEL_WARNING,"creeper_sim"}; Ort::SessionOptions options; std::unique_ptr<Ort::Session> session;
#endif
};
OnnxPolicy::OnnxPolicy():impl_(new Impl){} OnnxPolicy::~OnnxPolicy()=default;
bool OnnxPolicy::runtimeAvailable(){return CREEPER_HAS_ONNX;}
bool OnnxPolicy::loaded()const{
#if CREEPER_HAS_ONNX
 return bool(impl_->session);
#else
 return false;
#endif
}
bool OnnxPolicy::load(const std::string&p,std::string&e){
#if CREEPER_HAS_ONNX
 try{impl_->options.SetIntraOpNumThreads(1);impl_->session=std::make_unique<Ort::Session>(impl_->env,p.c_str(),impl_->options);if(impl_->session->GetInputCount()!=1||impl_->session->GetOutputCount()!=1){e="ONNX requires exactly one input and output";return false;}Ort::AllocatorWithDefaultOptions a;auto in=impl_->session->GetInputNameAllocated(0,a),out=impl_->session->GetOutputNameAllocated(0,a);auto ii=impl_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo(),oi=impl_->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();if(std::strcmp(in.get(),"obs")||std::strcmp(out.get(),"actions")||ii.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||oi.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||ii.GetShape()!=std::vector<int64_t>({1,48})||oi.GetShape()!=std::vector<int64_t>({1,12})){e="ONNX contract must be float32 obs[1,48] -> actions[1,12]";impl_->session.reset();return false;}return true;}catch(const Ort::Exception&x){e=x.what();return false;}
#else
 (void)p;e="this build has no ONNX Runtime support";return false;
#endif
}
bool OnnxPolicy::infer(const std::array<float,48>&o,std::array<float,12>&a,std::string&e){
#if CREEPER_HAS_ONNX
 if(!loaded()){e="policy not loaded";return false;}try{auto mem=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);std::array<int64_t,2> shape{1,48};auto input=Ort::Value::CreateTensor<float>(mem,const_cast<float*>(o.data()),o.size(),shape.data(),2);const char* ins[]={"obs"};const char* outs[]={"actions"};auto values=impl_->session->Run(Ort::RunOptions{nullptr},ins,&input,1,outs,1);const float*p=values[0].GetTensorData<float>();std::copy(p,p+12,a.begin());return true;}catch(const Ort::Exception&x){e=x.what();return false;}
#else
 (void)o;(void)a;e="this build has no ONNX Runtime support";return false;
#endif
}
}
