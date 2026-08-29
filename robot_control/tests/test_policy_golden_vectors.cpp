#include "policy_observation.h"

#include <onnxruntime_cxx_api.h>

#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace json {

struct Value {
    enum class Type { Null, Number, String, Array, Object, Boolean } type = Type::Null;
    double number = 0.0;
    bool boolean = false;
    std::string string;
    std::vector<Value> array;
    std::map<std::string, Value> object;

    const Value& at(const std::string& key) const {
        const auto it = object.find(key);
        if (type != Type::Object || it == object.end())
            throw std::runtime_error("JSON 缺少字段: " + key);
        return it->second;
    }
};

class Parser {
public:
    explicit Parser(std::string text) : text_(std::move(text)) {}
    Value parse() {
        Value result = value();
        space();
        if (pos_ != text_.size()) fail("根值之后仍有内容");
        return result;
    }

private:
    void space() { while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
    [[noreturn]] void fail(const std::string& why) const {
        throw std::runtime_error("JSON 解析错误 @" + std::to_string(pos_) + ": " + why);
    }
    bool take(char c) { space(); if (pos_ < text_.size() && text_[pos_] == c) { ++pos_; return true; } return false; }
    void literal(const char* word) {
        while (*word) { if (pos_ >= text_.size() || text_[pos_++] != *word++) fail("非法字面量"); }
    }
    std::string string() {
        if (!take('"')) fail("需要字符串");
        std::string out;
        while (pos_ < text_.size()) {
            char c = text_[pos_++];
            if (c == '"') return out;
            if (c == '\\') {
                if (pos_ >= text_.size()) fail("字符串转义不完整");
                const char e = text_[pos_++];
                switch (e) {
                case '"': case '\\': case '/': out.push_back(e); break;
                case 'b': out.push_back('\b'); break; case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break; case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                default: fail("测试文件不支持该字符串转义");
                }
            } else out.push_back(c);
        }
        fail("字符串未结束");
    }
    Value value() {
        space();
        if (pos_ >= text_.size()) fail("意外到达文件末尾");
        if (text_[pos_] == '{') return object();
        if (text_[pos_] == '[') return array();
        if (text_[pos_] == '"') { Value v; v.type = Value::Type::String; v.string = string(); return v; }
        if (text_[pos_] == 't') { literal("true"); Value v; v.type=Value::Type::Boolean; v.boolean=true; return v; }
        if (text_[pos_] == 'f') { literal("false"); Value v; v.type=Value::Type::Boolean; return v; }
        if (text_[pos_] == 'n') { literal("null"); return {}; }
        return number();
    }
    Value number() {
        space();
        const std::size_t begin = pos_;
        if (pos_ < text_.size() && text_[pos_] == '-') ++pos_;
        while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
        if (pos_ < text_.size() && text_[pos_] == '.') { ++pos_; while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
        if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
            ++pos_; if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) ++pos_;
            while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
        }
        if (begin == pos_) fail("需要数值");
        Value v; v.type = Value::Type::Number;
        v.number = std::stod(text_.substr(begin, pos_ - begin));
        return v;
    }
    Value array() {
        Value v; v.type = Value::Type::Array; take('[');
        if (take(']')) return v;
        do { v.array.push_back(value()); } while (take(','));
        if (!take(']')) fail("数组未结束");
        return v;
    }
    Value object() {
        Value v; v.type = Value::Type::Object; take('{');
        if (take('}')) return v;
        do { const std::string key = string(); if (!take(':')) fail("对象字段缺少冒号"); v.object.emplace(key, value()); } while (take(','));
        if (!take('}')) fail("对象未结束");
        return v;
    }
    std::string text_;
    std::size_t pos_ = 0;
};
} // namespace json

struct ErrorPeak {
    float value = 0.0f;
    int frame = -1;
    int index = -1;
    void observe(float actual, float expected, int f, int i) {
        const float error = std::abs(actual - expected);
        if (error > value) { value = error; frame = f; index = i; }
    }
};

static std::vector<float> numbers(const json::Value& object, const std::string& key,
                                  std::size_t expected_size)
{
    const auto& value = object.at(key);
    if (value.type != json::Value::Type::Array || value.array.size() != expected_size)
        throw std::runtime_error(key + " 维数不正确");
    std::vector<float> result;
    result.reserve(expected_size);
    for (const auto& item : value.array) {
        if (item.type != json::Value::Type::Number || !std::isfinite(item.number))
            throw std::runtime_error(key + " 包含非有限数值");
        result.push_back(static_cast<float>(item.number));
    }
    return result;
}

int main(int argc, char** argv)
{
    const std::string json_path = argc > 1 ? argv[1] : "robot_control/golden_vectors/policy_golden_vectors.json";
    const std::string model_path = argc > 2 ? argv[2] : "robot_control/creeper_flat_model_700_actor.onnx";
    try {
        std::ifstream input(json_path);
        if (!input) throw std::runtime_error("无法打开黄金向量: " + json_path);
        std::ostringstream buffer; buffer << input.rdbuf();
        std::cout << "[Golden] 读取 JSON " << buffer.str().size() << " bytes" << std::endl;
        const json::Value root = json::Parser(buffer.str()).parse();
        std::cout << "[Golden] JSON 解析完成" << std::endl;
        const auto& frames = root.at("frames").array;
        if (frames.empty()) throw std::runtime_error("黄金向量没有帧");

        const auto default_pose_vector = numbers(frames.front(), "default_joint_position", 12);
        float default_pose[12];
        for (int i = 0; i < 12; ++i) default_pose[i] = default_pose_vector[i];
        PolicyObservationBuilder builder(default_pose);

        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "GoldenVectorTest");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        Ort::Session session(env, model_path.c_str(), options);
        std::cout << "[Golden] ONNX 加载完成" << std::endl;
        if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
            throw std::runtime_error("ONNX 必须恰好有一个输入和一个输出");
        const auto input_type = session.GetInputTypeInfo(0);
        const auto output_type = session.GetOutputTypeInfo(0);
        const auto in_info = input_type.GetTensorTypeAndShapeInfo();
        const auto out_info = output_type.GetTensorTypeAndShapeInfo();
        const auto input_model_shape = in_info.GetShape();
        const auto output_model_shape = out_info.GetShape();
        if (input_model_shape.size() != 2 || input_model_shape[0] != 1 || input_model_shape[1] != 48 ||
            output_model_shape.size() != 2 || output_model_shape[0] != 1 || output_model_shape[1] != 12 ||
            in_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            out_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("ONNX I/O shape 或 dtype 与 [1,48] -> [1,12] float32 不符");

        ErrorPeak obs_peak, action_peak, target_peak, history_peak;
        Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        const std::array<int64_t, 2> input_shape{{1, 48}};
        const char* input_names[] = {"obs"};
        const char* output_names[] = {"actions"};

        for (std::size_t f = 0; f < frames.size(); ++f) {
            const auto& frame = frames[f];
            const auto lin = numbers(frame, "base_linear_velocity_body", 3);
            const auto ang = numbers(frame, "base_angular_velocity_body", 3);
            const auto gravity = numbers(frame, "projected_gravity_body", 3);
            const auto command = numbers(frame, "command", 3);
            const auto q = numbers(frame, "joint_position", 12);
            const auto dq = numbers(frame, "joint_velocity", 12);
            const auto expected_history = numbers(frame, "previous_raw_action", 12);
            const auto expected_obs = numbers(frame, "observation_final", 48);
            const auto expected_action = numbers(frame, "onnx_actor_raw_action", 12);
            const auto expected_target = numbers(frame, "joint_position_target", 12);

            EstimatedState state;
            for (int i = 0; i < 3; ++i) {
                state.body_linear_velocity[i] = lin[i];
                state.angular_velocity[i] = ang[i];
                state.projected_gravity[i] = gravity[i];
            }
            for (int i = 0; i < 12; ++i) {
                state.joint_position[i] = q[i];
                state.joint_velocity[i] = dq[i];
                history_peak.observe(builder.previousAction()[i], expected_history[i], static_cast<int>(f), i);
            }
            if (!builder.setVelocityCommand({{command[0], command[1], command[2]}}))
                throw std::runtime_error("速度命令被意外拒绝");
            const auto& observation = builder.build(state);
            for (int i = 0; i < 48; ++i)
                obs_peak.observe(observation[i], expected_obs[i], static_cast<int>(f), i);

            auto tensor = Ort::Value::CreateTensor<float>(
                memory, const_cast<float*>(observation.data()), observation.size(),
                input_shape.data(), input_shape.size());
            auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &tensor, 1, output_names, 1);
            const float* action = outputs[0].GetTensorData<float>();
            for (int i = 0; i < 12; ++i) {
                action_peak.observe(action[i], expected_action[i], static_cast<int>(f), i);
                const float target = default_pose[i] + 0.25f * action[i];
                target_peak.observe(target, expected_target[i], static_cast<int>(f), i);
            }
            if (!builder.commitRawAction(action)) throw std::runtime_error("ONNX 输出包含 NaN/Inf");
        }

        const auto print_peak = [](const char* name, const ErrorPeak& peak) {
            std::cout << std::left << std::setw(28) << name << " max_abs=" << std::scientific
                      << peak.value << " frame=" << peak.frame << " index=" << peak.index << '\n';
        };
        std::cout << "[Golden] frames=" << frames.size() << '\n';
        print_peak("C++ observation vs Python", obs_peak);
        print_peak("previous raw-action history", history_peak);
        print_peak("C++ ONNX vs golden ONNX", action_peak);
        print_peak("C++ q_des vs Python q_des", target_peak);

        constexpr float kObservationTolerance = 1e-6f;
        constexpr float kInferenceTolerance = 1e-5f;
        const bool passed = obs_peak.value <= kObservationTolerance &&
                            history_peak.value <= kObservationTolerance &&
                            action_peak.value <= kInferenceTolerance &&
                            target_peak.value <= kInferenceTolerance;
        if (!passed) {
            std::cerr << "[FAIL] 黄金向量对拍超过容差" << std::endl;
            return 1;
        }
        std::cout << "[PASS] 20-frame policy golden-vector replay" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[FAIL] " << error.what() << std::endl;
        return 1;
    }
}
