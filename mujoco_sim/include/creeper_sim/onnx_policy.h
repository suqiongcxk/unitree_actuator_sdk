#pragma once
#include <array>
#include <memory>
#include <string>
namespace creeper_sim {
class OnnxPolicy {
public:
 OnnxPolicy(); ~OnnxPolicy();
 bool load(const std::string&, std::string& error);
 bool infer(const std::array<float,48>&, std::array<float,12>&, std::string& error);
 bool loaded() const;
 static bool runtimeAvailable();
private: struct Impl; std::unique_ptr<Impl> impl_;
};
}
