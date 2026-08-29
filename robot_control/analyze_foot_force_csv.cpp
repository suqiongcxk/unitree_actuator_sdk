#include "foot_force_estimator.h"
#include "leg_kinematics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct Frame {
    uint64_t timestamp_ns = 0;
    int frame = 0;
    float q[12] = {0.0f};
    float dq[12] = {0.0f};
    float tau[12] = {0.0f};
};

struct Sample {
    FootForceEstimate estimate;
    float torque_norm = 0.0f;
    float foot_speed = 0.0f;
    bool contact = false;
    bool torque_contact = false;
    bool used_force = false;
};

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

float percentile(std::vector<float> values, float fraction) {
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());
    const float index = fraction * static_cast<float>(values.size() - 1);
    const size_t lower = static_cast<size_t>(index);
    const size_t upper = std::min(lower + 1, values.size() - 1);
    const float alpha = index - static_cast<float>(lower);
    return values[lower] * (1.0f - alpha) + values[upper] * alpha;
}

void printSummary(const char* label,
                  const std::vector<std::array<Sample, 4>>& samples,
                  size_t begin, size_t end) {
    static const char* names[4] = {"FL", "FR", "RL", "RR"};
    if (begin >= end || end > samples.size()) return;
    std::cout << "\n[" << label << "] frames=" << (end - begin) << std::endl;
    for (int leg = 0; leg < 4; ++leg) {
        std::vector<float> fz;
        std::vector<float> normal;
        double sum_fx = 0.0, sum_fy = 0.0, sum_fz = 0.0;
        double sum_torque = 0.0, sum_normal = 0.0;
        double sum_torque_sq = 0.0, sum_normal_sq = 0.0, sum_cross = 0.0;
        size_t valid = 0;
        size_t contact_frames = 0;
        size_t force_frames = 0;
        size_t torque_contact_frames = 0;
        size_t disagreement_frames = 0;
        size_t transitions = 0;
        bool previous_contact = samples[begin][leg].contact;
        for (size_t index = begin; index < end; ++index) {
            const auto& sample = samples[index][leg];
            contact_frames += sample.contact ? 1 : 0;
            force_frames += sample.used_force ? 1 : 0;
            torque_contact_frames += sample.torque_contact ? 1 : 0;
            disagreement_frames += sample.contact != sample.torque_contact ? 1 : 0;
            if (index > begin && sample.contact != previous_contact) ++transitions;
            previous_contact = sample.contact;
            if (!sample.estimate.valid) continue;
            ++valid;
            const float signed_fz = sample.estimate.force_body[2];
            fz.push_back(signed_fz);
            normal.push_back(sample.estimate.normal_force);
            sum_fx += sample.estimate.force_body[0];
            sum_fy += sample.estimate.force_body[1];
            sum_fz += signed_fz;
            sum_torque += sample.torque_norm;
            sum_normal += sample.estimate.normal_force;
            sum_torque_sq += sample.torque_norm * sample.torque_norm;
            sum_normal_sq += sample.estimate.normal_force * sample.estimate.normal_force;
            sum_cross += sample.torque_norm * sample.estimate.normal_force;
        }
        const double count = static_cast<double>(std::max<size_t>(1, valid));
        const double torque_mean = sum_torque / count;
        const double normal_mean = sum_normal / count;
        const double covariance = sum_cross / count - torque_mean * normal_mean;
        const double torque_variance = sum_torque_sq / count - torque_mean * torque_mean;
        const double normal_variance = sum_normal_sq / count - normal_mean * normal_mean;
        const double denominator = std::sqrt(std::max(0.0, torque_variance)
                                           * std::max(0.0, normal_variance));
        const double correlation = denominator > 1.0e-12 ? covariance / denominator : 0.0;
        std::cout << std::fixed << std::setprecision(3)
                  << "  " << names[leg]
                  << " valid=" << valid << "/" << (end - begin)
                  << " contact=" << contact_frames << "/" << (end - begin)
                  << " transitions=" << transitions
                  << " force_used=" << force_frames << "/" << (end - begin)
                  << " old_torque_contact=" << torque_contact_frames
                  << " disagree=" << disagreement_frames
                  << " Fxyz_mean=(" << sum_fx / count << ","
                  << sum_fy / count << "," << sum_fz / count << ")N"
                  << " Fz[p05/p50/p95]=(" << percentile(fz, 0.05f) << ","
                  << percentile(fz, 0.50f) << "," << percentile(fz, 0.95f) << ")N"
                  << " support[p05/p50/p95]=(" << percentile(normal, 0.05f) << ","
                  << percentile(normal, 0.50f) << "," << percentile(normal, 0.95f) << ")N"
                  << " corr(tau_norm,support)=" << correlation << std::endl;
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "用法: " << argv[0]
                  << " <nn_csv> [foot_force_output.csv]" << std::endl;
        return 1;
    }
    const std::string input_path = argv[1];
    const std::string output_path = argc == 3
        ? argv[2] : "/tmp/foot_force_estimates.csv";
    std::ifstream input(input_path);
    if (!input) {
        std::cerr << "无法打开CSV: " << input_path << std::endl;
        return 1;
    }

    std::string line;
    if (!std::getline(input, line)) {
        std::cerr << "CSV为空" << std::endl;
        return 1;
    }
    const auto header = splitCsv(line);
    std::unordered_map<std::string, size_t> column;
    for (size_t index = 0; index < header.size(); ++index)
        column[header[index]] = index;
    const auto require = [&](const std::string& name) {
        if (column.count(name)) return true;
        std::cerr << "CSV缺少列: " << name << std::endl;
        return false;
    };
    bool columns_ok = require("timestamp_ns") && require("frame");
    for (int joint = 0; joint < 12; ++joint) {
        columns_ok &= require("j" + std::to_string(joint) + "_pos");
        columns_ok &= require("j" + std::to_string(joint) + "_vel");
        columns_ok &= require("j" + std::to_string(joint) + "_tau");
    }
    if (!columns_ok) return 1;

    std::vector<Frame> frames;
    while (std::getline(input, line)) {
        const auto fields = splitCsv(line);
        if (fields.size() != header.size()) continue;
        try {
            Frame frame;
            frame.timestamp_ns = std::stoull(fields[column["timestamp_ns"]]);
            frame.frame = std::stoi(fields[column["frame"]]);
            for (int joint = 0; joint < 12; ++joint) {
                const std::string prefix = "j" + std::to_string(joint);
                frame.q[joint] = std::stof(fields[column[prefix + "_pos"]]);
                frame.dq[joint] = std::stof(fields[column[prefix + "_vel"]]);
                frame.tau[joint] = std::stof(fields[column[prefix + "_tau"]]);
            }
            frames.push_back(frame);
        } catch (...) {
            // 单个损坏行不应阻止其余实机日志的离线分析。
        }
    }
    if (frames.empty()) {
        std::cerr << "没有可解析的数据帧" << std::endl;
        return 1;
    }

    CreeperLegKinematics kinematics;
    FootForceEstimator force_estimator;
    std::vector<std::array<Sample, 4>> samples(frames.size());
    bool contact_state[4] = {false, false, false, false};
    int on_count[4] = {0, 0, 0, 0};
    int off_count[4] = {0, 0, 0, 0};
    bool torque_contact_state[4] = {false, false, false, false};
    int torque_on_count[4] = {0, 0, 0, 0};
    int torque_off_count[4] = {0, 0, 0, 0};
    constexpr float kFeedbackTorqueToJoint = 6.333f;
    constexpr float kForceOn = 7.60f;
    constexpr float kForceOff = 3.80f;
    constexpr float kTorqueOn = 0.28f;
    constexpr float kTorqueOff = 0.20f;
    constexpr float kMaxFootSpeed = 2.0f;
    std::ofstream output(output_path);
    if (!output) {
        std::cerr << "无法创建输出CSV: " << output_path << std::endl;
        return 1;
    }
    output << "timestamp_ns,frame";
    static const char* names[4] = {"FL", "FR", "RL", "RR"};
    for (const char* name : names)
        output << "," << name << "_Fx," << name << "_Fy," << name << "_Fz,"
               << name << "_support," << name << "_valid,"
               << name << "_condition," << name << "_residual,"
               << name << "_tau_norm," << name << "_foot_speed,"
               << name << "_contact," << name << "_used_force";
        // 旧力矩范数状态仅供离线对比，不参与运行时接触判断。
    for (const char* name : names) output << "," << name << "_old_torque_contact";
    output << "\n";

    for (size_t index = 0; index < frames.size(); ++index) {
        const Frame& frame = frames[index];
        output << frame.timestamp_ns << "," << frame.frame;
        for (int leg = 0; leg < 4; ++leg) {
            const auto kinematic = kinematics.compute(leg, frame.q, frame.dq);
            const int ids[3] = {leg, 4 + leg, 8 + leg};
            float torque[3] = {
                frame.tau[ids[0]] * kFeedbackTorqueToJoint,
                frame.tau[ids[1]] * kFeedbackTorqueToJoint,
                frame.tau[ids[2]] * kFeedbackTorqueToJoint};
            Sample& sample = samples[index][leg];
            if (kinematic.valid)
                sample.estimate = force_estimator.estimate(kinematic.jacobian, torque);
            // 保留旧判据对比时使用原始转子侧力矩范数。
            sample.torque_norm = std::sqrt(
                frame.tau[ids[0]] * frame.tau[ids[0]]
              + frame.tau[ids[1]] * frame.tau[ids[1]]
              + frame.tau[ids[2]] * frame.tau[ids[2]]);
            sample.foot_speed = std::sqrt(
                kinematic.foot_velocity[0] * kinematic.foot_velocity[0]
              + kinematic.foot_velocity[1] * kinematic.foot_velocity[1]
              + kinematic.foot_velocity[2] * kinematic.foot_velocity[2]);
            sample.used_force = sample.estimate.valid;
            const float evidence = sample.used_force
                ? sample.estimate.normal_force : sample.torque_norm;
            const float on_threshold = sample.used_force ? kForceOn : kTorqueOn;
            const float off_threshold = sample.used_force ? kForceOff : kTorqueOff;
            const bool touchdown = evidence >= on_threshold
                && sample.foot_speed <= kMaxFootSpeed;
            const bool liftoff = evidence < off_threshold;
            if (!contact_state[leg]) {
                off_count[leg] = 0;
                on_count[leg] = touchdown ? on_count[leg] + 1 : 0;
                if (on_count[leg] >= 3) {
                    contact_state[leg] = true;
                    on_count[leg] = 0;
                }
            } else {
                on_count[leg] = 0;
                off_count[leg] = liftoff ? off_count[leg] + 1 : 0;
                if (off_count[leg] >= 2) {
                    contact_state[leg] = false;
                    off_count[leg] = 0;
                }
            }
            sample.contact = contact_state[leg];

            const bool torque_touchdown = sample.torque_norm >= kTorqueOn
                && sample.foot_speed <= kMaxFootSpeed;
            const bool torque_liftoff = sample.torque_norm < kTorqueOff;
            if (!torque_contact_state[leg]) {
                torque_off_count[leg] = 0;
                torque_on_count[leg] = torque_touchdown
                    ? torque_on_count[leg] + 1 : 0;
                if (torque_on_count[leg] >= 3) {
                    torque_contact_state[leg] = true;
                    torque_on_count[leg] = 0;
                }
            } else {
                torque_on_count[leg] = 0;
                torque_off_count[leg] = torque_liftoff
                    ? torque_off_count[leg] + 1 : 0;
                if (torque_off_count[leg] >= 2) {
                    torque_contact_state[leg] = false;
                    torque_off_count[leg] = 0;
                }
            }
            sample.torque_contact = torque_contact_state[leg];
            output << "," << sample.estimate.force_body[0]
                   << "," << sample.estimate.force_body[1]
                   << "," << sample.estimate.force_body[2]
                   << "," << sample.estimate.normal_force
                   << "," << (sample.estimate.valid ? 1 : 0)
                   << "," << sample.estimate.normalized_determinant
                   << "," << sample.estimate.torque_residual
                   << "," << sample.torque_norm
                   << "," << sample.foot_speed
                   << "," << (sample.contact ? 1 : 0)
                   << "," << (sample.used_force ? 1 : 0);
        }
        for (int leg = 0; leg < 4; ++leg)
            output << "," << (samples[index][leg].torque_contact ? 1 : 0);
        output << "\n";
    }

    printSummary("ALL", samples, 0, samples.size());
    const size_t last_begin = samples.size() > 1000 ? samples.size() - 1000 : 0;
    printSummary("LAST_1000", samples, last_begin, samples.size());
    std::cout << "\n逐帧结果: " << output_path << std::endl;
    return 0;
}
