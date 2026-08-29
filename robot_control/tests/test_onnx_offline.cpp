/**
 * @file test_onnx_offline.cpp
 *
 * 离线 ONNX 模型测试 — 不需要任何硬件！
 *
 * 
    * 现在你可以用的工具

    # 无硬件离线测试 (已跑通)
    ./test_onnx_offline robot_control/creeper_flat_model_700_actor.onnx
    ./test_onnx_offline robot_control/creeper_flat_model_700_actor.onnx --random 50 --benchmark 500

    # 有硬件时用 robot_control
    sudo ./build/robot_control --onnx robot_control/creeper_flat_model_700_actor.onnx \
        --dry-run --log --compare --validate    # 第1步: 干运行验证
    sudo ./build/robot_control --onnx robot_control/creeper_flat_model_700_actor.onnx \
        --log --validate                        # 第2步: 实际控制
    sudo ./build/robot_control --onnx robot_control/creeper_flat_model_700_actor.onnx  # 第3步: 正式运行
 * 用途: 拿到 ONNX 模型后第一步验证
 *   1. 自动检测模型的输入/输出名称、形状、类型
 *   2. 构造合理范围内的假输入，跑多轮推理
 *   3. 检查输出是否在关节限位内、有无 NaN
 *   4. 统计延迟 (avg/p50/p99)
 *   5. 多轮输出一致性检查
 *
 * 编译:
 *   g++ -std=c++14 -O2 -I thirdparty/onnxruntime/include \
 *       robot_control/tests/test_onnx_offline.cpp \
 *       -L thirdparty/onnxruntime/lib -lonnxruntime \
 *       -Wl,-rpath,thirdparty/onnxruntime/lib \
 *       -o test_onnx_offline
 *
 * 使用:
 *   ./test_onnx_offline robot_control/creeper_flat_model_700_actor.onnx
 *   ./test_onnx_offline robot_control/creeper_flat_model_700_actor.onnx --verbose
 *   ./test_onnx_offline robot_control/creeper_flat_model_700_actor.onnx --benchmark 500
 */

#include <onnxruntime_cxx_api.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <random>
#include <fstream>

// ═══════════════════════════════════════════════════════════════════════════════
//  关节限位 (与 nn_validation.h 一致, 输出端 rad)
// ═══════════════════════════════════════════════════════════════════════════════

// Z字排序: [0..3]=hip, [4..7]=thigh, [8..11]=lower_leg (与 default_joint_pos 一致)
// 来自 ZeroPointCalibration.h
constexpr float JOINT_LIMITS[12][2] = {
    // hip (Joint 0-3)    [-60°, 60°]
    {-1.0472f,  1.0472f},
    {-1.0472f,  1.0472f},
    {-1.0472f,  1.0472f},
    {-1.0472f,  1.0472f},
    // thigh (Joint 4-5)  [-90°, 200°], (Joint 6-7) [-30°, 260°]
    {-1.5708f,  3.4907f},
    {-0.5236f,  4.5379f},
    {-1.5708f,  3.4907f},
    {-0.5236f,  4.5379f},
    // lower_leg (Joint 8-11) [-156°, -48°]
    {-2.7227f, -0.83776f},
    {-2.7227f, -0.83776f},
    {-2.7227f, -0.83776f},
    {-2.7227f, -0.83776f},
};

// 与 ZeroPointCalibration.cpp 的 default_joint_pos 一致 (Z字排序)
constexpr float DEFAULT_POSE[12] = {
    0.1f, -0.1f, 0.1f, -0.1f,  // hip
    0.8f,  0.8f, 1.0f,  1.0f,  // thigh
   -1.5f, -1.5f, -1.5f, -1.5f   // lower_leg
};

constexpr float ACTION_SCALE = 0.25f;

// ═══════════════════════════════════════════════════════════════════════════════

static const char* onnxTypeName(ONNXTensorElementDataType t) {
    switch (t) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:  return "float32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE: return "float64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:  return "int64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:  return "int32";
    default:                                   return "other";
    }
}

static void printSeparator(const char* title) {
    std::cout << "\n" << std::string(60, '-') << std::endl;
    std::cout << "  " << title << std::endl;
    std::cout << std::string(60, '-') << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <model.onnx> [--verbose] [--benchmark N] [--random N]" << std::endl;
        std::cerr << "  --verbose      打印每轮推理的输入/输出前几个值" << std::endl;
        std::cerr << "  --benchmark N  跑 N 轮推理并输出延迟统计 (默认100)" << std::endl;
        std::cerr << "  --random N     用 N 组不同的随机输入测试 (默认10)" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    bool verbose = false;
    int  benchmark_iters = 100;
    int  random_iters = 10;

    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (std::strcmp(argv[i], "--benchmark") == 0 && i+1 < argc) {
            benchmark_iters = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--random") == 0 && i+1 < argc) {
            random_iters = std::stoi(argv[++i]);
        }
    }

    std::cout << "╔══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  ONNX Offline Test — No Hardware Required            ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\n模型: " << model_path << std::endl;

    // ═══════════════════════════════════════════════════════════════════════════
    // 第1步: 加载模型, 检查 I/O 规格
    // ═══════════════════════════════════════════════════════════════════════════

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "OfflineTest");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    std::unique_ptr<Ort::Session> session;
    try {
        session = std::make_unique<Ort::Session>(env, model_path.c_str(), opts);
    } catch (const Ort::Exception& e) {
        std::cerr << "\n✗ 模型加载失败: " << e.what() << std::endl;
        return 1;
    }
    Ort::AllocatorWithDefaultOptions alloc;

    printSeparator("INPUT SPEC");

    size_t n_inputs = session->GetInputCount();
    std::vector<const char*> input_names(n_inputs);
    std::vector<std::vector<int64_t>> input_shapes(n_inputs);
    std::vector<size_t> input_sizes(n_inputs);

    for (size_t i = 0; i < n_inputs; ++i) {
        auto name = session->GetInputNameAllocated(i, alloc);
        input_names[i] = strdup(name.get());  // strdup for simplicity

        auto ti = session->GetInputTypeInfo(i);
        auto ts = ti.GetTensorTypeAndShapeInfo();
        auto shape = ts.GetShape();
        auto type  = ts.GetElementType();

        input_shapes[i] = shape;

        // 计算具体元素数 (dynamic dim → 1)
        input_sizes[i] = 1;
        bool has_dynamic = false;
        for (auto d : shape) {
            if (d == -1) { has_dynamic = true; d = 1; }
            input_sizes[i] *= static_cast<size_t>(d);
        }

        std::cout << "  [" << i << "] \"" << name.get() << "\"" << std::endl;
        std::cout << "      type=" << onnxTypeName(type)
                  << "  shape=[";
        for (size_t j = 0; j < shape.size(); ++j) {
            if (j > 0) std::cout << ",";
            if (shape[j] == -1) std::cout << "batch";
            else std::cout << shape[j];
        }
        std::cout << "]" << std::endl;
        std::cout << "      elements=" << input_sizes[i]
                  << "  size=" << std::fixed << std::setprecision(1)
                  << (input_sizes[i] * 4.0f / 1024.0f) << "KB" << std::endl;
    }

    printSeparator("OUTPUT SPEC");

    size_t n_outputs = session->GetOutputCount();
    std::vector<const char*> output_names(n_outputs);
    std::vector<std::vector<int64_t>> output_shapes(n_outputs);
    size_t total_output_elements = 0;

    for (size_t i = 0; i < n_outputs; ++i) {
        auto name = session->GetOutputNameAllocated(i, alloc);
        output_names[i] = strdup(name.get());

        auto ti = session->GetOutputTypeInfo(i);
        auto ts = ti.GetTensorTypeAndShapeInfo();
        auto shape = ts.GetShape();
        auto type  = ts.GetElementType();

        output_shapes[i] = shape;

        size_t elems = 1;
        for (auto d : shape) {
            if (d == -1) d = 1;
            elems *= static_cast<size_t>(d);
        }
        total_output_elements += elems;

        std::cout << "  [" << i << "] \"" << name.get() << "\"" << std::endl;
        std::cout << "      type=" << onnxTypeName(type)
                  << "  shape=[";
        for (size_t j = 0; j < shape.size(); ++j) {
            if (j > 0) std::cout << ",";
            if (shape[j] == -1) std::cout << "batch";
            else std::cout << shape[j];
        }
        std::cout << "]  elements=" << elems << std::endl;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // 第2步: 用全零输入做单次推理, 验证 pipeline 能走通
    // ═══════════════════════════════════════════════════════════════════════════

    printSeparator("SANITY CHECK (zero input)");

    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    // 构造全零输入
    std::vector<Ort::Value> zero_inputs;
    std::vector<std::vector<float>> zero_buffers(n_inputs);
    for (size_t i = 0; i < n_inputs; ++i) {
        zero_buffers[i].resize(input_sizes[i], 0.0f);
        // concrete shape: dynamic → 1
        std::vector<int64_t> cshape;
        for (auto d : input_shapes[i]) cshape.push_back(d == -1 ? 1 : d);
        zero_inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, zero_buffers[i].data(), input_sizes[i],
            cshape.data(), cshape.size()));
    }

    try {
        auto t0 = std::chrono::high_resolution_clock::now();

        auto outputs = session->Run(Ort::RunOptions{nullptr},
            input_names.data(), zero_inputs.data(), n_inputs,
            output_names.data(), n_outputs);

        auto t1 = std::chrono::high_resolution_clock::now();
        int first_latency = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        std::cout << "  ✓ 推理成功! 延迟: " << first_latency << " us ("
                  << std::fixed << std::setprecision(1) << first_latency/1000.0
                  << " ms)" << std::endl;

        // 打印输出
        int total_output_count = 0;
        for (size_t i = 0; i < n_outputs; ++i) {
            float* data = outputs[i].GetTensorMutableData<float>();
            size_t count = outputs[i].GetTensorTypeAndShapeInfo().GetElementCount();
            total_output_count += count;

            std::cout << "\n  " << output_names[i] << " (" << count << " 值):" << std::endl;

            // 检查 NaN/Inf
            int nan_c = 0, inf_c = 0, zero_c = 0;
            float min_v = 1e9, max_v = -1e9, sum_v = 0, sum_abs = 0;
            for (size_t j = 0; j < count; ++j) {
                float v = data[j];
                if (std::isnan(v)) { nan_c++; continue; }
                if (std::isinf(v)) { inf_c++; continue; }
                if (v == 0.0f) zero_c++;
                min_v = std::min(min_v, v);
                max_v = std::max(max_v, v);
                sum_v += v;
                sum_abs += std::abs(v);
            }
            float mean_v = sum_v / static_cast<float>(count);
            float mean_abs = sum_abs / static_cast<float>(count);

            std::cout << "      min=" << std::setw(10) << std::fixed << std::setprecision(4) << min_v
                      << "  max=" << std::setw(10) << max_v
                      << "  mean=" << std::setw(10) << mean_v << std::endl;
            std::cout << "      |mean|=" << std::setw(10) << mean_abs
                      << "  NaN:" << nan_c << "  Inf:" << inf_c
                      << "  zeros:" << zero_c << "/" << count << std::endl;

            // 前12个值 — 应用 delta 公式后打印绝对角度
            if (count >= 12) {
                std::cout << "      模型原始输出 [0..11]: ";
                for (size_t j = 0; j < 12; ++j) {
                    std::cout << std::fixed << std::setprecision(3) << data[j];
                    if (j < 11) std::cout << ", ";
                }
                std::cout << std::endl;

                // 施加 delta 公式: target = default + scale * output
                std::cout << "      绝对目标角度 [0..11]: ";
                for (int j = 0; j < 12; ++j) {
                    float target = DEFAULT_POSE[j] + ACTION_SCALE * data[j];
                    std::cout << std::fixed << std::setprecision(3) << target;
                    if (j < 11) std::cout << ", ";
                }
                std::cout << std::endl;

                // 关节限位检查 (对绝对角度检查)
                if (count == 12) {
                    int oob = 0;
                    for (int j = 0; j < 12; ++j) {
                        float target = DEFAULT_POSE[j] + ACTION_SCALE * data[j];
                        float lo = JOINT_LIMITS[j][0] - 0.05f;
                        float hi = JOINT_LIMITS[j][1] + 0.05f;
                        if (target < lo || target > hi) oob++;
                    }
                    if (oob > 0) {
                        std::cout << "      ⚠ " << oob << "/12 绝对角度越界!" << std::endl;
                    } else {
                        std::cout << "      ✓ 全部在限位内" << std::endl;
                    }
                }
            }

            // 如果是全零输入，输出也全零 → 检查是否是合理的 zero-policy
            if (mean_abs < 1e-6) {
                std::cout << "      ℹ 零输入 → 零输出 (模型可能输出的是相对于默认姿态的偏移量)" << std::endl;
            }
        }

        if (total_output_count == 12) {
            std::cout << "\n  ✓ 输出恰好 12 个值，匹配 12 关节目标" << std::endl;
        } else {
            std::cout << "\n  ⚠ 总输出 " << total_output_count
                      << " 个值 (期望 12, 对应 12 关节)" << std::endl;
            std::cout << "    需要在 ONNXPolicy::infer() 中做输出重映射" << std::endl;
        }

    } catch (const Ort::Exception& e) {
        std::cerr << "  ✗ 推理失败: " << e.what() << std::endl;
        return 1;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // 第3步: 随机输入测试
    // ═══════════════════════════════════════════════════════════════════════════

    printSeparator("RANDOM INPUT TEST");

    std::mt19937 rng(42);  // 固定种子, 可复现
    std::normal_distribution<float> normal(0.0f, 0.5f);  // 均值0, 标准差0.5
    std::uniform_real_distribution<float> uniform(-1.0f, 1.0f);

    bool any_nan = false, any_oob = false;
    std::vector<float> all_outputs;

    for (int iter = 0; iter < random_iters; ++iter) {
        // 构造随机输入
        std::vector<Ort::Value> rnd_inputs;
        std::vector<std::vector<float>> rnd_buffers(n_inputs);

        for (size_t i = 0; i < n_inputs; ++i) {
            rnd_buffers[i].resize(input_sizes[i]);
            for (size_t j = 0; j < input_sizes[i]; ++j) {
                // 一半 normal, 一半 uniform 混合, 更接近真实传感器数据
                rnd_buffers[i][j] = (j % 2 == 0)
                    ? std::max(-3.0f, std::min(3.0f, normal(rng)))
                    : uniform(rng);
            }

            std::vector<int64_t> cshape;
            for (auto d : input_shapes[i]) cshape.push_back(d == -1 ? 1 : d);
            rnd_inputs.push_back(Ort::Value::CreateTensor<float>(
                mem, rnd_buffers[i].data(), input_sizes[i],
                cshape.data(), cshape.size()));
        }

        try {
            auto outputs = session->Run(Ort::RunOptions{nullptr},
                input_names.data(), rnd_inputs.data(), n_inputs,
                output_names.data(), n_outputs);

            for (size_t i = 0; i < n_outputs; ++i) {
                float* data = outputs[i].GetTensorMutableData<float>();
                size_t count = outputs[i].GetTensorTypeAndShapeInfo().GetElementCount();

                for (size_t j = 0; j < count; ++j) {
                    float v = data[j];
                    if (std::isnan(v)) any_nan = true;
                    all_outputs.push_back(v);

                    // 关节限位检查 (对绝对角度)
                    if (count == 12 && j < 12) {
                        float target = DEFAULT_POSE[j] + ACTION_SCALE * v;
                        float lo = JOINT_LIMITS[j][0] - 0.05f;
                        float hi = JOINT_LIMITS[j][1] + 0.05f;
                        if (target < lo || target > hi) any_oob = true;
                    }
                }
            }

            if (verbose && iter < 3) {
                // 从第一个输出中取前几个值打印
                float* data = outputs[0].GetTensorMutableData<float>();
                size_t n = outputs[0].GetTensorTypeAndShapeInfo().GetElementCount();
                std::cout << "  iter " << iter << " output[0.."
                          << std::min<size_t>(n,6)-1 << "]: ";
                for (size_t j = 0; j < std::min<size_t>(n, 6); ++j) {
                    std::cout << std::fixed << std::setprecision(3) << data[j] << " ";
                }
                std::cout << std::endl;
            }

        } catch (const Ort::Exception& e) {
            std::cerr << "  ✗ iter " << iter << " 推理失败: " << e.what() << std::endl;
        }
    }

    // 统计所有输出
    if (!all_outputs.empty()) {
        std::sort(all_outputs.begin(), all_outputs.end());
        float min_all = all_outputs.front();
        float max_all = all_outputs.back();
        float sum_all = 0;
        for (float v : all_outputs) sum_all += v;
        float mean_all = sum_all / all_outputs.size();

        std::cout << "  " << random_iters << " 轮随机输入 → "
                  << all_outputs.size() << " 个输出值" << std::endl;
        std::cout << "    全局 min=" << std::fixed << std::setprecision(4) << min_all
                  << "  max=" << max_all
                  << "  mean=" << mean_all << std::endl;
        std::cout << "    NaN: " << (any_nan ? "⚠ YES!" : "✓ none")
                  << "  越界: " << (any_oob ? "⚠ YES!" : "✓ none")
                  << std::endl;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // 第4步: Benchmark
    // ═══════════════════════════════════════════════════════════════════════════

    printSeparator("BENCHMARK");

    {
        std::vector<int> latencies;
        latencies.reserve(benchmark_iters);

        // 复用全零输入
        for (int i = 0; i < benchmark_iters; ++i) {
            auto b0 = std::chrono::high_resolution_clock::now();
            auto _ = session->Run(Ort::RunOptions{nullptr},
                input_names.data(), zero_inputs.data(), n_inputs,
                output_names.data(), n_outputs);
            auto b1 = std::chrono::high_resolution_clock::now();
            latencies.push_back(std::chrono::duration_cast<std::chrono::microseconds>(b1 - b0).count());
        }

        std::sort(latencies.begin(), latencies.end());
        long long sum = 0;
        for (int l : latencies) sum += l;

        int avg   = static_cast<int>(sum / benchmark_iters);
        int p50   = latencies[benchmark_iters / 2];
        int p95   = latencies[benchmark_iters * 95 / 100];
        int p99   = latencies[benchmark_iters * 99 / 100];
        int worst = latencies.back();
        int best  = latencies.front();

        float hz_max = 1e6f / static_cast<float>(avg);

        std::cout << "  " << benchmark_iters << " 次推理:" << std::endl;
        std::cout << "    best:  " << std::setw(6) << best  << " us" << std::endl;
        std::cout << "    avg:   " << std::setw(6) << avg   << " us  ("
                  << std::fixed << std::setprecision(1) << avg/1000.0 << " ms)" << std::endl;
        std::cout << "    p50:   " << std::setw(6) << p50   << " us" << std::endl;
        std::cout << "    p95:   " << std::setw(6) << p95   << " us" << std::endl;
        std::cout << "    p99:   " << std::setw(6) << p99   << " us" << std::endl;
        std::cout << "    worst: " << std::setw(6) << worst << " us" << std::endl;
        std::cout << "    → 理论最大频率: " << std::fixed << std::setprecision(0)
                  << hz_max << " Hz" << std::endl;

        if (worst > 20000) {
            std::cout << "\n    ⚠ p99>20ms! 50Hz 推理(20ms窗口)不可行" << std::endl;
            std::cout << "       考虑: 模型量化 / TensorRT / 降低NN频率 / GPU加速" << std::endl;
        } else if (p99 > 10000) {
            std::cout << "\n    ⚠ p99>10ms, 50Hz 推理紧张, 可能出现掉帧" << std::endl;
        } else if (p99 > 5000) {
            std::cout << "\n    ⚡ p99<10ms, 50Hz OK, 100Hz 紧张" << std::endl;
        } else if (p99 > 2000) {
            std::cout << "\n    ⚡ p99<5ms, 50Hz 绰绰有余, 100Hz OK" << std::endl;
        } else {
            std::cout << "\n    🚀 p99<2ms! 可以轻松跑到 100-200Hz" << std::endl;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // 第5步: 确定性检查 (同样的输入 → 同样的输出?)
    // ═══════════════════════════════════════════════════════════════════════════

    printSeparator("DETERMINISM CHECK");

    {
        auto r1 = session->Run(Ort::RunOptions{nullptr},
            input_names.data(), zero_inputs.data(), n_inputs,
            output_names.data(), n_outputs);
        auto r2 = session->Run(Ort::RunOptions{nullptr},
            input_names.data(), zero_inputs.data(), n_inputs,
            output_names.data(), n_outputs);

        float* d1 = r1[0].GetTensorMutableData<float>();
        float* d2 = r2[0].GetTensorMutableData<float>();
        size_t n = r1[0].GetTensorTypeAndShapeInfo().GetElementCount();

        float max_diff = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            float diff = std::abs(d1[i] - d2[i]);
            if (diff > max_diff) max_diff = diff;
        }

        if (max_diff < 1e-6) {
            std::cout << "  ✓ 确定性: 相同输入→完全相同输出 (max_diff=" << max_diff << ")" << std::endl;
        } else {
            std::cout << "  ⚠ 不确定! 相同输入→输出差异 max=" << max_diff << std::endl;
            std::cout << "    模型可能含随机层(dropout/noise), 推理时要固定 seed" << std::endl;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════

    std::cout << "\n╔══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  ✓ 离线测试完成                                       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;

    // 免费 strdup 内存 (不值当写 RAII 包装...)
    for (auto p : input_names)  free((void*)p);
    for (auto p : output_names) free((void*)p);

    return 0;
}
