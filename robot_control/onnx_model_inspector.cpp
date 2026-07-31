/**
 * @file onnx_model_inspector.cpp
 *
 * ONNX 模型检查器 — 不需要连接硬件即可运行
 *
 * 用途: 拿到 ONNX 模型后第一步验证
 *   1. 模型能否正常加载
 *   2. 输入 tensor 的名称、形状、类型
 *   3. 输出 tensor 的名称、形状、类型
 *   4. 用随机数据跑一次推理，检查延迟
 *
 * 编译 (需要 onnxruntime):
 *   g++ -std=c++14 -O2 \
 *       -I/usr/include/onnxruntime \
 *       robot_control/onnx_model_inspector.cpp \
 *       -lonnxruntime -o onnx_model_inspector
 *
 * 使用:
 *   ./onnx_model_inspector model.onnx
 */

#ifdef ONNXRUNTIME_AVAILABLE

#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <cstring>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════════
//  辅助: 打印 ONNX 类型名称
// ═══════════════════════════════════════════════════════════════════════════════

static const char* onnxTypeName(ONNXTensorElementDataType type)
{
    switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:  return "float32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE: return "float64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:  return "int64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:  return "int32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:   return "bool";
    default:                                   return "unknown";
    }
}

// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <model.onnx> [--benchmark N]" << std::endl;
        std::cerr << "  --benchmark N  跑 N 次推理并输出延迟统计" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    int benchmark_iters = 0;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--benchmark") == 0 && i + 1 < argc) {
            benchmark_iters = std::stoi(argv[++i]);
        }
    }

    std::cout << "╔══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  ONNX Model Inspector                                ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\n模型: " << model_path << "\n" << std::endl;

    try {
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "Inspector");
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        Ort::Session session(env, model_path.c_str(), opts);
        Ort::AllocatorWithDefaultOptions allocator;

        // ═══════════════════════════════════════════════════════════════════════
        //  1. 输入信息
        // ═══════════════════════════════════════════════════════════════════════

        size_t num_inputs = session.GetInputCount();
        std::cout << "── 输入 (" << num_inputs << " 个) ──" << std::endl;

        std::vector<const char*> input_names;
        std::vector<std::vector<int64_t>> input_shapes;
        input_names.reserve(num_inputs);
        input_shapes.reserve(num_inputs);

        for (size_t i = 0; i < num_inputs; ++i) {
            auto name_ptr = session.GetInputNameAllocated(i, allocator);
            Ort::TypeInfo type_info = session.GetInputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();

            auto shape = tensor_info.GetShape();
            auto type  = tensor_info.GetElementType();
            size_t total_elements = 1;
            bool has_dynamic = false;

            std::cout << "  [" << i << "] " << name_ptr.get()
                      << "  type=" << onnxTypeName(type)
                      << "  shape=[";

            for (size_t j = 0; j < shape.size(); ++j) {
                if (j > 0) std::cout << ", ";
                if (shape[j] == -1) {
                    std::cout << "dynamic";
                    has_dynamic = true;
                } else {
                    std::cout << shape[j];
                    total_elements *= static_cast<size_t>(shape[j]);
                }
            }
            std::cout << "]";

            if (!has_dynamic) {
                std::cout << "  elements=" << total_elements;
                float size_kb = (total_elements * sizeof(float)) / 1024.0f;
                std::cout << "  size=" << std::fixed << std::setprecision(1) << size_kb << "KB";
            }
            std::cout << std::endl;

            input_names.push_back(name_ptr.release());
            input_shapes.push_back(shape);
        }

        // ═══════════════════════════════════════════════════════════════════════
        //  2. 输出信息
        // ═══════════════════════════════════════════════════════════════════════

        size_t num_outputs = session.GetOutputCount();
        std::cout << "\n── 输出 (" << num_outputs << " 个) ──" << std::endl;

        std::vector<const char*> output_names;
        output_names.reserve(num_outputs);

        for (size_t i = 0; i < num_outputs; ++i) {
            auto name_ptr = session.GetOutputNameAllocated(i, allocator);
            Ort::TypeInfo type_info = session.GetOutputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();

            auto shape = tensor_info.GetShape();
            auto type  = tensor_info.GetElementType();
            size_t total_elements = 1;

            std::cout << "  [" << i << "] " << name_ptr.get()
                      << "  type=" << onnxTypeName(type)
                      << "  shape=[";

            for (size_t j = 0; j < shape.size(); ++j) {
                if (j > 0) std::cout << ", ";
                if (shape[j] == -1) {
                    std::cout << "dynamic";
                } else {
                    std::cout << shape[j];
                    total_elements *= static_cast<size_t>(shape[j]);
                }
            }
            std::cout << "]";

            if (shape.size() > 0 && shape[0] != -1) {
                float size_kb = (total_elements * sizeof(float)) / 1024.0f;
                std::cout << "  elements=" << total_elements << "  size="
                          << std::fixed << std::setprecision(1) << size_kb << "KB";
            }
            std::cout << std::endl;

            output_names.push_back(name_ptr.release());
        }

        // ═══════════════════════════════════════════════════════════════════════
        //  3. 试运行推理 (用全零输入, 验证 pipeline 能走通)
        // ═══════════════════════════════════════════════════════════════════════

        std::cout << "\n── 试运行推理 ──" << std::endl;

        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);

        // 为每个输入创建 tensor (全零)
        std::vector<Ort::Value> input_tensors;
        input_tensors.reserve(num_inputs);

        for (size_t i = 0; i < num_inputs; ++i) {
            auto& shape = input_shapes[i];
            std::vector<int64_t> concrete_shape;
            size_t num_elements = 1;
            for (auto dim : shape) {
                int64_t concrete = (dim == -1) ? 1 : dim;
                concrete_shape.push_back(concrete);
                num_elements *= static_cast<size_t>(concrete);
            }

            std::vector<float> zeros(num_elements, 0.0f);
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                mem_info, zeros.data(), num_elements,
                concrete_shape.data(), concrete_shape.size()));
        }

        // 计时
        auto t0 = std::chrono::high_resolution_clock::now();

        auto output_tensors = session.Run(
            Ort::RunOptions{nullptr},
            input_names.data(), input_tensors.data(), input_tensors.size(),
            output_names.data(), output_names.size());

        auto t1 = std::chrono::high_resolution_clock::now();
        int latency_us = static_cast<int>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

        std::cout << "  ✓ 推理成功! 延迟: " << latency_us << " us ("
                  << std::fixed << std::setprecision(1) << (latency_us / 1000.0)
                  << " ms)" << std::endl;

        // 打印输出前几个值
        for (size_t i = 0; i < num_outputs; ++i) {
            float* data = output_tensors[i].GetTensorMutableData<float>();
            size_t count = output_tensors[i].GetTensorTypeAndShapeInfo().GetElementCount();

            std::cout << "  " << output_names[i] << " (前 " << std::min<size_t>(count, 15) << " 值): [";
            for (size_t j = 0; j < std::min<size_t>(count, 15); ++j) {
                if (j > 0) std::cout << ", ";
                std::cout << std::fixed << std::setprecision(4) << data[j];
            }
            if (count > 15) std::cout << ", ...";
            std::cout << "]" << std::endl;

            // 统计
            float min_val = data[0], max_val = data[0], sum = 0.0f;
            int nan_count = 0, inf_count = 0;
            for (size_t j = 0; j < count; ++j) {
                float v = data[j];
                if (std::isnan(v)) { nan_count++; continue; }
                if (std::isinf(v)) { inf_count++; continue; }
                if (v < min_val) min_val = v;
                if (v > max_val) max_val = v;
                sum += v;
            }
            float mean = sum / static_cast<float>(count);
            std::cout << "      统计: min=" << min_val << " max=" << max_val
                      << " mean=" << mean;
            if (nan_count > 0 || inf_count > 0) {
                std::cout << " NaN:" << nan_count << " Inf:" << inf_count;
            }
            std::cout << std::endl;
        }

        // ═══════════════════════════════════════════════════════════════════════
        //  4. Benchmark (可选)
        // ═══════════════════════════════════════════════════════════════════════

        if (benchmark_iters > 0) {
            std::cout << "\n── Benchmark (" << benchmark_iters << " 次推理) ──" << std::endl;

            std::vector<int> latencies;
            latencies.reserve(benchmark_iters);

            for (int iter = 0; iter < benchmark_iters; ++iter) {
                auto b0 = std::chrono::high_resolution_clock::now();

                auto results = session.Run(
                    Ort::RunOptions{nullptr},
                    input_names.data(), input_tensors.data(), input_tensors.size(),
                    output_names.data(), output_names.size());

                auto b1 = std::chrono::high_resolution_clock::now();
                latencies.push_back(static_cast<int>(
                    std::chrono::duration_cast<std::chrono::microseconds>(b1 - b0).count()));
            }

            // 统计
            std::sort(latencies.begin(), latencies.end());
            int sum_lat = 0;
            for (int l : latencies) sum_lat += l;

            int avg   = sum_lat / benchmark_iters;
            int p50   = latencies[benchmark_iters / 2];
            int p99   = latencies[benchmark_iters * 99 / 100];
            int worst = latencies.back();

            std::cout << "  avg:  " << avg   << " us (" << (avg/1000.0)   << " ms)" << std::endl;
            std::cout << "  p50:  " << p50   << " us (" << (p50/1000.0)   << " ms)" << std::endl;
            std::cout << "  p99:  " << p99   << " us (" << (p99/1000.0)   << " ms)" << std::endl;
            std::cout << "  worst:" << worst << " us (" << (worst/1000.0) << " ms)" << std::endl;

            float actual_hz = 1'000'000.0f / static_cast<float>(avg);
            std::cout << "  → 理论最大频率: " << std::fixed << std::setprecision(0) << actual_hz << " Hz" << std::endl;

            if (p99 > 10000) {
                std::cout << "  ⚠ p99 延迟 >10ms, 50Hz 推理可能有风险!" << std::endl;
            } else if (p99 > 5000) {
                std::cout << "  ⚠ p99 延迟 >5ms, 注意观测运行时是否有掉帧" << std::endl;
            } else {
                std::cout << "  ✓ p99 延迟 <5ms, 50Hz 推理绰绰有余" << std::endl;
            }
        }

        // ═══════════════════════════════════════════════════════════════════════
        //  5. 兼容性检查
        // ═══════════════════════════════════════════════════════════════════════

        std::cout << "\n── 兼容性检查 ──" << std::endl;

        // 检查输出形状是否合理 (期望 12 关节目标)
        if (num_outputs == 1) {
            auto& shape = input_shapes[0];
            auto oshape = session.GetOutputTypeInfo(0)
                           .GetTensorTypeAndShapeInfo().GetShape();
            size_t out_elems = 1;
            for (auto d : oshape) if (d != -1) out_elems *= static_cast<size_t>(d);

            if (out_elems == 12) {
                std::cout << "  ✓ 输出维度=12, 匹配 12 关节目标" << std::endl;
            } else {
                std::cout << "  ⚠ 输出维度=" << out_elems << ", 期望 12 (12 关节)" << std::endl;
                std::cout << "    → 需要在 ONNXPolicy::infer() 中调整输出映射" << std::endl;
            }
        }

        // 检查输入形状
        bool has_batch_dim = false;
        for (size_t i = 0; i < num_inputs; ++i) {
            auto& shape = input_shapes[i];
            if (shape.size() >= 2 && (shape[0] == -1 || shape[0] == 1)) {
                has_batch_dim = true;
            }
        }
        if (has_batch_dim) {
            std::cout << "  ✓ 检测到 batch 维度, 推理时设为 1" << std::endl;
        }

        std::cout << "\n✓ 模型检查完成" << std::endl;

    } catch (const Ort::Exception& e) {
        std::cerr << "\n✗ ONNX Runtime 错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

#else
// ── 无 onnxruntime 时编译成提示工具 ──

#include <iostream>

int main(int argc, char* argv[])
{
    (void)argc; (void)argv;
    std::cout << "此工具需要 onnxruntime 库。" << std::endl;
    std::cout << "请先安装: sudo apt install libonnxruntime-dev" << std::endl;
    std::cout << "然后在编译时定义 ONNXRUNTIME_AVAILABLE 宏。" << std::endl;
    return 1;
}

#endif  // ONNXRUNTIME_AVAILABLE
