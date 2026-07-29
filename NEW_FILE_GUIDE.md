# 新建 .cpp 文件接入指南

> 适用项目：`unitree_actuator_sdk`  
> 构建系统：CMake（**非** Python，没有 `__init__.py` 或 `import` 机制）  
> 最后更新：2026-07-28

---

## 1. 核心概念

本项目使用 **CMake** 管理编译。C++ 中不存在 Python 的 `import` —— 取而代之的是两个独立步骤：

| 步骤 | C++ 机制 | 对应 CMake 命令 |
|------|----------|-----------------|
| **编译期** 让其他文件"看见"你的头文件 | `#include` | `include_directories(...)` |
| **链接期** 让其他库/可执行文件调用你的函数 | 符号链接 | `target_link_libraries(...)` |

**关键文件只有一个**：项目根目录的 **`CMakeLists.txt`**。添加任何新 `.cpp` 都必须修改它。

---

## 2. 当前项目布局

```
unitree_actuator_sdk/
├── CMakeLists.txt          ← 唯一需要改动的文件
├── include/                ← 公共头文件目录（已在 include path 中）
├── motor_lib/              ← 电机控制库（已在 include path 中）
│   ├── motor_controller.cpp / .h
│   ├── fast_gpio.cpp / .h
│   ├── parallel_bus.cpp / .h
│   └── ...
├── State_Estimation/       ← 状态估计（已在 include path 中）
│   ├── jy901s.cpp / .h
│   └── test_jy901s.cpp
├── example/
├── lib/
└── build/
```

**已有 include path**（CMakeLists.txt 第 7 行）：
```cmake
include_directories(include motor_lib State_Estimation)
```

这意味着这三个目录下的所有 `.h` 文件，可以被项目任意位置的 `.cpp` 通过 `#include "xxx.h"` 直接引用，**无需额外配置**。

---

## 3. 场景一：在 State_Estimation 中新建一个独立的库

> **适用场景**：你要写一个全新模块（如神经网络推理、卡尔曼滤波器），需要同时提供 `.h` 和 `.cpp`，并可能被 `motor_lib` 或其它模块调用。

### 步骤

#### ① 创建源文件

```cpp
// State_Estimation/nn_predictor.h
#ifndef NN_PREDICTOR_H
#define NN_PREDICTOR_H

#include <vector>

class NNPredictor {
public:
    NNPredictor();
    ~NNPredictor();

    /// 加载模型权重
    bool loadModel(const char* model_path);

    /// 输入传感器数据，输出关节力矩预测
    std::vector<float> predict(const std::vector<float>& sensor_input);

private:
    void* model_handle_;  // 内部模型句柄
};

#endif  // NN_PREDICTOR_H
```

```cpp
// State_Estimation/nn_predictor.cpp
#include "nn_predictor.h"
#include <cstdio>

NNPredictor::NNPredictor() : model_handle_(nullptr) {}

NNPredictor::~NNPredictor() {
    // 释放模型资源
}

bool NNPredictor::loadModel(const char* model_path) {
    printf("[NNPredictor] Loading model from %s\n", model_path);
    // TODO: 实现模型加载
    return true;
}

std::vector<float> NNPredictor::predict(const std::vector<float>& sensor_input) {
    // TODO: 实现推理
    return {};
}
```

#### ② 修改 CMakeLists.txt

在文件末尾添加两行：

```cmake
# 神经网络预测器
add_library(nn_predictor State_Estimation/nn_predictor.cpp)
```

如果 `nn_predictor` 自身依赖 `jy901s` 的数据结构，则追加链接：

```cmake
target_link_libraries(nn_predictor jy901s)
```

#### ③ 让 motor_lib 中的代码能调用它

假设 `motor_controller` 想使用 `nn_predictor`，只需在已有 `target_link_libraries` 中追加：

```cmake
# 修改前：
target_link_libraries(motor_controller fast_gpio ${EXTRA_LIBS})

# 修改后：
target_link_libraries(motor_controller fast_gpio nn_predictor ${EXTRA_LIBS})
```

然后在 `motor_controller.cpp` 中：

```cpp
#include "nn_predictor.h"   // 头文件已在 include path 中，可直接引用

// 使用示例：
NNPredictor predictor;
predictor.loadModel("/path/to/model.bin");
auto torques = predictor.predict(sensor_data);
```

> **为什么 `#include "nn_predictor.h"` 能直接工作？**  
> 因为 `State_Estimation/` 已在 `include_directories()` 中，CMake 会将该目录传递给编译器的 `-I` 参数。

---

## 4. 场景二：新建一个纯头文件（header-only）

> **适用场景**：只需定义数据结构、常量、或内联工具函数，不需要 `.cpp`。

### 步骤

#### ① 创建头文件

```cpp
// State_Estimation/sensor_types.h
#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

struct IMUData {
    float acc_x, acc_y, acc_z;    // 加速度 (m/s²)
    float gyro_x, gyro_y, gyro_z; // 角速度 (rad/s)
    float roll, pitch, yaw;       // 欧拉角 (度)
    float q_w, q_x, q_y, q_z;     // 四元数
};

struct JointState {
    float position;   // rad
    float velocity;   // rad/s
    float torque;     // N·m
};

#endif
```

#### ② 无需修改 CMakeLists.txt

因为 `State_Estimation/` 已在 `include_directories` 中，任何位置的 `.cpp` 都可以直接：

```cpp
#include "sensor_types.h"
```

**不需要**修改 CMakeLists.txt 的库或可执行目标定义。

---

## 5. 场景三：新建一个可执行测试程序

> **适用场景**：你要写一个独立的测试/调试工具，编译后直接运行。

### 步骤

#### ① 创建源文件

```cpp
// State_Estimation/test_nn_predictor.cpp
#include <iostream>
#include "nn_predictor.h"

int main() {
    NNPredictor pred;
    if (!pred.loadModel("model.bin")) {
        std::cerr << "Failed to load model\n";
        return 1;
    }

    std::vector<float> input = {0.1, 0.2, 0.3, 9.8};
    auto output = pred.predict(input);

    std::cout << "Prediction OK, output size = " << output.size() << "\n";
    return 0;
}
```

#### ② 修改 CMakeLists.txt

在文件末尾添加：

```cmake
add_executable(test_nn_predictor State_Estimation/test_nn_predictor.cpp)
target_link_libraries(test_nn_predictor nn_predictor jy901s)
```

#### ③ 编译和运行

```bash
cd build
cmake .. && make test_nn_predictor
sudo ./test_nn_predictor
```

---

## 6. 场景四：在 motor_lib 中新建文件

> **适用场景**：新文件属于电机控制范畴，放在 `motor_lib/` 下更合理。

步骤与场景一完全相同，只是路径从 `State_Estimation/` 换成 `motor_lib/`：

```cmake
# CMakeLists.txt
add_library(your_new_lib motor_lib/your_new_file.cpp)
target_link_libraries(your_new_lib fast_gpio ${EXTRA_LIBS})  # 按需链接
```

---

## 7. 完整速查表

| 你要做什么 | 创建文件 | 需改 CMakeLists.txt？ | 改哪里 |
|-----------|---------|----------------------|--------|
| 添加纯头文件（只有 `.h`） | `xx/foo.h` | **否** | 无（`include_directories` 已覆盖） |
| 添加新库（`.h` + `.cpp`） | `xx/foo.h` + `xx/foo.cpp` | **是** | 加 `add_library(foo xx/foo.cpp)` |
| 让 `motor_controller` 调用新库 | — | **是** | 在 `target_link_libraries(motor_controller ...)` 中追加 `foo` |
| 添加测试/可执行文件 | `xx/test_foo.cpp` | **是** | 加 `add_executable(test_foo xx/test_foo.cpp)` + `target_link_libraries(test_foo foo ...)` |
| 新目录不在 include path 中 | — | **是** | `include_directories(...)` 中追加新目录名 |

---

## 8. 常见错误与排查

### ❌ `fatal error: xxx.h: No such file or directory`

**原因**：头文件所在目录不在 `include_directories` 中。

**修复**：
```cmake
# 在 CMakeLists.txt 第 7 行追加新目录
include_directories(include motor_lib State_Estimation 你的新目录)
```

### ❌ `undefined reference to 'YourClass::yourMethod()'`

**原因**：编译通过但链接失败——调用了某库的函数但未链接该库。

**修复**：在调用方的 `target_link_libraries` 中追加被调用的库名。

### ❌ 修改 CMakeLists.txt 后不生效

**原因**：CMake 缓存未更新。

**修复**：
```bash
cd build
cmake ..          # 重新生成 Makefile
make -j$(nproc)   # 重新编译
```

如果仍然异常，清除缓存重建：
```bash
rm -rf build/CMakeCache.txt build/CMakeFiles
cd build && cmake .. && make -j$(nproc)
```

---

## 9. 依赖关系图示例

假设你新增了 `nn_predictor`，以下是目标间的依赖拓扑：

```
test_nn_predictor ──→ nn_predictor ──→ jy901s
                                          │
motor_controller ──→ fast_gpio ──→ ${EXTRA_LIBS}
    │
    └──→ nn_predictor   ← 新增！让 motor_controller 也能调用它
```

对应的 CMakeLists.txt 片段：

```cmake
# 每行对应图中一个节点
add_library(jy901s         State_Estimation/jy901s.cpp)

add_library(nn_predictor   State_Estimation/nn_predictor.cpp)
target_link_libraries(nn_predictor jy901s)

add_library(motor_controller motor_lib/motor_controller.cpp)
target_link_libraries(motor_controller fast_gpio nn_predictor ${EXTRA_LIBS})
#                                                   ↑ 新增

add_executable(test_nn_predictor State_Estimation/test_nn_predictor.cpp)
target_link_libraries(test_nn_predictor nn_predictor jy901s)
```

---

## 10. 注意事项

1. **没有 `__init__.cpp` 这种机制** —— C++/CMake 不同于 Python，每个 `.cpp` 必须显式写在 `add_library` 或 `add_executable` 中。

2. **没有环境变量需要设置** —— 所有编译配置都在 `CMakeLists.txt` 中声明。

3. **头文件守卫** —— 每个 `.h` 必须用 `#ifndef` / `#define` / `#endif` 包裹，或使用 `#pragma once`，防止重复包含。

4. **include path 是全局的** —— `include_directories()` 对项目内所有 target 生效。如果两个目录有同名头文件，编译器会按 `include_directories` 中参数的顺序查找，先匹配到的生效。

5. **链接库的顺序有时有讲究** —— 如果 A 依赖 B，通常写 `target_link_libraries(A B)`（B 在后）。CMake 会自动处理大部分情况，但如果遇到奇怪的 `undefined reference`，可以尝试调整顺序。

6. **增量编译** —— 修改 `.cpp` 后只需 `make`，修改 `.h` 或 `CMakeLists.txt` 后建议重新执行 `cmake .. && make`。
