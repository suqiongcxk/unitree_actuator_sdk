cmake_minimum_required(VERSION 3.1.0)
project(unitree_actuator_sdk)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 独立 18 维 Linear KF 模块仅依赖 Eigen；缺少 Eigen 时不影响现有电机程序。
find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR Eigen/Core PATH_SUFFIXES eigen3)
    if(EIGEN3_INCLUDE_DIR)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
        set(Eigen3_FOUND TRUE)
    endif()
endif()
if(Eigen3_FOUND)
    add_library(linear_kf_position_velocity_estimator
        robot_control/linear_kf_position_velocity_estimator.cpp)
    target_link_libraries(linear_kf_position_velocity_estimator PUBLIC Eigen3::Eigen)
    target_include_directories(linear_kf_position_velocity_estimator PUBLIC robot_control)
    set_target_properties(linear_kf_position_velocity_estimator PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    add_executable(test_linear_kf_estimator robot_control/tests/test_linear_kf_estimator.cpp)
    target_link_libraries(test_linear_kf_estimator linear_kf_position_velocity_estimator)
    set_target_properties(test_linear_kf_estimator PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    add_executable(test_linear_kf_adapter
        robot_control/tests/test_linear_kf_adapter.cpp
        robot_control/linear_kf_state_estimator_adapter.cpp
        robot_control/state_estimator.cpp
        robot_control/leg_kinematics.cpp
        robot_control/legged_odometry.cpp)
    target_link_libraries(test_linear_kf_adapter linear_kf_position_velocity_estimator)
    target_compile_definitions(test_linear_kf_adapter PRIVATE LINEAR_KF_AVAILABLE)
    set_target_properties(test_linear_kf_adapter PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
else()
    message(STATUS "Eigen3 not found — independent Linear KF target disabled")
endif()

include_directories(include motor_lib State_Estimation robot_control)
link_directories(lib)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
    set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

#example
add_executable(example_a1_motor example/example_a1_motor.cpp)
target_link_libraries(example_a1_motor ${EXTRA_LIBS})

add_executable(example_a1_motor_output example/example_a1_motor_output.cpp)
target_link_libraries(example_a1_motor_output ${EXTRA_LIBS})

add_executable(example_b1_motor example/example_b1_motor.cpp)
target_link_libraries(example_b1_motor ${EXTRA_LIBS})

add_executable(example_goM8010_6_motor example/example_goM8010_6_motor.cpp)
target_link_libraries(example_goM8010_6_motor ${EXTRA_LIBS})

add_executable(changeID example/changeID.cpp)
target_link_libraries(changeID ${EXTRA_LIBS})

set(LIBRARY_OUTPUT_PATH "../lib")
add_subdirectory(thirdparty/pybind11)
pybind11_add_module(unitree_actuator_sdk thirdparty/python_wrapper/wrapper.cpp)
target_link_libraries(unitree_actuator_sdk PRIVATE ${EXTRA_LIBS})
set_target_properties(unitree_actuator_sdk PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")

add_executable(motor_test example/example_goM8010_6_orangepi.cpp)
add_executable(gpio_toggle_test motor_lib/gpio_toggle_test.cpp)
target_link_libraries(gpio_toggle_test fast_gpio)
target_link_libraries(motor_test ${EXTRA_LIBS})

# motor_lib — GO-M8010-6 motor control library
add_library(motor_controller
    motor_lib/motor_controller.cpp
    motor_lib/ZeroPointCalibration.cpp
    motor_lib/emergency_stop.cpp
)
target_link_libraries(motor_controller fast_gpio ${EXTRA_LIBS})

add_executable(example_usage motor_lib/example_usage.cpp)
target_link_libraries(example_usage motor_controller fast_gpio ${EXTRA_LIBS})

# 多路并行总线（fast_gpio + parallel_bus）
add_library(fast_gpio motor_lib/fast_gpio.cpp)
target_link_libraries(fast_gpio ${EXTRA_LIBS})

add_library(parallel_bus motor_lib/parallel_bus.cpp)
target_link_libraries(parallel_bus fast_gpio motor_controller ${EXTRA_LIBS})

add_executable(example_multi_bus motor_lib/example_multi_bus.cpp)
target_link_libraries(example_multi_bus parallel_bus fast_gpio motor_controller ${EXTRA_LIBS} pthread)

# JY901S IMU 传感器驱动
add_library(jy901s State_Estimation/jy901s.cpp)

add_executable(test_jy901s State_Estimation/test_jy901s.cpp)
target_link_libraries(test_jy901s jy901s)

# ── Robot Control System (multi-threaded, IMU + motors + estimation + NN) ──

# ONNX Runtime (可选, 用于神经网络推理)
set(ONNXRUNTIME_DIR "${CMAKE_SOURCE_DIR}/thirdparty/onnxruntime")
if(EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
    message(STATUS "Found ONNX Runtime at ${ONNXRUNTIME_DIR}")
    set(ONNX_FOUND TRUE)
else()
    message(STATUS "ONNX Runtime not found — ONNXPolicy will be disabled")
    set(ONNX_FOUND FALSE)
endif()

add_executable(robot_control
    robot_control/robot_control_main.cpp
    robot_control/robot_controller.cpp
    robot_control/state_estimator.cpp
    robot_control/linear_kf_state_estimator_adapter.cpp
    robot_control/leg_kinematics.cpp
    robot_control/legged_odometry.cpp
    robot_control/policy_observation.cpp
    robot_control/nn_policy.cpp
    robot_control/nn_validation.cpp
    robot_control/motion_safety.cpp
)

# NN 安全验证纯软件测试（不访问 GPIO、串口或电机）
add_executable(test_nn_validation
    robot_control/tests/test_nn_validation.cpp
    robot_control/nn_validation.cpp
)

# Phase 3共享快照并发压力测试（单生产者、快/慢双消费者）。
add_executable(test_double_buffer robot_control/tests/test_double_buffer.cpp)
target_link_libraries(test_double_buffer pthread)

add_executable(test_runtime_safety robot_control/tests/test_runtime_safety.cpp)
target_link_libraries(test_runtime_safety pthread)

# 与策略无关的目标变化率、反馈速度和多关节聚合保护测试。
add_executable(test_motion_safety
    robot_control/tests/test_motion_safety.cpp
    robot_control/motion_safety.cpp)

add_library(velocity_command_manager
    robot_control/velocity_command_manager.cpp)
target_include_directories(velocity_command_manager PUBLIC robot_control)
target_link_libraries(velocity_command_manager pthread)

add_executable(test_velocity_command_manager
    robot_control/tests/test_velocity_command_manager.cpp)
target_link_libraries(test_velocity_command_manager velocity_command_manager)

# 按需构建：cmake --build build --target test_double_buffer_tsan
add_executable(test_double_buffer_tsan EXCLUDE_FROM_ALL
    robot_control/tests/test_double_buffer.cpp)
target_compile_options(test_double_buffer_tsan PRIVATE
    -fsanitize=thread -fno-omit-frame-pointer -g)
target_link_options(test_double_buffer_tsan PRIVATE -fsanitize=thread)
target_link_libraries(test_double_buffer_tsan pthread)

# 状态估计 Step 1 纯软件测试（不访问 IMU、GPIO、串口或电机）
add_executable(test_state_estimator
    robot_control/tests/test_state_estimator.cpp
    robot_control/state_estimator.cpp
    robot_control/leg_kinematics.cpp
    robot_control/legged_odometry.cpp
)

# Step 3 IMU 实机诊断（只访问 I2C，不访问 GPIO、串口或电机）
add_executable(test_projected_gravity
    robot_control/tests/test_projected_gravity.cpp
    robot_control/state_estimator.cpp
    robot_control/leg_kinematics.cpp
    robot_control/legged_odometry.cpp
)

# Step 2 陀螺仪零偏/低通实机诊断（只访问 I2C，不访问电机）
add_executable(test_gyro_calibration
    robot_control/tests/test_gyro_calibration.cpp
    robot_control/state_estimator.cpp
    robot_control/leg_kinematics.cpp
    robot_control/legged_odometry.cpp
)

# Step 7 腿部运动学、接触与速度估计纯软件测试
add_executable(test_legged_odometry
    robot_control/tests/test_legged_odometry.cpp
    robot_control/leg_kinematics.cpp
    robot_control/legged_odometry.cpp
)

# 足端力估计与实机CSV离线分析（纯软件，不访问任何硬件）
add_library(foot_force_estimator robot_control/foot_force_estimator.cpp)
target_include_directories(foot_force_estimator PUBLIC robot_control)

# 所有包含 legged_odometry.cpp 的目标都需要足端力求解器。
if(TARGET test_linear_kf_adapter)
    target_link_libraries(test_linear_kf_adapter foot_force_estimator)
endif()
target_link_libraries(robot_control foot_force_estimator)
target_link_libraries(test_state_estimator foot_force_estimator)
target_link_libraries(test_projected_gravity foot_force_estimator)
target_link_libraries(test_gyro_calibration foot_force_estimator)
target_link_libraries(test_legged_odometry foot_force_estimator)

add_executable(test_foot_force_estimator
    robot_control/tests/test_foot_force_estimator.cpp
    robot_control/leg_kinematics.cpp)
target_link_libraries(test_foot_force_estimator foot_force_estimator)

add_executable(analyze_foot_force_csv
    robot_control/analyze_foot_force_csv.cpp
    robot_control/leg_kinematics.cpp)
target_link_libraries(analyze_foot_force_csv foot_force_estimator)
target_link_libraries(test_projected_gravity jy901s)
target_link_libraries(test_gyro_calibration jy901s)

# Step 6 观测布局与上一帧动作纯软件测试
add_executable(test_policy_observation
    robot_control/tests/test_policy_observation.cpp
    robot_control/policy_observation.cpp
)

# 训练端黄金向量逐元素对拍（纯软件，不链接或访问任何硬件）
if(ONNX_FOUND)
    add_executable(test_policy_golden_vectors
        robot_control/tests/test_policy_golden_vectors.cpp
        robot_control/policy_observation.cpp
    )
    target_include_directories(test_policy_golden_vectors PRIVATE
        robot_control "${ONNXRUNTIME_DIR}/include")
    target_link_directories(test_policy_golden_vectors PRIVATE "${ONNXRUNTIME_DIR}/lib")
    target_link_libraries(test_policy_golden_vectors onnxruntime)
    set_target_properties(test_policy_golden_vectors PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
        INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
        BUILD_RPATH "${ONNXRUNTIME_DIR}/lib")

    # ONNX 接管平滑与外层安全验证纯软件测试（不访问任何硬件）
    add_executable(test_smooth_takeover
        robot_control/tests/test_smooth_takeover.cpp
        robot_control/nn_policy.cpp
        robot_control/nn_validation.cpp
        robot_control/policy_observation.cpp
    )
    target_include_directories(test_smooth_takeover PRIVATE
        robot_control "${ONNXRUNTIME_DIR}/include")
    target_link_directories(test_smooth_takeover PRIVATE "${ONNXRUNTIME_DIR}/lib")
    target_link_libraries(test_smooth_takeover onnxruntime)
    set_target_properties(test_smooth_takeover PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
        INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
        BUILD_RPATH "${ONNXRUNTIME_DIR}/lib")
endif()

# Step 4 关节 ID/URDF 映射纯软件测试（不打开任何硬件设备）
add_executable(test_joint_mapping robot_control/tests/test_joint_mapping.cpp)
target_link_libraries(test_joint_mapping motor_controller fast_gpio ${EXTRA_LIBS} pthread)

# Step 4 实机反馈诊断：仅发送低阻尼零速度报文以取得反馈，不发送位置目标。
add_executable(test_joint_feedback robot_control/tests/test_joint_feedback.cpp)
target_link_libraries(test_joint_feedback
    parallel_bus fast_gpio motor_controller ${EXTRA_LIBS} pthread)

# Step 4 同步 MotorBus 对照诊断：复用机械标定已验证的收发路径。
add_executable(test_joint_feedback_sync robot_control/tests/test_joint_feedback_sync.cpp)
target_link_libraries(test_joint_feedback_sync
    motor_controller fast_gpio ${EXTRA_LIBS} pthread)

# 四路总线500 Hz阻尼通信丢包率实机测试。
add_executable(test_damping_packet_loss robot_control/tests/test_damping_packet_loss.cpp)
target_link_libraries(test_damping_packet_loss
    parallel_bus fast_gpio motor_controller ${EXTRA_LIBS} pthread)
target_include_directories(robot_control PRIVATE robot_control)
target_link_libraries(robot_control
    velocity_command_manager
    parallel_bus
    fast_gpio
    motor_controller
    jy901s
    ${EXTRA_LIBS}
    pthread
)
if(Eigen3_FOUND)
    target_link_libraries(robot_control linear_kf_position_velocity_estimator)
    target_compile_definitions(robot_control PRIVATE LINEAR_KF_AVAILABLE)
    set_target_properties(robot_control PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
endif()
if(ONNX_FOUND)
    target_include_directories(robot_control PRIVATE "${ONNXRUNTIME_DIR}/include")
    target_link_directories(robot_control PRIVATE "${ONNXRUNTIME_DIR}/lib")
    target_link_libraries(robot_control onnxruntime)
    target_compile_definitions(robot_control PRIVATE ONNXRUNTIME_AVAILABLE)
    # 设置 rpath 让可执行文件在运行时找到 .so
    set_target_properties(robot_control PROPERTIES
        INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
        BUILD_RPATH "${ONNXRUNTIME_DIR}/lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
endif()
