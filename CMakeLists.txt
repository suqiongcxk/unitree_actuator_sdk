cmake_minimum_required(VERSION 3.1.0)
project(unitree_actuator_sdk)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

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
add_library(motor_controller motor_lib/motor_controller.cpp motor_lib/ZeroPointCalibration.cpp)
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
    robot_control/nn_policy.cpp
    robot_control/nn_validation.cpp
)
target_include_directories(robot_control PRIVATE robot_control)
target_link_libraries(robot_control
    parallel_bus
    fast_gpio
    motor_controller
    jy901s
    ${EXTRA_LIBS}
    pthread
)
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