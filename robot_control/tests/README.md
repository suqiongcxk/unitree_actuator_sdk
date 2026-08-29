# 测试源码目录

本目录只存放`robot_control`相关的`test_*.cpp`测试与实机诊断入口。

- 纯软件测试可直接从`build/`运行，不访问电机、GPIO或IMU。
- `test_joint_feedback`、`test_damping_packet_loss`等实机诊断程序会访问硬件，必须按源码说明和安全流程人工启动。
- 新增测试时在根目录`CMakeLists.txt`中使用`robot_control/tests/test_xxx.cpp`路径。
- 公共实现仍放在`robot_control/`根目录，测试文件不要复制生产代码。
