# 调试与验证文档目录

本目录用于保存状态估计、策略验证、实机推进计划和阶段性技术结论。

- `STATE_ESTIMATION_PLAN.md`：状态估计及实机测试的主推进记录。
- `LINEAR_KF_ESTIMATOR.md`：Linear KF接口、坐标系和验证说明。
- `LOW_SPEED_POLICY_RETRAINING_SPEC.md`：新低速策略的训练端审计与部署门槛。

系统长期有效的数据流契约保留在`robot_control/DATA_FLOW.md`，避免与阶段性调试记录混放。
终端 1：启动机器人控制程序

```bash
cd /home/orangepi/unitree_actuator_sdk

sudo ./build/robot_control \
  --onnx robot_control/creeper_flat_model_700_actor.onnx \
  --log \
  --log-file /tmp/keyboard_walk_test.csv \
  2>&1 | tee -i /tmp/keyboard_walk_test.log
```

等待校准、开环站立和 NN 平滑接管全部完成。

终端 2：启动键盘控制

```bash
cd /home/orangepi/unitree_actuator_sdk

python3 robot_control/keyboard_velocity_console.py \
  --vx 0.30 \
  --vy 0.20 \
  --yaw 0.40
```

操作：

- `W/S/A/D/Q/E`：运动
- `+/-`：调整平移速度
- `]/[`：调整转向速度
- `空格`：平滑回零
- `X` 或 `Ctrl+C`：安全停机