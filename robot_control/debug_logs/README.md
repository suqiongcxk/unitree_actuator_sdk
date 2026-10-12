# 调试与验证文档目录

./push_wechat_project.sh "完成微信登录界面"



本目录用于保存状态估计、策略验证、实机推进计划和阶段性技术结论。

当前PC键盘多键遥控的修改原因、安全设计和使用方法见
`../../remote_control/docs/KEYBOARD_REMOTE_CONTROL.md`。真正的组合键客户端必须运行在连接物理
键盘的Windows PC本地，不能在SSH终端内运行。

- `STATE_ESTIMATION_PLAN.md`：状态估计及实机测试的主推进记录。
- `LINEAR_KF_ESTIMATOR.md`：Linear KF接口、坐标系和验证说明。
- `LOW_SPEED_POLICY_RETRAINING_SPEC.md`：新低速策略的训练端审计与部署门槛。
- `URDF_20260906_MIGRATION.md`：新旧 URDF 差异、代码同步范围与复查结果。
- `MODEL_950_DEPLOYMENT_VALIDATION.md`：model_950 交付物、数值对拍和实机准入结论。


请为新策略模型提供一个独立的版本化交付目录，包含：

1. creeper_model_<版本号>_actor.onnx
2. creeper_model_<版本号>_actor.onnx.sha256
3. model_<版本号>.pt
4. policy_golden_vectors.json
5. deployment_manifest.json
6. simulation_report.md

要求：
- ONNX输入严格为 float32 [1,48]，名称为 obs；
- ONNX输出严格为 float32 [1,12]，名称为 actions；
- golden vectors 至少包含100个连续帧；
- 每帧包含完整48维 observation、12维Actor原始输出和12维q_des；
- golden vectors 必须包含 previous_raw_action 的连续递归关系；
- manifest记录模型版本、checkpoint SHA256、ONNX SHA256、Git提交、
  URDF SHA256、opset、策略频率、default pose和action_scale；
- simulation report至少报告零速度站立、前进、后退、左右平移、
  左右转向测试，以及NaN、跌倒、非法接触和电机错误统计。

系统长期有效的数据流契约保留在`robot_control/DATA_FLOW.md`，避免与阶段性调试记录混放。
终端 1：启动机器人控制程序

```bash
cd /home/orangepi/unitree_actuator_sdk

sudo ./build/robot_control \
  --log \
  --log-file /tmp/v45_keyboard_walk.csv \
  2>&1 | tee -i /tmp/v45_keyboard_walk.log
```

等待校准、开环站立和 NN 平滑接管全部完成。

终端 2：启动键盘控制

```bash
cd /home/orangepi/unitree_actuator_sdk

python3 remote_control/keyboard/keyboard_velocity_console.py \
  --vx 0.20 \
  --vy 0.20 \
  --yaw 0.50
```

操作：

- `W/S/A/D/Q/E`：运动
- `+/-`：以0.1 m/s独立调整前后速度，范围0.2~1.0 m/s
- `>/<`：以0.1 m/s独立调整横移速度，范围0.2~0.3 m/s
- `]/[`：以 0.1 rad/s 调整转向速度；最低为 0.5 rad/s
- `空格`：平滑回零
- `X` 或 `Ctrl+C`：安全停机
