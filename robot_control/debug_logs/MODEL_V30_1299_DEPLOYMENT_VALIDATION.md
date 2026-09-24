# V30 model_1299 部署与验证记录

日期：2026-09-22

## 版本身份

```text
Delivery ID: creeper_v30_model1299
ONNX: model_deliveries/model_0922_candidate/creeper_v30_model1299/creeper_model_v30_model1299_actor.onnx
ONNX SHA256: 280e3429e72573be96955a1341a9c00e492838db004b0b51c982279e97f45382
Checkpoint: model_1299.pt
Checkpoint SHA256: 4285d2262ed122da182bba8e0790a8c477f11ad31940ad2fb1d502721da3a7cd
```

旧 `model_950` 与 `model_700` 均保留，未覆盖或删除。自2026-09-24起，上面的
版本化ONNX路径已设为 `robot_control` 默认模型；`--onnx <path>` 可覆盖默认模型，
`--standing-policy` 可显式回退StandingPolicy。`--step7-diagnostics` 仍自动使用
StandingPolicy，不改变原有诊断语义。

## 接口核对

- 输入：`obs`, float32 `[1,48]`；
- 输出：`actions`, float32 `[1,12]`；
- observation 顺序与实机 `PolicyObservationBuilder` 一致；
- action/关节顺序：`FL,FR,RL,RR` 的 hip、thigh、calf Z 字排列；
- 默认关节位置与 `RobotControlConfig::default_standing_pose` 一致；
- `action_scale=0.25`；
- `q_des=default_joint_position+0.25*raw_action`；
- `observation[36:48]` 为上一帧 Actor 原始输出；
- 策略频率 50 Hz。

## 香橙派纯软件验收

1. ONNX 自带 SHA256：PASS。
2. deployment manifest 记录的 50 个文件大小与 SHA256：全部 PASS。
3. V30/model1299 200 帧 C++/ONNX golden-vector replay：PASS。

```text
obs command semantics        max_abs=0
obs raw-action history       max_abs=0
fixed-input ONNX parity      max_abs=7.152557e-07
C++ q_des vs Python q_des    max_abs=2.384186e-07
```

4. model_700 旧格式 20 帧回归：PASS。
5. model_950 旧格式 100 帧回归：PASS。
6. `test_smooth_takeover`：PASS，100 帧接管最大单帧变化 `0.00359952 rad`。
7. `test_policy_observation`：PASS。
8. `test_nn_validation`：PASS。
9. `robot_control` 完整重新编译：PASS。

交付外层 `model_0922_candidate/README.md` 仍写着“本轮未生成V30”，与实际 V30
目录、manifest、哈希和验证证据冲突，判定为训练端遗留模板文字。未修改交付原件；
它不改变已校验文件的内容，但训练端后续应修正文档，避免版本审计歧义。

`test_policy_golden_vectors` 已兼容旧展开格式和新版 compact delivery v1。新版文件
不含拆分后的 IMU/关节原始状态，因此用它验证最终48维 observation 的命令位置、
raw-action 历史、ONNX输出和 q_des；`PolicyObservationBuilder` 的完整拼接继续由旧
golden vectors和独立单元测试覆盖。

## 准入状态

```text
文件完整性：PASS
接口兼容性：PASS
香橙派数值对拍：PASS
生产程序编译：PASS
实机电机测试：NOT TESTED
```

训练端报告明确声明硬件状态为 `NOT_TESTED`。其离线 PASS 使用用户暂定的
`0.50 rad/20 ms` 门槛；历史 `0.10 rad/20 ms` 标准仍为 FAIL。以上结果允许进入
分级实机验证，但不等于已经证明真实电机、电源和机械结构安全。

## 后续分级命令

第一阶段只加载模型并让电机保持阻尼，不执行校准、站立或 NN 电机下发：

```bash
cd /home/orangepi/unitree_actuator_sdk

sudo ./build/robot_control \
  --onnx model_deliveries/model_0922_candidate/creeper_v30_model1299/creeper_model_v30_model1299_actor.onnx \
  --monitor-only \
  --log \
  --log-file /tmp/model_v30_1299_monitor.csv \
  2>&1 | tee -i /tmp/model_v30_1299_monitor.log
```

通过 monitor-only 后，下一阶段才允许使用 `--dry-run --compare` 执行既有机械标定
和开环站立；真实 NN 接管必须再次单独批准，不得由部署步骤自动执行。

## 回滚

如新模型验证失败，停止程序并继续显式使用旧模型：

```text
model_deliveries/model_950/creeper_model_950_actor.onnx
```

模型选择不依赖软链接，因此不存在软链接被静默改写导致版本身份不清的问题。
