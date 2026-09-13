# Creeper V5 model_950 Actor 导出与部署验证报告

## 结论

Checkpoint 已从真实 V5 task 和 RSL-RL 配置成功恢复，Actor ONNX 导出及 Python/ONNX 数值对拍通过。Isaac 10-seed 行为测试没有跌倒、base contact 或 non-foot contact，但单帧 `q_des` 跳变明显超过真机 `0.10 rad/frame` 门槛。因此总体结论为 **FAIL_FOR_ACTUATED_HARDWARE**；当前只允许香橙派离线对拍和不驱动电机的 dry-run。

## 模型与资产

- Checkpoint：`/workspace/isaaclab/logs/rsl_rl/creeper_20260906_flat_v5/2026-09-11_11-47-06_scratch_seed42_balanced_commands_contact_safe/model_950.pt`
- SHA-256：`df9eb646aa3fbde7b60a4f7f7aed2ffd4d2f07a0681988f7d040161c967bb5ec`
- 顶层键：`model_state_dict`、`optimizer_state_dict`、`iter`、`infos`
- Actor：`48 → 128 → 128 → 128 → 12`，隐藏层 `ELU`，线性输出，无 `tanh`
- 类型：标准 feed-forward `ActorCritic`，非 recurrent
- observation normalizer：`Identity`
- 导出内容：仅确定性 Actor mean；无 critic、采样噪声或 privileged observation
- V5 保存的 `env.yaml` 加载 `Creeper20260906.usd`；转换配置的源文件为已校验 URDF。
- URDF SHA-256：`37ff462cfe9a9c353fa3bb0befc1cda09372f63837fafc30aba96adb04c9ebf0`
- 运行时总质量：`11.932221412658691 kg`，与 URDF `11.932221638634982 kg` 的差异为 float32 表示误差。

## ONNX 接口和数值对拍

- 文件：`creeper_model_950_actor.onnx`
- opset：18
- 输入：`obs`, `float32[1,48]`，固定 batch
- 输出：`actions`, `float32[1,12]`，固定 batch
- CPUExecutionProvider：通过
- 1000 组随机输入最大绝对误差：`9.5367431640625e-7`
- 平均绝对误差：`6.998416068881852e-8`
- 最大相对误差：`0.0015215272`（接近零的输出导致相对误差放大）
- 最大误差位置：样本 345，action index 8
- 连续 100 帧最大绝对误差：`7.152557373046875e-7`
- 首帧 previous action：严格全零
- `obs[t][36:48]` 对上一帧 PyTorch raw action 最大误差：`0.0`
- observation 七个分段及 `q_des` 公式逐元素最大误差：`0.0`
- 机械限位越界：0

## Isaac 10-seed 固定速度验收

协议：每个 seed 独立 reset；5 秒零命令、1 秒 ramp、10 秒保持、1 秒 ramp 回零、5 秒站立。Seeds 为 42–51。

| vx (m/s) | 实际平均 vx | 四足周期步态 | reset/base/non-foot | 回零稳定 |
|---:|---:|---:|---:|---:|
| 0.00 | -0.000058 | 不适用，四脚持续承重 | 0 / 0 / 0 | 10/10 |
| 0.05 | 0.001155 | 0/10 | 0 / 0 / 0 | 10/10 |
| 0.10 | 0.002080 | 0/10 | 0 / 0 / 0 | 10/10 |
| 0.20 | 0.170835 | 10/10 | 0 / 0 / 0 | 10/10 |
| 0.40 | 0.407658 | 10/10 | 0 / 0 / 0 | 10/10 |

`0.05/0.10 m/s` 表现为站立而非“前倾并周期抬腿”。`0.20/0.40 m/s` 四条腿均参与周期步态。`0.40 m/s` 位于训练范围 `lin_vel_x=[-1,1] m/s` 内。

最大关节速度分别为：standing `0.072`、0.05 `0.212`、0.10 `0.673`、0.20 `7.319`、0.40 `8.368 rad/s`。最大力矩分别为 `7.35/8.02/8.40/9.03/11.50 Nm`。

## 阻塞风险

- 连续黄金段最大 `Δq_des=0.218947 rad/frame`，其中 17 个 joint-frame 超过 `0.10 rad`。
- 全行为测试最大 `Δq_des=0.373347 rad/frame`。
- 这些是 Actor 输出经 `default_pose + 0.25*raw_action` 得到的真实目标变化，不是 ONNX 数值误差。
- 在保留“previous_raw_action 必须是未修改 Actor 输出”的接口前提下，部署安全层不能反向覆盖 observation 历史；但实际电机目标仍必须有独立、经验证的安全处理后才可能进入吊带测试。
- 平移部署必须执行 deadzone：`|translation|<0.2 m/s → 0`，并把非零移动起点设为 `0.2 m/s`。

原始逐 trial 数据见 `model_950_simulation_raw.json`，完整 Actor/action 统计见 `validation_results.json`，连续输入输出见 `model_950_policy_golden_vectors.json`。
