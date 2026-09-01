# Creeper 低速四足策略重新训练、导出与部署交付任务书

> 将本文件完整交给训练端 Codex。训练端 Codex 应直接检查工程、修改配置、训练、评估并导出交付物，不能只给建议或示例代码。

## 0. 任务目标与安全边界

为 Creeper 四足机器人训练一个新的低速平地行走 Actor，替代存在明显缺陷的 `model_700.pt`。第一版必须同时做到：

- 零命令稳定站立；
- `vx=0.05~0.20 m/s` 时产生四条腿均参与、可重复的周期步态，而不是只前倾或长期固定某条腿；
- 指令变化连续，关节速度、加速度、力矩、冲击和滑移合理；
- 导出的 ONNX 与当前 C++17 部署端接口逐元素一致；
- 提供足够的仿真证据和黄金向量，使部署端可以进行离线回放、dry-run 和吊带实机分级验收。

`model_700.pt` 及其 ONNX 禁止继续用于真机非零速度控制，但可作为接口对拍和失败行为对照基线。

必须区分以下三个概念：

1. 仿真资产的物理上限；
2. 训练中的软平滑约束；
3. 真机首次调试时的保护阈值。

不得把部署端过去用于排查旧模型故障的 `3 rad/s` 反馈速度门槛当成训练硬上限。它不是已确认的 GD8010 机械极限，会过度压制步态。当前 URDF 中的 `30.1 rad/s` 也只是仿真资产上限，且历史文档表明它参考了 Go2 参数；在没有电机厂商数据或实测依据前，同样不能宣称它是 Creeper 真机可长期运行速度。

本任务交付的是“接口兼容并通过仿真验收的实机候选模型”。任何 ONNX 都不能跳过 Python/C++ 对拍、离线回放、dry-run、吊带和短脉冲实机验证而直接自由行走。

## 1. 部署端不可改变的接口契约

### 1.1 坐标系、单位与频率

- 机体系：`+X` 向前，`+Y` 向左，`+Z` 向上；
- 线速度：`m/s`；
- 角速度：`rad/s`；
- 关节位置：输出端 `rad`；
- 关节速度：输出端 `rad/s`；
- 策略频率：`50 Hz`，每帧 `0.02 s`；
- 仿真可使用 `physics_dt=0.005 s`、`decimation=4`，但 Actor 必须仍以 50 Hz 更新。

### 1.2 关节顺序

以下顺序在 observation、action、默认姿态、日志和 ONNX 输出中都必须完全一致：

```text
0  FL_hip_joint
1  FR_hip_joint
2  RL_hip_joint
3  RR_hip_joint
4  FL_thigh_joint
5  FR_thigh_joint
6  RL_thigh_joint
7  RR_thigh_joint
8  FL_calf_joint
9  FR_calf_joint
10 RL_calf_joint
11 RR_calf_joint
```

默认关节位置为：

```text
[ 0.1, -0.1,  0.1, -0.1,
  0.8,  0.8,  1.0,  1.0,
 -1.5, -1.5, -1.5, -1.5 ] rad
```

训练仿真的关节输出端 PD 基准为：

```text
Kp_joint = 25.0
Kd_joint = 0.5
gear_ratio = 6.333
```

GO-M8010-6 真机接口中的增益是电机转子端语义，部署时应按减速比平方换算：

```text
Kp_motor = Kp_joint / gear_ratio^2 ≈ 0.623
Kd_motor = Kd_joint / gear_ratio^2 ≈ 0.0125
```

当前实机使用的约 `Kp=0.625、Kd=0.0125` 与上述换算一致。训练端必须保留输出端 `25/0.5` 的动力学语义，不能把真机转子端的小增益直接填入仿真，也不能把减速比重复换算两次。若训练工程中的执行器模型不是理想关节 PD，必须写清它如何实现相同的输出端等效增益和力矩限制。

### 1.3 Actor 的 48 维 observation

Actor 输入必须是未经额外归一化、裁剪、堆叠或历史拼接的 `float32[1,48]`：

| 下标 | 数量 | 内容 | 坐标系/语义 |
|---|---:|---|---|
| `0..2` | 3 | `base_linear_velocity` | 机体系 `vx,vy,vz`，m/s |
| `3..5` | 3 | `base_angular_velocity` | 机体系 `wx,wy,wz`，rad/s |
| `6..8` | 3 | `projected_gravity` | 世界重力方向投影到机体系，无量纲 |
| `9..11` | 3 | `velocity_command` | `[vx, vy, yaw_rate]`，机体系 |
| `12..23` | 12 | `joint_position - default_joint_position` | 上述关节顺序，rad |
| `24..35` | 12 | `joint_velocity` | 上述关节顺序，rad/s；默认速度为 0 |
| `36..47` | 12 | `previous_raw_action` | 上一策略帧 Actor 的原始 12 维输出 |

首帧 `previous_raw_action` 必须全零；后续第 `t` 帧的 `observation[36:48]` 必须等于第 `t-1` 帧 Actor 原始输出，不能使用安全层修改后的目标、平滑目标或由目标位置反算的 action。

### 1.4 Actor 输出和目标位置公式

- 单输出 `actions`，类型 `float32`，形状严格为 `[1,12]`；
- 输出是确定性 Actor mean 的原始 action；
- 输出层不增加 `tanh`，除非训练环境原本就在完全相同的位置使用并且导出/部署对拍证明一致；
- 目标关节位置必须按下式生成：

```text
q_des[i] = default_joint_position[i] + 0.25 * raw_action[i]
```

不得把减速比再次乘进 observation、action、`q_des`、关节位置或关节速度。上述量全部是关节输出端语义。

### 1.5 ONNX 格式

候选 ONNX 必须满足：

```text
input name:   obs
input shape:  [1, 48]
input dtype:  float32
output name:  actions
output shape: [1, 12]
output dtype: float32
opset:        18（若工具链确有约束，可使用部署端 ONNX Runtime 支持的版本并说明）
provider:     CPUExecutionProvider 可运行
```

Actor 不得依赖 critic 的 privileged observation、随机噪声、外部 recurrent state 或部署端没有的数据。

## 2. 修改前必须完成的代码审计

先定位并记录文件名、行号、原始值和结论，然后继续完成修复与训练，不要只停在审查报告：

1. observation 拼接是否与第 1.3 节逐项一致；
2. action 顺序、默认姿态和 `action_scale=0.25` 是否与第 1.2、1.4 节一致；
3. 四条腿的关节 axis、方向、限位、执行器、初始状态、足端 link、碰撞体和接触传感器是否成对对称；
4. 所有按腿索引的 reward、termination、reset、contact、gait phase 和 curriculum 是否包含 FL/FR/RL/RR，是否存在漏腿、错序或符号错误；
5. 训练使用的是关节输出端还是电机转子端量，是否错误重复应用 `6.333` 减速比；
6. PD 是否遵守第 1.2 节的输出端/转子端换算，力矩限幅、速度限幅、关节摩擦、延迟和随机化是否有来源，是否与真实机器人量级一致；
7. 命令采样是否真正覆盖零命令和 `vx=0.05/0.10/0.15/0.20 m/s`，低速命令保持时间是否足以形成完整步态；
8. `model_700.pt` 为什么会只前倾、不形成完整四足步态，并给出可复现基线数据。

如果发现资产或索引错误，先做最小修复并保存 diff；不要用提高某个总 reward 权重掩盖根因。

## 3. 训练目标与课程

### 3.1 第一版命令范围

第一版聚焦平地：

```text
零命令: vx=0
前进:   vx=0.05~0.20 m/s
vy=0
yaw_rate=0
```

每个 episode 中应包含足够比例的零命令，并使用连续斜坡而不是瞬间阶跃进入和退出速度命令。建议先学习稳定站立和 `0.10~0.20 m/s`，再逐步扩展到 `0.05 m/s`，因为极低速下步态更容易退化为前倾或拖步。

第一版通过后再扩展后退、侧移、偏航和复杂地形，不能为了覆盖更多命令牺牲当前低速步态完整性。

### 3.2 必须优化的行为

- 零命令下保持稳定四足支撑，不持续漂移或高频抖动；
- 非零命令下四条腿均周期离地和落地，不允许任一条腿在整个稳定段固定；
- 跟踪目标速度，同时减少足端拖地、支撑期切向滑移、落足冲击、base contact 和姿态大幅摆动；
- 保持左右、前后合理对称，但不要强迫瞬时 action 完全对称；
- 对 action rate、关节加速度、力矩变化率和机械功率使用软惩罚，权重应通过消融实验调整；
- 不得通过把关节速度硬裁剪到 `3 rad/s`、把每帧动作压得极小，或限制“同一帧最多只能动 3 个关节”来换取平滑。这些做法会阻碍正常协调步态。

### 3.3 随机化与 sim-to-real

在策略已经学会正确名义步态后逐步加入，而不是一开始就施加最大随机化：

- 质量、质心、惯量；
- 地面摩擦和轻微坡度/不平整；
- 电机强度、PD、关节摩擦；
- observation 噪声和偏置；
- 0~2 个控制帧的延迟或与实机测量一致的延迟范围；
- 小幅外力扰动。

所有范围必须写入最终 manifest；禁止使用无法解释的超大随机化掩盖模型或资产错误。

## 4. 仿真验收协议

### 4.1 测试矩阵

至少使用 10 个未参与调参的固定 seed。每个 seed、每个 `vx=0.05/0.10/0.15/0.20 m/s` 都从 fresh reset 开始并执行：

```text
零命令稳定 5 s
1 s 连续斜坡到目标速度
目标速度保持 10 s
1 s 连续斜坡回零
回零后稳定 5 s
```

另做不少于 50 个 episode 的域随机化压力测试，并单独报告名义环境和随机化环境结果，不能混成一个平均值。

### 4.2 必须全部满足的硬门槛

- 无 NaN/Inf、摔倒、base contact、非 time-out reset 或物理关节限位越界；
- 命令稳定段四条腿都存在重复离地事件；每条腿至少 2 次，且不得长期固定某条腿；
- 离地事件判定至少同时使用接触力和相对落足基线的足端高度，阈值与持续时间写入报告；
- 速度跟踪误差、横向漂移、yaw 漂移和 base 姿态不得随时间发散；
- 所有帧最大 `|delta q_des| <= 0.10 rad`，即 50 Hz 下目标轨迹变化率不超过 `5 rad/s`；
- `|delta q_des|` 的全关节合并 `p99.9 <= 0.06 rad/帧`，绝对最大值只允许是孤立事件，不能连续贴近 `0.10 rad/帧`；
- 实际关节速度全关节合并 `p99.9 <= 8 rad/s`、绝对峰值 `<=12 rad/s`，且峰值不得连续出现或伴随限位碰撞、异常冲击或力矩饱和；
- 回零连续，不得突然切换默认姿态或集中跳变，最终恢复稳定四足支撑。

这里的 `8/12 rad/s` 是低速策略候选模型的性能与风险验收包络，不是 GD8010 厂商机械极限，也不应作为训练时逐步硬裁剪值。它比过去的 `3 rad/s` 调试门槛宽松很多，足以避免过度限制自然步态；若合格步态仍需要超过该包络，必须提交关节、相位、持续帧数、力矩、接触冲击、URDF/电机依据和消融对比后再评审，不能自行放宽。

不再把 `sum(|delta q_des|)` 或“同帧变化关节数”设为训练硬门槛，因为四足步态本来就需要多个关节协同。但必须报告它们的 mean、p99、p99.9、max 和最长连续高值帧数，供部署端根据新模型分布重新设置保护阈值。

### 4.3 必须报告的统计量

对每个 seed、速度、腿和关节分别输出，而不仅是所有样本的平均值：

- 实际 `vx` 的 mean/std/RMSE/p5/p95，`vy` 和 yaw 漂移；
- 每条腿离地次数、首次离地时间、摆动/支撑时长、duty factor、足端净空和滑移距离；
- base 高度、roll、pitch、角速度；
- raw action、`q_des`、`delta q_des`、实际关节位置/速度/加速度；
- 力矩、功率、力矩饱和占比和落足冲击；
- `sum(|delta q_des|)`、同帧超过 `0.03/0.05/0.08 rad` 的关节数；
- 每项的 mean、p99、p99.9、max、峰值所在关节/腿/时间和连续帧数。

必须同时运行同一评估脚本测试 `model_700.pt`，以证明新策略确实消除了“只前倾、无完整步态和持续大目标跳变”，而不是仅获得更高 reward。

## 5. 导出、数值对拍与交付物

### 5.1 导出前自检

固定 checkpoint、配置、seed 和导出脚本。验证：

1. PyTorch Actor 与 ONNX 对相同 observation 的输出逐元素一致；
2. observation 确实是 Actor 最终收到的 48 维张量，不是手工重建的近似值；
3. 首帧 previous action 为零，后续帧等于上一帧 raw action；
4. `q_des` 严格符合 `default + 0.25 * raw_action`；
5. ONNX 不包含训练专用随机噪声、critic 或外部状态。

至少导出以下连续轨迹的黄金向量：

- 零命令 20 帧；
- 斜坡进入 `vx=0.05/0.10/0.20 m/s` 各不少于 100 帧；
- 稳定行走和回零各不少于 250 帧；
- 合计不少于 1000 个连续策略帧，不能把彼此独立的单帧拼接冒充时序测试。

每帧保存：48 维 observation、12 维 raw action、12 维 `q_des`、12 维实际关节位置/速度、base 状态、四足接触和命令。

PyTorch/ONNX 对拍建议门槛：

```text
max_abs_error <= 1e-5
max_rel_error <= 1e-4（接近零的元素以绝对误差为准）
```

### 5.2 固定交付目录

最终创建一个独立目录 `creeper_low_speed_policy_delivery/`，至少包含：

```text
creeper_low_speed_actor.onnx
model_final.pt
deployment_manifest.json
policy_golden_vectors.json
policy_golden_vectors.npz
evaluation_summary.csv
evaluation_summary.json
contact_phase_plots/
training_curves/
TRAINING_AND_DEPLOYMENT_REPORT.md
training_changes.patch
```

`deployment_manifest.json` 必须至少记录：

- checkpoint 和 ONNX 的 SHA-256；
- git commit、IsaacLab/RSL-RL/PyTorch/ONNX/ONNX Runtime 版本；
- task 名称、seed、训练步数和最终配置路径；
- 48 维 observation 顺序、12 维关节顺序、默认姿态和 action scale；
- 策略/物理频率、PD、力矩/速度/关节限位；
- 训练与评估 command 范围、域随机化范围；
- 全部验收统计和 pass/fail；
- 根据新模型实测分布建议的部署端 `max_target_velocity`、`max_feedback_velocity`、聚合变化和连续超限阈值；这些是建议值，不能静默修改真机代码。

### 5.3 最终回复格式

训练端 Codex 的最终回复必须：

1. 给出交付目录的绝对路径；
2. 列出 ONNX/checkpoint/manifest/golden vectors 的 SHA-256；
3. 用表格报告四个目标速度和 10 个 seed 的关键验收结果；
4. 明确写出 `PASS` 或 `FAIL`，逐条对应第 4.2 节；
5. 列出实际修改的文件和原因；
6. 给出部署端 Python/C++ 对拍命令；
7. 给出仍然存在的风险，禁止用“训练 reward 很高”代替部署证据。

若训练未收敛、任何硬门槛失败或缺少交付物，必须标记 `FAIL`，继续诊断和迭代；不得把不合格 checkpoint 命名为最终可部署模型。若因算力、依赖或数据缺失确实无法继续，保留已完成产物并准确报告阻塞点，不得伪造训练或评估结果。

## 6. 部署端收到模型后的固定验收顺序

训练端只需在报告中原样保留此顺序，不得声称仿真通过即可直接自由行走：

1. 检查 SHA-256、ONNX I/O 和 manifest；
2. Python 与 C++ 黄金向量逐元素对拍；
3. 现有 4477 帧及后续实机日志离线回放；
4. `robot_control --dry-run --log`，电机只保持安全阻尼；
5. 吊带下零命令接管；
6. 吊带下 `0.05 m/s`、1 秒短脉冲；
7. 依次测试 `0.10/0.15/0.20 m/s`，逐级延长；
8. 每一级确认无 MotionSafety 失败、电机 MError、过流/欠压、异常温升、滑移、撞限位和剧烈冲击后才能进入下一级。

所有实机阶段必须保留命令看门狗、状态陈旧检测、12 电机统一阻尼停机和 `Ctrl+C` 急停。部署端保护阈值应根据新模型仿真/回放分布和电机资料单独调整，不能为了让模型通过而直接关闭保护。
