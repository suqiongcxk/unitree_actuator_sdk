# V46 model_1250 ONNX与Sim2Sim验收报告

日期：2026-10-01

## 验收状态

- Isaac Status: PASS
- MuJoCo Status: PASS
- Hardware Status: NOT_TESTED

## 模型与接口

- Checkpoint：`model_1250.pt`，SHA-256：`739fe6e3215f4df0fee38a037114f3f5475ce85f91c043275372b90c010c2b94`
- ONNX SHA-256：`2893426e5d3cf0c42dbbaef6236cbc3b38e889f89fda8acc8ce464197f913cfc`
- 接口：`obs float32[1,48] -> actions float32[1,12]`，静态shape。
- PyTorch/ONNX最大绝对误差：`2.384185791015625e-06`。
- `q_des=default_joint_position+0.25*raw_action`；50 Hz policy、200 Hz physics。
- Actor确定性raw action包含逐关节平滑安全边界；ONNX内部不包含default pose、action scale、PD、filter或rate limiter。

## Isaac结果

原始证据：`v46_model1250_isaac_multiseed.json`。seeds 42/43/44、每工况4环境，324条轨迹；覆盖standing、`vx=±0.2～±1.0`、`vy=±0.2/±0.3`、`yaw=±0.5/±1.0`、前进+横移、前进+转向、三轴组合、启动、停车和方向切换。

- 约972,000个关节-policy控制步的`q_des`越界：0。
- 相对实机0.05 rad安全范围的最小余量：`0.1162 rad`。
- reset/base/non-foot contact：`0/0/0`。
- 最大`Δq_des`：`0.4791 rad/20 ms`。
- 实际速度：`vx=0.2 -> 0.194`、`vx=0.4 -> 0.391`、`vx=1.0 -> 0.973 m/s`；`vy=0.3 -> 0.287 m/s`；`yaw=1.0 -> 0.986 rad/s`。

## MuJoCo结果

原始证据位于`evidence/mujoco/`，使用未修改的`Creeper20260906.xml`、PD和控制链。

- 19组固定命令/启停测试及连续`+0.4 -> -0.4 m/s`方向切换均无fall/base/non-foot contact。
- 全部逐帧`q_des`越界：0；最小安全余量：`0.2199 rad`。
- 最大`Δq_des`：`0.3459 rad/20 ms`；方向切换阶段最大`0.1380 rad/20 ms`。
- 实际速度：`vx=0.2 -> 0.199`、`vx=0.4 -> 0.400`、`vx=1.0 -> 1.009 m/s`；`vy=0.3 -> 0.290 m/s`；`yaw=1.0 -> 0.970 rad/s`。
- standing平均height=`0.284 m`、平均pitch约`-0.006 rad`。

## q_des安全边界

实机calf安全范围为`[-2.6727,-0.88776] rad`。V46确定性Actor使用机械限位内缩0.06 rad的范围，即calf为`[-2.6627,-0.89776] rad`。该0.06 rad由实机要求0.05 rad和0.01 rad数值储备构成。极端输入离线扫描768组，仍未越过实机安全范围。

## 判定边界

- Isaac与MuJoCo离线验收：PASS。
- 当前`0.50 rad/20 ms`只是暂定离线工程门槛，不是硬件物理极限。
- Hardware：NOT_TESTED。实机仍需悬空、急停、低速限幅、telemetry、电流/温度/跟踪误差和真实前倾复核。
- V45曾在实机发生`FR_calf_joint`目标越界；本报告不追溯修改V45历史结果，也不将V46离线PASS表述为实机安全PASS。
