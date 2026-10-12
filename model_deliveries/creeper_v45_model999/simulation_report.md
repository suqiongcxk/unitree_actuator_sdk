# V45 model_999 ONNX与Sim2Sim验收报告

日期：2026-10-01

## 验收状态

- Isaac Status: PASS（采用训练一致的command ramp及暂定`0.50 rad/20 ms`离线门槛）
- MuJoCo Status: PASS（采用相同控制接口、command ramp及暂定离线门槛）
- Hardware Status: NOT_TESTED

## 模型与接口

- Checkpoint：`/workspace/isaaclab/logs/rsl_rl/creeper_20260906_flat_v45_width_height_balance/2026-10-01_18-17-51_scratch_seed42_1000iter_width04_trans015_height40/model_999.pt`
- Checkpoint SHA-256：`505e139cfc73af1c4bd4cad56331fdb826622f0db12b9328d7ed676f22707f24`
- ONNX：`creeper_v45_model999_actor.onnx`
- ONNX SHA-256：`77ddfdf7642ba0010e90ddd7360745505ec94989a73e7a3d9632164ed5d489d9`
- 接口：`float32[1,48] -> float32[1,12]`，静态shape；标准raw action Actor。
- PyTorch/ONNX最大绝对误差：`4.291534423828125e-06`，PASS（门槛`1e-5`）。

## 控制链

- MJCF：`models/Creeper20260906.xml`，未修改动力学参数。
- `q_des=default_joint_pos+0.25*raw_action`。
- PD：`Kp=25.0`、`Kd=0.5`；torque/saturation limit=`23.7 Nm`。
- Physics=`200 Hz`，policy=`50 Hz`；无action filter、rate limiter或Hybrid standing。
- V45训练命令斜率：translation acceleration=`0.3 m/s²`、deceleration=`0.8 m/s²`；最终MuJoCo验收按此复现。

## MuJoCo结果

正式结果位于`mujoco_training_ramp/`。零命令standing记录20秒，其余固定命令稳态记录5秒。

| 工况 | 实际`vx/vy/yaw` | height mean/min | front/rear width | max Δq_des | fall/non-foot/base |
|---|---|---|---|---|---|
| standing | `-0.001/-0.000/-0.007` | `0.284/0.282 m` | `0.347/0.299 m` | `0.013 rad` | `0/0/0` |
| vx=+0.2 | `0.193/0.007/0.023` | `0.284/0.281 m` | `0.326/0.298 m` | `0.192 rad` | `0/0/0` |
| vx=-0.2 | `-0.167/-0.001/0.001` | `0.285/0.283 m` | `0.317/0.308 m` | `0.150 rad` | `0/0/0` |
| vx=+0.5 | `0.500/0.002/0.001` | `0.291/0.288 m` | `0.303/0.301 m` | `0.269 rad` | `0/0/0` |
| vx=-0.5 | `-0.482/-0.001/-0.000` | `0.294/0.292 m` | `0.299/0.303 m` | `0.186 rad` | `0/0/0` |
| vx=+1.0 | `1.006/0.001/0.001` | `0.295/0.293 m` | `0.284/0.297 m` | `0.327 rad` | `0/0/0` |
| vx=-1.0 | `-0.992/-0.001/-0.004` | `0.294/0.292 m` | `0.292/0.300 m` | `0.240 rad` | `0/0/0` |
| vy=+0.2 | `0.013/0.181/-0.001` | `0.285/0.283 m` | `0.307/0.315 m` | `0.153 rad` | `0/0/0` |
| vy=-0.2 | `0.010/-0.179/0.002` | `0.285/0.283 m` | `0.307/0.314 m` | `0.161 rad` | `0/0/0` |
| yaw=+0.5 | `-0.010/-0.008/0.419` | `0.288/0.281 m` | `0.303/0.293 m` | `0.169 rad` | `0/0/0` |
| yaw=-0.5 | `-0.010/0.007/-0.417` | `0.288/0.279 m` | `0.300/0.293 m` | `0.174 rad` | `0/0/0` |
| mixed +yaw | `0.194/-0.005/0.449` | `0.286/0.283 m` | `0.325/0.302 m` | `0.235 rad` | `0/0/0` |
| mixed -yaw | `0.197/0.003/-0.449` | `0.286/0.282 m` | `0.324/0.303 m` | `0.213 rad` | `0/0/0` |

`vx=1.0 -> 0`按`0.8 m/s²`停车：`1.61 s`进入稳定零速，移动约`0.738 m`，最大`Δq_des=0.274 rad/20 ms`，无non-foot/base contact。

## Isaac动作连续性复核

旧评估脚本的停车工况直接把command从`1.0`跳到`0`，其`0.542 rad`是瞬时阶跃压力测试，不符合V45训练/部署命令斜坡。修正临时评估脚本后，按`0.8 m/s²`连续减速，三seed、12条轨迹最大`Δq_des=0.3162 rad/20 ms`，无reset/base/non-foot contact。原始阶跃结果保留，不追溯删除。

## 判定边界

- ONNX一致性：PASS。
- Isaac固定命令与训练一致启停：PASS。
- MuJoCo Sim2Sim：PASS（现行暂定`0.50 rad/20 ms`离线门槛）。
- 历史`0.10 rad/20 ms`参考：仍不满足。
- 实机安全/部署：NOT_TESTED。`0.50`不是经过当前电机telemetry证实的硬件极限；本报告不能替代悬空台架、低速限幅、急停和真实电机温度/电流/跟踪误差验证。
