# Creeper20260906 URDF 迁移记录

## 对比来源

- 旧版：`robot_description/Creeper20260714/urdf/Creeper20260714.urdf`
- 新版：`robot_description/Creeper20260906/urdf/Creeper20260906.urdf`
- 新版 SHA-256：`37ff462cfe9a9c353fa3bb0befc1cda09372f63837fafc30aba96adb04c9ebf0`

## 对控制有意义的几何差异

| 项目 | Creeper20260714 | Creeper20260906 | 影响 |
|---|---:|---:|---|
| 前腿 hip→thigh X | `+0.05825 m` | `+0.06825 m` | 前足名义 X 增加 10 mm |
| 后腿 hip→thigh X | `-0.05825 m` | `-0.06825 m` | 后足名义 X 减少 10 mm |
| base→hip X/Y | `±0.1366/±0.0650 m` | 不变 | 无 |
| hip→thigh Y | `±0.0186 m` | 不变 | 无 |
| thigh→calf | `Y=±0.0572, Z=-0.210 m` | 不变 | 无 |
| calf→foot / 足半径 | `-0.208 / 0.020 m` | 不变 | 无 |

关节 axis、限位、effort、velocity 和 motor-ID 顺序未变。URDF 中 calf joint origin 的其余差异为浮点导出尾数，不是机构变化。

## 质量与惯量

- URDF 总质量：`8.734596037 kg → 11.932221639 kg`。
- base：`3.741831344 kg → 5.087401639 kg`。
- hip 总质量：`1.281026772 kg → 2.408200000 kg`，新版每个 hip 均为 `0.60205 kg`。
- 前腿 thigh 每个 `0.87467 kg`，后腿 thigh 每个 `0.87728 kg`，calf 每个 `0.23193 kg`。
- base、hip、thigh、calf 的质心和惯量张量均已重新导出；详细值以新 URDF 为唯一来源。

## 部署代码影响

- `CreeperLegKinematics` 与 `EigenLegKinematics` 必须共同使用 `0.06825 m` hip→thigh X 偏移。
- 足端位置、角速度交叉项、腿部里程计和 Linear KF 足端世界坐标因此更新。
- 当前实机代码不实现质量矩阵或刚体动力学预测，所以 URDF 质量不应被错误用来缩放电机反馈力矩、PD 增益或触地阈值。
- 质量、质心和惯量必须在仿真/重新训练资产中完整更新；用旧 URDF 训练的 `model_700` 不能视为新动力学的合格策略。

## 遗漏复查补充

- MuJoCo 足球已与 URDF 对齐为半径 `0.020 m`、质量 `0.001 kg`、惯量 `diag(1e-6)`，不再使用历史仿真近似 `0.026 m/0.03 kg`。
- MuJoCo IMU 固定体位置已与 URDF 对齐为 `(-0.02557, 0, 0.04232) m`，并计入 `0.001 kg` 质量。
