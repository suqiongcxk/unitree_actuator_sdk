# Creeper V30 model_1299 Simulation Report

## Identity and decision

- Checkpoint: `model_1299.pt`
- Checkpoint SHA256: `4285d2262ed122da182bba8e0790a8c477f11ad31940ad2fb1d502721da3a7cd`
- Actor interface: `obs float32[1,48] -> actions float32[1,12]`
- Policy period: `20 ms` (`50 Hz`); physics period: `5 ms` (`200 Hz`)
- Offline continuity criterion: `max |q_des[t]-q_des[t-1]| <= 0.50 rad/20 ms`
- Criterion status: user-selected provisional offline engineering criterion; **not** a hardware-validated motor limit.
- Isaac status: **PASS** under the provisional `0.50 rad/20 ms` criterion.
- MuJoCo status: **PASS** under the provisional `0.50 rad/20 ms` criterion.
- Real hardware status: **NOT_TESTED**. No hardware telemetry or motor command was used for this delivery.

The historical `0.10 rad/20 ms` result remains **FAIL** and is not overwritten by this report.

## Isaac

Status: **PASS**

Evaluation used task `Isaac-Velocity-Flat-Creeper-20260906-V30-Play-v0`, seed 42, four environments,
noise/external force/base-mass randomization disabled, 2 s warmup and 5 s recording at 50 Hz. Standing also has a
20 s record. The extended fixed-command evidence covers standing, forward/backward, lateral, both yaw directions,
mixed commands and command transitions.

| Case | Actual primary velocity | Height/attitude evidence | Reset/base contact | max/P95/P99 Δq_des (rad) | >0.50 |
|---|---:|---|---:|---:|---:|
| standing | `vx≈-0.0002 m/s` | see archived standing record | `0/0` | `0.00035/0.00003/0.00005` | `0%` |
| vx=+0.2 | `0.1911 m/s` | archived per-frame metrics | `0/0` | `0.1623/0.0684/0.1022` | `0%` |
| vx=+0.5 | `0.4877 m/s` | archived per-frame metrics | `0/0` | `0.2085/0.1075/0.1536` | `0%` |
| vx=+1.0 | `0.9688 m/s` | archived per-frame metrics | `0/0` | `0.2487/0.1481/0.1885` | `0%` |
| 0→-1.0 abrupt | `-0.9727 m/s` | mean/min height `0.3129/0.2757 m`; max roll/pitch `0.0685/0.1234 rad` | `0/0` | `0.3415/0.1450/0.1921` | `0%` |
| vy=±0.2 | `+0.1937/-0.1921 m/s` | archived per-frame metrics | `0/0` | max `0.1559/0.1685` | `0%` |
| yaw=±0.5 | `+0.4528/-0.4591 rad/s` | archived per-frame metrics | `0/0` | max `0.1268/0.0982` | `0%` |
| mixed ± | `vx≈0.197, yaw≈±0.511` | archived per-frame metrics | `0/0` | max `0.1600/0.1595` | `0%` |

No NaN/Inf was reported. The abrupt reverse case deliberately bypasses the formal command ramp and remains below the
provisional threshold; it does not replace the required deployed ramp.

## MuJoCo

Status: **PASS**

Evaluation used `Creeper20260906.xml`, the unchanged PD/DCMotor torque-speed control, physics dt `0.005 s`, policy dt
`0.02 s`, the 48D ONNX and policy-only control. Friction, dynamics and action processing were not adjusted to pass.

| Case | Actual primary velocity | Mean/min height; max roll/pitch | Fall/non-foot/base | max/P95/P99 Δq_des (rad) | >0.50 |
|---|---:|---|---:|---:|---:|
| standing 20 s | `vx≈-0.0001 m/s` | `0.2844/0.2841 m`; `0.0019/0.0300 rad` | `0/0/0` | `0.00011/0.00008/0.00009` | `0%` |
| vx=+0.2 | `0.2088 m/s` | `0.2886/0.2850 m`; `0.0236/0.0326 rad` | `0/0/0` | `0.1652/0.0703/0.1049` | `0%` |
| vx=+0.5 | `0.5045 m/s` | archived raw metrics | `0/0/0` | `0.2063/0.1115/0.1544` | `0%` |
| vx=+1.0 | `1.0045 m/s` | archived raw metrics | `0/0/0` | `0.2063/0.1491/0.1795` | `0%` |
| vx=-1.0 | `-0.9945 m/s` | archived raw metrics | `0/0/0` | `0.2415/0.1334/0.1687` | `0%` |
| vy=±0.2 | `+0.1959/-0.1917 m/s` | archived raw metrics | `0/0/0` | max `0.1656/0.1800` | `0%` |
| yaw=±0.5 | `+0.4352/-0.3998 rad/s` | archived raw metrics | `0/0/0` | max `0.0957/0.0928` | `0%` |
| mixed ± | `vx≈0.2095, yaw≈±0.515` | archived raw metrics | `0/0/0` | max `0.1294/0.1265` | `0%` |

The ramped keyboard sequence (`+1→-1`, lateral/yaw/mixed reversal, then stop) had max/P95/P99
`0.2063/0.0864/0.1468 rad`, `>0.30=0%`, `>0.50=0%`, and no illegal contact. The stop test measured
`1.27 s/0.310 m`. Available files record simulated joint velocity, tracking error and torque; these are simulation
outputs, not real motor telemetry.

## Motor/control errors

- Simulated NaN/Inf, fall, base contact and non-foot contact in accepted scenarios: `0` according to archived evidence.
- A standardized real-motor error counter was not exercised: **NOT_TESTED**.
- Real joint speed/current/torque/temperature/latency telemetry: **NOT_TESTED**.

## Limitations and evidence

MuJoCo yaw is under-tracked and asymmetric (`+0.435/-0.400 rad/s` for `±0.5`). The original `0.10 rad/20 ms`
reference is still exceeded in moving cases, especially around calf contact transitions. Offline PASS does not establish
motor, power, communication, mechanical or emergency-stop safety.

Authoritative evidence copied into this delivery includes the original reacceptance `REPORT.md`, `summary.json`, Isaac
extended fixed-command JSON, MuJoCo standing/fixed-command/recomputed continuity JSON, keyboard sequence JSON/NPZ and
the original `SHA256SUMS`. Paths and hashes are recorded in `deployment_manifest.json`.
