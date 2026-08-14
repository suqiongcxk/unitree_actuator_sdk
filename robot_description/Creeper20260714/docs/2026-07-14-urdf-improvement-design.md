# URDF Improvement Design: Creeper20260714 for Isaac Sim RL Training

## Goal

Refine the freshly-exported Creeper20260714 URDF from SolidWorks to be ready for
reinforcement learning (flat-ground locomotion) in Isaac Sim.

## Reference

- Old version: `Creeper20260617（6061）` — verified joint topology, same motor specs
- Motor reference: `unitree-go2/go2.urdf` — joint limits, effort, velocity, collision design

## Changes

### 1. Axis alignment (match Go2)

| Joint | Before | After |
|-------|--------|-------|
| FL_hip, RL_hip, RR_hip | `-1 0 0` | `1 0 0` |
| FR_thigh, RR_thigh | `0 -1 0` | `0 1 0` |

After this, all 12 joint axes are identical to Go2, so limits can be copied
without sign flipping.

### 2. Joint limits / effort / velocity

| Joint | lower (rad) | upper (rad) | effort (Nm) | velocity (rad/s) |
|-------|-------------|-------------|-------------|------------------|
| 4× hip | -1.047 | 1.047 | 23.7 | 30.1 |
| FL/FR thigh | -1.571 | 3.491 | 23.7 | 30.1 |
| RL/RR thigh | -0.524 | 4.538 | 23.7 | 30.1 |
| 4× calf | -2.723 | -0.838 | 23.7 | 30.1 |

Calf effort/velocity = 23.7/30.1 (NOT 35.55/20.07 as Go2) — Creeper has no
gear transmission on the calf joint.

### 3. Simplified collision geometry

STL mesh collisions replaced with primitive shapes based on bounding-box
measurements from the exported STL files.

| Link | Shape | Size (m) | Origin xyz | Origin rpy |
|------|-------|----------|------------|------------|
| base | box | 0.40 0.23 0.14 | 0.057 0 0 | 0 0 0 |
| FL/RL hip | cylinder | r=0.045 l=0.04 | 0 0.08 0 | 1.57 0 0 |
| FR/RR hip | cylinder | r=0.045 l=0.04 | 0 -0.08 0 | 1.57 0 0 |
| FL/RL thigh | box | 0.10 0.025 0.035 | 0 0 -0.10 | 0 1.57 0 |
| FR/RR thigh | box | 0.10 0.025 0.035 | 0 0 -0.10 | 0 1.57 0 |
| 4× calf | cylinder | r=0.013 l=0.12 | 0.01 0 -0.07 | 0 -0.2 0 |
| 4× foot | sphere | r=0.02 | 0 0 -0.22 | 0 0 0 |

### 4. New links

| Link | Purpose | Parent | Joint type |
|------|---------|--------|------------|
| base_footprint | World root frame | (root) | fixed |
| imu | IMU sensor reading in Isaac Sim | base | fixed |
| FL_foot | Foot contact detection | FL_calf | fixed, dont_collapse |
| FR_foot | Foot contact detection | FR_calf | fixed, dont_collapse |
| RL_foot | Foot contact detection | RL_calf | fixed, dont_collapse |
| RR_foot | Foot contact detection | RR_calf | fixed, dont_collapse |

### 5. Files to update

- `urdf/Creeper20260714.urdf` — all changes above
- `launch/display.launch` — already correct, no change needed
- `launch/gazebo.launch` — already correct, no change needed
- `config/joint_names_Creeper20260714.yaml` — no change needed
- `package.xml` — no change needed
- `CMakeLists.txt` — no change needed
