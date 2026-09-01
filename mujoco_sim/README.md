# Creeper standalone MuJoCo simulator

This directory is a C++17, headless-first simulator for Creeper20260714. It has no ROS, Unitree SDK, serial, GPIO, I2C, motor, or physical-sensor dependency. The default-pose controller works without an ONNX file; ONNX Runtime is discovered at build time and is required only for `--onnx`.

## Dependencies and build

Install CMake, a C++17 compiler, Eigen 3, and MuJoCo. Optionally install ONNX Runtime C/C++ for policy inference. Point `CMAKE_PREFIX_PATH` or `mujoco_DIR` at non-system installations.

```bash
cmake -S mujoco_sim -B mujoco_sim/build
cmake --build mujoco_sim/build -j
ctest --test-dir mujoco_sim/build --output-on-failure
```

For a development machine with network access, `-DCREEPER_FETCH_MUJOCO=ON` fetches the pinned MuJoCo 3.3.5 source if no installed package is found. Normal deployment builds do not download anything.

## Run

```bash
./mujoco_sim/build/mujoco_robot_control \
  --model mujoco_sim/models/creeper/creeper.xml \
  --headless --duration 10 --no-policy

./mujoco_sim/build/mujoco_robot_control \
  --model mujoco_sim/models/creeper/creeper.xml \
  --onnx /path/to/model.onnx --mode ground-truth \
  --command 0.05 0 0 --duration 10 --log simulation.csv
```

`--mode sensor-emulation` reads MuJoCo IMU, encoder, actuator-force, and contact sensors. Its portable estimator integrates gravity-compensated specific force and applies a stance-contact drift correction; use ground truth first for policy/model validation because this simple estimator is not a claim of byte-for-byte JY901S equivalence. `--realtime` paces simulation to wall time; without it, headless mode runs as fast as possible. Rendering is intentionally not linked into this independent headless control executable.

On Ctrl+C or a latched fault, Actor execution stops and no more `mj_step` calls are issued. The process records the reason and exits; it does not emulate the real robot's damping command.

## Sparse checkout

```bash
git clone --filter=blob:none --no-checkout <REPOSITORY_URL> creeper_sim_repo
cd creeper_sim_repo
git sparse-checkout init --cone
git sparse-checkout set mujoco_sim
git checkout <BRANCH>
cmake -S mujoco_sim -B mujoco_sim/build
cmake --build mujoco_sim/build -j
./mujoco_sim/build/mujoco_robot_control \
  --model mujoco_sim/models/creeper/creeper.xml \
  --headless --duration 10 --no-policy
```

The URDF values `23.7 N·m` and `30.1 rad/s` are unverified simulation-asset limits, not claimed continuous hardware ratings.
