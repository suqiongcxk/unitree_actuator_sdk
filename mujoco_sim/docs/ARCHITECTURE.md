# Architecture and source audit

The simulator is a deterministic single control thread: read state, run the Actor every fourth step, compute output-side joint PD every step, clamp torque, and call `mj_step`. `physics_dt=0.005 s` gives 200 Hz physics and decimation 4 gives 50 Hz policy control. Wall-clock pacing never changes simulation-time scheduling.

The fixed array order is FL, FR, RL, RR hip; FL, FR, RL, RR thigh; FL, FR, RL, RR calf. All joint values are reducer-output radians, rad/s, and N·m. The MJCF joint axis is +X for hips and +Y for thighs/calves. Floating-base qpos/qvel precede hinges, so the backend resolves every address by name.

Portable behavior was audited from `robot_control/shared_data.h`, `policy_observation.*`, `nn_policy.*`, `motion_safety.*`, `state_estimator.*`, `linear_kf_position_velocity_estimator.*`, `leg_kinematics.*`, `robot_controller.*`, `debug_logs/LOW_SPEED_POLICY_RETRAINING_SPEC.md`, and `robot_description/Creeper20260714/urdf/Creeper20260714.urdf`. Synchronization rule: changes to observation order, joint order, default pose, action scale, coordinate convention, or policy frequency in those files must be explicitly mirrored here and covered by `test_core.cpp`; never include or link parent sources.

Reusable pure algorithms are observation construction, raw-action history, quaternion gravity projection, kinematic/KF concepts, finite checks, bounds checks, and command-jump diagnostics. JY901S, calibration, UART/RS-485, GPIO, Unitree motor transport, real-time multi-bus threads, rotor zero offsets, and physical emergency damping are deliberately isolated and absent.

Ground truth uses MuJoCo free-body/joint state. Sensor emulation reads frame quaternion, gyro, accelerometer, joint position/velocity, actuator force, and touch sensors. MuJoCo reports the site frame quaternion as body-to-world `(w,x,y,z)`; projected gravity is `R^T [0,0,-1]`. The static test verifies horizontal gravity `[0,0,-1]`, zero gyro, and accelerometer `+9.80665` on local Z rather than assuming JY901S sign equivalence.
