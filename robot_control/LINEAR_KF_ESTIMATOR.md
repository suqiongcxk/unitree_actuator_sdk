# Independent 18-state Linear KF

This module is a clean C++17/Eigen implementation inspired by the estimator
structure used in MIT Cheetah-Software. It does not include or copy ROS, LCM,
vendor SDK, or Cheetah source code.

## Frames and units

- Body: right-handed, `+X` forward, `+Y` left, `+Z` up.
- World: right-handed, `+Z` up; its origin is selected at estimator reset.
- Quaternion: `body -> world`, Eigen constructor/order semantics `(w,x,y,z)`.
- Position: metres; velocity: metres/second; acceleration: metres/second squared.
- Joint position/velocity: radians and radians/second, in URDF coordinates.
- IMU acceleration is specific force. A level stationary IMU reads approximately
  `[0,0,+9.80665]`; the estimator computes `a_world = R_WB*f_body + [0,0,-g]`.

## State and observations

The state is

```text
x = [p_WB(3), v_WB(3), p_WF_FL(3), p_WF_FR(3), p_WF_RL(3), p_WF_RR(3)]
```

For each foot, forward kinematics supplies `p_BF` and `J*dq`. The filter uses:

```text
p_WF - p_WB = R_WB * p_BF
v_WB = -R_WB * (omega_B x p_BF + J*dq)   (stationary support foot)
```

Because angular velocity is not part of the requested input, `omega_B` is
derived from consecutive body-to-world quaternions and `dt`; the first update
uses zero angular velocity.

Confidence below the configured threshold, or a velocity innovation indicating
slip, multiplies that foot's measurement covariance by 100. Its observation is
also blended toward the prediction so a swing/airborne foot cannot slowly force
the estimated base velocity to zero.

Absolute global X/Y position remains subject to the normal inertial/leg-odometry
drift. Initial Z chooses the mean trusted contact-foot plane as world `z=0`;
without an external position or terrain-height sensor, absolute world position
is not globally observable.

## Selecting the runtime backend

The default backend remains the existing complementary IMU/leg-odometry chain:

```cpp
RobotControlConfig config = RobotController::getDefaultConfig();
config.estimator_backend = StateEstimatorBackend::COMPLEMENTARY;
```

Select the 18-state estimator with:

```cpp
config.estimator_backend = StateEstimatorBackend::LINEAR_KF;
```

The executable also accepts `--estimator complementary` and
`--estimator kalman`. Kalman selection is rejected before hardware
initialization when Eigen support was not compiled in; it never silently falls
back to the complementary estimator.
