# Model conversion notes

The SolidWorks-exported `Creeper20260906.urdf` is the source for joint
origins, axes, limits, masses, centers of mass, full inertia tensors, foot
radius, and the fixed IMU origin. The MJCF uses a free joint, plane gravity
environment, four three-link chains, separate visual/collision geoms, sphere
feet, and named sites/sensors. Analytic capsules and boxes replace STL
collision and visualization so sparse checkout has no package URI, mesh-unit
ambiguity, or complex-triangle contacts.

The default standing keyframe uses control-order pose `[0.1,-0.1,0.1,-0.1, 0.8,0.8,1,1, -1.5,-1.5,-1.5,-1.5]`; MJCF qpos values are serialized in body-tree order, which differs. Tests query each joint's qpos and dof address and compare values in control order.

The MJCF `imu` body uses the URDF fixed-joint translation
`[-0.02557, 0, 0.04232] m`. The JY901S sensor axes are aligned with the base
convention (+X forward, +Y left, +Z up) because the URDF fixed joint has zero
rotation. Any additional physical mounting transform not represented by the
URDF must be documented and applied before claiming sensor-emulation parity
with hardware.
