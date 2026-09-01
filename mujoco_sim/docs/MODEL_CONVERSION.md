# Model conversion notes

The SolidWorks-exported URDF was treated as the source for joint origins, axes, limits, masses, diagonalized inertias, and nominal link lengths. The MJCF uses a free joint, plane gravity environment, four symmetric three-link chains, separate visual/collision geoms, sphere feet, and named sites/sensors. Analytic capsules and boxes replace STL collision and visualization so sparse checkout has no package URI, mesh-unit ambiguity, or complex-triangle contacts.

The default standing keyframe uses control-order pose `[0.1,-0.1,0.1,-0.1, 0.8,0.8,1,1, -1.5,-1.5,-1.5,-1.5]`; MJCF qpos values are serialized in body-tree order, which differs. Tests query each joint's qpos and dof address and compare values in control order.

The JY901S physical mounting transform was not unambiguously encoded in the audited URDF. `imu_site` is aligned with the base convention (+X forward, +Y left, +Z up); any measured non-identity mounting transform must be documented and applied before claiming sensor-emulation parity with hardware.
