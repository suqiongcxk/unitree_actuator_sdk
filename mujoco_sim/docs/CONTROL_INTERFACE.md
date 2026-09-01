# Control interface

Actor input is exactly float32 `obs[1,48]`: body linear velocity 0..2, body angular velocity 3..5, projected gravity 6..8, `[vx,vy,yaw_rate]` 9..11, `q-default` 12..23, `dq` 24..35, and previous raw Actor action 36..47. The first raw-action history is zero; it is committed immediately after successful inference and is never replaced with a blended, clamped, or safety-modified command.

Actor output must be exactly float32 `actions[1,12]`. Names, types, ranks, and dimensions are rejected on mismatch. Desired position is `default[i] + 0.25 * raw_action[i]`. Optional takeover blends desired positions from standing without altering raw-action history.

Every physics step applies output-side `tau = 25 * (q_des-q) - 0.5 * dq`, clamps it to the configured MJCF asset limit, and counts saturation. No 6.333 ratio is applied. The real-controller gains 0.625/0.0125 and shutdown damping 0.136 have rotor/hardware semantics and are not used.

Faults latch on invalid state/target, bounds or velocity violation, base contact, or inference failure. CSV records simulation time, height, control cadence, base contact, saturation count, and maximum target jump. Process exit ends physics; it never accesses a device node.
