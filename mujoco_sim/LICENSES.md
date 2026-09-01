# Licenses and provenance

- Portable C++ code, MJCF, documentation, and scripts in this directory: same project license as the repository containing this directory.
- Eigen: MPL-2.0; supplied by the target system, not vendored.
- MuJoCo: Apache-2.0; supplied by the target system or fetched from its upstream release, not vendored.
- ONNX Runtime: MIT; optional target-system dependency, not vendored.
- No STL or other binary asset is copied from `robot_description`. The MJCF uses analytic geometry and audited numeric dimensions from the repository's `Creeper20260714.urdf`.
