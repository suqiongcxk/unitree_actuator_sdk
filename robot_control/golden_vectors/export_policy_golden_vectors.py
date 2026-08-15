#!/usr/bin/env python3
"""Export continuous Creeper rollout golden vectors for C++ policy parity tests.

The Actor input is captured from the real ``obs["policy"]`` returned by the
IsaacLab environment.  The Actor output is captured with a forward hook on the
loaded RSL-RL actor.  Reconstructed observations are used only for assertions;
they are never fed to the policy.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import random
import sys
from pathlib import Path
from typing import Any

RSL_RL_SCRIPT_DIR = "/workspace/isaaclab/scripts/reinforcement_learning/rsl_rl"
sys.path.insert(0, RSL_RL_SCRIPT_DIR)

from isaaclab.app import AppLauncher

import cli_args  # noqa: E402


DEFAULT_CHECKPOINT = (
    "/workspace/isaaclab/logs/rsl_rl/creeper_flat/"
    "2026-07-27_16-45-03/model_700.pt"
)
DEFAULT_ONNX = (
    "/root/gpufree-data/CreeperLab/deployment/creeper_policy/model_700/"
    "creeper_flat_model_700_actor.onnx"
)
DEFAULT_OUTPUT = (
    "/root/gpufree-data/Note/04_Policy_Evaluation/policy_golden_vectors.json"
)
DEFAULT_ORT_DEPS = (
    "/root/gpufree-data/CreeperLab/deployment/creeper_policy/python_deps"
)


parser = argparse.ArgumentParser(description="Export Creeper policy golden vectors from a real rollout.")
parser.add_argument("--task", default="Isaac-Velocity-Flat-Creeper-Play-v0")
parser.add_argument("--agent", default="rsl_rl_cfg_entry_point")
parser.add_argument("--onnx", default=DEFAULT_ONNX)
parser.add_argument("--output", default=DEFAULT_OUTPUT)
parser.add_argument("--frames", type=int, default=20, choices=range(10, 21), metavar="[10-20]")
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--ort-deps", default=DEFAULT_ORT_DEPS)
cli_args.add_rsl_rl_args(parser)
parser.set_defaults(checkpoint=DEFAULT_CHECKPOINT)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import onnx  # noqa: E402
import torch  # noqa: E402
from onnx import numpy_helper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

import isaaclab_tasks  # noqa: E402, F401
from isaaclab_tasks.utils import load_cfg_from_registry, parse_env_cfg  # noqa: E402


ACTION_SCALE = np.float32(0.25)
EXPECTED_OBSERVATION_SIZE = 48
EXPECTED_ACTION_SIZE = 12
COMMAND_ORDER = ["lin_vel_x", "lin_vel_y", "ang_vel_z"]


def sha256sum(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def as_numpy(tensor: torch.Tensor) -> np.ndarray:
    return tensor.detach().cpu().numpy().astype(np.float32, copy=True)


def vector(tensor: torch.Tensor) -> np.ndarray:
    return as_numpy(tensor[0])


def json_vector(array: np.ndarray) -> list[float]:
    return np.asarray(array, dtype=np.float32).tolist()


def tensor_shape(value_info: onnx.ValueInfoProto) -> list[int | str | None]:
    shape: list[int | str | None] = []
    for dim in value_info.type.tensor_type.shape.dim:
        if dim.HasField("dim_value"):
            shape.append(dim.dim_value)
        elif dim.HasField("dim_param"):
            shape.append(dim.dim_param)
        else:
            shape.append(None)
    return shape


def command_for_frame(frame: int, frame_count: int) -> tuple[str, tuple[float, float, float]]:
    """Cover all requested command axes, then retain a forward-walking segment."""
    if frame < 2:
        return "standing", (0.0, 0.0, 0.0)
    if frame < 4:
        return "forward_probe", (0.4, 0.0, 0.0)
    if frame < 6:
        return "lateral_probe", (0.0, 0.25, 0.0)
    if frame < 8:
        return "yaw_probe", (0.0, 0.0, 0.5)
    # Frames 8..19 form the longest continuous normal forward-walking sample
    # possible while keeping the requested file to at most 20 policy frames.
    return "forward_walking", (0.4, 0.0, 0.0)


def set_command(command_term: Any, command: tuple[float, float, float]) -> None:
    value = torch.tensor(command, device=command_term.device, dtype=command_term.vel_command_b.dtype)
    command_term.vel_command_b[:] = value
    command_term.is_heading_env[:] = False
    command_term.is_standing_env[:] = False
    command_term.time_left[:] = float("inf")


class ErrorTracker:
    """Track float32 absolute and relative errors with exact locations."""

    def __init__(self) -> None:
        self.results: dict[str, dict[str, Any]] = {}

    def check(
        self,
        name: str,
        actual: np.ndarray,
        expected: np.ndarray,
        frame: int,
        index_offset: int = 0,
    ) -> None:
        actual = np.asarray(actual, dtype=np.float32).reshape(-1)
        expected = np.asarray(expected, dtype=np.float32).reshape(-1)
        if actual.shape != expected.shape:
            raise AssertionError(f"{name}: shape mismatch {actual.shape} != {expected.shape}")
        abs_error = np.abs(actual - expected).astype(np.float32)
        denominator = np.maximum(np.abs(expected), np.float32(1.0e-12))
        rel_error = (abs_error / denominator).astype(np.float32)
        abs_index = int(np.argmax(abs_error))
        rel_index = int(np.argmax(rel_error))
        current = self.results.setdefault(
            name,
            {
                "max_abs_error": 0.0,
                "max_abs_location": {"frame_index": 0, "index": index_offset},
                "max_rel_error": 0.0,
                "max_rel_location": {"frame_index": 0, "index": index_offset},
            },
        )
        if float(abs_error[abs_index]) > current["max_abs_error"]:
            current["max_abs_error"] = float(abs_error[abs_index])
            current["max_abs_location"] = {
                "frame_index": frame,
                "index": abs_index + index_offset,
            }
        if float(rel_error[rel_index]) > current["max_rel_error"]:
            current["max_rel_error"] = float(rel_error[rel_index])
            current["max_rel_location"] = {
                "frame_index": frame,
                "index": rel_index + index_offset,
            }


def verify_actor_weights(checkpoint_path: Path, model: onnx.ModelProto) -> dict[str, Any]:
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    state_dict = checkpoint["model_state_dict"]
    initializer_errors: dict[str, float] = {}
    all_equal = True
    for initializer in model.graph.initializer:
        if initializer.name.startswith("actor."):
            onnx_value = numpy_helper.to_array(initializer)
            checkpoint_value = state_dict[initializer.name].detach().cpu().numpy()
            error = float(np.max(np.abs(onnx_value - checkpoint_value)))
            initializer_errors[initializer.name] = error
            all_equal = all_equal and np.array_equal(onnx_value, checkpoint_value)
    if len(initializer_errors) != 8:
        raise AssertionError(f"Expected 8 actor initializers, found {len(initializer_errors)}")
    if not all_equal:
        raise AssertionError(f"ONNX/checkpoint Actor weights differ: {initializer_errors}")
    return {
        "checkpoint_iteration": int(checkpoint["iter"]),
        "actor_initializer_count": len(initializer_errors),
        "all_actor_initializers_bitwise_equal": all_equal,
        "per_initializer_max_abs_error": initializer_errors,
    }


def main() -> None:
    checkpoint_path = Path(args_cli.checkpoint).expanduser().resolve()
    onnx_path = Path(args_cli.onnx).expanduser().resolve()
    output_path = Path(args_cli.output).expanduser().resolve()
    npz_path = output_path.with_suffix(".npz")
    for path in (checkpoint_path, onnx_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    random.seed(args_cli.seed)
    np.random.seed(args_cli.seed)
    torch.manual_seed(args_cli.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args_cli.seed)

    model = onnx.load(str(onnx_path))
    onnx.checker.check_model(model)
    weights_check = verify_actor_weights(checkpoint_path, model)
    input_info = model.graph.input[0]
    output_info = model.graph.output[0]
    if input_info.name != "obs" or output_info.name != "actions":
        raise AssertionError(f"Unexpected ONNX I/O names: {input_info.name}, {output_info.name}")
    if tensor_shape(input_info) != [1, 48] or tensor_shape(output_info) != [1, 12]:
        raise AssertionError("Unexpected ONNX input/output shape")
    if input_info.type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
        raise AssertionError("ONNX input must be float32")
    if output_info.type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
        raise AssertionError("ONNX output must be float32")

    ort_deps = Path(args_cli.ort_deps).expanduser().resolve()
    if ort_deps.is_dir():
        sys.path.insert(0, str(ort_deps))
    import onnxruntime as ort  # noqa: PLC0415

    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 1
    session_options.inter_op_num_threads = 1
    session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    ort_session = ort.InferenceSession(
        str(onnx_path), sess_options=session_options, providers=["CPUExecutionProvider"]
    )

    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    agent_cfg = load_cfg_from_registry(args_cli.task, args_cli.agent)
    env_cfg.seed = args_cli.seed
    env_cfg.scene.num_envs = 1
    env_cfg.observations.policy.enable_corruption = False
    agent_cfg.seed = args_cli.seed
    if args_cli.device is not None:
        agent_cfg.device = args_cli.device
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    try:
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(str(checkpoint_path))
        policy_nn = runner.alg.policy
        policy_nn.eval()
        if bool(policy_nn.is_recurrent):
            raise AssertionError("This exporter expects the deployed non-recurrent Actor")
        policy = runner.get_inference_policy(device=env.unwrapped.device)

        normalizer = policy_nn.actor_obs_normalizer
        if type(normalizer).__name__ != "Identity":
            raise AssertionError(f"Unexpected Actor observation normalizer: {type(normalizer).__name__}")

        captured_actor_inputs: list[torch.Tensor] = []
        captured_actor_outputs: list[torch.Tensor] = []

        def capture_actor(_module: torch.nn.Module, inputs: tuple[torch.Tensor, ...], output: torch.Tensor) -> None:
            captured_actor_inputs.append(inputs[0].detach().clone())
            captured_actor_outputs.append(output.detach().clone())

        hook = policy_nn.actor.register_forward_hook(capture_actor)
        base_env = env.unwrapped
        robot = base_env.scene["robot"]
        command_term = base_env.command_manager.get_term("base_velocity")
        action_term = base_env.action_manager.get_term("joint_pos")
        dt = np.float32(base_env.step_dt)
        joint_names = list(robot.joint_names)
        if joint_names != list(action_term._joint_names):
            raise AssertionError("Robot and action-term joint orders differ")
        if len(joint_names) != EXPECTED_ACTION_SIZE:
            raise AssertionError(f"Unexpected joint count: {len(joint_names)}")

        reset_result = env.reset()
        if isinstance(reset_result, tuple):
            reset_obs = reset_result[0]
        else:
            reset_obs = reset_result
        del reset_obs

        frames: list[dict[str, Any]] = []
        arrays: dict[str, list[np.ndarray]] = {
            key: []
            for key in (
                "base_linear_velocity_body",
                "base_angular_velocity_body",
                "projected_gravity_body",
                "command",
                "joint_position",
                "default_joint_position",
                "joint_position_relative",
                "joint_velocity",
                "default_joint_velocity",
                "previous_raw_action",
                "observation_final",
                "actor_input_after_normalizer",
                "actor_raw_action",
                "onnx_actor_raw_action",
                "joint_position_target",
            )
        }
        tracker = ErrorTracker()
        previous_actor_action: np.ndarray | None = None

        for frame_index in range(args_cli.frames):
            if not simulation_app.is_running():
                raise RuntimeError("Isaac Sim stopped before the requested frames were captured")
            command_case, command_value = command_for_frame(frame_index, args_cli.frames)
            set_command(command_term, command_value)

            # This is the actual ObservationManager/RSL-RL wrapper output used by policy().
            obs = env.get_observations()
            observation_manager_output = vector(obs["policy"])
            if observation_manager_output.shape != (EXPECTED_OBSERVATION_SIZE,):
                raise AssertionError(f"Unexpected observation shape: {observation_manager_output.shape}")

            base_lin_vel = vector(robot.data.root_lin_vel_b)
            base_ang_vel = vector(robot.data.root_ang_vel_b)
            projected_gravity = vector(robot.data.projected_gravity_b)
            command = vector(command_term.vel_command_b)
            joint_position = vector(robot.data.joint_pos)
            default_joint_position = vector(robot.data.default_joint_pos)
            joint_position_relative = (joint_position - default_joint_position).astype(np.float32)
            joint_velocity = vector(robot.data.joint_vel)
            default_joint_velocity = vector(robot.data.default_joint_vel)
            joint_velocity_relative = (joint_velocity - default_joint_velocity).astype(np.float32)
            previous_raw_action = vector(base_env.action_manager.action)

            captured_actor_inputs.clear()
            captured_actor_outputs.clear()
            with torch.inference_mode():
                actor_action_tensor = policy(obs)
            if len(captured_actor_inputs) != 1 or len(captured_actor_outputs) != 1:
                raise AssertionError("Actor hook did not capture exactly one forward pass")
            actor_input = vector(captured_actor_inputs[0])
            actor_raw_action = vector(captured_actor_outputs[0])
            policy_returned_action = vector(actor_action_tensor)
            if not np.array_equal(actor_raw_action, policy_returned_action):
                raise AssertionError("Inference policy return differs from captured Actor output")

            onnx_action = ort_session.run(
                [output_info.name], {input_info.name: observation_manager_output.reshape(1, -1)}
            )[0].astype(np.float32, copy=False)[0]

            _, _, dones, _ = env.step(actor_action_tensor)
            if bool(dones[0].item()):
                raise RuntimeError(f"Environment reset during continuous capture at frame {frame_index}")
            actual_joint_target = vector(action_term.processed_actions)

            # Assertions reconstruct values only for parity checking, never for Actor inference.
            tracker.check("joint_position_relative", joint_position_relative, joint_position - default_joint_position, frame_index)
            tracker.check("observation_joint_position", observation_manager_output[12:24], joint_position_relative, frame_index, 12)
            tracker.check("observation_joint_velocity", observation_manager_output[24:36], joint_velocity_relative, frame_index, 24)
            formula_target = (default_joint_position + ACTION_SCALE * actor_raw_action).astype(np.float32)
            tracker.check("joint_position_target", actual_joint_target, formula_target, frame_index)
            tracker.check("observation_base_linear_velocity", observation_manager_output[0:3], base_lin_vel, frame_index, 0)
            tracker.check("observation_base_angular_velocity", observation_manager_output[3:6], base_ang_vel, frame_index, 3)
            tracker.check("observation_projected_gravity", observation_manager_output[6:9], projected_gravity, frame_index, 6)
            tracker.check("observation_command", observation_manager_output[9:12], command, frame_index, 9)
            tracker.check("actor_input_after_normalizer", actor_input, observation_manager_output, frame_index)
            tracker.check("pytorch_actor_vs_onnxruntime", actor_raw_action, onnx_action, frame_index)
            if frame_index == 0:
                tracker.check("first_previous_raw_action_is_zero", previous_raw_action, np.zeros(12, dtype=np.float32), frame_index, 36)
            else:
                assert previous_actor_action is not None
                tracker.check("previous_raw_action_temporal", previous_raw_action, previous_actor_action, frame_index, 36)

            raw_arrays = {
                "base_linear_velocity_body": base_lin_vel,
                "base_angular_velocity_body": base_ang_vel,
                "projected_gravity_body": projected_gravity,
                "command": command,
                "joint_position": joint_position,
                "default_joint_position": default_joint_position,
                "joint_position_relative": joint_position_relative,
                "joint_velocity": joint_velocity,
                "default_joint_velocity": default_joint_velocity,
                "previous_raw_action": previous_raw_action,
                "observation_final": observation_manager_output,
                "actor_input_after_normalizer": actor_input,
                "actor_raw_action": actor_raw_action,
                "onnx_actor_raw_action": onnx_action,
                "joint_position_target": actual_joint_target,
            }
            for name, value in raw_arrays.items():
                if value.dtype != np.float32:
                    raise AssertionError(f"{name} is {value.dtype}, expected float32")
                if not np.isfinite(value).all():
                    raise FloatingPointError(f"NaN/Inf in {name} at frame {frame_index}")
                arrays[name].append(value.copy())

            frames.append(
                {
                    "frame_index": frame_index,
                    "command_case": command_case,
                    "dt": float(dt),
                    **{name: json_vector(value) for name, value in raw_arrays.items()},
                }
            )
            previous_actor_action = actor_raw_action.copy()

        hook.remove()
        stacked_arrays = {name: np.stack(values).astype(np.float32) for name, values in arrays.items()}
        all_values = np.concatenate([value.reshape(-1) for value in stacked_arrays.values()])
        if not np.isfinite(all_values).all():
            raise FloatingPointError("Generated dataset contains NaN/Inf")

        dynamic_dimensions = any(
            isinstance(dim, str) or dim is None
            for info in (input_info, output_info)
            for dim in tensor_shape(info)
        )
        metadata = {
            "generator": str(Path(__file__).resolve()),
            "task": args_cli.task,
            "seed": args_cli.seed,
            "model_path": str(onnx_path),
            "model_sha256": sha256sum(onnx_path),
            "checkpoint_path": str(checkpoint_path),
            "checkpoint_sha256": sha256sum(checkpoint_path),
            "checkpoint_iteration": weights_check["checkpoint_iteration"],
            "checkpoint_onnx_weight_match": weights_check,
            "observation_size": EXPECTED_OBSERVATION_SIZE,
            "action_size": EXPECTED_ACTION_SIZE,
            "action_scale": float(ACTION_SCALE),
            "joint_order": joint_names,
            "command_order": COMMAND_ORDER,
            "coordinate_convention": "+X forward, +Y left, +Z up; base velocities and projected gravity are expressed in body frame",
            "quaternion_order": "IsaacLab internal root quaternion is [w, x, y, z] and represents body-to-world orientation; quaternion is not an Actor input",
            "first_previous_action_rule": "all zeros after ActionManager.reset()",
            "subsequent_previous_action_rule": "observation[t][36:48] equals Actor raw output from policy step t-1",
            "observation_dtype": "float32",
            "inference_dtype": "float32",
            "observation_source": "env.get_observations()['policy'] (actual Actor/ONNX input)",
            "actor_output_semantics": "deterministic Actor mean; final Linear output; no tanh",
            "observation_processing": {
                "term_scale": None,
                "term_clipping": None,
                "play_noise_or_corruption": False,
                "actor_normalizer": type(normalizer).__name__,
                "frame_stacking": False,
                "history": False,
                "privileged_observation_in_actor": False,
            },
            "action_processing": {
                "rsl_rl_clip_actions": agent_cfg.clip_actions,
                "joint_action_clip": action_term.cfg.clip,
                "formula": "joint_position_target = default_joint_position + float32(0.25) * actor_raw_action",
                "action_hold_seconds": float(dt),
                "decimation": int(base_env.cfg.decimation),
                "physics_dt_seconds": float(base_env.cfg.sim.dt),
            },
            "onnx": {
                "opset": [entry.version for entry in model.opset_import if entry.domain in ("", "ai.onnx")],
                "input_name": input_info.name,
                "input_shape": tensor_shape(input_info),
                "input_dtype": "float32",
                "output_name": output_info.name,
                "output_shape": tensor_shape(output_info),
                "output_dtype": "float32",
                "has_dynamic_dimensions": dynamic_dimensions,
                "runtime_version": ort.__version__,
                "runtime_providers": ort_session.get_providers(),
            },
            "capture": {
                "frame_count": args_cli.frames,
                "continuous_single_episode": True,
                "command_schedule": [
                    {"frames": [0, 1], "case": "standing", "command": [0.0, 0.0, 0.0]},
                    {"frames": [2, 3], "case": "forward_probe", "command": [0.4, 0.0, 0.0]},
                    {"frames": [4, 5], "case": "lateral_probe", "command": [0.0, 0.25, 0.0]},
                    {"frames": [6, 7], "case": "yaw_probe", "command": [0.0, 0.0, 0.5]},
                    {"frames": [8, args_cli.frames - 1], "case": "forward_walking", "command": [0.4, 0.0, 0.0]},
                ],
            },
        }
        report = {
            "status": "passed",
            "float_semantics": "all checked arrays and arithmetic are float32",
            "relative_error_denominator": "max(abs(expected), 1e-12)",
            "contains_nan_or_inf": False,
            "checks": tracker.results,
        }
        payload = {"metadata": metadata, "self_check": report, "frames": frames}
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        np.savez_compressed(
            npz_path,
            **stacked_arrays,
            frame_index=np.arange(args_cli.frames, dtype=np.int32),
            dt=np.full(args_cli.frames, dt, dtype=np.float32),
            metadata_json=np.asarray(json.dumps(metadata, ensure_ascii=False)),
        )
        print(json.dumps({"output_json": str(output_path), "output_npz": str(npz_path), **report}, indent=2), flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
