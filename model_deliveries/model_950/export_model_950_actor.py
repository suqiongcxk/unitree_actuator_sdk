#!/usr/bin/env python3
"""Export the deterministic Creeper V5 actor from its real IsaacLab/RSL-RL configuration."""
from __future__ import annotations
import argparse, hashlib, json, os, sys
from pathlib import Path
sys.path.insert(0, "/workspace/isaaclab/scripts/reinforcement_learning/rsl_rl")
from isaaclab.app import AppLauncher
import cli_args

CHECKPOINT=Path("/workspace/isaaclab/logs/rsl_rl/creeper_20260906_flat_v5/2026-09-11_11-47-06_scratch_seed42_balanced_commands_contact_safe/model_950.pt")
EXPECTED="df9eb646aa3fbde7b60a4f7f7aed2ffd4d2f07a0681988f7d040161c967bb5ec"
parser=argparse.ArgumentParser();parser.add_argument("--task",default="Isaac-Velocity-Flat-Creeper-20260906-V5-Play-v0");parser.add_argument("--output-dir",type=Path,default=Path(__file__).resolve().parent)
cli_args.add_rsl_rl_args(parser);AppLauncher.add_app_launcher_args(parser);args,hydra=parser.parse_known_args();sys.argv=[sys.argv[0],*hydra]
simulation_app=AppLauncher(args).app
import gymnasium as gym, onnx, torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
import isaaclab_tasks  # noqa
from isaaclab_tasks.utils import load_cfg_from_registry,parse_env_cfg

def sha(p):
    h=hashlib.sha256()
    with open(p,"rb") as f:
        for b in iter(lambda:f.read(1<<20),b""):h.update(b)
    return h.hexdigest()

def shape(v):
    return [d.dim_value if d.HasField("dim_value") else d.dim_param for d in v.type.tensor_type.shape.dim]

def main():
    ck=Path(args.checkpoint).resolve() if args.checkpoint else CHECKPOINT.resolve();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True);onnx_path=out/"creeper_model_950_actor.onnx"
    if sha(ck)!=EXPECTED:raise RuntimeError(f"checkpoint SHA mismatch: {sha(ck)}")
    raw=torch.load(ck,map_location="cpu",weights_only=False)
    env_cfg=parse_env_cfg(args.task,device=args.device,num_envs=1);env_cfg.scene.num_envs=1;env_cfg.observations.policy.enable_corruption=False
    agent_cfg=load_cfg_from_registry(args.task,"rsl_rl_cfg_entry_point");agent_cfg.device=args.device
    env=RslRlVecEnvWrapper(gym.make(args.task,cfg=env_cfg),clip_actions=agent_cfg.clip_actions)
    try:
        runner=OnPolicyRunner(env,agent_cfg.to_dict(),log_dir=None,device=agent_cfg.device);runner.load(str(ck));policy=runner.alg.policy.eval()
        normalizer=policy.actor_obs_normalizer
        if type(policy).__name__!="ActorCritic" or policy.is_recurrent:raise RuntimeError(f"unsupported policy {type(policy).__name__}, recurrent={policy.is_recurrent}")
        if type(normalizer).__name__!="Identity":raise RuntimeError(f"non-identity normalizer: {normalizer}")
        actor=policy.actor.cpu().eval()
        torch.onnx.export(actor,torch.zeros(1,48,dtype=torch.float32),str(onnx_path),input_names=["obs"],output_names=["actions"],opset_version=18,dynamic_axes=None,dynamo=False)
        model=onnx.load(str(onnx_path));onnx.helper.set_model_props(model,{"robot":"Creeper20260906","task":args.task,"checkpoint":str(ck),"checkpoint_sha256":sha(ck),"normalizer":"Identity","policy":"deterministic actor mean","action_scale":"0.25"});onnx.save(model,str(onnx_path));onnx.checker.check_model(onnx.load(str(onnx_path)))
        model=onnx.load(str(onnx_path));inp=model.graph.input[0];op=model.graph.output[0]
        if inp.name!="obs" or shape(inp)!=[1,48] or op.name!="actions" or shape(op)!=[1,12]:raise RuntimeError("ONNX interface mismatch")
        actor_state=[{"name":n,"shape":list(t.shape),"dtype":str(t.dtype)} for n,t in actor.state_dict().items()]
        info={"checkpoint":str(ck),"checkpoint_sha256":sha(ck),"checkpoint_top_level_keys":list(raw.keys()),"checkpoint_iteration":int(raw["iter"]),"policy_class":type(policy).__name__,"is_recurrent":bool(policy.is_recurrent),"actor":str(actor),"actor_layers":actor_state,"activation":"ELU","output_tanh":False,"actor_observation_normalizer":type(normalizer).__name__,"actor_input_size":48,"actor_output_size":12,"onnx":str(onnx_path),"onnx_sha256":sha(onnx_path),"opset":18,"input":{"name":"obs","shape":[1,48],"dtype":"float32","dynamic":False},"output":{"name":"actions","shape":[1,12],"dtype":"float32","dynamic":False}}
        (out/"actor_audit.json").write_text(json.dumps(info,indent=2)+"\n");print(json.dumps(info,indent=2),flush=True)
    finally:env.close()
if __name__=="__main__":
    try:main()
    finally:simulation_app.close()
