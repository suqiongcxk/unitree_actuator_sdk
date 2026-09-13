#!/usr/bin/env python3
"""Validate V5 PyTorch/ONNX parity, export golden vectors, and run Isaac behavior acceptance."""
from __future__ import annotations
import argparse, copy, hashlib, importlib.metadata, json, math, os, subprocess, sys
from pathlib import Path
sys.path.insert(0,"/workspace/isaaclab/scripts/reinforcement_learning/rsl_rl")
sys.path.insert(0,"/root/gpufree-data/CreeperLab/deployment/creeper_policy/python_deps")
from isaaclab.app import AppLauncher
import cli_args
ROOT=Path(__file__).resolve().parent
CK=Path("/workspace/isaaclab/logs/rsl_rl/creeper_20260906_flat_v5/2026-09-11_11-47-06_scratch_seed42_balanced_commands_contact_safe/model_950.pt")
parser=argparse.ArgumentParser();parser.add_argument("--task",default="Isaac-Velocity-Flat-Creeper-20260906-V5-Play-v0");parser.add_argument("--output-dir",type=Path,default=ROOT)
cli_args.add_rsl_rl_args(parser);AppLauncher.add_app_launcher_args(parser);args,hydra=parser.parse_known_args();sys.argv=[sys.argv[0],*hydra]
app=AppLauncher(args).app
import gymnasium as gym, numpy as np, onnx, onnxruntime as ort, torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
import isaaclab_tasks  # noqa
from isaaclab_tasks.utils import load_cfg_from_registry,parse_env_cfg

JOINTS=["FL_hip_joint","FR_hip_joint","RL_hip_joint","RR_hip_joint","FL_thigh_joint","FR_thigh_joint","RL_thigh_joint","RR_thigh_joint","FL_calf_joint","FR_calf_joint","RL_calf_joint","RR_calf_joint"]
DEFAULT=np.array([.1,-.1,.1,-.1,.8,.8,1.,1.,-1.5,-1.5,-1.5,-1.5],np.float32)
CASES=[0.,.05,.10,.20,.40];SEEDS=list(range(42,52));LEGS=["FL","FR","RL","RR"]
def sha(p):
 h=hashlib.sha256()
 with open(p,"rb") as f:
  for b in iter(lambda:f.read(1<<20),b""):h.update(b)
 return h.hexdigest()
def f32list(x):return np.asarray(x,dtype=np.float32).tolist()
def setcmd(term,c):term.vel_command_b[:]=torch.tensor(c,device=term.device);term.is_heading_env[:]=False;term.is_standing_env[:]=False;term.time_left[:]=1e9
def maxrun(a):
 best=cur=0
 for x in a:
  cur=cur+1 if x else 0;best=max(best,cur)
 return best

def main():
 out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True);onnx_path=out/"creeper_model_950_actor.onnx";ck=Path(args.checkpoint).resolve() if args.checkpoint else CK
 env_cfg=parse_env_cfg(args.task,device=args.device,num_envs=1);env_cfg.scene.num_envs=1;env_cfg.seed=42;env_cfg.episode_length_s=30.0;env_cfg.observations.policy.enable_corruption=False;env_cfg.events.add_base_mass=None;env_cfg.events.base_external_force_torque=None;env_cfg.events.push_robot=None
 agent_cfg=load_cfg_from_registry(args.task,"rsl_rl_cfg_entry_point");agent_cfg.device=args.device
 env=RslRlVecEnvWrapper(gym.make(args.task,cfg=env_cfg),clip_actions=agent_cfg.clip_actions);base=env.unwrapped;robot=base.scene["robot"];sensor=base.scene.sensors["contact_forces"];term=base.command_manager.get_term("base_velocity")
 assert robot.joint_names==JOINTS,(robot.joint_names,JOINTS);assert np.allclose(robot.data.default_joint_pos[0].cpu().numpy(),DEFAULT,atol=1e-7)
 fs=[sensor.body_names.index(f"{x}_foot") for x in LEGS];nonfoot=torch.ones(len(sensor.body_names),dtype=torch.bool,device=base.device);nonfoot[fs]=False
 runner=OnPolicyRunner(env,agent_cfg.to_dict(),log_dir=None,device=agent_cfg.device);runner.load(str(ck));nn=runner.alg.policy.eval();policy=runner.get_inference_policy(device=base.device);actor=copy.deepcopy(nn.actor).cpu().eval()
 session=ort.InferenceSession(str(onnx_path),providers=["CPUExecutionProvider"]);dt=float(base.step_dt)
 # 1000 bounded random observations, including explicit command values and boundary-biased samples.
 rng=np.random.default_rng(42);rand=np.empty((1000,48),np.float32);rand[:,0:3]=rng.uniform(-1.5,1.5,(1000,3));rand[:,3:6]=rng.uniform(-2,2,(1000,3));rand[:,6:9]=rng.uniform(-1,1,(1000,3));rand[:,9:12]=rng.uniform([-1,-.3,-1],[1,.3,1],(1000,3));rand[:,12:24]=rng.uniform(-1.2,1.2,(1000,12));rand[:,24:36]=rng.uniform(-25,25,(1000,12));rand[:,36:48]=rng.uniform(-4,4,(1000,12));rand[:10]*=.95
 for i,v in enumerate([0,.05,.1,.2,.4,-.2,.2,.2,.4,-.4]):rand[i,9:12]=[v,0,0] if i<6 else ([0,.2,0] if i==6 else ([0,-.2,0] if i==7 else ([0,0,.5] if i==8 else [0,0,-.5])))
 with torch.inference_mode():pt=actor(torch.from_numpy(rand)).numpy()
 oo=np.concatenate([session.run(["actions"],{"obs":x[None]})[0] for x in rand],axis=0);diff=np.abs(pt-oo);den=np.maximum(np.abs(pt),1e-8);idx=np.unravel_index(np.argmax(diff),diff.shape)
 parity={"random_count":1000,"max_abs_error":float(diff.max()),"mean_abs_error":float(diff.mean()),"max_relative_error":float((diff/den).max()),"max_error_sample":int(idx[0]),"max_error_action_index":int(idx[1]),"finite":bool(np.isfinite(rand).all() and np.isfinite(pt).all() and np.isfinite(oo).all())}
 if parity["max_abs_error"]>1e-5:raise RuntimeError(parity)
 # Golden rollout: actual ObservationManager output; exactly one history reset at frame zero.
 torch.manual_seed(42000);env.seed(42000);obs,_=env.reset();nn.reset(torch.ones(1,dtype=torch.bool,device=base.device));setcmd(term,(.2,0,0));obs=env.get_observations();frames=[];roll_pt=[];roll_oo=[]
 for k in range(100):
  setcmd(term,(.2,0,0));obs=env.get_observations();o=obs["policy"][0].detach().cpu().numpy().astype(np.float32)
  with torch.inference_mode():a=policy(obs)[0].detach().cpu().numpy().astype(np.float32)
  ao=session.run(["actions"],{"obs":o[None]})[0][0].astype(np.float32);q=robot.data.joint_pos[0].cpu().numpy().astype(np.float32);dq=robot.data.joint_vel[0].cpu().numpy().astype(np.float32);qd=(DEFAULT+np.float32(.25)*ao).astype(np.float32)
  raw={"frame_index":k,"timestamp":float(np.float32(k*dt)),"default_joint_position":f32list(DEFAULT),"base_linear_velocity_body":f32list(robot.data.root_lin_vel_b[0].cpu()),"base_angular_velocity_body":f32list(robot.data.root_ang_vel_b[0].cpu()),"projected_gravity_body":f32list(robot.data.projected_gravity_b[0].cpu()),"command":f32list([.2,0,0]),"joint_position":f32list(q),"joint_velocity":f32list(dq),"previous_raw_action":f32list(o[36:48]),"observation_final":f32list(o),"pytorch_actor_raw_action":f32list(a),"onnx_actor_raw_action":f32list(ao),"joint_position_target":f32list(qd)};frames.append(raw);roll_pt.append(a);roll_oo.append(ao)
  obs,_,done,_=env.step(torch.from_numpy(a[None]).to(base.device));nn.reset(done)
 ro=np.asarray([f["observation_final"] for f in frames],np.float32);ra=np.asarray(roll_oo,np.float32);rq=DEFAULT+np.float32(.25)*ra
 roll_pt_array=np.asarray(roll_pt,np.float32);history_pt_err=float(np.max(np.abs(ro[1:,36:48]-roll_pt_array[:-1])));history_onnx_err=float(np.max(np.abs(ro[1:,36:48]-ra[:-1])));first_prev=float(np.max(np.abs(ro[0,36:48])));roll_diff=np.abs(roll_pt_array-ra)
 if history_pt_err!=0.0 or first_prev!=0.0:raise RuntimeError(f"history mismatch first={first_prev}, pytorch_previous={history_pt_err}")
 dqdes=np.abs(np.diff(rq,axis=0));sumdq=dqdes.sum(1);limits=robot.data.joint_pos_limits[0].cpu().numpy();limit_viol=int(np.sum((rq<limits[:,0])|(rq>limits[:,1])))
 action_stats={JOINTS[i]:{"min":float(ra[:,i].min()),"max":float(ra[:,i].max()),"mean":float(ra[:,i].mean()),"std":float(ra[:,i].std()),"max_delta":float(np.abs(np.diff(ra[:,i])).max()),"q_des_min":float(rq[:,i].min()),"q_des_max":float(rq[:,i].max())} for i in range(12)}
 rollout={"pytorch_onnx_max_abs_error":float(roll_diff.max()),"history_vs_previous_pytorch_max_abs_error":history_pt_err,"history_vs_previous_onnx_max_abs_error":history_onnx_err,"first_previous_action_max_abs":first_prev,"max_delta_q_des":float(dqdes.max()),"sum_abs_delta_q_des":{"max":float(sumdq.max()),"p50":float(np.quantile(sumdq,.5)),"p95":float(np.quantile(sumdq,.95)),"p99":float(np.quantile(sumdq,.99))},"joint_limit_violations":limit_viol,"finite":bool(np.isfinite(ro).all() and np.isfinite(ra).all() and np.isfinite(rq).all()),"per_joint":action_stats}
 try:git_commit=subprocess.check_output(["git","-C","/root/gpufree-data/CreeperLab","rev-parse","HEAD"],text=True,stderr=subprocess.DEVNULL).strip()
 except:git_commit="not_available"
 metadata={"checkpoint":str(ck),"checkpoint_sha256":sha(ck),"onnx":str(onnx_path),"onnx_sha256":sha(onnx_path),"urdf":"/root/gpufree-data/Creeper20260906/urdf/Creeper20260906.urdf","urdf_sha256":sha("/root/gpufree-data/Creeper20260906/urdf/Creeper20260906.urdf"),"git_commit":git_commit,"versions":{"torch":torch.__version__,"onnx":onnx.__version__,"onnxruntime":ort.__version__,"rsl_rl":importlib.metadata.version("rsl-rl-lib")},"actor":"48-128-128-128-12 ELU, linear output","observation_normalizer":"Identity","joint_order":JOINTS,"observation_order":["base_linear_velocity_body[3]","base_angular_velocity_body[3]","projected_gravity_body[3]","command[vx,vy,yaw_rate]","joint_position-default_joint_position[12]","joint_velocity[12]","previous_raw_action[12]"],"action_scale":.25,"policy_frequency_hz":50,"physics_dt":.005,"decimation":4,"policy_dt":.02,"coordinate_convention":"body +X forward, +Y left, +Z up","seed":42000,"pytorch_onnx_max_abs_error":max(parity["max_abs_error"],rollout["pytorch_onnx_max_abs_error"])}
 (out/"model_950_policy_golden_vectors.json").write_text(json.dumps({"metadata":metadata,"frames":frames},indent=2)+"\n")
 # 10 explicit seeds per speed, complete 5+1+10+1+5 second sequence.
 trials=[]
 for target in CASES:
  for seed in SEEDS:
   torch.manual_seed(seed);env.seed(seed);obs,_=env.reset();nn.reset(torch.ones(1,dtype=torch.bool,device=base.device));prev=None;contacts=[];slides=[];vel=[];heights=[];dqv=[];tau=[];qjump=[];dones=basehits=nonhits=0;post=[]
   for k in range(round(22/dt)):
    t=k*dt
    if t<5:c=0
    elif t<6:c=target*(t-5)
    elif t<16:c=target
    elif t<17:c=target*(17-t)
    else:c=0
    setcmd(term,(c,0,0));obs=env.get_observations()
    with torch.inference_mode():a=policy(obs)
    qd=robot.data.default_joint_pos+np.float32(.25)*a
    if prev is not None:qjump.append(float(torch.max(torch.abs(qd-prev))))
    prev=qd.clone();obs,_,done,_=env.step(a);nn.reset(done);dones+=int(done[0]);basehits+=int(base.termination_manager.get_term("base_contact")[0])
    force=torch.linalg.vector_norm(sensor.data.net_forces_w[:,fs],dim=-1)[0];con=force>1;allf=torch.linalg.vector_norm(sensor.data.net_forces_w,dim=-1)[0];nonhits+=int((allf[nonfoot]>1).any())
    if 6<=t<16:
     contacts.append(con.cpu().numpy());footspd=torch.linalg.vector_norm(robot.data.body_lin_vel_w[:,[robot.body_names.index(f"{x}_foot") for x in LEGS],:2],dim=-1)[0];slides.append((footspd*con).cpu().numpy());vel.append(float(robot.data.root_lin_vel_b[0,0]));heights.append(float(robot.data.root_link_pos_w[0,2]));dqv.append(float(robot.data.joint_vel[0].abs().max()));tau.append(float(robot.data.applied_torque[0].abs().max()))
    if t>=17:post.append(float(robot.data.root_lin_vel_b[0,0]))
   con=np.asarray(contacts,bool);trans=np.sum(con[1:]!=con[:-1],axis=0);duty=con.mean(0);sl=np.asarray(slides)
   trials.append({"target_vx":target,"seed":seed,"mean_vx_hold":float(np.mean(vel)),"std_vx_hold":float(np.std(vel)),"mean_height_hold":float(np.mean(heights)),"contact_duty":duty.tolist(),"contact_transitions":trans.tolist(),"periodic_liftoff_each_leg":bool(np.all(trans>=4)),"long_term_nonparticipating_leg":bool(np.any((duty>.995)&(trans<2))),"mean_contact_slip_mps":[float(sl[:,i][con[:,i]].mean()) if con[:,i].any() else 0 for i in range(4)],"max_joint_velocity_radps":max(dqv),"max_torque_nm":max(tau),"max_delta_q_des":max(qjump),"reset_count":dones,"base_contact_frames":basehits,"nonfoot_contact_frames":nonhits,"post_zero_mean_abs_vx":float(np.mean(np.abs(post[-round(3/dt):]))),"post_zero_stable":bool(np.mean(np.abs(post[-round(3/dt):]))<.05)})
   print(f"[BEHAVIOR] vx={target:.2f} seed={seed}",flush=True)
 summaries={}
 for target in CASES:
  a=[x for x in trials if x["target_vx"]==target];summaries[str(target)]={"trial_count":len(a),"mean_actual_vx":float(np.mean([x["mean_vx_hold"] for x in a])),"std_across_trials":float(np.std([x["mean_vx_hold"] for x in a])),"periodic_liftoff_all_trials":all(x["periodic_liftoff_each_leg"] for x in a),"periodic_trial_count":sum(x["periodic_liftoff_each_leg"] for x in a),"any_nonparticipating_leg":any(x["long_term_nonparticipating_leg"] for x in a),"reset_trials":sum(x["reset_count"]>0 for x in a),"base_contact_trials":sum(x["base_contact_frames"]>0 for x in a),"nonfoot_contact_trials":sum(x["nonfoot_contact_frames"]>0 for x in a),"post_zero_stable_trials":sum(x["post_zero_stable"] for x in a),"max_joint_velocity_radps":max(x["max_joint_velocity_radps"] for x in a),"max_torque_nm":max(x["max_torque_nm"] for x in a),"max_delta_q_des":max(x["max_delta_q_des"] for x in a),"mean_contact_duty":np.mean([x["contact_duty"] for x in a],axis=0).tolist(),"mean_contact_slip_mps":np.mean([x["mean_contact_slip_mps"] for x in a],axis=0).tolist()}
 behavior={"task":args.task,"checkpoint":str(ck),"seeds":SEEDS,"protocol":"5s zero + 1s ramp + 10s hold + 1s ramp down + 5s zero","foot_contact_threshold_n":1.,"summaries":summaries,"trials":trials};(out/"model_950_simulation_raw.json").write_text(json.dumps(behavior,indent=2)+"\n")
 validation={"random_parity":parity,"rollout_parity_and_actions":rollout,"behavior":behavior};(out/"validation_results.json").write_text(json.dumps(validation,indent=2)+"\n");print("[DONE]",out,flush=True);env.close()
if __name__=="__main__":
 try:main()
 finally:app.close()
