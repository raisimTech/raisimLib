from turtle import update
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import MPC_multiprocessing
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import os
import math
import time
import torch
import argparse
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver
#######################my import###################################
import wandb
import datetime
import numpy as np
from raisimGymTorch.env.envs.MPC_multiprocessing.my_helper import MyHelper
from raisimGymTorch.mpc.mpc_controller_multiprocessing.control import Controller
###################################################################

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg_test.yaml", 'r'))

# create environment from the configuration file
env = VecEnv(MPC_multiprocessing.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

################# my wandb #####################################
wandb.init(project="MPC_guide",group="1112",job_type="test",name="MPC_multiprocessing",notes="It's just MPC but with multiprocessing"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
################################################################

# Constants
env_obs_dim = env.num_obs
env_act_dim = env.num_acts
sim_time_step = cfg['environment']['simulation_dt']
num_env = cfg['environment']['num_envs']

rl_obs_dim = env.num_obs + 4 #(add desired leg state)
rl_act_dim = 12 # contact forces

ctrl_time_step = cfg['environment']['control_dt']
ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD

num_threads = cfg['environment']['num_threads']
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * num_env

my_helper = MyHelper(env,cfg,None,rl_obs_dim,rl_act_dim,n_steps)

# simulation
env.reset()
done_sum = 0
average_dones = 0.
start_step_id = 0

# actual steps
my_helper.on_visualize(update = 0,istest=True)
env.reset()
init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
init_joint_angle = init_ctrl_obs['jointPos']
controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)

for step in range(total_steps*2):
    frame_start = time.time()

    raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
    ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
    
    u_mpc = controller.get_mpc_contact_force(ctrl_obs)    
    st = time.perf_counter()
    env_act = controller.control_with_conactForce(u_mpc,ctrl_obs,step)
    print(time.perf_counter()-st)

    reward, dones = env.step(env_act,step)
    for i in range(num_env):
        if dones[i]:
            init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
            init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
            init_joint_angle = init_ctrl_obs['jointPos']
            controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)
    
    step_data = {"step":step}
    step_data["x_vel"] = ctrl_obs['baseLinVel'][0][0]
    step_data["reward"] = reward[0]
    # for key in contact_forces:
    #         step_data[key] = contact_forces[key]
    # for key in u_rl_contact_forces:
    #     step_data[key] = u_rl_contact_forces[key]
    # for key in u_rl_contact_forces_normed:
    #     step_data[key] = u_rl_contact_forces_normed[key]
    # step_data["loss"] = np.mean((u_rl.reshape(-1,rl_act_dim)-u_mpc_normed)**2)
    wandb.log(step_data)
    
# recording end and reset
my_helper.off_visualize()
print("Finished at the maximum visualization steps")