from turtle import update
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import os
import math
import time
import torch
import argparse
#######################my import###################################
import wandb
import datetime
import numpy as np
from copy import copy,deepcopy
from raisimGymTorch.env.bin import MPC
from raisimGymTorch.env.envs.MPC.my_helper import MyHelper
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver
from raisimGymTorch.mpc.mpc_controller_DAgger_all.control import Controller
###################################################################

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg_test.yaml", 'r'))

# create environment from the configuration file
env = VecEnv(MPC.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

################# my wandb #####################################
wandb.init(project="MPC_guide",group="1129",job_type="test",name="MPC",notes="It's just MPC"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
################################################################

# Constants
env_obs_dim = env.num_obs
env_act_dim = env.num_acts
sim_time_step = cfg['environment']['simulation_dt']
num_env = cfg['environment']['num_envs']

ctrl_time_step = cfg['environment']['control_dt']
ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD

num_threads = cfg['environment']['num_threads']
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * num_env

my_helper = MyHelper(env,cfg,n_steps)

# actual steps
my_helper.on_visualize(record=True)
# reset environment and controller
env.reset()
init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
init_joint_angle = init_ctrl_obs['jointPos']
controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)

normed_torque = []
normed_joint_pos = []
torque = []
joint_pos = []

rewards = []
normed_joint_pos_obs = []
normed_joint_pos_obs.append(MyHelper.get_ctrl_obs(my_helper.change_normed_obs(init_raw_obs))['jointPos'])
num_peripheral = 2

simulation_steps = total_steps*3 
for step in range(simulation_steps):

    frame_start = time.time()

    raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
    ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
    normed_joint_pos_obs.append(MyHelper.get_ctrl_obs(my_helper.change_normed_obs(raw_obs))['jointPos'])
    # # perturbed data collect
    # for _ in range(num_peripheral):
    #     tmp_controller = deepcopy(controller)
    #     perturbed_raw_obs,_ = env.observe(update_statistics=False, isnoise=True)
    #     perturbed_normed_obs = my_helper.change_normed_obs(raw_obs,perturbed_raw_obs)
    #     perturbed_ctrl_obs = MyHelper.get_ctrl_obs(perturbed_raw_obs)
    #     perturbed_u_mpc = tmp_controller.get_mpc_contact_force(perturbed_ctrl_obs)
    #     perturbed_mpc_action = tmp_controller.control_with_conactForce(perturbed_u_mpc,perturbed_ctrl_obs,step).reshape(-1,ctrl_act_dim)
    #     perturbed_mpc_action_normed = my_helper.normalize_target(perturbed_mpc_action)

    # mpc action
    u_mpc = controller.get_mpc_contact_force(ctrl_obs)    
    env_act = controller.control_with_conactForce(u_mpc,ctrl_obs,step).reshape(-1,ctrl_act_dim)
    torque.append(env_act[:,12:24])
    joint_pos.append(env_act[:,:12])

    # normalized action
    env_act = my_helper.normalize_target(env_act)
    normed_torque.append(env_act[:,12:24])
    normed_joint_pos.append(env_act[:,:12])

    # unnormalize and step
    env_act = my_helper.process_rl_action(env_act)
    reward, dones = env.step(env_act,step)
    rewards.append(reward)
    for i in range(num_env):
        if dones[i]:
            init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
            init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
            init_joint_angle = init_ctrl_obs['jointPos']
            controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)
            normed_joint_pos_obs.append(MyHelper.get_ctrl_obs(my_helper.change_normed_obs(init_raw_obs))['jointPos'])

# convert to numpy
torque = np.array(torque).reshape(-1,12)
joint_pos = np.array(joint_pos).reshape(-1,12)
normed_torque = np.array(normed_torque).reshape(-1,12)
normed_joint_pos = np.array(normed_joint_pos).reshape(-1,12)
rewards = np.array(rewards)
normed_joint_pos_obs = np.array(normed_joint_pos_obs).reshape(-1,12)
assert normed_torque.shape[0] == simulation_steps, " do it again"
assert normed_joint_pos.shape[0] == simulation_steps, " do it again"
assert torque.shape[0] == simulation_steps, " do it again"
assert joint_pos.shape[0] == simulation_steps, " do it again"

# record wandb
step_data = {}
for i in range(simulation_steps):
    for j in range(4):
        for k in range(3):
            # step_data["(normed)leg"+str(j+1)+"ff"+str(k+1)] = normed_torque[i][k+j*3]
            # step_data["(normed)leg"+str(j+1)+"joint"+str(k+1)] = normed_joint_pos[i][k+j*3]
            step_data["leg"+str(j+1)+"ff"+str(k+1)] = torque[i][k+j*3]
            step_data["leg"+str(j+1)+"joint"+str(k+1)] = joint_pos[i][k+j*3]
            
    step_data["reward"] = rewards[i]
    step_data["step"] = i
    wandb.log(step_data)     
# recording end and reset
my_helper.off_visualize(False)


















# from turtle import update
# from ruamel.yaml import YAML, dump, RoundTripDumper
# from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
# import os
# import math
# import time
# import torch
# import argparse
# #######################my import###################################
# import wandb
# import datetime
# import numpy as np
# from raisimGymTorch.env.bin import MPC
# from raisimGymTorch.env.envs.MPC.my_helper import MyHelper
# from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver
# from raisimGymTorch.mpc.mpc_controller_DAgger_all.control import Controller
# ###################################################################

# # directories
# task_path = os.path.dirname(os.path.realpath(__file__))
# home_path = task_path + "/../../../../.."

# # config
# cfg = YAML().load(open(task_path + "/cfg_test.yaml", 'r'))

# # create environment from the configuration file
# env = VecEnv(MPC.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

# ################# my wandb #####################################
# wandb.init(project="MPC_guide",group="1129",job_type="test",name="MPC",notes="It's just MPC"
#             ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
# ################################################################

# # Constants
# env_obs_dim = env.num_obs
# env_act_dim = env.num_acts
# sim_time_step = cfg['environment']['simulation_dt']
# num_env = cfg['environment']['num_envs']

# ctrl_time_step = cfg['environment']['control_dt']
# ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
# ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD

# num_threads = cfg['environment']['num_threads']
# n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
# total_steps = n_steps * num_env

# my_helper = MyHelper(env,cfg,n_steps)

# # actual steps
# my_helper.on_visualize(record=True)
# # reset environment and controller
# env.reset()
# init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
# init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
# init_joint_angle = init_ctrl_obs['jointPos']
# controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)
# torque = []
# joint_pos = []
# rewards = []
# joint_pos_obs = []
# joint_pos_obs.append(init_ctrl_obs['jointPos'])

# simulation_steps = total_steps*3 
# for step in range(simulation_steps):

#     frame_start = time.time()

#     raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
#     ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
#     joint_pos_obs.append(ctrl_obs['jointPos'])

#     u_mpc = controller.get_mpc_contact_force(ctrl_obs)    
#     env_act = controller.control_with_conactForce(u_mpc,ctrl_obs,step).reshape(-1,ctrl_act_dim)
#     torque.append(env_act[:,12:24])
#     joint_pos.append(env_act[:,:12])
#     reward, dones = env.step(env_act,step)
#     rewards.append(reward)
#     for i in range(num_env):
#         if dones[i]:
#             init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
#             init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
#             init_joint_angle = init_ctrl_obs['jointPos']
#             controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)
#             joint_pos_obs.append(init_ctrl_obs['jointPos'])


# torque = np.array(torque).reshape(-1,12)
# joint_pos = np.array(joint_pos).reshape(-1,12)
# rewards = np.array(rewards)
# joint_pos_obs = np.array(joint_pos_obs).reshape(-1,12)

# torque_mean = np.mean(torque,axis=0)
# torque_var = np.var(torque,axis=0)

# joint_pos_mean = np.mean(joint_pos,axis=0)
# joint_pos_var = np.var(joint_pos,axis=0)

# joint_pos_obs_mean = np.mean(joint_pos_obs,axis=0)
# joint_pos_obs_var = np.var(joint_pos_obs,axis=0)

# print("torque_mean")
# print(torque_mean)
# print("torque_var")
# print(torque_var)
# print("joint_mean")
# print(joint_pos_mean)
# print("joint_var")
# print(joint_pos_var)
# print("joint_pos_obs_mean")
# print(joint_pos_obs_mean)
# print("joint_pos_obs_var")
# print(joint_pos_obs_var)

# assert torque.shape[0] == simulation_steps, " do it again"
# assert joint_pos.shape[0] == simulation_steps, " do it again"
# i = 0
# step_data = {}
# for i in range(simulation_steps):
#     for j in range(4):
#         for k in range(3):
#             step_data["leg"+str(j+1)+"ff"+str(k+1)] = torque[i][k+j*3]
#             step_data["leg"+str(j+1)+"joint"+str(k+1)] = joint_pos[i][k+j*3]
#     step_data["reward"] = rewards[i]
#     step_data["step"] = i
#     wandb.log(step_data)     
# # recording end and reset
# my_helper.off_visualize(False)