from email.policy import default
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin.MPC_DAgger import RaisimGymEnv
from raisimGymTorch.env.bin.MPC_DAgger import NormalSampler
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param
import os
import math
import time
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
import torch.nn as nn
import numpy as np
import torch
import datetime
import argparse
#######################my import###################################
import wandb
import raisimpy as raisim
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
import random
from raisimGymTorch.mpc import control
from raisimGymTorch.helper import wandb_helper
###################################################################

# task specification
task_name = "MPC_DAgger"

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
weight_path = args.weight

# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# wandb initialization
wandb.init(project="MPC_guide",group="1005",job_type="train",name="grf imitation",
            notes="RL policy that imitat grf"
            ,config=cfg,save_code=True, mode = 'disabled')

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))

# shortcuts
control_ob_dim = env.num_obs
control_act_dim = env.num_acts
ob_dim = env.num_obs
act_dim = 4
num_threads = cfg['environment']['num_threads']

# Training
## n_steps: horizon size
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs

avg_rewards = [] # average rewards over all the updates

actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim),
                         ppo_module.MultivariateGaussianDiagonalCovariance(act_dim,
                                                                           env.num_envs,
                                                                           1.0,
                                                                           NormalSampler(act_dim),
                                                                           cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, ob_dim, 1),
                           device)

saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])

ppo = PPO.PPO(actor=actor,
              critic=critic,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              num_mini_batches=4,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False,
              )


if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

# update is number of epochs
for update in range(2601):
    env.reset()
    # per update variables
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.
    rewardinfo = {}
    velocities = {"forward": 0.0, "total": 0.0, "yawing": 0.0}


    # Initialize controller
    controller = []
    obs = env.observe(False,True)
    for i in range(cfg['environment']['num_envs']):
        joint_pos_init_obs = obs[i,8:20]
        controller.append(control._setup_controller())
        controller[i].reset(0.0,joint_pos_init_obs)
        
    time_step = cfg['environment']['simulation_dt']
    ctr_time_step = cfg['environment']['control_dt']

    command_function = control._generate_example_linear_angular_speed

    env.turn_on_visualization()
    for step in range(n_steps):
        obs = env.observe(False,True)
        action = np.zeros((cfg['environment']['num_envs'],control_act_dim))
        lin_speed, ang_speed, e_stop = command_function(ctr_time_step*step)
        for i in range(obs.shape[0]):
            control._update_controller_params(controller[i],np.array([0.6,0.0,0.0]),0.0)
            # control._update_controller_params(controller[i],lin_speed,ang_speed)
            controller[i].update(ctr_time_step*step,obs[i])
            action_tmp,_ = controller[i].get_action(obs[i])
            pos_action = action_tmp[control.POSITION_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            tq_action = action_tmp[control.TORQUE_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            kp_action = action_tmp[control.POSITION_GAIN_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            kd_action = action_tmp[control.VELOCITY_GAIN_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            action[i] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
        action = action.astype(np.float32)
        reward, dones = env.step(action)
        for i in range(dones.shape[0]):
            if dones[i]:
                obs = env.observe(update_statistics=False,israw=True)
                joint_pos_init_obs = obs[i,8:20]
                controller[i].reset(ctr_time_step*(step+1),joint_pos_init_obs)
                