from email.policy import default
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin.MPC_BC import RaisimGymEnv
from raisimGymTorch.env.bin.MPC_BC import NormalSampler
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param
import os
import math
import time
import raisimGymTorch.algo.ppo_BC.module as ppo_module
import raisimGymTorch.algo.ppo_BC.ppo_BC as PPO_BC
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
from raisimGymTorch.env.envs.MPC_BC.my_helper import MyHelper
from raisimGymTorch.mpc.mpc_controller_BC.control import Controller
###################################################################


# task specification
task_name = "MPC_BC"
algo_name = "ppo_BC"
mpc_name = "mpc_controller_BC"
# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."
algo_path = home_path+"/raisimGymTorch/raisimGymTorch/algo/"+algo_name
vecenv_path = home_path+"/raisimGymTorch/raisimGymTorch/env/"
mpc_path = home_path+"/raisimGymTorch/raisimGymTorch/mpc/"+mpc_name

# arguments
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
weight_path = args.weight
# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# wandb initialization
wandb.init(project="MPC_guide",group="1108",job_type="train",name="PPO with BC Loss",
            notes="PPO training with BC Loss"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])

wandb.save(task_path+"/*") # save all files in envs
wandb.save(mpc_path+"/*") # save mpc codes
wandb.save(algo_path+"/*") # save algo
wandb.save(vecenv_path+"/*") # save common environment files

saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])

# Environment
env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))

env_obs_dim = env.num_obs
env_act_dim = env.num_acts
sim_time_step = cfg['environment']['simulation_dt']
num_env = cfg['environment']['num_envs']

# RL
rl_obs_dim = env.num_obs + 4 #(add desired leg state)
rl_act_dim = 12 # contact forces
init_std = 0.5
min_std = 0.05

actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, rl_obs_dim, rl_act_dim),
                         ppo_module.MultivariateGaussianDiagonalCovariance(rl_act_dim,
                                                                           env.num_envs,
                                                                           init_std,
                                                                           NormalSampler(rl_act_dim),
                                                                           cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, rl_obs_dim, 1),
                           device)
# Control
ctrl_time_step = cfg['environment']['control_dt']
ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD.

# etc constants
video_header = cfg['video_header']
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs
avg_rewards = []
num_peripheral = cfg['mpc']['num_peripheral']
my_helper = MyHelper(env,cfg,saver,rl_obs_dim,rl_act_dim,n_steps)

# collect mpc trajectories
controller = controller = my_helper.reset_env_controller()
reward_ll_sum = 0.0
done_sum = 0.0
mpc_obs = np.zeros((num_env,n_steps*(1+num_peripheral),rl_obs_dim))
mpc_act = np.zeros((num_env,n_steps*(1+num_peripheral),rl_act_dim))
counter=0
for step in range(n_steps):
    raw_obs,_ = env.observe(update_statistics=False,isnoise=False)
    ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
    normed_obs = my_helper.change_normed_obs(raw_obs)
    desired_leg_state = MyHelper.change_foot_contact_range(controller.get_desired_leg_state()) # get desired leg state for rl obs
    rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state) # get rl obs

    u_mpc = controller.get_mpc_contact_force(ctrl_obs).reshape(-1,rl_act_dim)
    u_mpc_normed = my_helper.normalize_target(u_mpc)
    mpc_obs[:,counter,:] = rl_obs
    mpc_act[:,counter,:] = u_mpc_normed
    counter+=1
    for _ in range(num_peripheral):
        perturbed_raw_obs,_ = env.observe(update_statistics=False, isnoise=True) # get perturbed observation
        perturbed_normed_obs = my_helper.change_normed_obs(raw_obs,perturbed_raw_obs) # get desired leg state for rl obs
        perturbed_ctrl_obs = MyHelper.get_ctrl_obs(perturbed_raw_obs)
        perturbed_rl_obs = MyHelper.get_rl_obs(perturbed_normed_obs,desired_leg_state)

        perturbed_u_mpc = controller.get_mpc_contact_force(perturbed_ctrl_obs).reshape(-1,rl_act_dim)
        perturbed_u_mpc_normed = my_helper.normalize_target(perturbed_u_mpc)
        
        mpc_obs[:,counter,:] = perturbed_rl_obs
        mpc_act[:,counter,:] = perturbed_u_mpc_normed
        counter+=1

    env_act = controller.control_with_conactForce(u_mpc.reshape(-1,4,3),ctrl_obs,step)
    reward, dones = env.step(env_act,step)  
    for i in range(num_env):
        if dones[i]:
            my_helper.reset_controller(i,step,controller)

# Training
ppo = PPO_BC.PPO(actor=actor,
              critic=critic,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              num_mini_batches=4,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=True,
              )   

if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

mpc_env_start_num = -5
assert (cfg['environment']['updates']//20)*5 > num_env, "Not all environemnts in MPC data are trained "

## train and evaluate policy
for update in range(cfg['environment']['updates']):
    start = time.time()
    if update%20 == 0:
        mpc_env_start_num+=5
        if mpc_env_start_num >= num_env:
            mpc_env_start_num = 0

    # save model and record video
    if update % cfg['environment']['eval_every_n'] == 0:
        print("Visualizing and evaluating the current policy") 
        # save trained model
        my_helper.save_model(actor=actor,critic=critic,ppo=ppo,update=update)
        # visualize, simulate and evaluate
        my_helper.on_visualize(record=True,update=update,istest=False)
        controller = my_helper.reset_env_controller()
        if cfg['environment']['render']:
            my_helper.simulate_evaluate(update=update,controller=controller)
        my_helper.off_visualize(record=True)
        # compare with MPC
        my_helper.on_visualize(record=False,update=update,istest=False)
        controller = my_helper.reset_env_controller()
        if cfg['environment']['render']:
            my_helper.compare_with_mpc(update=update,controller=controller)
        my_helper.off_visualize(record=False)
    
    # collect trajectories
    controller = controller = my_helper.reset_env_controller()
    reward_ll_sum = 0.0
    done_sum = 0.0
    for step in range(n_steps):
        raw_obs,_ = env.observe(update_statistics=False,isnoise=False)
        normed_obs = my_helper.change_normed_obs(raw_obs)
        desired_leg_state = MyHelper.change_foot_contact_range(controller.get_desired_leg_state()) # get desired leg state for rl obs
        rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state) # get rl obs
        ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)

        u_rl = ppo.act(rl_obs)
        u_rl_unnormed = my_helper.unnormalize(u_rl)
        u_rl_unnormed = u_rl_unnormed.reshape(-1,4,3)
        env_act = controller.control_with_conactForce(u_rl_unnormed,ctrl_obs,step)
        reward, dones = env.step(env_act,step)
        ppo.step(value_obs=rl_obs, rews=reward, dones=dones)
        done_sum = done_sum + np.sum(dones)
        reward_ll_sum = reward_ll_sum + np.sum(reward)
        for i in range(num_env):
            if dones[i]:
                my_helper.reset_controller(i,step,controller)

    # take st step to get value obs(last value) and update RL policy
    raw_obs,_ = env.observe(update_statistics=False, isnoise=False)
    normed_obs = my_helper.change_normed_obs(raw_obs)
    desired_leg_state = MyHelper.change_foot_contact_range(controller.get_desired_leg_state()) # get desired leg state for rl obs
    rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state) # get rl obs
    ppo.add_mpc_data(mpc_obs = mpc_obs[mpc_env_start_num:mpc_env_start_num+5,:,:], \
                    mpc_act = mpc_act[mpc_env_start_num:mpc_env_start_num+5,:,:])
    ppo.update(actor_obs=rl_obs, value_obs=rl_obs, log_this_iteration=update % 5 == 0, 
            update=update,w_im = cfg['learner']['w_im'], w_rl = cfg['learner']['w_rl'])
    average_ll_performance = reward_ll_sum / total_steps
    average_dones = done_sum / num_env
    if update%5==0:
        wandb.log({"reward(training)":average_ll_performance,"dones(training)":average_dones,"update":update})
    # if cfg['record_video'] == 'yes':
    #     wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name,\
    #             caption=str(update), fps=4, format="mp4")}) 
    ###############################################################################################################
    actor.update()
    actor.distribution.enforce_minimum_std((torch.ones(rl_act_dim)*min_std).to(device))

    # curriculum update. Implement it in Environment.hpp
    env.curriculum_callback()

    end = time.time()

    print('----------------------------------------------------')
    print('{:>6}th iteration'.format(update))
    print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(average_ll_performance)))
    print('{:<40} {:>6}'.format("dones: ", '{:0.6f}'.format(average_dones)))
    print('{:<40} {:>6}'.format("time elapsed in this iteration: ", '{:6.4f}'.format(end - start)))
    print('{:<40} {:>6}'.format("fps: ", '{:6.0f}'.format(total_steps / (end - start))))
    print('{:<40} {:>6}'.format("real time factor: ", '{:6.0f}'.format(total_steps / (end - start)
                                                                       * cfg['environment']['control_dt'])))
    print('----------------------------------------------------\n')
###################################################################