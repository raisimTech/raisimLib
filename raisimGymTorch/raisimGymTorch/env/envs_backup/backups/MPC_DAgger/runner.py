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
import random
from raisimGymTorch.mpc import control
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
control_obs_dim = env.num_obs
control_act_dim = env.num_acts
rl_obs_dim = env.num_obs
rl_act_dim = 12 # leg times 3 dim
num_threads = cfg['environment']['num_threads']

# Training
## n_steps: horizon size
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs

avg_rewards = [] # average rewards over all the updates

actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, rl_obs_dim, rl_act_dim),
                         ppo_module.MultivariateGaussianDiagonalCovariance(rl_act_dim,
                                                                           env.num_envs,
                                                                           1.0,
                                                                           NormalSampler(rl_act_dim),
                                                                           cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, rl_obs_dim, 1),
                           device)

saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])


if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

# update is number of epochs
for update in range(2601):
    start = time.time()
    env.reset()
    
    # containers for one update
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.    
    mean_reward_per_step = {'reward_sum': []}
    mean_reward_one_update = {'reward_sum': 0.0}
    for key in cfg['environment']['reward']:
        mean_reward_per_step[key] = []
        mean_reward_one_update[key] = 0.0
    mean_vel_per_step = {"total": [], "forward": [], "yawing": []}
    beta = 1.0

    # Initialize controller for one update
    controller = []
    control_init_obs = env.observe(update_statistics= False,israw= True)
    for i in range(cfg['environment']['num_envs']):
        joint_pos_init_obs = control_init_obs[i,8:20]
        controller.append(control._setup_controller())
        controller[i].reset(0.0,joint_pos_init_obs)
    ctr_time_step = cfg['environment']['control_dt']

    # save model and record video
    if update % cfg['environment']['eval_every_n'] == 0:
        print("Visualizing and evaluating the current policy")
        # save trained model
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'optimizer_state_dict': ppo.optimizer.state_dict(),
        }, saver.data_dir+"/full_"+str(update)+'.pt')

        # if cfg['environment']['render']:
        #     # we create another graph just to demonstrate the save/load method
        #     loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, rl_obs_dim, rl_act_dim)
        #     loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])

        #     env.turn_on_visualization()
        #     if cfg['record_video'] == 'yes':
        #         vid_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
        #         env.start_video_recording(vid_name)

        #     #simulate
        #     # TODO: 
        #     # write code to save the frame so that the video can be reproduced in real time
        #     # write simulation code
        #     control_act = np.zeros((cfg['environment']['num_envs'],control_act_dim))
        #     for step in range(n_steps*2):
        #         with torch.no_grad():
        #             frame_start = time.time()
        #             # observe
        #             obs = env.observe(update_statistics= False, israw= False)
        #             control_obs = env.observe(update_statistics= False, israw= True)
        #             #update controller by its time
        #             control._update_controller_params(controller[0],np.array([0.6,0.0,0.0]),0.0)
        #             controller[0].update(ctr_time_step*step,control_obs[0])
        #             # action
        #             contact_force = loaded_graph.architecture(torch.from_numpy(obs).cpu())
        #             contact_force = contact_force[0].reshape((4,3))
        #             control_act_tmp = controller[0].get_control_action(contact_force,control_obs[0])
        #             pos_action = control_act_tmp[control.POSITION_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
        #             tq_action = control_act_tmp[control.TORQUE_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
        #             kp_action = control_act_tmp[control.POSITION_GAIN_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
        #             kd_action = control_act_tmp[control.VELOCITY_GAIN_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
        #             control_act[0] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
        #             control_act = control_act.astype(np.float32)
        #             reward_ll, dones = env.step(control_act.cpu().detach().numpy())
        #             frame_end = time.time()
        #             wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
        #             if wait_time > 0.:
        #                 time.sleep(wait_time)

        #     if cfg['record_video'] == 'yes':
        #         env.stop_video_recording()

        #     env.turn_off_visualization()
        #     env.reset()

        env.save_scaling(saver.data_dir, str(update))
    
    # Data generation for one step
    for step in range(n_steps):
        # observe for all envs
        control_obs = env.observe(update_statistics = False, israw= True)
        rl_obs = env.observe(update_statistics = True, israw=False)
        control_act = np.zeros((cfg['environment']['num_envs'],control_act_dim))
        rl_act = ppo.act(rl_obs)

        #control each envs
        for i in range(control_obs.shape[0]):
            # control update with observation and time
            control._update_controller_params(controller[i],np.array([0.6,0.0,0.0]),0.0)
            controller[i].update(ctr_time_step*step,control_obs[i])
            # action
            mpc_act = controller[i].get_contact_force(control_obs[i]) # 4 contact forces
            contact_force = beta * mpc_act + (1 - beta) * rl_act[i].reshape((4,3))
            control_act_tmp = controller[i].get_control_action(contact_force,control_obs[i])
            pos_action = control_act_tmp[control.POSITION_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            tq_action = control_act_tmp[control.TORQUE_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            kp_action = control_act_tmp[control.POSITION_GAIN_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            kd_action = control_act_tmp[control.VELOCITY_GAIN_INDEX::control.MOTOR_COMMAND_DIMENSION].copy()
            control_act[i] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
        # control and gather data
        control_act = control_act.astype(np.float32)
        reward, dones = env.step(control_act)
        ppo.step(value_obs=rl_obs, rews=reward, dones=dones)
        # record performance
        done_sum = done_sum + np.sum(dones)
        reward_ll_sum = reward_ll_sum + np.sum(reward)
        tmp_rewardinfo = env.getRewardinfo()
        rewardinfo_map = {'reward_sum': []}
        for key in cfg['environment']['reward']:
            rewardinfo_map[key] = []
        for i in range(len(tmp_rewardinfo)):
            for key in tmp_rewardinfo[i]:
                rewardinfo_map[key].append(tmp_rewardinfo[i][key])
        for key in rewardinfo_map:
            mean_reward_one_update[key] += np.sum(np.array(rewardinfo_map[key])) / total_steps
            mean_reward_per_step[key].append(np.mean(np.array(rewardinfo_map[key])))
        # record velocity
        obs_raw = env.observe(update_statistics= False,israw= True)
        mean_vel_per_step["forward"].append(np.mean(obs_raw[:,20]))
        mean_vel_per_step["total"].append(np.mean(np.linalg.norm(obs_raw[:,20:23],axis=1)))
        mean_vel_per_step["yawing"].append(np.mean(np.abs(obs_raw[:,25])))

        # dealing in the case of falling
        for i in range(dones.shape[0]):
            if dones[i]:
                control_reset_obs = env.observe(update_statistics=False,israw=True)
                joint_pos_reset_obs = control_reset_obs[i,8:20]
                controller[i].reset(ctr_time_step*(step+1),joint_pos_reset_obs)

    