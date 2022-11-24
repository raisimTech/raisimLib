from email.policy import default
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin.MPC_guide_buffer_mode import RaisimGymEnv
from raisimGymTorch.env.bin.MPC_guide_buffer_mode import NormalSampler
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
###################################################################

# choose experiments mode
MPC_imitation = True

# task specification
task_name = "MPC_guide"

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
wandb.init(project="MPC_guide",group="0926",job_type="train",name="train_with_synchronized",
            notes="Imitation learning with synchronized MPC freqeuncy data. Also, action noramlization has been changed to normalize torque data"
            ,config=cfg,save_code=True, mode = 'disabled')

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts
num_threads = cfg['environment']['num_threads']

# Training
## n_steps: horizon size
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs

avg_rewards = []

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

###########################3 my save environment.hpp ###################################
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/MPC_guide_buffer_mode/Environment.hpp")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/MPC_guide_buffer_mode/cfg.yaml")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/algo/ppo/ppo.py")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/algo/ppo/module.py")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/algo/ppo/storage.py")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/Common.hpp")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/RaisimGymEnv.hpp")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/RaisimGymVecEnv.py")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/Reward.hpp")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/VectorizedEnvironment.hpp")
#########################################################################################


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
              MPC = MPC_imitation,
              )   


# 1. in for loop, get data
    # 2. process action data(action normalization)
    # 3. stack data(stack normalized action)
# 4. process obs data(obs normalization)  -> or should it be normalized by mean, var csv data?
if MPC_imitation:
    traj_size = 3000
    num_traj = 200
    whole_mpc_obs = [] # whole observation tarjectory. should be (# of traj, size of traj, obs_dim)
    whole_mpc_acts = [] # whole action tarjectory (# of traj, size of traj, act_dim+12(kps))
    for i in range(num_traj):
        # get data
        acts_name = "/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/mpc/mpc_data_traj_frequencyMatch/action_traj" \
                            + str(i)+".csv"
        obs_name = "/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/mpc/mpc_data_traj_frequencyMatch/observation_traj"\
                            + str(i)+".csv"
        act_mpc = np.loadtxt(acts_name,dtype=np.float32)
        obs_mpc = np.loadtxt(obs_name,dtype=np.float32)
        # normalize action
        mpc_kp = act_mpc[:,24:].copy()
        mpc_kp_mask = (mpc_kp==0) 

        initial_pos = np.array([0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8])
        initial_pos = np.tile(initial_pos,(traj_size,1))
        initial_pos[mpc_kp_mask] = 0
        
        act_pos = act_mpc[:,:12].copy()
        act_pos[mpc_kp_mask] = 0
        act_pos = (act_pos - initial_pos)/0.3

        act_torque = act_mpc[:,12:24].copy()
        act_torque = act_torque/5

        act_mpc_norm = np.hstack((act_pos,act_torque)) # (size of traj, action_dimension)
        assert act_mpc_norm.shape[0] == traj_size, "Wrong traj size!"

        # stack data
        whole_mpc_obs.append(obs_mpc)
        whole_mpc_acts.append(act_mpc_norm)
    whole_mpc_obs = np.array(whole_mpc_obs)
    whole_mpc_acts = np.array(whole_mpc_acts)

    # normalize observation
    tmp_whole_obs = whole_mpc_obs.reshape(-1,whole_mpc_obs.shape[-1]).copy()
    obs_mpc_mean = np.mean(tmp_whole_obs,axis = 0)
    obs_mpc_var = np.mean((tmp_whole_obs - obs_mpc_mean)**2,axis = 0)
    obs_cnt = float(traj_size*num_traj)
    env.setObStatistics(obs_mpc_mean, obs_mpc_var,obs_cnt)

if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

### my MPC trajectory sampler ###
if MPC_imitation:
    traj_batch_size = random.randint(1,10)
    mpc_traj_indices =list(BatchSampler(SubsetRandomSampler(range(num_traj)), traj_batch_size, drop_last=False))
    traj_batch_idx = 0

# update is number of epochs
for update in range(2601):
    start = time.time()
    env.reset()
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.
    rewardinfo = {}
    velocities = {"forward": 0.0, "total": 0.0, "yawing": 0.0}

    if MPC_imitation:
        if traj_batch_idx == len(mpc_traj_indices):
            traj_batch_size = random.randint(1,10)
            mpc_traj_indices = list(BatchSampler(SubsetRandomSampler(range(num_traj)), traj_batch_size, drop_last=False))
            traj_batch_idx = 0
        sampled_traj = mpc_traj_indices[traj_batch_idx]
        mpc_obs = []
        mpc_act = []
        for i in range(len(sampled_traj)):
            mpc_obs.extend(whole_mpc_obs[sampled_traj[i]])
            mpc_act.extend(whole_mpc_acts[sampled_traj[i]])
        traj_batch_idx += 1
        mpc_obs = np.array(mpc_obs)
        mpc_act = np.array(mpc_act)

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
        wandb.save(saver.data_dir+"/full_"+str(update)+'.pt')

        # # we create another graph just to demonstrate the save/load method
        # loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim)
        # loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])

        # env.turn_on_visualization()

        # # record video
        # vid_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
        # env.start_video_recording(vid_name)

        # #simulate
        # for step in range(n_steps*2):
        #     with torch.no_grad():
        #         frame_start = time.time()
        #         obs = env.observe(False,False)
        #         action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())
        #         reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
        #         frame_end = time.time()
        #         wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
        #         if wait_time > 0.:
        #             time.sleep(wait_time)

        # env.stop_video_recording()
        # env.turn_off_visualization()
        # env.reset()
        env.save_scaling(saver.data_dir, str(update))
        _, _, env.count = env.getObStatistics()
        wandb.log({"num_obs":env.count}) #### debugging to see if it only increases(to see if normalization includes all episodes)
    
    mean_reward_per_step = {'reward_sum': []}
    mean_reward_one_update = {'reward_sum': 0.0}
    mean_vel_per_step = {"forward": [], "total": [], "yawing": []}

    for key in cfg['environment']['reward']:
        mean_reward_one_update[key] = 0.0
        mean_reward_per_step[key] = []

    # actual training for one iter(one horizon)
    # all data are from current policy -> on-policy!
    for step in range(n_steps):
        obs = env.observe()
        action = ppo.act(obs)
        reward, dones = env.step(action)
        ppo.step(value_obs=obs, rews=reward, dones=dones)
        done_sum = done_sum + np.sum(dones)
        reward_ll_sum = reward_ll_sum + np.sum(reward)

        # ############################ my reward by reward and velocities ###############################

        # Record velocity for one step. Expectation over environments
        obs_raw = env.observe(False,True)
        mean_vel_per_step["forward"].append(np.mean(obs_raw[:,16]))

        mean_vel_per_step["total"].append(np.mean(np.linalg.norm(obs_raw[:,16:19],axis=1)))

        mean_vel_per_step["yawing"].append(abs(obs_raw[0][21]))

        
        # Record reward for one step. Will be expectation over environments
        tmp_rewardinfo = env.getRewardinfo()
        # make reward information in MAP. Just for my convinience
        rewardinfo_map = {'reward_sum': []}
        for key in cfg['environment']['reward']:
            rewardinfo_map[key] = []
        for i in range(len(tmp_rewardinfo)):
            for key in tmp_rewardinfo[i]:
                rewardinfo_map[key].append(tmp_rewardinfo[i][key])
        # Record reward
        for key in rewardinfo_map:
            mean_reward_one_update[key] += np.sum(np.array(rewardinfo_map[key])) / total_steps
            mean_reward_per_step[key].append(np.mean(np.array(rewardinfo_map[key])))           

    # ##################################################################################
    # take st step to get value obs(last value)
    obs = env.observe()
    ## my ##
    if MPC_imitation:
        mpc_obs_mean, mpc_obs_var, _ = env.getObStatistics()
        mpc_obs = (mpc_obs - mpc_obs_mean)/np.sqrt(mpc_obs_var+1e-8)
        ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 10 == 0, 
                update=update,mpc_obs = mpc_obs, mpc_acts = mpc_act)
    else:
        ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 10 == 0, update=update)
    average_ll_performance = reward_ll_sum / total_steps
    average_dones = done_sum / total_steps
    avg_rewards.append(average_ll_performance)
    ####my code####################################################################################################
    # Record step by step reward and velocity for certain update

    if update%cfg['environment']['eval_every_n'] == 0:
        # collect stepwise data
        step_data = {"step": []}
        for key in mean_reward_per_step:
            step_data[key+"(reward per step)"] = mean_reward_per_step[key]
        for key in mean_vel_per_step:
            step_data[key+"(velocity)"] = mean_vel_per_step[key]

        for i in range(n_steps):
            step_data["step"].append((update//cfg['environment']['eval_every_n']*n_steps+i))
        # log stepwise data
        for i in range(n_steps):
            wandb.log({key: step_data[key][i] for key in step_data})
        # log video
        # wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name,\
        #         caption=str(update), fps=4, format="mp4")}) 
         
    #Record average reward for certain update.
    if update%10 == 0:
        # define update_wise data
        update_data = {"update": update}
        for key in mean_reward_one_update:
            update_data[key+"(reward per update)"] = mean_reward_one_update[key]
        update_data["dones"] = average_dones
        # log update wise data
        wandb.log({key: update_data[key] for key in update_data})
    ###############################################################################################################
    actor.update()
    actor.distribution.enforce_minimum_std((torch.ones(act_dim)*0.2).to(device))

    # curriculum update. Implement it in Environment.hpp
    env.curriculum_callback()

    end = time.time()

    print('----------------------------------------------------')
    print('{:>6}th iteration'.format(update))
    print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(average_ll_performance)))
    # print('{:<40} {:>6}'.format("forward velocity: ", '{:0.10f}'.format(velocities['forward'] / total_steps)))
    # print('{:<40} {:>6}'.format("total velocity: ", '{:0.10f}'.format(velocities['total'] / total_steps)))
    # print('{:<40} {:>6}'.format("yawing velocity: ", '{:0.10f}'.format(velocities['yawing'] / total_steps)))
    print('{:<40} {:>6}'.format("dones: ", '{:0.6f}'.format(average_dones)))
    print('{:<40} {:>6}'.format("time elapsed in this iteration: ", '{:6.4f}'.format(end - start)))
    print('{:<40} {:>6}'.format("fps: ", '{:6.0f}'.format(total_steps / (end - start))))
    print('{:<40} {:>6}'.format("real time factor: ", '{:6.0f}'.format(total_steps / (end - start)
                                                                       * cfg['environment']['control_dt'])))
    print('----------------------------------------------------\n')
###################################################################