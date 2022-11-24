from email.policy import default
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin.MPC_BC_perturb import RaisimGymEnv
from raisimGymTorch.env.bin.MPC_BC_perturb import NormalSampler
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param
import os
import math
import time
import raisimGymTorch.algo.ppo_BC_perturb.module as ppo_module
import raisimGymTorch.algo.ppo_BC_perturb.ppo_BC_perturb as PPO_BC_perturb
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
from raisimGymTorch.mpc_BC_perturb import control

###################################################################


# task specification
task_name = "MPC_BC_perturb"
algo_name = "ppo_BC_perturb"
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
algo_path = home_path+"/raisimGymTorch/raisimGymTorch/algo/"+algo_name
env_path = home_path+"/raisimGymTorch/raisimGymTorch/env/"
# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# wandb initialization
wandb.init(project="MPC_guide",group="1017",job_type="train",name="with BC perturbation",
            notes="with BC loss but with perturbation"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))

# shortcuts(constants)
control_obs_dim = env.num_obs
control_act_dim = env.num_acts
rl_obs_dim = env.num_obs
rl_act_dim = 12 # leg times 3 dim
num_threads = cfg['environment']['num_threads']
time_step = cfg['environment']['simulation_dt']
ctr_time_step = cfg['environment']['control_dt']
num_env = cfg['environment']['num_envs']
video_header = "BC"
F_x_mean = -2.5
F_z_mean_R = -40
F_z_mean_F = -20
F_y_mean = 0
F_std = 50

# Training
## n_steps: horizon size
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs

avg_rewards = []

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


###########################3 my save environment.hpp ###################################
wandb.save(task_path+"/*")
wandb.save(task_path+"/MPC_controller/*")
wandb.save(algo_path+"/*")
wandb.save(env_path+"/*")
#########################################################################################


ppo = PPO_BC_perturb.PPO(actor=actor,
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

## data generation and store

# Initialize data generation controller
env.reset()
control_obs = env.observe(False,True)
controller = control.Controller(control_obs,ctr_time_step,control_act_dim,num_env)
mpc_data = {"state":[], "action":[]}

# data generation using MPC

if cfg['environment']['render']:
    env.turn_on_visualization()
    if cfg['record_video'] == 'yes':
        env.start_video_recording(video_header+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "mpc_data"+'.mp4')

sigma = 0.025
for step in range(2*n_steps):
    jointPos_noise = np.random.normal(0.0,5*sigma,(num_env,12))
    vel_noise = np.random.normal(0.0,10*sigma,(num_env,6))
    noise = np.append(np.append(np.zeros((num_env,8)),jointPos_noise,axis = 1),vel_noise,axis = 1)

    control_obs = env.observe(update_statistics = False, israw= True) # (num_env,obs_dim)
    env.observe(update_statistics = True, israw= False) # Just to update statistics
    perturbed_ctrl_obs = (control_obs+noise).astype(np.float32)

    mpc_act,control_act = controller.control(control_obs,step) # mpc_act: (num_env,4,3)
    perturbed_mpc_act,_ = controller.control(perturbed_ctrl_obs,step) # mpc_act: (num_env,4,3)

    control_act = control_act.astype(np.float32)

    mpc_data['state'].append(control_obs)
    mpc_data['state'].append(perturbed_ctrl_obs)
    
    mpc_act[:,:,0]-=F_x_mean
    mpc_act[:,:,1]-=F_y_mean
    mpc_act[:,:2,2]-=F_z_mean_F
    mpc_act[:,2:4,2]-=F_z_mean_R
    mpc_act = mpc_act/F_std

    perturbed_mpc_act[:,:,0]-=F_x_mean
    perturbed_mpc_act[:,:,1]-=F_y_mean
    perturbed_mpc_act[:,:2,2]-=F_z_mean_F
    perturbed_mpc_act[:,2:4,2]-=F_z_mean_R
    perturbed_mpc_act = perturbed_mpc_act/F_std

    mpc_data['action'].append(mpc_act)
    mpc_data['action'].append(perturbed_mpc_act)

    reward, dones = env.step(control_act)
    for i in range(num_env):
        if dones[i]:
            control_obs = env.observe(update_statistics=False,israw=True)
            joint_pos_init_obs = control_obs[i,8:20]
            controller.reset(ctr_time_step*(step+1),joint_pos_init_obs,i)
mpc_data['state'] = np.array(mpc_data['state']).reshape(-1,control_obs_dim)
mpc_data['action'] = np.array(mpc_data['action']).reshape(-1,rl_act_dim)
for i in range(2*n_steps):
    wandb.log({ "mpc_forceFRx":mpc_data['action'][i*num_env,0],\
                "mpc_forceFRy":mpc_data['action'][i*num_env,1],\
                "mpc_forceFRz":mpc_data['action'][i*num_env,2],\
                "mpc_forceFLx":mpc_data['action'][i*num_env,3],\
                "mpc_forceFLy":mpc_data['action'][i*num_env,4],\
                "mpc_forceFLz":mpc_data['action'][i*num_env,5],\
                "mpc_forceRRx":mpc_data['action'][i*num_env,6],\
                "mpc_forceRRy":mpc_data['action'][i*num_env,7],\
                "mpc_forceRRz":mpc_data['action'][i*num_env,8],\
                "mpc_forceRLx":mpc_data['action'][i*num_env,9],\
                "mpc_forceRLy":mpc_data['action'][i*num_env,10],\
                "mpc_forceRLz":mpc_data['action'][i*num_env,11],\
                "mpc_size": i})

if cfg['environment']['render']:
    if cfg['record_video'] == 'yes':
        env.stop_video_recording()
    env.turn_off_visualization()
env.reset()

if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

## train and evaluate policy
for update in range(cfg['environment']['updates']):
    # define data container
    start = time.time()
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.
    rewardinfo = {}
    mean_reward_per_step = {'reward_sum': []}
    mean_reward_one_update = {'reward_sum': 0.0}
    mean_vel_per_step = {"forward": [], "total": []}
    for key in cfg['environment']['reward']:
        mean_reward_one_update[key] = 0.0
        mean_reward_per_step[key] = []
    mean_rl_act = {"FRx":[],"FRy":[],"FRz":[],"FLx":[],"FLy":[],"FLz":[],\
                    "RRx":[],"RRy":[],"RRz":[],"RLx":[],"RLy":[],"RLz":[]}
    # Initialize controller
    env.reset()
    control_obs = env.observe(False,True)
    controller = control.Controller(control_obs,ctr_time_step,control_act_dim,num_env)

    # save model and record video
    if update % cfg['environment']['eval_every_n'] == 0:
        print("Visualizing and evaluating the current policy")   
        # save trained model
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'optienv.turn_on_visualization()mizer_state_dict': ppo.optimizer.state_dict(),
        }, saver.data_dir+"/full_"+str(update)+'.pt')
        wandb.save(saver.data_dir+"/full_"+str(update)+'.pt')
        
        if cfg['environment']['render']:
            env.turn_on_visualization()
            # record video
            if cfg['record_video'] == 'yes':
                vid_name = video_header+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
                env.start_video_recording(vid_name)
            # we create another graph just to demonstrate the save/load method
            loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, rl_obs_dim, rl_act_dim)
            loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])
            #simulate
            for step in range(n_steps*2):
                with torch.no_grad():
                    frame_start = time.time()
                    rl_obs = env.observe(False,False)
                    control_obs = env.observe(False,True)
                    rl_act = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
                    rl_act = rl_act.reshape((num_env,4,3))*F_std
                    rl_act[:,:,0]+=F_x_mean
                    rl_act[:,:,1]+=F_y_mean
                    rl_act[:,:2,2]+=F_z_mean_F
                    rl_act[:,2:4,2]+=F_z_mean_R

                    simulate_control_act = controller.control_with_action(rl_act,control_obs,step)
                    simulate_control_act = simulate_control_act.astype(np.float32)
                    reward, dones = env.step(simulate_control_act)
                    for i in range(num_env):
                        if dones[i]:
                            control_obs = env.observe(update_statistics=False,israw=True)
                            joint_pos_init_obs = control_obs[i,8:20]
                            controller.reset(ctr_time_step*(step+1),joint_pos_init_obs,i)
                    frame_end = time.time()
                    wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
                    if wait_time > 0.:
                        time.sleep(wait_time)
            if cfg['record_video'] == 'yes':
                env.stop_video_recording()
            env.turn_off_visualization()
            env.reset()
        env.save_scaling(saver.data_dir, str(update))

        # Initialize controller
        env.reset()
        control_obs = env.observe(False,True)
        controller = control.Controller(control_obs,ctr_time_step,control_act_dim,num_env)

    # collect trajectories
    for step in range(n_steps):
        rl_obs = env.observe(update_statistics=True, israw=False)
        control_obs = env.observe(update_statistics=False, israw=True)
        rl_act = ppo.act(rl_obs)
        tmpi=0
        for key in mean_rl_act:
            mean_rl_act[key].append(np.mean(rl_act[:,tmpi]))
            tmpi+=1
        rl_act = rl_act.reshape((num_env,4,3))*F_std
        rl_act[:,:,0]+=F_x_mean
        rl_act[:,:,1]+=F_y_mean
        rl_act[:,:2,2]+=F_z_mean_F
        rl_act[:,2:,2]+=F_z_mean_R
        simulate_control_act = controller.control_with_action(rl_act,control_obs,step)
        simulate_control_act = simulate_control_act.astype(np.float32)
        reward, dones = env.step(simulate_control_act)
        for i in range(num_env):
            if dones[i]:
                control_obs = env.observe(update_statistics=False,israw=True)
                joint_pos_init_obs = control_obs[i,8:20]
                controller.reset(ctr_time_step*(step+1),joint_pos_init_obs,i)

        ppo.step(value_obs=rl_obs, rews=reward, dones=dones)
        done_sum = done_sum + np.sum(dones)
        reward_ll_sum = reward_ll_sum + np.sum(reward)
        
        # collect performance info
        obs_raw = env.observe(False,True)
        mean_vel_per_step["forward"].append(np.mean(obs_raw[:,20]))
        mean_vel_per_step["total"].append(np.mean(np.linalg.norm(obs_raw[:,20:23],axis=1)))

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

    # take st step to get value obs(last value)
    obs = env.observe(update_statistics=True, israw=False)
    obs_mean, obs_var, _ = env.getObStatistics()
    mpc_obs = mpc_data["state"][(update%num_env)::num_env,:] #should have 2*n_steps,ctrl_obs_dim
    mpc_obs = (mpc_obs - obs_mean)/np.sqrt(obs_var+1e-8)
    mpc_act = mpc_data["action"] ## action data is normalized
    
    ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 10 == 0, 
            update=update,w_im_arg = 0.05, w_rl_arg = 1.0, mpc_obs = mpc_obs, mpc_acts = mpc_act)
    average_ll_performance = reward_ll_sum / total_steps
    average_dones = done_sum / total_steps
    avg_rewards.append(average_ll_performance)

    if update%cfg['environment']['eval_every_n'] == 0:
        # collect stepwise data
        step_data = {"step": []}
        for key in mean_reward_per_step:
            step_data[key+"(reward per step)"] = mean_reward_per_step[key]
        for key in mean_vel_per_step:
            step_data[key+"(velocity)"] = mean_vel_per_step[key]
        for key in mean_rl_act:
            step_data[key] = mean_rl_act[key]
        for i in range(n_steps):
            step_data["step"].append((update//cfg['environment']['eval_every_n']*n_steps+i))
        # log stepwise data
        for i in range(n_steps):
            wandb.log({key: step_data[key][i] for key in step_data})
        # log video
        if cfg['environment']['render']:
            if cfg['record_video'] == 'yes':
                wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name,\
                        caption=str(update), fps=4, format="mp4")}) 
         
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
    actor.distribution.enforce_minimum_std((torch.ones(rl_act_dim)*0.2).to(device))

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