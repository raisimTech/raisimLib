from email.policy import default
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin.a1_default import RaisimGymEnv
from raisimGymTorch.env.bin.a1_default import NormalSampler
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
###################################################################

# task specification
task_name = "a1_default"

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


################################ my wandb initialization ############################################
wandb.init(project="0912",group="MPC_guide",job_type="train",name="a1_default_motion_0.6",
            notes="A1 motion without any imitation. Target velocity is set to 0.6m/s.\
             forward velocity coefficient goes up to 2.0 and 1.5 times torque coeff to make a good feedback"
            ,config=cfg,save_code=True, mode = 'online')
######################################################################################

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
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/a1_default/Environment.hpp")
wandb.save("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/a1_default/cfg.yaml")
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
              )


if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)
for update in range(5000):
    start = time.time()
    env.reset()
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.
    
    # # save model and record video
    if update % cfg['environment']['eval_every_n'] == 0:
        print("Visualizing and evaluating the current policy")
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'optimizer_state_dict': ppo.optimizer.state_dict(),
        }, saver.data_dir+"/full_"+str(update)+'.pt')
        wandb.save(saver.data_dir+"/full_"+str(update)+'.pt')

        # we create another graph just to demonstrate the save/load method
        loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim)
        loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])

        env.turn_on_visualization()
        vid_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
        env.start_video_recording(vid_name)
        
        for step in range(n_steps*2):
            with torch.no_grad():
                frame_start = time.time()
                obs = env.observe(False)
                action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())
                reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
                frame_end = time.time()
                wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
                if wait_time > 0.:
                    time.sleep(wait_time)

        env.stop_video_recording()
        env.turn_off_visualization()
        env.reset()
        env.save_scaling(saver.data_dir, str(update))
        

    mean_reward_per_step = {'reward_sum': []}
    mean_reward_one_update = {'reward_sum': 0.0}
    mean_vel_per_step = {"forward": [], "total": [], "yawing": []}
    mean_torque_per_step = {"torque":[]}

    for key in cfg['environment']['reward']:
        mean_reward_one_update[key] = 0.0
        mean_reward_per_step[key] = []
    
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
            if key == "torque":
                mean_torque_per_step[key].append(np.mean(np.array(rewardinfo_map[key]))/\
                    cfg['environment']['reward']['torque']['coeff'])
            
    # ##################################################################################
    # take st step to get value obs
    obs = env.observe()
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
        for key in mean_torque_per_step:
            step_data[key] = mean_torque_per_step[key]
        for i in range(n_steps):
            step_data["step"].append((update//cfg['environment']['eval_every_n']*n_steps+i))
        # log stepwise data
        for i in range(n_steps):
            wandb.log({key: step_data[key][i] for key in step_data})
        # log video
        wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name,\
                caption=str(update), fps=4, format="mp4")})      
    #Record average reward for certain update.
    if update%10 == 0:
        # define update_wise data
        update_data = {"update": update}
        for key in mean_reward_one_update:
            update_data[key+"(reward per update)"] = mean_reward_one_update[key]
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