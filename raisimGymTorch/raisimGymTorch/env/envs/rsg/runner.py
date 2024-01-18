from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher, RaisimLogger
from raisimGymTorch.env.bin.rsg import NormalSampler
from raisimGymTorch.env.bin.rsg import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
from raisimGymTorch.deploy_utils.angle_utils import get_last_position
from raisimGymTorch.deploy_utils.runner_util import run_model_with_pt_input_modify, list_pt
import os
import math
import time
import torch.nn as nn
import numpy as np
import torch
import datetime
import argparse

from unitree_utils.Waiter import Waiter
from raisimGymTorch.deploy_log.draw_map import Drawer



# task specification
task_name = "rsg_test"

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
parser.add_argument('-u', '--update', help='update times', type=int, default=300)
parser.add_argument('-p', '--cfg_path', help='where to find the path', type=str, default=None)
# parser.add_argument('-b', '--load_best', help='load best file in last train', type=bool, default=False)
args = parser.parse_args()
mode = args.mode
weight_path = args.weight
cfg_path = args.cfg_path
# load_best = args.load_best
# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# device = torch.device('cpu')
print(device)
# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
if cfg_path is not None:
    cfg = YAML().load(open(cfg_path, 'r'))
else:
    cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))
print(cfg['environment'])

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))
env.seed(cfg['seed'])

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts
num_threads = cfg['environment']['num_threads']

# Training
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
logger = RaisimLogger(saver.data_dir+"/train.log")

if mode =='train' or mode == 'retrain':
    tensorboard_launcher(saver.data_dir+"/..")  # press refresh (F5) after the first ppo update
num_envs = cfg['environment']['num_envs']
ppo = PPO.PPO(actor=actor,
              critic=critic,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps * 5,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              num_mini_batches=16,
              # learning_rate_schedule='',
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False,
              # learning_rate=cfg['environment']['learnning_rate']
              )

reward_analyzer = RewardAnalyzer(env, ppo.writer)
biggest_reward = 0
biggest_iter = 0
if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

# his_util=[0, 0.523, -1.046] * 4
his_util=[0] * 12
check_done = lambda a, b: a + 1 if not b else 0
check_history = lambda a, b: a if not b else his_util


print = logger.info
on_p_rate =cfg['on_policy']['rate']
on_p_kb = cfg['on_policy']['kb']
env_idx = np.zeros(num_envs)
mode == 'train'
history_act = np.array([his_util] * num_envs)
history_act_0 = np.array([his_util] * num_envs)
total_update = args.update
if mode =='train' or mode == 'retrain':
    schedule = cfg['environment']['schedule']
    for update in range(total_update):
        start = time.time()
        env.reset()
        reward_sum = 0
        envs_idx = env_idx
        history_act=history_act_0
        done_sum = 0
        average_dones = 0.

        if update % cfg['environment']['eval_every_n'] == 0:
            # draw_his = Drawer('history')
            waiter = Waiter(0.01)
            print("Visualizing and evaluating the current policy")
            torch.save({
                'actor_architecture_state_dict': actor.architecture.state_dict(),
                'actor_distribution_state_dict': actor.distribution.state_dict(),
                'critic_architecture_state_dict': critic.architecture.state_dict(),
                'optimizer_state_dict': ppo.optimizer.state_dict(),
            }, saver.data_dir+"/full_"+str(update)+'.pt')

            env.turn_on_visualization()
            env.start_video_recording(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4')
            envs_idx = env_idx
            waiter.update_start()
            for step in range(n_steps):
                with torch.no_grad():
                    waiter.wait()
                    frame_start = time.time()


                    obs = env.observe(False)
                    action = ppo.act(obs)
                    action, history_act = run_model_with_pt_input_modify(action, envs_idx, schedule, history_act, kb=on_p_kb, rate=on_p_rate)
                    reward, dones = env.step(action)


                    history_act = np.array([check_history(history_act[i], dones[i]) for i in range(num_envs)])
                    envs_idx     = np.where(dones  ==1, 0, envs_idx+1)
                    reward_analyzer.add_reward_info(env.get_reward_info())
                    frame_end = time.time()
                    wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)

            env.stop_video_recording()
            env.turn_off_visualization()
            # history_act = np.array([his_util] * num_envs)
            reward_analyzer.analyze_and_plot(update)
            env.reset()
            env.save_scaling(saver.data_dir, str(update))
        envs_idx = env_idx
        history_act=history_act_0
        for step in range(n_steps):
            obs = env.observe(False)
            action = ppo.act(obs)
            action, history_act = run_model_with_pt_input_modify(action, envs_idx, schedule, history_act, kb=on_p_kb, rate=on_p_rate)
            reward, dones = env.step(action)
            history_act = np.array([check_history(history_act[i], dones[i]) for i in range(num_envs)])
            envs_idx = np.where(dones == 1, 0, envs_idx + 1)
            ppo.step(value_obs=obs, rews=reward, dones=dones)



            done_sum = done_sum + np.sum(dones)
            reward_sum = reward_sum + np.sum(reward)
            reward_analyzer.add_reward_info(env.get_reward_info())
            reward_analyzer.analyze_and_plot(update)
        # take st step to get value obs
        obs = env.observe(False)
        ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 2 == 0, update=update)
        average_ll_performance = reward_sum / total_steps


        if average_ll_performance > biggest_reward:
            biggest_iter = update
            biggest_reward = average_ll_performance
            torch.save({
                'actor_architecture_state_dict': actor.architecture.state_dict(),
                'actor_distribution_state_dict': actor.distribution.state_dict(),
                'critic_architecture_state_dict': critic.architecture.state_dict(),
                'optimizer_state_dict': ppo.optimizer.state_dict(),
            }, saver.data_dir+"/full_"+str(update)+'.pt')
            env.save_scaling(saver.data_dir, str(update))
        average_dones = done_sum / total_steps
        avg_rewards.append(average_ll_performance)

        actor.update()
        actor.distribution.enforce_minimum_std((torch.ones(12)).to(device))

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


print(f'biggest:{biggest_reward},rate = {biggest_iter}')


