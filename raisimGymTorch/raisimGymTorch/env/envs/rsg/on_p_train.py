import concurrent.futures
import threading

from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher, RaisimLogger
from raisimGymTorch.env.bin.rsg import NormalSampler
from raisimGymTorch.env.bin.rsg import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
from raisimGymTorch.deploy_utils.angle_utils import get_last_position
from raisimGymTorch.deploy_utils.runner_util import run_model_with_pt_input_modify, list_pt, step_reset
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
from raisimGymTorch.deploy_log.csv_saver import CSV_saver

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
print(env.num_envs)
actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim),
                         ppo_module.MultivariateGaussianDiagonalCovariance(act_dim,
                                                                           env.num_envs,
                                                                           1,
                                                                           NormalSampler(act_dim),
                                                                           cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, ob_dim, 1),
                           device)

saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])
logger = RaisimLogger(saver.data_dir+"/train.log")

def updating():
    obs = env.observe(False)
    ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 2 == 0, update=update)
    average_ll_performance = reward_sum / total_steps


    if update % cfg['environment']['eval_every_n'] == 0:
        biggest_iter = update
        biggest_reward = average_ll_performance
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'optimizer_state_dict': ppo.optimizer.state_dict(),
        }, saver.data_dir + "/full_" + str(update) + '.pt')
        env.save_scaling(saver.data_dir, str(update))
        save_act.save()
        save_observe.save()
    avg_rewards.append(average_ll_performance)

    actor.update()
    actor.distribution.enforce_minimum_std((torch.ones(12)).to(device))

    print('----------------------------------------------------')
    print('{:>6}th iteration'.format(update))
    print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(average_ll_performance)))
    print('----------------------------------------------------\n')



if mode =='train' or mode == 'retrain':
    tensorboard_launcher(saver.data_dir+"/..")  # press refresh (F5) after the first ppo update
num_envs = cfg['environment']['num_envs']
ppo = PPO.PPO(actor=actor,
              critic=critic,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps *5,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              num_mini_batches=16,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False,
              )

reward_analyzer = RewardAnalyzer(env, ppo.writer)
biggest_reward = 0
biggest_iter = 0
if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

save_observe = CSV_saver('./observation_saver')
save_act = CSV_saver('./act_saver')
print = logger.info


on_p_rate =cfg['on_policy']['rate']
on_p_kb = cfg['on_policy']['kb']
mode == 'train'
his_util = [0] * 12
history_act = np.array([his_util] * num_envs)
total_update = args.update
if mode =='train' or mode == 'retrain':
    env.turn_on_visualization()
    schedule = cfg['environment']['schedule']
    envs_idx = 0
    waiter = Waiter(0.01)
    waiter.update_start()
    for update in range(total_update):
        reward_sum = 0

        for step in range(n_steps):
            # if step>=200:
            #     time.sleep(5)
            waiter.wait()
            obs = env.observe(False)
            # print(obs)
            action = ppo.act(obs)
            # action = np.zeros_like(action)
            action, history_act = run_model_with_pt_input_modify(action, envs_idx, schedule, history_act, kb=on_p_kb, rate=on_p_rate)
            reward, _ = env.step(action)

            envs_idx +=1
            ppo.step(value_obs=obs, rews=reward, dones=_)

            save_observe.add_list(obs[0])
            save_act.add_list(action[0])

            reward_sum = reward_sum + np.sum(reward)
            reward_analyzer.add_reward_info(env.get_reward_info())
            reward_analyzer.analyze_and_plot(update)
        # take st step to get value obs
        update_thread = threading.Thread(target=updating)
        update_thread.start()

        cnt = 0
        for i in range(4 * schedule):
            # print('running optimize position ')
            waiter.wait()
            acc, history_act =step_reset(action, cnt, schedule, history_act, kb=on_p_kb, rate=on_p_rate)
            env.step(acc)
            cnt +=1
        # for i in range(2 * schedule):
        #     # print('running optimize position ')
        #     waiter.wait()
        #     # acc, history_act =run_model_with_pt_input_modify(action, cnt, schedule, history_act, kb=on_p_kb, rate=on_p_rate)
        #     env.step(acc)
        #     cnt +=1
        while update_thread.is_alive():
            waiter.wait()
            # print('threading running')
            env.step(acc)
        history_act=np.array([his_util] * num_envs)
        print('updating finished')
    env.turn_off_visualization()
print(f'biggest:{biggest_reward},rate = {biggest_iter}')


