import concurrent.futures
import threading

import asn1crypto.cms
import asn1crypto.core
from raisimGymTorch.env.bin.rsg import NormalSampler
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher, RaisimLogger
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.env.bin.rsg import RaisimGymEnv
from raisimGymTorch.deploy_utils.angle_utils import get_last_position
from raisimGymTorch.deploy_utils.runner_util import run_model_with_pt_input_modify, list_pt, u0_rad
import os
import math
import time
import torch.nn as nn
import numpy as np
import torch
import datetime
import argparse
import signal
from raisimGymTorch.deploy_log.draw_map import Drawer
from raisimGymTorch.deploy_log.csv_saver import CSV_saver
# task specification
# from fake_a1 import *
from robot_utils import *

task_name = "real_time_train_test"

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

# shortcuts


env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))
env.seed(cfg['seed'])

ob_dim = a1.ob_dim()
act_dim = a1.act_dim()

# Training
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * 1

avg_rewards = []
num_envs=1
actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim),
                           ppo_module.MultivariateGaussianDiagonalCovariance(act_dim,
                                                                           num_envs,
                                                                           1.0,
                                                                           NormalSampler(act_dim),
                                                                           cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, ob_dim, 1),
                           device)

saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])
logger = RaisimLogger(saver.data_dir+"/train.log")

def real_handler(signal, frame):
    print('You pressed Ctrl+C!')
    a1.quit_robot()
    # debug_draw_gry.draw()
    draw_reward.draw()
    save_reward.save()
    sys.exit(0)

signal.signal(signal.SIGINT, real_handler)

init_e_x = 0
init_e_y = 0
# init_v_x = 0
# init_v_y = 0
def cal_reward(robot):
    """

    :param robot: a1
    :return: reward number
    """
    global init_e_x
    global init_e_y
    # global init_v_x
    # global init_v_y
    obs = robot.observe()[0]
    reward = 0
    # reward -= (abs(obs[0] - init_e_x )+ abs(obs[1] - init_e_y))
    # reward -=  (abs(obs[2]) + abs(obs[3])) * 0.1
    # reward = reward * 0.4#stable
    # reward += 1
    # reward = reward + (0.5 - abs(obs[4] +   0.5)) * 2 # wheel
    reward = reward + obs[4]# wheel

    return  np.array([reward])

def updating():
    obs = a1.observe()
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
        save_act.save()
        save_observe.save()

        # draw_reward.draw()

    avg_rewards.append(average_ll_performance)
    draw_reward.add_map_list([average_ll_performance])
    save_reward.add_list([average_ll_performance])
    actor.update()
    actor.distribution.enforce_minimum_std((torch.ones(12)).to(device))

    print('----------------------------------------------------')
    print('{:>6}th iteration'.format(update))
    print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(average_ll_performance)))
    print('----------------------------------------------------\n')



num_envs = cfg['environment']['num_envs']
ppo = PPO.PPO(actor=actor,
              critic=critic,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              num_mini_batches=16,
              device=device,
              log_dir=saver.data_dir,
              )


biggest_reward = 0
biggest_iter = 0

save_observe = CSV_saver('./observation_saver')
save_act = CSV_saver('./act_saver')
save_reward = CSV_saver('./reward_saver')
draw_reward = Drawer('./draw_reward')
print = logger.info



on_p_rate =cfg['on_policy']['rate']
on_p_kb = cfg['on_policy']['kb']
mode == 'train'
his_util = [0] * 12
history_act = np.array([his_util] * num_envs)
total_update = args.update

obs = a1.observe()
a1.torque_limit =31
ppo.act(obs)
act = a1.position
a1.kp=[150] * 12
a1.kd = [7] * 12
envs_idx = 0
schedule = cfg['environment']['schedule']

action, _ = run_model_with_pt_input_modify(np.zeros((1,12)), 0, schedule, history_act, kb=on_p_kb,
                                                     rate=on_p_rate)

a1.init_motor(act)
init_position(action[0].tolist(), 250)
a1.observe()
print(f'actural position {a1.position}')
for i in range(500):
    if i >30 and i <=40:
        obs = a1.observe()
        init_e_x += obs[0][0]
        init_e_y += obs[0][1]
    if i==40:
        init_e_y /= 10
        init_e_x /= 10
    a1.hold_on()
    # print(a1.position)
# obs = a1.observe()

print(f'actual zero angle : {init_e_x, init_e_y}')

# obb = obs.copy()
# ppo_act = threading.Thread(target=ppo.act(obs))
# ppo_act.start()
# while ppo_act.is_alive():
#     a1.hold_on()
#     print('holding')
# print('finished hold /on ')
envs_idx = 0
for update in range(total_update):
    reward_sum = 0
    if update==0:
        a1.hold_on()
        # print('1')
    for step in range(n_steps):
        # waiter.wait()
        obs = a1.observe()
        # print(3)
        action = ppo.act(obs)
        if update == 0 and step == 0 :
            a1.hold_on()
            # print('2')

        # action = np.zeros_like(action)

        # action=np.clip(action-1, -1, 1)
        # print(4)
        action, history_act = run_model_with_pt_input_modify(action, envs_idx, schedule, history_act, kb=on_p_kb, rate=on_p_rate)
        action=action[0]
        if step==0 and update ==0:
            print(f"pre take action the first time ")
        # print('take action ')
        a1.take_action(action.tolist())
        reward = cal_reward(a1)
        envs_idx +=1

        ppo.step(value_obs=obs, rews=reward, dones=np.array([False]))

        save_observe.add_list(obs[0])
        save_act.add_list(action[0])

        reward_sum = reward_sum + np.sum(reward)

    # take st step to get value obs
    update_thread = threading.Thread(target=updating)
    update_thread.start()
    while update_thread.is_alive():
        # waiter.wait()
        print('threading running')
        a1.take_action(action.tolist())
    print('updating finished')
print(f'biggest:{biggest_reward},rate = {biggest_iter}')


