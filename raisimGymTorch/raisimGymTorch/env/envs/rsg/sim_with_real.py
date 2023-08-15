from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher, RaisimLogger
from raisimGymTorch.env.bin.rsg import NormalSampler
from raisimGymTorch.env.bin.rsg import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
from raisimGymTorch.env.deploy.angle_utils import transfer
from unitree_utils.Waiter import Waiter
import os
import math
import time
import torch.nn as nn
import numpy as np
import torch
import datetime
import argparse
# import pandas as pd
# from sine_generator import sine_generator
from unitree_deploy.angle_utils import  sine_generator
""""
todo 
    1. left-hand right-hand check
    2. raisim data check 
    
    env cannot get linear-acc

"""





# task specification
task_name = "sim2real"



# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
parser.add_argument('-u', '--update', help='update times', type=int, default=120)
parser.add_argument('-p', '--cfg_path', help='where to find the path', type=str, default=None)
# parser.add_argument('-b', '--load_best', help='load best file in last train', type=bool, default=False)
args = parser.parse_args()
mode = args.mode
weight_path = args.weight
cfg_path = args.cfg_path
# load_best = args.load_best
# check if gpu is available
device = torch.device('cpu')

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

dt = cfg['environment']['control_dt']
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
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              num_mini_batches=4,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False,
              learning_rate=cfg['environment']['learnning_rate']
              )

reward_analyzer = RewardAnalyzer(env, ppo.writer)
biggest_reward = 0
biggest_iter = 0
if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

check_done = lambda a, b: a + 1 if not b else 0

print = logger.info


total_update = args.update

schedule = cfg['environment']['schedule']
angle_rate = cfg['environment']['angle_rate']
act_rate = cfg['environment']['action_std'] # how many action generated use for work
act_rate = float(act_rate)

# act_rate = 0

input(f"Are you sure to execute the action in schedule: {schedule}, angle_rate: {angle_rate}, act_rate(how many action generated from NN used for work): {act_rate}")


moving_robot = False
virtual = True

his_util=[0, 0.523, -1.046] * 4
# check_done = lambda a, b: a + 1 if not b else 0
check_history = lambda a, b: a if not b else his_util
history_act = np.array([his_util] * 100)
# init virtual part
if virtual:
    waiter= Waiter(0.01)
    env.reset()
    start = time.time()
    onnx_flag = False
    if onnx_flag:
        cnt_onnx = 0
        from raisimGymTorch.env.deploy import onnx_deploy
    else:
        load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

    envs_idx = [0] * num_envs


# NOTE : a1 init


if moving_robot:
    from robot_utils import *
    a1.torque_limit = 33.15
    init_robot(dt)

    ori_posi = sine_generator(0, schedule, rate=angle_rate).tolist() # initial position
    # a1.kp = cfg['environment']['kp']
    # ori_posi = a1.position
    init_position(ori_posi,200)
    print(f"""
        now_posi: {a1.position},
        angle_rate : {angle_rate}
        schedule: {schedule}
    """)
    real_idx = 0
    for i in range(schedule):
        a1.hold_on()
    a1.kp = [100] * 12
    a1.kd = [7] * 12
    a1.init_k(a1.kp, a1.kd)

for step in range(n_steps * 10):
    if not moving_robot:
        if step == 0:
            waiter.update_start()
        print('moving?')
        time.sleep(0.005)
        obs = env.observe(False)


    #virtual bot part
    if virtual:
        if onnx_flag:
            action = onnx_deploy.run_model(obs, cnt_onnx, 50)
            action = np.array(action)[0]
            cnt_onnx += 1
        else:
            gen_action = ppo.act(obs)
            sine = sine_generator(envs_idx, schedule, angle_rate)
            action = transfer(gen_action, sine, act_rate, history_act=history_act).astype(np.float32)
            print(f"virtual_obs:{obs} \n virtual_gen_action:{gen_action} \n virtual_action:{action}")

    # obs = np.zeros((1,31))
    # gen_act = ppo.act(obs)
    # sine = sine_generator(real_idx, schedule, angle_rate)
    # act = transfer(gen_act, sine, act_rate)



    # real bot part
    if moving_robot:
        if step ==0:
            real_history_act = np.array(his_util)
        print('b-observe')
        real_obs = a1.observe()
        # real_obs = np.array([a1.observe()])
        print(f"real_obs {real_obs}")
        # input('now cancel it ')
        # print()
        print('b-ppo')
        gen_action = ppo.act(real_obs)
        print('b-sin')
        real_sine = sine_generator(real_idx, schedule, angle_rate)
        print('b-trans')
        real_action = transfer(gen_action, real_sine, act_rate, history_act=real_history_act)
        # real_action = real_action
        print(f"real_obs:{real_obs} \n gen_action : {gen_action}\n for work_action:{real_action} \n now_tau_est:{a1.tau}")
        # waiter.wait()
        if real_action.shape[0] == 1:
            real_action = real_action[0]
        a1.take_action(real_action.tolist())
        real_history_act = real_sine
        # envs_idx = list(map(check_done, real_idx, dones))
        # history_act = np.array([check_history(history_act[i], 0) for i in range(num_envs)])
        real_idx += 1

    # update virtual bot
    if virtual:
        waiter.wait()
        reward, dones = env.step(action)
        print('exec')
        # envs_idx = list(map(check_done, envs_idx, dones))
        history_act = sine
        envs_idx = list(map(check_done, envs_idx, dones))
        history_act = np.array([check_history(history_act[i], dones[i]) for i in range(num_envs)])

print(f'biggest:{biggest_reward},rate = {biggest_iter}')


