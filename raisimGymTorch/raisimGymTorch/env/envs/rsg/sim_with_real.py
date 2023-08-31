from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher, RaisimLogger
from raisimGymTorch.env.bin.rsg import NormalSampler
from raisimGymTorch.env.bin.rsg import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
from raisimGymTorch.deploy_utils.angle_utils import transfer
from unitree_utils.Waiter import Waiter
from raisimGymTorch.deploy_utils.angle_utils import get_last_position
from raisimGymTorch.deploy_log.draw_map import  Drawer
from raisimGymTorch.deploy_log.csv_saver import CSV_saver
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
from raisimGymTorch.deploy_utils.angle_utils import  deg_rad, rad_deg
from raisimGymTorch.deploy_utils.runner_util import run_model_with_pt_input_modify, list_pt


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


moving_robot = True
onnx_flag = False
virtual = False
on_p_rate =cfg['on_policy']['rate']
on_p_kb = cfg['on_policy']['kb']
# his_util=[0, 0.523, -1.046] * 4
his_util =[0] * 12
# his_util=[0, 0.523, -1.046] * 4
# check_done = lambda a, b: a + 1 if not b else 0
check_history = lambda a, b: a if not b else his_util
history_act = np.array([his_util] * num_envs)
# init virtual part
if virtual:
    waiter= Waiter(0.01)
    env.reset()
    start = time.time()
    onnx_flag = False
    if onnx_flag:
        cnt_onnx = 0
        from raisimGymTorch.deploy_utils import onnx_deploy
    else:
        load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

    envs_idx = [0] * num_envs


# NOTE : a1 init


if moving_robot:
    history_obs = None

    if onnx_flag:
        cnt_onnx = 0

        from raisimGymTorch.deploy_utils import onnx_deploy
    else:
        load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

    from robot_utils import *
    a1.torque_limit = 50
    # init_robot(dt)

    # ori_posi = sine_generator(0, schedule, rate=angle_rate).tolist() # initial position
    # a1.kp = cfg['environment']['kp']
    # ori_posi = a1.position
    # init_position(ori_posi,250)
    gen_action = ppo.act(a1.observe())
    a1.kp = [100] * 12
    a1.kd = [3] * 12
    # a1.init_k(a1.kp, a1.kd)
    ori_posi,_ = run_model_with_pt_input_modify(np.zeros_like(gen_action), 0, schedule, history_act, kb=on_p_kb,
                                                                 rate=on_p_rate)
    a1.stand_up(200, ori_posi[0].tolist())
    print(f"""
        now_posi: {a1.position},
        angle_rate : {angle_rate}
        schedule: {schedule}
    """)
    real_idx = 0
    for i in range(schedule):
        a1.hold_on()

    # history_obs = a1.observe()[ :5]
history_act = np.array([his_util] * num_envs)
for step in range(n_steps * 10):
    #virtual bot part
    if virtual:
        # if step%1    ==0 :
        #     input()
        if step == 0:
            draw_vir_obs = Drawer('vir_obs')
            draw_vir_action = Drawer('vir_sent_act')
            draw_vir_recv_action = Drawer('vir_recv_act')
            waiter.update_start()
            # save_gen_act = CSV_saver('virtual_gen_act', './')
            save_obs = CSV_saver('virtual_obs', './')
            save_for_work = CSV_saver('virtual_sent', './')
            save_recv = CSV_saver('virtual_recv', './')
        obs = env.observe(False)
        print('saving')
        last_p = get_last_position(obs)
        print(f'last{last_p}')
        save_recv.add_list(last_p[0])
        draw_vir_recv_action.add_map_list(last_p[0])
        if onnx_flag:
            # obs = rad_deg(obs)
            # print(obs.shape)
            action, obs = onnx_deploy.run_model(obs, cnt_onnx, 40)
            action = np.array(action)[0]
            cnt_onnx += 1
        else:
            gen_action = ppo.act(obs)
            action, history_act = run_model_with_pt_input_modify(gen_action, envs_idx, schedule, history_act, kb=on_p_kb,
                                                                 rate=on_p_rate)
            # sine = sine_generator(envs_idx, schedule, angle_rate)
            # action = transfer(gen_action, sine, act_rate, history_act=history_act).astype(np.float32)
            # history_act =
        print(f"virtual_obs:{obs} \n virtual_action:{action}")
        save_obs.add_list(obs[0])
        # draw_vir_recv_action.add_map_list(action[0])
        save_for_work.add_list(action[0])




    # real bot part
    if moving_robot:

        if step ==0:
            draw_real_obs = Drawer('real_obs')
            draw_sent_action = Drawer('real_sent_act')
            draw_recv_action = Drawer('real_a_recv_act')
            real_history_act = np.array(his_util)
            save_real_obs = CSV_saver('real_a_obs')
            save_real_action = CSV_saver('real_a_action')
        # print('b-observe')
        # obbs = a1.observe()
        # obbs[:5] -= history_obs
        real_obs = a1.observe()
        # real_obs = np.array([a1.observe()])
        # print(f"real_obs {real_obs}")
        # input('now cancel it ')
        # print()
        # print('b-ppo')


        if onnx_flag:
            action, real_obs = onnx_deploy.run_model(real_obs, cnt_onnx, 40)
            real_obs = real_obs[0]
            real_action = np.array(action)[0]
            cnt_onnx += 1

        else:
            gen_action = ppo.act(real_obs)
            # print('b-sin')
            # real_sine = sine_generator(real_idx, schedule, angle_rate)
            # print('b-trans')
            # real_action = transfer(gen_action, real_sine, act_rate, history_act=real_history_act)
            # gen_action = ppo.act(obs)
            action, history_act = run_model_with_pt_input_modify(gen_action, real_idx, schedule, history_act, kb=on_p_kb,
                                                                 rate=on_p_rate)

        # real_action = real_action
        # add_map_list(real_action)
        draw_real_obs.add_map_list(real_obs[:5])
        save_real_obs.add_list(real_obs)
        save_real_action.add_list(a1.position)
        draw_recv_action.add_map_list(a1.position)
        draw_sent_action.add_map_list(action[0])
        print(f"real_obs:{real_obs} \n for work_action:{action} ")
        # waiter.wait()
        if action.shape[0] == 1:
            action = action[0]
        a1.take_action(action.tolist())
        # real_history_act = real_sine
        real_idx +=1


    # update virtual bot
    if virtual:
        waiter.wait()
        reward, dones = env.step(action)
        envs_idx = list(map(check_done, envs_idx, dones))

        draw_vir_obs.add_map_list(obs[0][:5])
        draw_vir_action.add_map_list(action[0])
        history_act = np.array([check_history(history_act[i], dones[i]) for i in range(num_envs)])
# draw()
if virtual:
    save_obs.save()
    save_for_work.save()
    draw_vir_obs.draw()
    draw_vir_action.draw()
    draw_vir_recv_action.draw()
    save_recv.save()
    # save_gen_act.save()
else:
    a1.back_safe()
    save_real_obs.save()
    save_real_action.save()
    draw_real_obs.draw()
    draw_sent_action.draw()
    draw_recv_action.draw()

print(f'biggest:{biggest_reward},rate = {biggest_iter}')


