from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher
from raisimGymTorch.env.bin.rsg_leg import NormalSampler
from raisimGymTorch.env.bin.rsg_leg import RaisimGymEnv
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

task_name = 'leg_locomotion'

parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train' )
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
weight_path = args.weight

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + '/../../../../..'

cfg = YAML().load(open(task_path + '/cfg.yaml', 'r'))

env = VecEnv(RaisimGymEnv(home_path + '/rsc/', dump(cfg['environment'], Dumper=RoundTripDumper)))
env.seed(cfg['seed'])

ob_dim = env.num_obs
act_dim = env.num_acts
num_threads = cfg['environment']['num_threads']

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


saver = ConfigurationSaver(log_dir=home_path + '/raisimGymTorch/data/' + task_name,
                           save_items=[task_path+'/cfg.yaml', task_path+'/Environment.hpp'])
tensorboard_launcher(saver.data_dir + '/..')


ppo = PPO.PPO(actor=actor,
              critic=critic,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              num_mini_batches=4,
              learning_rate=1e-2,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False)

if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)


for update in range(10000):
    start = time.time()
    env.reset()
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0

    if update % cfg['environment']['eval_every_n'] == 0:
        print('Visualizing and evaluating the current policy')
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'ppo_optimizer_state_dict': ppo.optimizer.state_dict()
        }, saver.data_dir+'/full_'+ str(update)+'.pt')

        loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim)
        loaded_graph.load_state_dict(torch.load(saver.data_dir+'/full_'+ str(update) + '.pt')['actor_architecture_state_dict'])

        env.turn_on_visualization()
        env.start_video_recording(datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + 'policy_' + str(update) + '.mp4')

        for step in range(n_steps * 2):
            with torch.no_grad():
                frame_start = time.time()
                obs = env.observe(False)
                action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())
                reward_ll, dones  =env.step(action_ll.cpu().detach().numpy())
                frame_end = time.time()
                wait_time = cfg['environment']['control_dt'] - (frame_end - frame_start)
                if wait_time> 0:
                    time.sleep(wait_time)

        env.stop_video_recording()
        env.turn_off_visualization()

        env.reset()
        env.save_scaling(saver.data_dir, str(update))

    for step in range(n_steps):
        obs = env.observe()
        action = ppo.act(obs)
        reward, dones = env.step(action)
        ppo.step(value_obs=obs, rews=reward, dones=dones)
        done_sum = done_sum + np.sum(dones)
        reward_ll_sum = reward_ll_sum + np.sum(reward)

    obs = env.observe()
    ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 10 ==0, update=update)
    average_ll_performance = reward_ll_sum / total_steps
    average_dones = done_sum / total_steps
    avg_rewards.append(average_ll_performance)

    actor.update()
    actor.distribution.enforce_minimum_std((torch.ones(5) * 0.2).to(device)) #

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
