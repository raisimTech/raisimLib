from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher, RaisimLogger
from raisimGymTorch.env.bin.rsg import NormalSampler
from raisimGymTorch.env.bin.rsg import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
from raisimGymTorch.env.deploy.angle_utils import transfer_f ,get_last_position
from raisimGymTorch.env.deploy.onnx_deploy import run_model_with_pt_input_modify, list_pt, run_model_with_pt_input
import os
import math
import time
import torch.nn as nn
import numpy as np
import torch
import datetime
import argparse
from unitree_deploy.angle_utils import sine_generator
from unitree_utils.Waiter import Waiter
from raisimGymTorch.deploy_log.draw_map import Drawer


# task specification
task_name = "rsg_test"


"""
todo:
    1. init model 
    2. change p-d info 
    3. debug and add_on sine_gait

"""


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

# his_util=[0, 0.523, -1.046] * 4
his_util=[0] * 12
check_done = lambda a, b: a + 1 if not b else 0
check_history = lambda a, b: a if not b else his_util
# need_to_review = lambda a,b: a if not b else  np.array([[0, 0.523, -1.046] * 4]) # todo make sure the shape
# if load_best == True:
#     weight_path = ""

print = logger.info

mode == 'train'
history_act = np.array([his_util] * num_envs)
total_update = args.update
if mode =='train' or mode == 'retrain':
    # print('start train')
    schedule = cfg['environment']['schedule']
    angle_rate = cfg['environment']['angle_rate']
    act_rate = cfg['environment']['action_std'] # how many action generated use for work
    act_rate = float(act_rate)
    for update in range(total_update):
        start = time.time()
        env.reset()
        reward_sum = 0
        done_sum = 0
        average_dones = 0.

        if update % cfg['environment']['eval_every_n'] == 0:
            waiter = Waiter(0.01)
            print("Visualizing and evaluating the current policy")
            torch.save({
                'actor_architecture_state_dict': actor.architecture.state_dict(),
                'actor_distribution_state_dict': actor.distribution.state_dict(),
                'critic_architecture_state_dict': critic.architecture.state_dict(),
                'optimizer_state_dict': ppo.optimizer.state_dict(),
            }, saver.data_dir+"/full_"+str(update)+'.pt')
            # we create another graph just to demonstrate the save/load method
            loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim)
            loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])

            # debug_draw_action = Drawer('debug_draw_action')
            # debug_draw_history = Drawer('debug_draw_history')

            env.turn_on_visualization()
            env.start_video_recording(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4')
            envs_idx = [0] * num_envs
            waiter.update_start()
            for step in range(n_steps):
                with torch.no_grad():
                    waiter.wait()
                    frame_start = time.time()
                    obs = env.observe(False)
                    # print(obs.shape)

                    # history_act = get_last_position(obs)
                    # history_act = np.array([ check_history(history_act[i], dones[i]) for i in range(num_envs)] )


                    action = loaded_graph.architecture(torch.from_numpy(obs).cpu())
                    action = action.cpu().detach().numpy()

                    # sine = sine_generator(envs_idx, schedule, angle_rate)
                    # action, history_act = transfer_f(action, sine, act_rate, history_act=history_act)
                    # action = action.astype(np.float32)
                    #

                    action, history_act = run_model_with_pt_input_modify(action, envs_idx, schedule, history_act)


                    # debug_draw_action.add_map_list(action[0])
                    # debug_draw_history.add_map_list(history_act[0])

                    reward, dones = env.step(action)

                    # history_act = sine
                    # envs_idx = list(map(check_done, envs_idx, dones))
                    # history_act = action
                    history_act = np.array([check_history(history_act[i], dones[i]) for i in range(num_envs)])

                    envs_idx = list(map(check_done, envs_idx, dones))

                    reward_analyzer.add_reward_info(env.get_reward_info())
                    frame_end = time.time()
                    wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
                    # if wait_time > 0.:
                    #     time.sleep(wait_time)

            env.stop_video_recording()
            env.turn_off_visualization()
            history_act = np.array([his_util] * num_envs)
            reward_analyzer.analyze_and_plot(update)
            env.reset()
            env.save_scaling(saver.data_dir, str(update))

            # debug_draw_action.draw()
            # debug_draw_history.draw()

        # actual training
        # sine = [0] * 12
        envs_idx = [0] * num_envs
        for step in range(n_steps):
            obs = env.observe(False)
            """
                1. z轴方向的加速度的处理方法
            
            """
            # history_act = get_last_position(obs)

            action = ppo.act(obs)


            # sine = sine_generator(envs_idx, schedule, angle_rate)
            # action, history_act = transfer_f(action, sine, act_rate, history_act=history_act)
            # action = action.astype(np.float32)
            # action1, history_act1 = run_model_with_pt_input(action,envs_idx,schedule,history_act)
            action, history_act = run_model_with_pt_input_modify(action, envs_idx, schedule, history_act)
            # print(f'action1 {action1}\n action{action}')
            # print(f'history1 {history_act1} \n history{history_act}')
            # input()
            # print(f'3{time.time()}')
            # todo the transfer has bug
            reward, dones = env.step(action)
            # input('1')
            # print(f"history : {history_act}")

            # history_act = sine
            # envs_idx = list(map(check_done, envs_idx, dones))
            # history_act = action
            history_act = np.array([check_history(history_act[i], dones[i]) for i in range(num_envs)])

            # reward = reward * envs_idx / n_steps * 2

            envs_idx = list(map(check_done, envs_idx, dones))
            ppo.step(value_obs=obs, rews=reward, dones=dones)
            done_sum = done_sum + np.sum(dones)
            reward_sum = reward_sum + np.sum(reward)
            reward_analyzer.add_reward_info(env.get_reward_info())
            reward_analyzer.analyze_and_plot(update)
        # take st step to get value obs
        obs = env.observe(False)
        ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 10 == 0, update=update)
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
else:
    env.reset()
    start = time.time()
    onnx_flag = False
    if onnx_flag:
        cnt_onnx = 0
        from raisimGymTorch.env.deploy import onnx_deploy

    # load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)
    for step in range(n_steps * 10):
        time.sleep(0.01)
        obs = env.observe(False)
        # print(obs.shape)
        if onnx_flag:
            action = onnx_deploy.run_model(obs, cnt_onnx, 50)
            action = np.array(action)[0]
            cnt_onnx += 1
            # action = np.array([act for x in range(100)])
        else:
            action = ppo.act(obs)


        reward, dones = env.step(action)

print(f'biggest:{biggest_reward},rate = {biggest_iter}')


