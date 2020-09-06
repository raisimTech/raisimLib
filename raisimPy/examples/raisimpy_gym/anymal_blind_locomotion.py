#!/usr/bin/env python
"""ANYmal blind locomotion RL example using Gym and raisimpy.

The example is mostly the same as the one given in [1], with few slight modifications.

References:
    - [1] https://github.com/leggedrobotics/raisimGym/blob/master/scripts/anymal_blind_locomotion.py
"""

import os
import math
import argparse
import time

from raisimpy_gym.envs.anymal import AnymalEnv, ANYMAL_CONFIG_DIRECTORY, ANYMAL_RESOURCE_DIRECTORY
from raisimpy_gym.envs import RaisimGymVecEnv
from raisimpy_gym.algos import PPO2
from raisimpy_gym.policies import MlpPolicy
from raisimpy_gym.helper import ConfigurationSaver, TensorboardLauncher
from raisimpy_gym.envs import load_yaml


# configuration
parser = argparse.ArgumentParser()
parser.add_argument('--cfg', type=str, default=os.path.abspath(ANYMAL_CONFIG_DIRECTORY + "/default_cfg.yaml"),
                    help='configuration file')
cfg_abs_path = parser.parse_args().cfg
cfg = load_yaml(cfg_abs_path)


# save the configuration and other files
rsg_root = os.path.dirname(os.path.abspath(__file__))
log_dir = rsg_root + '/data'
saver = ConfigurationSaver(log_dir=log_dir + '/ANYmal_blind_locomotion',
                           save_items=[rsg_root + '/envs/anymal/env.py', cfg_abs_path])


# create environment from the configuration file
env = RaisimGymVecEnv(AnymalEnv, config=cfg_abs_path, resource_directory=ANYMAL_RESOURCE_DIRECTORY)

# Get algorithm
model = PPO2(
    tensorboard_log=saver.data_dir,
    policy=MlpPolicy,
    policy_kwargs=dict(net_arch=[dict(pi=[128, 128], vf=[128, 128])]),
    env=env,
    gamma=0.998,
    n_steps=math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt']),
    ent_coef=0,
    learning_rate=1e-3,
    vf_coef=0.5,
    max_grad_norm=0.5,
    lam=0.95,
    nminibatches=1,
    noptepochs=10,
    cliprange=0.2,
    verbose=1,
)

# TensorBoard
# Make sure that your chrome browser is already on.
TensorboardLauncher(saver.data_dir + '/PPO2_1')

# PPO run
# model.learn(total_timesteps=400000000, eval_every_n=50, log_dir=saver.data_dir, record_video=cfg['record_video'])
model.learn(total_timesteps=400000, eval_every_n=50, log_dir=saver.data_dir, record_video=cfg['record_video'])

environment = env.environments[0]
environment.turn_on_visualization()
reward_sum = 0
total_steps = model.n_steps
start = time.time()
obs = environment.reset()

for step in range(total_steps):
    action = model.get_next_action(obs.reshape(1, -1))
    obs, reward, done, info = environment.step(action.reshape(-1))
    reward_sum += reward

end = time.time()
environment.turn_off_visualization()
print("average reward: " + str(reward_sum / total_steps))
print("time elapsed in this iteration: " + str(end - start))


# Need this line if you want to keep TensorFlow alive after training
input("Press Enter to exit... TensorBoard will be closed after exit\n")
