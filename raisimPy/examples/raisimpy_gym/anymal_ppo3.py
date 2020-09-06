#!/usr/bin/env python
"""ANYmal PPO3 RL example using Gym and raisimpy.

The example is mostly the same as the one given in [1], with few slight modifications.

References:
    - [1] https://github.com/leggedrobotics/raisimGym/blob/master/scripts/anymal_ppo3.py
"""

import os
import math
import argparse
import time

from raisimpy_gym.envs.anymal import AnymalEnv, ANYMAL_CONFIG_DIRECTORY, ANYMAL_RESOURCE_DIRECTORY
from raisimpy_gym.envs import RaisimGymVecEnv
from raisimpy_gym.algos import PPO3
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
saver = ConfigurationSaver(log_dir=log_dir + '/anymal_ppo3',
                           save_items=[rsg_root + '/envs/anymal/env.py', cfg_abs_path])

# create environment from the configuration file
env = RaisimGymVecEnv(AnymalEnv, config=cfg_abs_path, resource_directory=ANYMAL_RESOURCE_DIRECTORY)

# Get algorithm
model = PPO3(
    tensorboard_log=saver.data_dir,
    policy=MlpPolicy,
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

# TensorBoard: make sure that your chrome browser is already on.
TensorboardLauncher(saver.data_dir + '/PPO3_1')

# PPO run
reward_sum = 0
total_steps = model.n_steps * model.n_envs

for update in range(10000):
    start = time.time()
    obs = env.reset()

    for step in range(model.n_steps):
        action = model.get_next_action(obs)
        obs, rewards, dones, info = env.step(action)  # got error
        reward_sum += sum(rewards)
        model.collect(obs, rewards, dones)

    model.learn(update=update, nupdates=10000)
    end = time.time()

    print("average reward: " + str(reward_sum / total_steps))
    print("time elapsed in this iteration: " + str(end - start))

    reward_sum = 0

# Need this line if you want to keep TensorFlow alive after training
input("Press Enter to exit... TensorBoard will be closed after exit\n")
