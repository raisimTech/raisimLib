#!/usr/bin/env python
"""RL environment using Gym and raisimpy.

The environment is the same one as the one that can be found on the `raisimGym` repository [1] but fully written in
Python using the `raisimpy` library (which is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

Note that in this class, we use a callable class which is given to the `set_control_callback` function.

References:
    - [1] raisimGym: https://github.com/leggedrobotics/raisimGym
    - [2] raisimLib: https://github.com/leggedrobotics/raisimLib
    - [3] raisimOgre: https://github.com/leggedrobotics/raisimOgre
"""

import os
import numpy as np
from multiprocessing.pool import ThreadPool
from functools import partial

import gym
from gym import spaces
from stable_baselines.common.vec_env import VecEnv

import raisimpy as raisim

from raisimpy_gym.envs.raisim_gym_env import RaisimGymEnv
from raisimpy_gym.envs.utils import keyboard_interrupt, load_yaml


__author__ = ["Jemin Hwangbo (C++, Python)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"


class RaisimGymVecEnv(VecEnv):
    r"""Raisim Gym Vectorized Environment.

    This is the Python translation of [1] and combined with [2] to form one class. In the original code, there are
    2 classes [1] and [2] because [1] is written in C++ (and then wrapped) and used by [2] which is written in Python.
    In our case, we don't need to wrap anything so [1] and [2] are combined here.

    References:
        - [1] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/VectorizedEnvironment.hpp
        - [2] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/RaisimGymVecEnv.py
    """

    def __init__(self, environment, config, resource_directory=os.path.dirname(os.path.abspath(__file__)) + '/../rsc/'):
        self.resource_directory = resource_directory
        if isinstance(config, str):
            config = load_yaml(config)
        elif not isinstance(config, dict):
            raise TypeError("Expecting the given 'config' argument to be dict or a str (path to the YAML file).")
        self.config = config

        if not issubclass(environment, RaisimGymEnv):
            raise TypeError("Expecting the given 'environment' to be a subclass of `RaisimGymEnv`.")
        self.environment = environment

        self.environments = []
        self.extra_info_names = []
        self.record_video = False
        self.already_closed = False
        self.pool = None

        self.num_envs = 1
        self.ob_dim, self.action_dim = 0, 0

        self.init()

        self.num_obs = self.get_ob_dim()
        self.num_acts = self.get_action_dim()
        observation_space = spaces.Box(np.ones(self.num_obs) * -np.inf, np.ones(self.num_obs) * np.inf,
                                       dtype=np.float32)
        action_space = spaces.Box(np.ones(self.num_acts) * -1., np.ones(self.num_acts) * 1., dtype=np.float32)

        self._observation = np.zeros([self.num_envs, self.num_obs], dtype=np.float32)
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self._done = np.zeros(self.num_envs, dtype=np.bool)
        self._extra_info = np.zeros([self.num_envs, len(self.extra_info_names)], dtype=np.float32)
        self.rewards = [[] for _ in range(self.num_envs)]

        super(RaisimGymVecEnv, self).__init__(self.num_envs, observation_space, action_space)

    #############
    # Operators #
    #############

    def __del__(self):
        self.close()

    ###################
    # Getters/Setters #
    ###################

    def get_extra_info_names(self):
        return self.extra_info_names

    def set_simulation_time_step(self, dt):
        for env in self.environments:
            env.set_simulation_time_step(dt)

    def set_control_time_step(self, dt):
        for env in self.environments:
            env.set_control_time_step(dt)

    def get_ob_dim(self):
        return self.ob_dim

    def get_action_dim(self):
        return self.action_dim

    def get_extra_info_dim(self):
        return len(self.extra_info_names)

    def get_num_envs(self):
        return self.num_envs

    ###########
    # Methods #
    ###########

    def init(self):
        # create multiple threads
        self.pool = ThreadPool(processes=int(self.config['environment']['num_threads']))
        self.num_envs = int(self.config['environment']['num_envs'])

        # initialize each environment
        for i in range(self.num_envs):
            # only the first environment is visualized
            environment = self.environment(self.config, self.resource_directory, visualizable=(i == 0))
            environment.set_simulation_time_step(float(self.config['environment']['simulation_dt']))
            environment.set_control_time_step(float(self.config['environment']['control_dt']))
            environment.init()
            environment.reset()
            self.environments.append(environment)

        # get observation and action dimensions
        self.ob_dim = self.environments[0].get_ob_dim()
        self.action_dim = self.environments[0].get_action_dim()
        if self.ob_dim == 0 or self.action_dim == 0:
            raise RuntimeError("Observation/Action dimension must be defined in the constructor of each environment!")

        # generate reward names: compute it once to get reward names. The actual value is not used.
        self.environments[0].update_extra_info()
        for info_name in self.environments[0].extra_info.keys():
            self.extra_info_names.append(info_name)

    def seed(self, seed=None):
        for env in self.environments:
            env.set_seed(seed)
            seed += 1

    def step(self, action, visualize=False):
        if not visualize:
            self._step(action, self._observation, self._reward, self._done, self._extra_info)
        else:
            self._test_step(action, self._observation, self._reward, self._done, self._extra_info)

        info = [{'extra_info': {
            self.extra_info_names[j]: self._extra_info[i, j],
        }} for j in range(0, len(self.extra_info_names)) for i in range(self.num_envs)]

        for i in range(self.num_envs):
            self.rewards[i].append(self._reward[i])

            if self._done[i]:
                eprew = sum(self.rewards[i])
                eplen = len(self.rewards[i])
                epinfo = {"r": eprew, "l": eplen}
                info[i]['episode'] = epinfo
                self.rewards[i].clear()

        return self._observation.copy(), self._reward.copy(), self._done.copy(), info.copy()

    def _step(self, action, observation, reward, done, extra_info):
        step = partial(self.__per_agent_step, action=action, observation=observation, reward=reward, done=done,
                       extra_info=extra_info)
        self.pool.map(step, [idx for idx in range(self.num_envs)])
        return self.observe()

    def _test_step(self, action, observation, reward, done, extra_info):
        # only the first environment
        self.environments[0].turn_on_visualization()
        self.__per_agent_step(0, action, observation, reward, done, extra_info)
        self.environments[0].turn_off_visualization()
        return self.environments[0].observe()

    def observe(self):
        return np.array([env.observe() for env in self.environments])  # (num_envs, ob_dim)

    def is_terminal_state(self):
        return [env.is_terminal_state()[0] for env in self.environments]

    def __per_agent_step(self, agent_id, action, observation, reward, done, extra_info):
        # perform one step in the environment
        # print(agent_id, len(action), len(self.environments))
        _, rew, over, info = self.environments[agent_id].step(action[agent_id])
        # the above line raises an error with TF; got Seg. fault., address not mapped

        reward[agent_id] = rew
        done[agent_id] = over

        for j, info_name in enumerate(self.extra_info_names):
            extra_info[agent_id, j] = info[info_name]

        if done[agent_id]:
            self.environments[agent_id].reset()

    def reset(self):
        self._reward = np.zeros(self.num_envs, dtype=np.float32)

        for env in self.environments:
            env.reset()

        self._observation = self.observe()
        return self._observation.copy()

    def reset_and_update_info(self):
        return self.reset(), self._update_epi_info()

    def _update_epi_info(self):
        info = [{} for _ in range(self.num_envs)]

        for i in range(self.num_envs):
            eprew = sum(self.rewards[i])
            eplen = len(self.rewards[i])
            epinfo = {"r": eprew, "l": eplen}
            info[i]['episode'] = epinfo
            self.rewards[i].clear()

        return info

    def render(self, mode='human'):
        raise RuntimeError('This method is not implemented')

    def close(self):
        if not self.already_closed:
            for env in self.environments:
                env.close()
            self.already_closed = True

    def start_recording_video(self, filename):
        self.environments[0].start_recording_video(filename)

    def stop_recording_video(self):
        self.environments[0].stop_recording_video()

    def curriculum_callback(self):
        for env in self.environments:
            env.curriculum_update()

    def step_async(self):
        raise RuntimeError('This method is not implemented')

    def step_wait(self):
        raise RuntimeError('This method is not implemented')

    def get_attr(self, attr_name, indices=None):
        """
        Return attribute from vectorized environment.

        :param attr_name: (str) The name of the attribute whose value to return
        :param indices: (list,int) Indices of envs to get attribute from
        :return: (list) List of values of 'attr_name' in all environments
        """
        raise RuntimeError('This method is not implemented')

    def set_attr(self, attr_name, value, indices=None):
        """
        Set attribute inside vectorized environments.

        :param attr_name: (str) The name of attribute to assign new value
        :param value: (obj) Value to assign to `attr_name`
        :param indices: (list,int) Indices of envs to assign value
        :return: (NoneType)
        """
        raise RuntimeError('This method is not implemented')

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        """
        Call instance methods of vectorized environments.

        :param method_name: (str) The name of the environment method to invoke.
        :param indices: (list,int) Indices of envs whose method to call
        :param method_args: (tuple) Any positional arguments to provide in the call
        :param method_kwargs: (dict) Any keyword arguments to provide in the call
        :return: (list) List of items returned by the environment's method call
        """
        raise RuntimeError('This method is not implemented')
