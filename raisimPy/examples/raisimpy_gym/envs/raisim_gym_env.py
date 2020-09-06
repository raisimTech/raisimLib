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
import gym

import raisimpy as raisim

from raisimpy_gym.envs.utils import keyboard_interrupt, load_yaml


__author__ = ["Jemin Hwangbo (C++, Python)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"


class RaisimGymEnv(object):  # gym.Env
    r"""Raisim Gym Env

    This is a Python translation of [1]. The ``RaisimGymEnv`` is the base abstract class from which the other robotic
    gym environments inherit from. Children of this class are given to vectorized environments that are then given to
    the RL algorithms.

    References:
        - [1] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/RaisimGymEnv.hpp
    """

    def __init__(self, config, resource_directory=os.path.dirname(os.path.abspath(__file__)) + '/../rsc/',
                 visualizable=False):
        """
        Initialize the raisim gym environment.

        Args:
            config (str): path to YAML configuration file.
            resource_directory (str): resource directory path. This directory should contain the URDF and meshes.
            visualizable (bool): if we should visualize or not the simulation.
        """
        # set variables
        self.resource_directory = resource_directory
        self.visualizable = visualizable
        if isinstance(config, str):
            config = load_yaml(config)
        elif not isinstance(config, dict):
            raise TypeError("Expecting the given 'config' argument to be dict or a str (path to the YAML file).")
        self.config = config

        # define other variables
        self.simulation_dt = 0.0025
        self.control_dt = 0.01
        self.extra_info = dict()  # {str: float}
        self.ob_dim, self.action_dim = 0, 0

        self.visualization_counter = 0
        self.visualize_this_step = visualizable
        self.desired_fps = 60.
        self.already_closed = False

        # create world
        self.world = raisim.World()
        self.world.set_time_step(self.simulation_dt)

    def init(self):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    def set_seed(self, seed):
        raise NotImplementedError

    def observe(self):
        raise NotImplementedError

    @keyboard_interrupt
    def step(self, action):
        raise NotImplementedError

    def is_terminal_state(self):
        raise NotImplementedError

    def curriculum_update(self):
        pass

    def close(self):
        pass

    def update_extra_info(self):
        pass

    def set_simulation_time_step(self, dt):
        self.simulation_dt = dt
        self.world.set_time_step(dt)

    def get_simulation_time_step(self):
        return self.simulation_dt

    @staticmethod
    def start_recording_video(filename):
        raisim.OgreVis.get().start_recording_video(filename)

    @staticmethod
    def stop_recording_video():
        raisim.OgreVis.get().stop_recording_video_and_save()

    def set_control_time_step(self, dt):
        self.control_dt = dt

    def get_control_time_step(self):
        return self.control_dt

    def get_ob_dim(self):
        return self.ob_dim

    def get_action_dim(self):
        return self.action_dim

    def get_extra_info_dim(self):
        return len(self.extra_info)

    def get_world(self):
        return self.world

    def turn_on_visualization(self):
        self.visualize_this_step = True

    def turn_off_visualization(self):
        self.visualize_this_step = False
