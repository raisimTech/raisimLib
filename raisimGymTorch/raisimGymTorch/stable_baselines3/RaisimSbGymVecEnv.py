# //----------------------------//
# // This file is part of RaiSim//
# // Copyright 2020, RaiSim Tech//
# //----------------------------//
from typing import Type, List

import numpy as np
import platform
import os
import gym.spaces.space
from stable_baselines3.common.vec_env.base_vec_env import VecEnv, VecEnvIndices, Any


class RaisimSbGymVecEnv(VecEnv):
    metadata = {}

    def __init__(self, impl, normalize_ob=True, seed=0, clip_obs=10.):
        if platform.system() == "Darwin":
            os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'
        self.normalize_ob = normalize_ob
        self.clip_obs = clip_obs
        self.wrapper = impl
        self.num_obs = self.wrapper.getObDim()
        self.num_acts = self.wrapper.getActionDim()
        self.observation_space = gym.spaces.Box(-np.full(self.num_obs, 1e6, np.float32), np.full(self.num_obs, 1e6, np.float32), dtype=np.float32)
        self.action_space = gym.spaces.Box(-np.full(self.num_acts, 1e6, np.float32), np.full(self.num_acts, 1e6, np.float32), dtype=np.float32)
        super(RaisimSbGymVecEnv, self).__init__(self.wrapper.getNumOfEnvs(), self.observation_space, self.action_space)

        self._observation = np.zeros([self.num_envs, self.num_obs], dtype=np.float32)
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self._done = np.zeros(self.num_envs, dtype=np.bool)
        self._info =[{} for k in range(self.num_envs)]
        self.rewards = [[] for _ in range(self.num_envs)]
        self.wrapper.setSeed(seed)
        self.actions = None
        self.count = 0.0
        self.mean = np.zeros(self.num_obs, dtype=np.float32)
        self.var = np.zeros(self.num_obs, dtype=np.float32)

    def seed(self, seed=None):
        self.wrapper.setSeed(seed)

    def turn_on_visualization(self):
        self.wrapper.turnOnVisualization()

    def turn_off_visualization(self):
        self.wrapper.turnOffVisualization()

    def start_video_recording(self, file_name):
        self.wrapper.startRecordingVideo(file_name)

    def stop_video_recording(self):
        self.wrapper.stopRecordingVideo()

    def step_async(self, actions: np.ndarray) -> None:
        self.actions = actions

    def step_wait(self):
        self.wrapper.step(self.actions, self._reward, self._done)
        return self.observe(self.normalize_ob).copy(), self._reward.copy(), self._done.copy(), self._info

    def env_method(self, method_name: str, *method_args, indices: VecEnvIndices = None, **method_kwargs):
        pass

    def get_attr(self, attr_name: str, indices: VecEnvIndices = None):
        pass

    def set_attr(self, attr_name: str, value: Any, indices: VecEnvIndices = None):
        pass

    def load_scaling(self, dir_name, iteration, count=1e5):
        mean_file_name = dir_name + "/mean" + str(iteration) + ".csv"
        var_file_name = dir_name + "/var" + str(iteration) + ".csv"
        self.count = count
        self.mean = np.loadtxt(mean_file_name, dtype=np.float32)
        self.var = np.loadtxt(var_file_name, dtype=np.float32)
        self.wrapper.setObStatistics(self.mean, self.var, self.count)

    def save_scaling(self, dir_name, iteration):
        mean_file_name = dir_name + "/mean" + iteration + ".csv"
        var_file_name = dir_name + "/var" + iteration + ".csv"
        self.wrapper.getObStatistics(self.mean, self.var, self.count)
        np.savetxt(mean_file_name, self.mean)
        np.savetxt(var_file_name, self.var)

    def observe(self, update_mean=True):
        self.wrapper.observe(self._observation, update_mean)
        return self._observation

    def reset(self):
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self.wrapper.reset()
        return self.observe(False)

    def close(self):
        self.wrapper.close()

    def curriculum_callback(self):
        self.wrapper.curriculumUpdate()

    def render(self, mode='human'):
        pass

    def env_is_wrapped(self, wrapper_class: Type[gym.Wrapper], indices: VecEnvIndices = None) -> List[bool]:
        """Check if worker environments are wrapped with a given wrapper"""
        target_envs = self._get_target_envs(indices)
        # Import here to avoid a circular import
        from stable_baselines3.common import env_util

        return [env_util.is_wrapped(env_i, wrapper_class) for env_i in target_envs]


