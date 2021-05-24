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

    def __init__(self, impl, cfg, normalize_ob=True, seed=0, normalize_rew=True, clip_obs=10.):
        if platform.system() == "Darwin":
            os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

        self.normalize_ob = normalize_ob
        self.normalize_rew = normalize_rew
        self.clip_obs = clip_obs
        self.wrapper = impl
        self.num_obs = self.wrapper.getObDim()
        self.num_acts = self.wrapper.getActionDim()
        self.observation_space = gym.spaces.Box(-np.full(self.num_obs, np.inf), np.full(self.num_obs, np.inf), dtype=np.float32)
        self.action_space = gym.spaces.Box(-np.full(self.num_acts, np.inf), np.full(self.num_acts, np.inf), dtype=np.float32)
        super(RaisimSbGymVecEnv, self).__init__(self.wrapper.getNumOfEnvs(), self.observation_space, self.action_space)

        self._observation = np.zeros([self.num_envs, self.num_obs], dtype=np.float32)
        self.obs_rms = RunningMeanStd(shape=[self.num_envs, self.num_obs])
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self._done = np.zeros(self.num_envs, dtype=np.bool)
        self.rewards = [[] for _ in range(self.num_envs)]
        self.seed(seed)
        self.actions = None

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
        return self.observe(True), self._reward.copy(), self._done.copy(), []

    def env_method(self, method_name: str, *method_args, indices: VecEnvIndices = None, **method_kwargs):
        pass

    def get_attr(self, attr_name: str, indices: VecEnvIndices = None):
        pass

    def set_attr(self, attr_name: str, value: Any, indices: VecEnvIndices = None):
        pass

    def load_scaling(self, dir_name, iteration, count=1e5):
        mean_file_name = dir_name + "/mean" + str(iteration) + ".csv"
        var_file_name = dir_name + "/var" + str(iteration) + ".csv"
        self.obs_rms.count = count
        self.obs_rms.mean = np.loadtxt(mean_file_name, dtype=np.float32)
        self.obs_rms.var = np.loadtxt(var_file_name, dtype=np.float32)

    def save_scaling(self, dir_name, iteration):
        mean_file_name = dir_name + "/mean" + iteration + ".csv"
        var_file_name = dir_name + "/var" + iteration + ".csv"
        np.savetxt(mean_file_name, self.obs_rms.mean)
        np.savetxt(var_file_name, self.obs_rms.var)

    def observe(self, update_mean=True):
        self.wrapper.observe(self._observation)

        if self.normalize_ob:
            if update_mean:
                self.obs_rms.update(self._observation)

            return self._normalize_observation(self._observation)
        else:
            return self._observation.copy()

    def reset(self):
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self.wrapper.reset()
        return self.observe(False)

    def _normalize_observation(self, obs):
        if self.normalize_ob:

            return np.clip((obs - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + 1e-8), -self.clip_obs,
                           self.clip_obs)
        else:
            return obs

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


class RunningMeanStd(object):
    def __init__(self, epsilon=1e-4, shape=()):
        """
        :param epsilon: (float) helps with arithmetic issues
        :param shape: (tuple) the shape of the data stream's output
        """
        self.mean = np.zeros(shape, 'float32')
        self.var = np.ones(shape, 'float32')
        self.count = epsilon

    def update(self, arr):
        batch_mean = np.mean(arr, axis=0)
        batch_var = np.var(arr, axis=0)
        batch_count = arr.shape[0]
        self.update_from_moments(batch_mean, batch_var, batch_count)

    def update_from_moments(self, batch_mean, batch_var, batch_count):
        delta = batch_mean - self.mean
        tot_count = self.count + batch_count

        new_mean = self.mean + delta * batch_count / tot_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m_2 = m_a + m_b + np.square(delta) * (self.count * batch_count / (self.count + batch_count))
        new_var = m_2 / (self.count + batch_count)

        new_count = batch_count + self.count

        self.mean = new_mean
        self.var = new_var
        self.count = new_count

