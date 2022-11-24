# //----------------------------//
# // This file is part of RaiSim//
# // Copyright 2020, RaiSim Tech//
# //----------------------------//

import numpy as np
import platform
import os
#######################3 my import ###########################3
import wandb
import time
###############################################################3


class RaisimGymVecEnv:

    def __init__(self, impl, normalize_ob=True, seed=0, clip_obs=10.):
        if platform.system() == "Darwin":
            os.environ['KMP_DUPLICATE_LIB_OK']='True'
        self.normalize_ob = normalize_ob
        self.clip_obs = clip_obs
        self.wrapper = impl
        self.num_obs = self.wrapper.getObDim()
        self.num_acts = self.wrapper.getActionDim()
        self._observation = np.zeros([self.num_envs, self.num_obs], dtype=np.float32)
        self.actions = np.zeros([self.num_envs, self.num_acts], dtype=np.float32)
        self.log_prob = np.zeros(self.num_envs, dtype=np.float32)
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self._done = np.zeros(self.num_envs, dtype=np.bool)
        self.rewards = [[] for _ in range(self.num_envs)]
        self.wrapper.setSeed(seed)
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

    def step(self, action, step):
        self.wrapper.step(action, self._reward, self._done, step)
        return self._reward.copy(), self._done.copy()

    def load_scaling(self, dir_name, iteration, count=1e5):
        mean_file_name = dir_name + "/mean" + str(iteration) + ".csv"
        var_file_name = dir_name + "/var" + str(iteration) + ".csv"
        self.count = count
        self.mean = np.loadtxt(mean_file_name, dtype=np.float32)
        self.var = np.loadtxt(var_file_name, dtype=np.float32)
        self.wrapper.setObStatistics(self.mean, self.var, self.count)

    ## my ##
    def setObStatistics(self, mean, var, count):
        self.count = count
        self.mean = mean
        self.var = var
        self.wrapper.setObStatistics(mean, var, count)
        
    ## my ##
    def getObStatistics(self):
        mean = np.zeros_like(self.mean)
        var = np.zeros_like(self.var)
        count = 0.0 # do not use it
        self.wrapper.getObStatistics(mean, var, count)
        return mean, var, count

    def save_scaling(self, dir_name, iteration):
        mean_file_name = dir_name + "/mean" + iteration + ".csv"
        var_file_name = dir_name + "/var" + iteration + ".csv"
        self.wrapper.getObStatistics(self.mean, self.var, self.count)
        np.savetxt(mean_file_name, self.mean)
        np.savetxt(var_file_name, self.var)
        ############################### my wandb save scale ###########################################
        wandb.save(mean_file_name)
        wandb.save(var_file_name)
        ###############################################################################################
############################ my comment #################################################
    # def observe(self, update_statistics=True):
    #     self.wrapper.observe(self._observation, update_statistics)
    #     return self._observation
#########################################################################################
############################# my observe ########################################
    def observe(self, update_statistics=True,isnoise=False):
        normed_obs = self._observation.copy()
        self.wrapper.observe(self._observation,normed_obs, update_statistics, isnoise)
        return self._observation.copy(), normed_obs
#################################################################################
################################### my function ###################################
    def getRewardinfo(self):
        return self.wrapper.rewardInfo()
####################################################################################
    def reset(self):
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self.wrapper.reset()

    def close(self):
        self.wrapper.close()

    def curriculum_callback(self):
        self.wrapper.curriculumUpdate()

    @property
    def num_envs(self):
        return self.wrapper.getNumOfEnvs()
