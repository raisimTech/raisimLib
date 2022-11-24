import numpy as np
import datetime
import wandb
import torch
import torch.nn as nn
import time
import math
from raisimGymTorch.mpc.mpc_controller_DAgger_all import a1
from raisimGymTorch.mpc.mpc_controller_DAgger_all import gait_generator as gait_generator_lib

SWING = 0
STANCE = 1
EARLY_CONTACT = 2
LOSE_CONTACT = 3

class MyHelper():
    def __init__(self,env,cfg,n_steps):
        self.env = env
        self.cfg = cfg
        self.num_env = cfg['environment']['num_envs']
        self.n_steps = n_steps
        self.count = 0
        self.ctrl_time_step = self.cfg['environment']['control_dt']        
        self.obs_mean = np.array([ 0.117 , 0.742 ,-1.934, -0.14,   0.74,  -1.923, -0.06,   0.787, -1.882,  0.039, 0.778, -1.872])
        self.obs_var = np.array([0.001, 0.012, 0.034, 0.001, 0.013, 0.039, 0.001, 0.013, 0.035, 0.001, 0.013, 0.04 ])
        self.joint_pos_mean = np.array([ 0.037,  0.304, -0.868, -0.048,  0.301, -0.865, -0.019,  0.316, -0.842,  0.01, 0.32,  -0.853])
        self.joint_pos_var = np.array([0.002, 0.144, 1.131, 0.004, 0.142, 1.14,  0.001, 0.158, 1.082, 0.0003,    0.16,  1.094])
        self.ff_torque_mean = np.array([ 1.804,  1.705,  3.791, -1.81,   1.717,  3.718,  1.888,  2.188,  6.759, -2.026, 1.995,  6.527])
        self.ff_torque_var = np.array([ 4.422,  2.901, 13.736,  4.501,  2.834, 12.734,  2.639,  8.499, 41.766,  3.096, 7.678, 40.002])

    @staticmethod    
    def get_ctrl_obs(env_obs):
        bodyOrientation = env_obs[:,0:4]
        footContacts = env_obs[:,4:8]
        jointPos = env_obs[:,8:20]
        baseLinVel = env_obs[:,20:23]
        baseAngVel = env_obs[:,23:26]
        return {'bodyOrientation': bodyOrientation, 'footContacts':footContacts,\
                'jointPos':jointPos,'baseLinVel':baseLinVel,'baseAngVel':baseAngVel}


    def change_normed_obs(self,raw_obs,perturbed_raw_obs=None):
        normed_obs = raw_obs.copy()
        normed_obs[:,8:20] = (normed_obs[:,8:20]-self.obs_mean)/np.sqrt(self.obs_var)
        return normed_obs
        

    def on_visualize(self,record=False,video_flag = None):
        if self.cfg['environment']['render']:
            self.env.turn_on_visualization()
            if record:
                if video_flag is None:
                    vid_name = self.cfg['video_header']+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "_MPC_"+'.mp4'
                else:
                    vid_name = self.cfg['video_header']+video_flag+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "_MPC_"+".mp4"
                self.env.start_video_recording(vid_name)

    def off_visualize(self,record=False):
        if self.cfg['environment']['render']:
            self.env.turn_off_visualization()
            if record:
                self.env.stop_video_recording()


    def normalize_target(self,action):
        # update normalization
        normed_action = action.copy()
        normed_action[:,:12] = (action[:,:12]-self.joint_pos_mean)/np.sqrt(self.joint_pos_var)
        normed_action[:,12:24] = (action[:,12:24]-self.ff_torque_mean)/np.sqrt(self.ff_torque_var)
        return normed_action


    def process_rl_action(self,rl_action):
        processed_rl_action = rl_action.copy()
        processed_rl_action[:,:12] = rl_action[:,:12]*np.sqrt(self.joint_pos_var)+self.joint_pos_mean
        processed_rl_action[:,12:24] = rl_action[:,12:24]*np.sqrt(self.ff_torque_var)+self.ff_torque_mean
        return processed_rl_action.astype(np.float32)

    def append_gains(self,action,gait_generator):
        kps = np.zeros((self.num_env,12))
        kds = np.zeros((self.num_env,12))
        for j in range(self.num_env):
            for leg_id in range(4):
                if gait_generator[j].desired_leg_state[leg_id] == gait_generator_lib.LegState.SWING:
                    kps[j,3*leg_id:3*(leg_id+1)] = [a1.ABDUCTION_P_GAIN,a1.HIP_P_GAIN,a1.KNEE_P_GAIN]
                    kds[j,3*leg_id:3*(leg_id+1)] = [a1.ABDUCTION_D_GAIN,a1.HIP_D_GAIN,a1.KNEE_D_GAIN]
        action[:,24:36] = kps
        action[:,36:] = kds
        return action

