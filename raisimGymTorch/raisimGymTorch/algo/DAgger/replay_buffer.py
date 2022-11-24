import numpy as np
from collections import namedtuple
import random

class ReplayBuffer():
    def __init__(self,capacity,rl_obs_dim,rl_act_dim):
        self.capacity = capacity
        self.rl_obs = np.zeros([self.capacity, rl_obs_dim], dtype=np.float32)
        self.u_mpc = np.zeros([self.capacity, rl_act_dim], dtype=np.float32)
        self.position = 0
        self.size = 0

    def push(self,rl_obs,u_mpc,num_env):
        """Saves a sample."""
        self.size = min(self.size + num_env, self.capacity)
        isoverflow = ((self.position+num_env)>=self.capacity)
        old_position = self.position
        self.position = (self.position + num_env) % self.capacity

        if isoverflow:
            self.rl_obs[old_position:,:] = rl_obs[:self.capacity-old_position,:]
            self.u_mpc[old_position:,:] = u_mpc[:self.capacity-old_position,:]
            self.rl_obs[:self.position,:] = rl_obs[self.capacity-old_position:,:]
            self.u_mpc[:self.position,:] = u_mpc[self.capacity-old_position:,:]
        else:
            self.rl_obs[old_position:self.position,:] = rl_obs
            self.u_mpc[old_position:self.position,:] = u_mpc
            

    def sample(self, batch_size):
        indices = np.random.choice(self.size,batch_size,replace=False)
        return self.rl_obs[indices,:], self.u_mpc[indices,:]

    def __len__(self):
        return self.size