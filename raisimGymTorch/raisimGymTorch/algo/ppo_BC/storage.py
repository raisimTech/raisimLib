import torch
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
import numpy as np


class RolloutStorage:
    def __init__(self, num_envs, num_transitions_per_env, actor_obs_shape, critic_obs_shape, actions_shape, device):
        self.device = device

        # Core
        self.critic_obs = np.zeros([num_transitions_per_env, num_envs, *critic_obs_shape], dtype=np.float32)
        self.actor_obs = np.zeros([num_transitions_per_env, num_envs, *actor_obs_shape], dtype=np.float32)
        self.rewards = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.actions = np.zeros([num_transitions_per_env, num_envs, *actions_shape], dtype=np.float32)
        self.dones = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.bool)

        # For PPO
        self.actions_log_prob = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.values = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.returns = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.advantages = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.mu = np.zeros([num_transitions_per_env, num_envs, *actions_shape], dtype=np.float32)
        self.sigma = np.zeros([num_transitions_per_env, num_envs, *actions_shape], dtype=np.float32)

        # torch variables
        self.critic_obs_tc = torch.from_numpy(self.critic_obs).to(self.device)
        self.actor_obs_tc = torch.from_numpy(self.actor_obs).to(self.device)
        self.actions_tc = torch.from_numpy(self.actions).to(self.device)
        self.actions_log_prob_tc = torch.from_numpy(self.actions_log_prob).to(self.device)
        self.values_tc = torch.from_numpy(self.values).to(self.device)
        self.returns_tc = torch.from_numpy(self.returns).to(self.device)
        self.advantages_tc = torch.from_numpy(self.advantages).to(self.device)
        self.mu_tc = torch.from_numpy(self.mu).to(self.device)
        self.sigma_tc = torch.from_numpy(self.sigma).to(self.device)

        self.num_transitions_per_env = num_transitions_per_env
        self.num_envs = num_envs
        self.device = device

        self.step = 0

    def add_transitions(self, actor_obs, critic_obs, actions, mu, sigma, rewards, dones, actions_log_prob):
        if self.step >= self.num_transitions_per_env:
            raise AssertionError("Rollout buffer overflow")
        self.critic_obs[self.step] = critic_obs
        self.actor_obs[self.step] = actor_obs
        self.actions[self.step] = actions
        self.mu[self.step] = mu
        self.sigma[self.step] = sigma
        self.rewards[self.step] = rewards.reshape(-1, 1)
        self.dones[self.step] = dones.reshape(-1, 1)
        self.actions_log_prob[self.step] = actions_log_prob.reshape(-1, 1)
        self.step += 1

    def add_mpc_data(self,mpc_obs,mpc_act):
        self.mpc_obs = mpc_obs.astype(np.float32)
        self.mpc_act = mpc_act.astype(np.float32)
        self.mpc_obs_tc = torch.from_numpy(self.mpc_obs).to(self.device)
        self.mpc_act_tc = torch.from_numpy(self.mpc_act).to(self.device)
        self.mpc_batch_size = mpc_obs.shape[0]*mpc_obs.shape[1]

    def clear(self):
        self.step = 0

    def compute_returns(self, last_values, critic, gamma, lam):
        with torch.no_grad():
            self.values = critic.predict(torch.from_numpy(self.critic_obs).to(self.device)).cpu().numpy()

        advantage = 0
        # GAE
        for step in reversed(range(self.num_transitions_per_env)):
            if step == self.num_transitions_per_env - 1:
                next_values = last_values.cpu().numpy()
                # next_is_not_terminal = 1.0 - self.dones[step].float()
            else:
                next_values = self.values[step + 1]
                # next_is_not_terminal = 1.0 - self.dones[step+1].float()

            next_is_not_terminal = 1.0 - self.dones[step]
            delta = self.rewards[step] + next_is_not_terminal * gamma * next_values - self.values[step]
            advantage = delta + next_is_not_terminal * gamma * lam * advantage
            self.returns[step] = advantage + self.values[step]

        # Compute and normalize the advantages
        self.advantages = self.returns - self.values
        self.advantages = (self.advantages - self.advantages.mean()) / (self.advantages.std() + 1e-8)

        # Convert to torch variables
        self.critic_obs_tc = torch.from_numpy(self.critic_obs).to(self.device)
        self.actor_obs_tc = torch.from_numpy(self.actor_obs).to(self.device)
        self.actions_tc = torch.from_numpy(self.actions).to(self.device)
        self.actions_log_prob_tc = torch.from_numpy(self.actions_log_prob).to(self.device)
        self.values_tc = torch.from_numpy(self.values).to(self.device)
        self.returns_tc = torch.from_numpy(self.returns).to(self.device)
        self.advantages_tc = torch.from_numpy(self.advantages).to(self.device)
        self.sigma_tc = torch.from_numpy(self.sigma).to(self.device)
        self.mu_tc = torch.from_numpy(self.mu).to(self.device)
        
    def mini_batch_generator_shuffle(self, num_mini_batches):
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches
        mpc_mini_batch_size = self.mpc_batch_size // num_mini_batches
        assert mpc_mini_batch_size*num_mini_batches == self.mpc_batch_size, "MPC data are not fully used"
        batch_indices = list(BatchSampler(SubsetRandomSampler(range(batch_size)), mini_batch_size, drop_last=True))
        mpc_batch_indices = list(BatchSampler(SubsetRandomSampler(range(self.mpc_batch_size)), mpc_mini_batch_size, drop_last=True))
        assert len(batch_indices) == len(mpc_batch_indices), "length of mpc batch and RL batch is different!"

        for idx_indices in range(len(batch_indices)):
            indices = batch_indices[idx_indices]
            mpc_indices = mpc_batch_indices[idx_indices]
            actor_obs_batch = self.actor_obs_tc.view(-1, *self.actor_obs_tc.size()[2:])[indices]
            critic_obs_batch = self.critic_obs_tc.view(-1, *self.critic_obs_tc.size()[2:])[indices]
            actions_batch = self.actions_tc.view(-1, self.actions_tc.size(-1))[indices]
            sigma_batch = self.sigma_tc.view(-1, self.sigma_tc.size(-1))[indices]
            mu_batch = self.mu_tc.view(-1, self.mu_tc.size(-1))[indices]
            values_batch = self.values_tc.view(-1, 1)[indices]
            returns_batch = self.returns_tc.view(-1, 1)[indices]
            old_actions_log_prob_batch = self.actions_log_prob_tc.view(-1, 1)[indices]
            advantages_batch = self.advantages_tc.view(-1, 1)[indices]
            mpc_obs_batch = self.mpc_obs_tc.view(-1,*self.mpc_obs_tc.size()[2:])[mpc_indices]
            mpc_act_batch = self.mpc_act_tc.view(-1,*self.mpc_act_tc.size()[2:])[mpc_indices]
            yield actor_obs_batch, critic_obs_batch, actions_batch, sigma_batch, mu_batch, values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch,\
                mpc_obs_batch,mpc_act_batch

    def mini_batch_generator_inorder(self, num_mini_batches):
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches
        mpc_mini_batch_size = self.mpc_batch_size // num_mini_batches
        assert mpc_mini_batch_size*num_mini_batches == self.mpc_batch_size, "MPC data are not fully used"

        for batch_id in range(num_mini_batches):
            yield self.actor_obs_tc.view(-1, *self.actor_obs_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.critic_obs_tc.view(-1, *self.critic_obs_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.actions_tc.view(-1, self.actions_tc.size(-1))[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.sigma_tc.view(-1, self.sigma_tc.size(-1))[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.mu_tc.view(-1, self.mu_tc.size(-1))[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.values_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.advantages_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.returns_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.actions_log_prob_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size],\
                self.mpc_obs_tc.view(-1,*self.mpc_obs_tc.size()[2:])[batch_id*mpc_mini_batch_size:(batch_id+1)*mpc_mini_batch_size],\
                self.mpc_act_tc.view(-1,*self.mpc_act_tc.size()[2:])[batch_id*mpc_mini_batch_size:(batch_id+1)*mpc_mini_batch_size]

    # def mini_batch_generator_shuffle(self, num_mini_batches,mpc_obs=None,mpc_acts=None,mpc_batch_size=None):
    #     batch_size = self.num_envs * self.num_transitions_per_env            
    #     mini_batch_size = batch_size // num_mini_batches
    #     assert mpc_obs.shape[0] == mpc_batch_size, "mpc data size is wrong!"
    #     mpc_mini_batch_size = mpc_batch_size // num_mini_batches
    #     mpc_obs_tc = torch.from_numpy(mpc_obs).to(self.device)
    #     mpc_acts_tc = torch.from_numpy(mpc_acts).to(self.device)
    #     batch_indices = list(BatchSampler(SubsetRandomSampler(range(batch_size)), mini_batch_size, drop_last=True))
    #     mpc_batch_indices = list(BatchSampler(SubsetRandomSampler(range(mpc_batch_size)), mpc_mini_batch_size, drop_last=True))
    #     assert len(batch_indices) == len(mpc_batch_indices), "length of mpc batch and RL batch is different!"
    #     for idx_indices in range(len(batch_indices)):
    #         actor_obs_batch = self.actor_obs_tc.view(-1, *self.actor_obs_tc.size()[2:])[batch_indices[idx_indices]]
    #         critic_obs_batch = self.critic_obs_tc.view(-1, *self.critic_obs_tc.size()[2:])[batch_indices[idx_indices]]
    #         actions_batch = self.actions_tc.view(-1, self.actions_tc.size(-1))[batch_indices[idx_indices]]
    #         sigma_batch = self.sigma_tc.view(-1, self.sigma_tc.size(-1))[batch_indices[idx_indices]]
    #         mu_batch = self.mu_tc.view(-1, self.mu_tc.size(-1))[batch_indices[idx_indices]]
    #         values_batch = self.values_tc.view(-1, 1)[batch_indices[idx_indices]]
    #         returns_batch = self.returns_tc.view(-1, 1)[batch_indices[idx_indices]]
    #         old_actions_log_prob_batch = self.actions_log_prob_tc.view(-1, 1)[batch_indices[idx_indices]]
    #         advantages_batch = self.advantages_tc.view(-1, 1)[batch_indices[idx_indices]]
    #         mpc_obs_batch = mpc_obs_tc.view(-1,mpc_obs_tc.size()[-1])[mpc_batch_indices[idx_indices]]
    #         mpc_acts_batch = mpc_acts_tc.view(-1,mpc_acts_tc.size()[-1])[mpc_batch_indices[idx_indices]]
    #         yield actor_obs_batch, critic_obs_batch, actions_batch, sigma_batch, mu_batch, values_batch, \
    #             advantages_batch, returns_batch, old_actions_log_prob_batch,mpc_obs_batch, mpc_acts_batch