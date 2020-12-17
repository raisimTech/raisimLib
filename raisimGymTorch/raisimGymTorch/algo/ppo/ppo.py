from datetime import datetime
import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
import numpy as np
from .storage import RolloutStorage


class PPO:
    def __init__(self,
                 actor,
                 critic,
                 num_envs,
                 num_transitions_per_env,
                 num_learning_epochs,
                 num_mini_batches,
                 clip_param=0.2,
                 gamma=0.998,
                 lam=0.95,
                 value_loss_coef=0.5,
                 entropy_coef=0.0,
                 learning_rate=5e-4,
                 max_grad_norm=0.5,
                 use_clipped_value_loss=True,
                 log_dir='run',
                 device='cpu',
                 mini_batch_sampling='shuffle',
                 log_intervals=10):

        # PPO components
        self.actor = actor
        self.critic = critic
        self.storage = RolloutStorage(num_envs, num_transitions_per_env, actor.obs_shape, critic.obs_shape, actor.action_shape, device)

        if mini_batch_sampling == 'shuffle':
            self.batch_sampler = self.storage.mini_batch_generator_shuffle
        elif mini_batch_sampling == 'in_order':
            self.batch_sampler = self.storage.mini_batch_generator_inorder
        else:
            raise NameError(mini_batch_sampling + ' is not a valid sampling method. Use one of the followings: shuffle, order')

        self.optimizer = optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=learning_rate)
        self.device = device

        # env parameters
        self.num_transitions_per_env = num_transitions_per_env
        self.num_envs = num_envs

        # PPO parameters
        self.clip_param = clip_param
        self.num_learning_epochs = num_learning_epochs
        self.num_mini_batches = num_mini_batches
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.gamma = gamma
        self.lam = lam
        self.max_grad_norm = max_grad_norm
        self.use_clipped_value_loss = use_clipped_value_loss

        # Log
        self.log_dir = os.path.join(log_dir, datetime.now().strftime('%b%d_%H-%M-%S'))
        self.writer = SummaryWriter(log_dir=self.log_dir, flush_secs=10)
        self.tot_timesteps = 0
        self.tot_time = 0
        self.ep_infos = []
        self.log_intervals = log_intervals

        # temps
        self.actions = None
        self.actions_log_prob = None
        self.actor_obs = None

    def observe(self, actor_obs):
        self.actor_obs = actor_obs
        self.actions, self.actions_log_prob = self.actor.sample(torch.from_numpy(actor_obs).to(self.device))
        # self.actions = np.clip(self.actions.numpy(), self.env.action_space.low, self.env.action_space.high)
        return self.actions.cpu().numpy()

    def step(self, value_obs, rews, dones, infos):
        values = self.critic.predict(torch.from_numpy(value_obs).to(self.device))
        self.storage.add_transitions(self.actor_obs, value_obs, self.actions, rews, dones, values,
                                     self.actions_log_prob)

        # Book keeping
        for info in infos:
            ep_info = info.get('episode')
            if ep_info is not None:
                self.ep_infos.append(ep_info)

    def update(self, actor_obs, value_obs, log_this_iteration, update):
        last_values = self.critic.predict(torch.from_numpy(value_obs).to(self.device))

        # Learning step
        self.storage.compute_returns(last_values.to(self.device), self.gamma, self.lam)
        mean_value_loss, mean_surrogate_loss, infos = self._train_step()
        self.storage.clear()
        # stop = time.time()

        # if log_this_iteration:
        #     self.log({**locals(), **infos, 'ep_infos': self.ep_infos, 'it': update})

        self.ep_infos.clear()

    def log(self, variables, width=80, pad=28):
        self.tot_timesteps += self.num_transitions_per_env * self.num_envs

        ep_string = f''
        for key in variables['ep_infos'][0]:
            value = np.mean([ep_info[key] for ep_info in variables['ep_infos']])
            self.writer.add_scalar('Episode/' + key, value, variables['it'])
            ep_string += f"""{f'Mean episode {key}:':>{pad}} {value:.4f}\n"""
        mean_std = self.actor.distribution.log_std.exp().mean()

        self.writer.add_scalar('Loss/value_function', variables['mean_value_loss'], variables['it'])
        self.writer.add_scalar('Loss/surrogate', variables['mean_surrogate_loss'], variables['it'])
        self.writer.add_scalar('Policy/mean_noise_std', mean_std.item(), variables['it'])

        log_string = (f"""{'#' * width}\n"""
                      f"""{'Value function loss:':>{pad}} {variables['mean_value_loss']:.4f}\n"""
                      f"""{'Surrogate loss:':>{pad}} {variables['mean_surrogate_loss']:.4f}\n"""
                      f"""{'Mean action noise std:':>{pad}} {mean_std.item():.2f}\n""")
        log_string += ep_string

        print(log_string)

    def _train_step(self):
        mean_value_loss = 0
        mean_surrogate_loss = 0
        for epoch in range(self.num_learning_epochs):
            for actor_obs_batch, critic_obs_batch, actions_batch, target_values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch \
                    in self.batch_sampler(self.num_mini_batches):

                actions_log_prob_batch, entropy_batch = self.actor.evaluate(actor_obs_batch, actions_batch)
                value_batch = self.critic.evaluate(critic_obs_batch)

                # Surrogate loss
                ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
                surrogate = -torch.squeeze(advantages_batch) * ratio
                surrogate_clipped = -torch.squeeze(advantages_batch) * torch.clamp(ratio, 1.0 - self.clip_param,
                                                                                   1.0 + self.clip_param)
                surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()

                # Value function loss
                if self.use_clipped_value_loss:
                    value_clipped = target_values_batch + (value_batch - target_values_batch).clamp(-self.clip_param,
                                                                                                    self.clip_param)
                    value_losses = (value_batch - returns_batch).pow(2)
                    value_losses_clipped = (value_clipped - returns_batch).pow(2)
                    value_loss = torch.max(value_losses, value_losses_clipped).mean()
                else:
                    value_loss = (returns_batch - value_batch).pow(2).mean()

                loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean()

                # Gradient step
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_([*self.actor.parameters(), *self.critic.parameters()], self.max_grad_norm)
                self.optimizer.step()

                mean_value_loss += value_loss.item()
                mean_surrogate_loss += surrogate_loss.item()

        num_updates = self.num_learning_epochs * self.num_mini_batches
        mean_value_loss /= num_updates
        mean_surrogate_loss /= num_updates

        return mean_value_loss, mean_surrogate_loss, locals()
