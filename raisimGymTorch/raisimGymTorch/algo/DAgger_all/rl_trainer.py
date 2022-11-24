import torch.optim as optim
import torch

class SupTrainer():
    def __init__(self,actor,critic,replay_buffer,learning_rate,device):
        self.actor = actor
        self.critic = critic
        self.replay_buffer = replay_buffer
        self.learning_rate = learning_rate
        self.optimizer = optim.Adam(self.actor.parameters(), lr=self.learning_rate)
        # self.scheduler = optim.lr_scheduler.MultiStepLR(self.optimizer,milestones = [1500,5000,10000],gamma=0.5)
        self.device = device
        self.mseloss = torch.nn.MSELoss()
        self.use_clipped_value_loss = True
        self.clip_param = 0.2

    def step(self,rl_obs,u_mpc,num_env):
        self.replay_buffer.push(rl_obs,u_mpc,num_env)
    
    def train(self,rl_obs_batch,u_mpc_batch):
        self.optimizer.zero_grad()
        pred_actions = self.actor.noiseless_action(rl_obs_batch)
        mseloss = self.mseloss(pred_actions,torch.from_numpy(u_mpc_batch).to(self.device))
        loss = mseloss
        loss.backward()
        self.optimizer.step()
        return loss.to('cpu').detach().numpy()