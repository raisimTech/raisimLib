from curses.ascii import ctrl
from email.policy import default
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin.MPC_DAgger import RaisimGymEnv
from raisimGymTorch.env.bin.MPC_DAgger import NormalSampler
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param
import os
import math
import time
import torch.nn as nn
import numpy as np
import torch
import datetime
import argparse
#######################my import###################################
import wandb
import random
from raisimGymTorch.mpc.mpc_controller_DAgger.control import Controller
from raisimGymTorch.env.envs.MPC_DAgger.my_helper import MyHelper
from raisimGymTorch.algo.DAgger import module
from raisimGymTorch.algo.DAgger.replay_buffer import ReplayBuffer
from raisimGymTorch.algo.DAgger.rl_trainer import SupTrainer
from torch.utils.tensorboard import SummaryWriter
###################################################################



######################################################################################################################################
######################################################################################################################################
## configuration and initial setting
######################################################################################################################################
######################################################################################################################################
writer = SummaryWriter()

parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
weight_path = args.weight

task_name = "MPC_DAgger"
algo_name = "DAgger"
mpc_name = "mpc_controller_DAgger"

# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."
algo_path = home_path+"/raisimGymTorch/raisimGymTorch/algo/"+algo_name
env_path = home_path+"/raisimGymTorch/raisimGymTorch/env/"
mpc_path = home_path+"/raisimGymTorch/raisimGymTorch/mpc/"+mpc_name

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# wandb initialization
wandb.init(project="MPC_guide",group="1101",job_type="train",name="MPC_DAgger",
            notes="Train with DAgger"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
            
wandb.save(task_path+"/*")
wandb.save(task_path+"/MPC_controller/*")
wandb.save(algo_path+"/*")
wandb.save(env_path+"/*")

saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])   


##############################################################################################################################
##############################################################################################################################
# Object generation(env, controller, rl) and constant define
##############################################################################################################################
##############################################################################################################################

# Environment object
env = VecEnv(RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)))

env_obs_dim = env.num_obs
env_act_dim = env.num_acts
sim_time_step = cfg['environment']['simulation_dt']
num_env = cfg['environment']['num_envs']


# RL object
rl_obs_dim = env.num_obs + 4 #(add desired leg state)
rl_act_dim = 12 # contact forces
buffer_capacity = 1000000
actor = module.Actor(module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, rl_obs_dim, rl_act_dim),
                        module.MultivariateGaussianDiagonalCovariance(rl_act_dim,
                                                                        env.num_envs,
                                                                        0.5,
                                                                        NormalSampler(rl_act_dim),
                                                                        cfg['seed']),
                    device)

replay_buffer = ReplayBuffer(buffer_capacity,rl_obs_dim,rl_act_dim)
learner = SupTrainer(actor,replay_buffer,learning_rate=5e-3,device=device)

# Control constant
ctrl_time_step = cfg['environment']['control_dt']
ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD

# etc constant
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs
maxIter = 100000
mpcDecimation = 500
batch_size = 2*buffer_capacity//mpcDecimation
num_peripheral = 4
my_helper = MyHelper(env,cfg,saver,rl_obs_dim,rl_act_dim,n_steps)
log_interval = 20


# train
for iter in range(maxIter):
    if iter % cfg['environment']['eval_every_n'] == 0:
        my_helper.save_model(actor,learner,iter)
    alpha = np.clip(1-iter/maxIter,0.2,1.0)
    # new data generation
    if iter%mpcDecimation == 0:
        start_time = time.time()
        my_helper.on_visualize(update=iter,istest=False)
        
        # reset env and set controller
        env.reset()
        init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
        init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
        init_joint_angle = init_ctrl_obs['jointPos']
        controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)
        for step in range(n_steps):
            # data collect
            raw_obs,_ = env.observe(update_statistics=False, isnoise = False)
            normed_obs = my_helper.change_normed_obs(raw_obs) # make leg state -1~1            
            desired_leg_state = controller.get_desired_leg_state() # get desired leg state for rl obs
            desired_leg_state = MyHelper.change_foot_contact_range(desired_leg_state) # make desired leg state -1~1
            rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state) # get rl obs
            ctrl_obs = MyHelper.get_ctrl_obs(raw_obs) # get control obs
            u_mpc = controller.get_mpc_contact_force(ctrl_obs).reshape(-1,rl_act_dim) # get expert label
            u_mpc_normed = my_helper.normalize_target(u_mpc)
            learner.step(rl_obs,u_mpc_normed,num_env) # add transition
            # perturbed data collect
            for _ in range(num_peripheral):
                perturbed_raw_obs,perturbed_normed_obs = env.observe(update_statistics=False, isnoise=True) # get perturbed observation
                perturbed_normed_obs = my_helper.change_normed_obs(raw_obs,perturbed_raw_obs) # get desired leg state for rl obs
                perturbed_ctrl_obs = MyHelper.get_ctrl_obs(perturbed_raw_obs)
                perturbed_rl_obs = MyHelper.get_rl_obs(perturbed_normed_obs,desired_leg_state)
                perturbed_u_mpc = controller.get_mpc_contact_force(perturbed_ctrl_obs).reshape(-1,rl_act_dim)
                perturbed_u_mpc_normed = my_helper.normalize_target(perturbed_u_mpc)
                learner.step(perturbed_rl_obs,perturbed_u_mpc_normed,num_env)
                
            # next step
            u_rl = actor.noiseless_action(rl_obs).cpu().detach().numpy() # deterministic policy
            u_rl_unnormalized = my_helper.unnormalize(u_rl)
            u_behavior = (alpha*u_mpc+(1-alpha)*u_rl_unnormalized).reshape(-1,4,3)
            env_act = controller.control_with_conactForce(u_behavior,ctrl_obs,step)
            reward, dones = env.step(env_act)                        
            for i in range(num_env):
                if dones[i]:
                    init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
                    init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
                    init_joint_angle = init_ctrl_obs['jointPos']
                    controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)
            
            # log normed joint pos, normed u_mpc, unnormed u_rl
            contact_forces = {"FRx":u_mpc_normed[0][0],"FRy":u_mpc_normed[0][1],"FRz":u_mpc_normed[0][2], \
                        "FLx":u_mpc_normed[0][3],"FLy":u_mpc_normed[0][4],"FLz":u_mpc_normed[0][5],\
                        "RRx":u_mpc_normed[0][6],"RRy":u_mpc_normed[0][7],"RRz":u_mpc_normed[0][8],\
                        "RLx":u_mpc_normed[0][9],"RLy":u_mpc_normed[0][10],"RLz":u_mpc_normed[0][11]}
            u_rl_contact_forces = {"RL_FRx":u_rl_unnormalized[0][0],"RL_FRy":u_rl_unnormalized[0][1],"RL_FRz":u_rl_unnormalized[0][2], \
                        "RL_FLx":u_rl_unnormalized[0][3],"RL_FLy":u_rl_unnormalized[0][4],"RL_FLz":u_rl_unnormalized[0][5],\
                        "RL_RRx":u_rl_unnormalized[0][6],"RL_RRy":u_rl_unnormalized[0][7],"RL_RRz":u_rl_unnormalized[0][8],\
                        "RL_RLx":u_rl_unnormalized[0][9],"RL_RLy":u_rl_unnormalized[0][10],"RL_RLz":u_rl_unnormalized[0][11]}
            u_rl_contact_forces_normed = {"normed_RL_FRx":u_rl[0][0],"normed_RL_FRy":u_rl[0][1],"normed_RL_FRz":u_rl[0][2], \
                        "normed_RL_FLx":u_rl[0][3],"normed_RL_FLy":u_rl[0][4],"normed_RL_FLz":u_rl[0][5],\
                        "normed_RL_RRx":u_rl[0][6],"normed_RL_RRy":u_rl[0][7],"normed_RL_RRz":u_rl[0][8],\
                        "normed_RL_RLx":u_rl[0][9],"normed_RL_RLy":u_rl[0][10],"normed_RL_RLz":u_rl[0][11]}
            step_data = {"hip": normed_obs[0,8],"shoulder": normed_obs[0,9],"knee": normed_obs[0,10],"step":step+(iter/mpcDecimation)*n_steps}
            for key in contact_forces:
                step_data[key] = contact_forces[key]
            for key in u_rl_contact_forces:
                step_data[key] = u_rl_contact_forces[key]
            for key in u_rl_contact_forces_normed:
                step_data[key] = u_rl_contact_forces_normed[key]
            wandb.log(step_data)

        my_helper.off_visualize()   
        print("time to generate data: {:6.4f}".format(time.time()-start_time))
    rl_obs_batch,u_mpc_batch = replay_buffer.sample(batch_size)
    loss = learner.train(rl_obs_batch,u_mpc_batch)
    if iter%log_interval:
        wandb.log({"loss": loss,"iter":iter})

writer.close()


# env.reset()
# init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
# init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
# init_joint_angle = init_ctrl_obs['jointPos']
# controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)

# for step in range(n_steps):
#     raw_obs, normed_obs = env.observe(update_statistics=True, isnoise=False)
#     desired_leg_state = controller.get_desired_leg_state()
#     desired_leg_state = MyHelper.change_foot_contact_range(desired_leg_state)
#     rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state)
#     ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
#     u_rl = actor.noiseless_action(rl_obs).cpu().detach().numpy().reshape(-1,4,3)
#     u_rl[:,:,2] -= 50
#     env_act = controller.control_with_conactForce(u_rl,ctrl_obs,step)
#     reward, dones = env.step(env_act)
#     for i in range(num_env):
#         if dones[i]:
#             init_raw_obs, init_normed_obs = env.observe(update_statistics=True, isnoise=False)
#             init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
#             init_joint_angle = init_ctrl_obs['jointPos']
#             controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)
    

# for step in range(n_steps):
#     raw_obs,normed_obs = env.observe(update_statistics=True, isnoise=False)
#     ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
#     u_mpc = controller.get_mpc_contact_force(ctrl_obs)
#     env_act = controller.control_with_conactForce(u_mpc,ctrl_obs,step)
#     env.step(env_act)