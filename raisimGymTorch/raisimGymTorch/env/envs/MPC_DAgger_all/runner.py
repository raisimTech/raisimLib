from curses.ascii import ctrl
from email.policy import default
from ruamel.yaml import YAML, dump, RoundTripDumper
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
import wandb
import random
#######################my import###################################
from raisimGymTorch.env.bin.MPC_DAgger_all import RaisimGymEnv
from raisimGymTorch.env.bin.MPC_DAgger_all import NormalSampler
from raisimGymTorch.mpc.mpc_controller_DAgger_all.control import Controller
from raisimGymTorch.mpc.mpc_controller_DAgger_all import control
from raisimGymTorch.env.envs.MPC_DAgger_all.my_helper import MyHelper
from raisimGymTorch.algo.DAgger_all import module
from raisimGymTorch.algo.DAgger_all.replay_buffer import ReplayBuffer
from raisimGymTorch.algo.DAgger_all.rl_trainer import SupTrainer
from copy import copy, deepcopy
from torch.utils.tensorboard import SummaryWriter
###################################################################



######################################################################################################################################
######################################################################################################################################
## configuration and initial setting
######################################################################################################################################
######################################################################################################################################
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
weight_path = args.weight

task_name = "MPC_DAgger_all"
algo_name = "DAgger_all"
mpc_name = "mpc_controller_DAgger_all"

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
wandb.init(project="MPC_guide",group="1129",job_type="train",name="MPC_DAgger_all",
            notes="Train with DAgger action space is controller"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
            
wandb.save(task_path+"/*")
wandb.save(mpc_path+"/*")
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
rl_obs_dim = 4+12+3+3+1+1+4 # foot contact, joint pose, body vel, body ang vel, height, body z angle,phase in full cyle
rl_act_dim = 12*2 # joint pos, joint torque
buffer_capacity = 1000000
actor = module.Actor(module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, rl_obs_dim, rl_act_dim),
                        module.MultivariateGaussianDiagonalCovariance(rl_act_dim,
                                                                        env.num_envs,
                                                                        0.5,
                                                                        NormalSampler(rl_act_dim),
                                                                        cfg['seed']),
                    device)
critic = module.Critic(module.MLP(cfg['architecture']['value_net'], nn.LeakyReLU, rl_obs_dim, 1),
                           device)
replay_buffer = ReplayBuffer(buffer_capacity,rl_obs_dim,rl_act_dim)
learner = SupTrainer(actor,critic,replay_buffer,learning_rate=5e-3,device=device)

# Control constant
ctrl_time_step = cfg['environment']['control_dt']
ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD

# etc constant
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs
maxIter = 100000
mpcDecimation = cfg['environment']['eval_every_n']
batch_size = 2*buffer_capacity//mpcDecimation
num_peripheral = 4
my_helper = MyHelper(env,cfg,saver,rl_obs_dim,rl_act_dim,n_steps, \
                    control._STANCE_DURATION_SECONDS,control._DUTY_FACTOR,control._INIT_PHASE_FULL_CYCLE,control._INIT_LEG_STATE)
log_interval = 20
data_amount = 0
record = cfg["record_video"]

# train
for iter in range(maxIter+1):
    if iter % cfg['environment']['eval_every_n'] == 0:
        my_helper.save_model(actor,learner,iter)
        my_helper.on_visualize(update=iter,record=record,istest=False)
        loaded_graph = module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, rl_obs_dim, rl_act_dim)
        loaded_graph.load_state_dict(torch.load(saver.data_dir+"/full_"+str(iter)+'.pt')['actor_architecture_state_dict'])
        env.reset()
        my_helper.simulate_evaluate(update=iter,loaded_graph=loaded_graph)
        my_helper.off_visualize(record=record)
    alpha = np.clip(1-iter/maxIter,0.1,1.0)
    # new data generation
    if iter%mpcDecimation == 0:
        start_time = time.time()
        my_helper.on_visualize(update=iter,record=record,istest=False,video_flag="_datacollection_")
        # reset env and set controller
        env.reset()
        init_raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
        init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
        init_joint_angle = init_ctrl_obs['jointPos']
        controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)
        for step in range(n_steps):
            # get observations
            raw_obs,_ = env.observe(update_statistics=False, isnoise = False)
            normed_obs = my_helper.change_normed_obs(raw_obs) # make leg state -1~1            
            phase_in_full_cycle = my_helper.get_phase_in_full_cycle(ctrl_time_step*step-controller.reset_time) # (num_env,4)

            # perturbed data collect
            for _ in range(num_peripheral):
                tmp_controller = deepcopy(controller)
                perturbed_raw_obs,_ = env.observe(update_statistics=False, isnoise=True) # get perturbed observation
                perturbed_normed_obs = my_helper.change_normed_obs(raw_obs,perturbed_raw_obs) # get desired leg state for rl obs
                perturbed_rl_obs = MyHelper.get_rl_obs(perturbed_normed_obs,phase_in_full_cycle,num_env)
                perturbed_ctrl_obs = MyHelper.get_ctrl_obs(perturbed_raw_obs)
                perturbed_u_mpc = tmp_controller.get_mpc_contact_force(perturbed_ctrl_obs)
                perturbed_mpc_action = tmp_controller.control_with_conactForce(perturbed_u_mpc,perturbed_ctrl_obs,step).reshape(-1,ctrl_act_dim)
                perturbed_mpc_action_normed = my_helper.normalize_target(perturbed_mpc_action)                
                learner.step(perturbed_rl_obs,perturbed_mpc_action_normed[:,:24],num_env)
                
            # data collect
            rl_obs = MyHelper.get_rl_obs(normed_obs,phase_in_full_cycle,num_env) # get rl obs
            ctrl_obs = MyHelper.get_ctrl_obs(raw_obs) # get control obs
            u_mpc = controller.get_mpc_contact_force(ctrl_obs)
            mpc_action = controller.control_with_conactForce(u_mpc,ctrl_obs,step).reshape(-1,ctrl_act_dim)
            mpc_action_normed = my_helper.normalize_target(mpc_action)
            learner.step(rl_obs,mpc_action_normed[:,:24],num_env)
            # next step
            rl_action = actor.noiseless_action(rl_obs).cpu().detach().numpy() # deterministic policy
            rl_action_control = my_helper.process_rl_action(rl_action) # unnormalize
            behavior_action = alpha*mpc_action[:,:24]+(1-alpha)*rl_action_control
            behavior_action = np.append(behavior_action,mpc_action[:,24:],axis = 1)
            reward, dones = env.step(behavior_action,step)
            for i in range(num_env):
                if dones[i]:
                    init_raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
                    init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
                    init_joint_angle = init_ctrl_obs['jointPos']
                    controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)

            step_data = {"step":step+(iter/mpcDecimation)*n_steps}
            for leg_num in range(4):
                for joint_num in range(3):
                    step_data["RL_leg"+str(leg_num+1)+"_joint"+str(joint_num+1)] = rl_action_control[0][3*leg_num+joint_num]
            for leg_num in range(4):
                for joint_num in range(3):
                    step_data["RL_leg"+str(leg_num+1)+"_torque"+str(joint_num+1)] = rl_action_control[0][3*leg_num+joint_num+12]
            for leg_num in range(4):
                for joint_num in range(3):
                    step_data["MPC_leg"+str(leg_num+1)+"_joint"+str(joint_num+1)] = mpc_action[0][3*leg_num+joint_num]
            for leg_num in range(4):
                for joint_num in range(3):
                    step_data["MPC_leg"+str(leg_num+1)+"_torque"+str(joint_num+1)] = mpc_action[0][3*leg_num+joint_num+12]
            
            # for leg_num in range(4):
            #     for joint_num in range(3):
            #         step_data["(normed)RL_leg"+str(leg_num+1)+"_joint"+str(joint_num+1)] = rl_action[0][3*leg_num+joint_num]
            # for leg_num in range(4):
            #     for joint_num in range(3):
            #         step_data["(normed)RL_leg"+str(leg_num+1)+"_torque"+str(joint_num+1)] = rl_action[0][3*leg_num+joint_num+12]
            # for leg_num in range(4):
            #     for joint_num in range(3):
            #         step_data["(normed)MPC_leg"+str(leg_num+1)+"_joint"+str(joint_num+1)] = mpc_action_normed[0][3*leg_num+joint_num]
            # for leg_num in range(4):
            #     for joint_num in range(3):
            #         step_data["(normed)MPC_leg"+str(leg_num+1)+"_torque"+str(joint_num+1)] = mpc_action_normed[0][3*leg_num+joint_num+12]

            wandb.log(step_data)

        my_helper.off_visualize(record=record)   
        print("time to generate data: {:6.4f}".format(time.time()-start_time))
    
    rl_obs_batch,u_mpc_batch = replay_buffer.sample(batch_size)
    loss = learner.train(rl_obs_batch,u_mpc_batch)
    data_amount += batch_size
    if iter%log_interval:
        wandb.log({"loss": loss,"data_amount":data_amount, "iter":iter})