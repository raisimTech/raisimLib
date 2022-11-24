from turtle import update
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import MPC_BC_baseline_no_randinit
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo_BC_baseline.module as ppo_module
import os
import math
import time
import torch
import argparse
#######################my import###################################
import wandb
import datetime
import numpy as np
from raisimGymTorch.mpc_BC_baseline_no_randinit import control
###################################################################


# configuration
weight_path_arg = '/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/data/MPC_BC_baseline_no_randinit/2022-10-15-17-50-50/full_2200.pt'
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='trained weight path', type=str, \
    default=weight_path_arg)
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1

################# my wandb #####################################
wandb.init(project="MPC_guide",group="1017",job_type="test",name="baseline",notes="baseline"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
################################################################

env = VecEnv(MPC_BC_baseline_no_randinit.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])


# shortcuts
control_obs_dim = env.num_obs
control_act_dim = env.num_acts
rl_obs_dim = env.num_obs
rl_act_dim = 12 # leg times 3 dim
num_threads = cfg['environment']['num_threads']
time_step = cfg['environment']['simulation_dt']
ctr_time_step = cfg['environment']['control_dt']
num_env = cfg['environment']['num_envs']
video_header = "BC"
F_x_mean = -2.5
F_z_mean_R = -40
F_z_mean_F = -20
F_y_mean = 0
F_std = 50

weight_path = args.weight
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'

if weight_path == "":
    print("Can't find trained weight, please provide a trained weight with --weight switch\n")

# simulation
else:
    print("Loaded weight from {}\n".format(weight_path))
    env.reset()
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.
    n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
    total_steps = n_steps * 1
    start_step_id = 0
    record_config = []

    print("Visualizing and evaluating the policy: ", weight_path)

    # load actor
    loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, rl_obs_dim, rl_act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

    # load sampler
    loaded_sampler = ppo_module.MultivariateGaussianDiagonalCovariance(rl_act_dim,env.num_envs,1.0,MPC_BC_baseline_no_randinit.NormalSampler(rl_act_dim),cfg['seed'])
    loaded_sampler.load_state_dict(torch.load(weight_path)['actor_distribution_state_dict'])

    # load observation normalizing scale
    env.load_scaling(weight_dir, int(iteration_number))

    # recording
    record_start = time.time()
    env.turn_on_visualization()
    vid_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_test"+'.mp4'
    env.start_video_recording(vid_name)

    # initialize controller
    control_obs = env.observe(False,True)
    controller = control.Controller(control_obs,ctr_time_step,control_act_dim,num_env)

    # actual steps
    for step in range(total_steps*2):
        
        frame_start = time.time()

        rl_obs = env.observe(update_statistics=False,israw=False)
        control_obs = env.observe(update_statistics=False,israw=True)

        # no sampling version
        rl_act = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()

        ## sampling version
        # action_mean = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu())
        # rl_act, _ = loaded_sampler.sample(action_mean.detach().numpy())

        rl_act = rl_act.reshape((num_env,4,3))*F_std
        rl_act[:,:,0]+=F_x_mean
        rl_act[:,:,1]+=F_y_mean
        rl_act[:,:2,2]+=F_z_mean_F
        rl_act[:,2:4,2]+=F_z_mean_R
        simulate_control_act = controller.control_with_action(rl_act,control_obs,step)
        simulate_control_act = simulate_control_act.astype(np.float32)
        reward_ll, dones = env.step(simulate_control_act)
        for i in range(num_env):
            if dones[i]:
                control_obs = env.observe(update_statistics=False,israw=True)
                joint_pos_init_obs = control_obs[i,8:20]
                controller.reset(ctr_time_step*(step+1),joint_pos_init_obs,i)

        frame_end = time.time()
        wait_time = cfg['environment']['control_dt'] - (frame_end - frame_start)
        if wait_time > 0.:
            time.sleep(wait_time)
        else:
            print("Calculation delayed about %fs. It might make it look slower than its control"%(-wait_time))

        ## record velocity and dones
        if not dones and step%5==0:
            obs_raw = env.observe(False,True)
            wandb.log({"Forward velocity": obs_raw[0][16], "Total velocity": \
                np.linalg.norm(obs_raw[0][16:19]),"yawing_velocity": abs(obs_raw[0][21]),"step": step})
        if dones or step == total_steps - 1:
            # ############################################3 my comment ################################################
            print('----------------------------------------------------')
            print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(reward_ll_sum / (step + 1 - start_step_id))))
            print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * cfg['environment']['control_dt'])))
            print('----------------------------------------------------\n')
            # #########################################################################################################
            wandb.log({"dones":dones,"step": step})
            start_step_id = step + 1
            reward_ll_sum = 0.0

    # recording end and reset
    env.turn_off_visualization()
    env.stop_video_recording()
    env.reset()
    ######################## my record to wandb ################################################
    time.sleep(20)
    wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name, caption="test", fps=4, format="mp4")})
    ############################################################################################
    print("Finished at the maximum visualization steps")