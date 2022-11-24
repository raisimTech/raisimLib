from turtle import update
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import MPC_DAgger
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.DAgger.module as module
import os
import math
import time
import torch
import argparse
#######################my import###################################
import wandb
import datetime
import numpy as np
from raisimGymTorch.mpc.mpc_controller_DAgger import control
from raisimGymTorch.helper.my_helper import MyHelper
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver
from raisimGymTorch.mpc.mpc_controller_DAgger.control import Controller
###################################################################


# configuration
weight_path_arg = '/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/data/MPC_DAgger/2022-10-31-01-27-46/full_83000.pt'
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='trained weight path', type=str, \
    default=weight_path_arg)
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg_test.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1
env = VecEnv(MPC_DAgger.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

################# my wandb #####################################
wandb.init(project="MPC_guide",group="1101",job_type="test",name="MPC-DAgger",notes="MPC-DAgger"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
################################################################

# Constants
env_obs_dim = env.num_obs
env_act_dim = env.num_acts
sim_time_step = cfg['environment']['simulation_dt']
num_env = cfg['environment']['num_envs']

rl_obs_dim = env.num_obs + 4 #(add desired leg state)
rl_act_dim = 12 # contact forces

ctrl_time_step = cfg['environment']['control_dt']
ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD

num_threads = cfg['environment']['num_threads']
num_env = cfg['environment']['num_envs']
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * 1

my_helper = MyHelper(env,cfg,None,rl_obs_dim,rl_act_dim,n_steps)

weight_path = args.weight
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'

if weight_path == "":
    print("Can't find trained weight, please provide a trained weight with --weight switch\n")

# simulation
else:
    print("Loaded weight from {}\n".format(weight_path))
    env.reset()
    done_sum = 0
    average_dones = 0.
    start_step_id = 0
    print("Visualizing and evaluating the policy: ", weight_path)

    # # load actor
    # loaded_graph = module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, rl_obs_dim, rl_act_dim)
    # loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

    # # load sampler
    # loaded_sampler = module.MultivariateGaussianDiagonalCovariance(rl_act_dim,env.num_envs,1.0,MPC_DAgger.NormalSampler(rl_act_dim),cfg['seed'])
    # loaded_sampler.load_state_dict(torch.load(weight_path)['actor_distribution_state_dict'])

    # load normalizing scale
    env.load_scaling(weight_dir, int(iteration_number))
    my_helper.load_scaling(weight_dir,int(iteration_number))

    # actual steps
    my_helper.on_visualize(iseval=False, update = iteration_number)    
    env.reset()
    init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
    init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
    init_joint_angle = init_ctrl_obs['jointPos']
    controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)

    contact_forces = {"FR":[], "FL":[],"RR":[],"RL":[]}
    for step in range(total_steps*2):
        frame_start = time.time()

        raw_obs, normed_obs = env.observe(update_statistics=False, isnoise=False)
        desired_leg_state = controller.get_desired_leg_state()
        desired_leg_state = MyHelper.change_foot_contact_range(desired_leg_state)
        rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state)
        ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
        
        # u_rl = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
        # u_rl = my_helper.unnormalize(u_rl).reshape(-1,4,3)
        u_rl = controller.get_mpc_contact_force(ctrl_obs)
        for i,key in enumerate(contact_forces):
            contact_forces[key].append(u_rl[:,i,2])
        env_act = controller.control_with_conactForce(u_rl,ctrl_obs,step)
        reward, dones = env.step(env_act)

        for i in range(num_env):
            if dones[i]:
                init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
                init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
                init_joint_angle = init_ctrl_obs['jointPos']
                controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)

        # frame_end = time.time()
        # wait_time = cfg['environment']['control_dt'] - (frame_end - frame_start)
        # if wait_time > 0.:
        #     time.sleep(wait_time)
        # else:
        #     print("Calculation delayed about %fs. It might make it look slower than its control"%(-wait_time))

        if dones or step == total_steps - 1:
            step_data = {"step":[]}
            for i in np.arange(start_step_id,step,1):
                step_data["step"].append(i)
            for key in contact_forces:
                step_data[key] = contact_forces[key]
            for i in range(step-start_step_id):
                wandb.log({key: step_data[key][i] for key in step_data})
            # ############################################3 my comment ################################################
            print('----------------------------------------------------')
            print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * cfg['environment']['control_dt'])))
            print('----------------------------------------------------\n')
            # #########################################################################################################
            start_step_id = step + 1

    # recording end and reset
    my_helper.off_visualize()
    env.reset()
    ######################## my record to wandb ################################################
    # time.sleep(20)
    # wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name, caption="test", fps=4, format="mp4")})
    ############################################################################################
    print("Finished at the maximum visualization steps")