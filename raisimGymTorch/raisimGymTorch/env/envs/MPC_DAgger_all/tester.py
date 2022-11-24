from turtle import update
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv

import os
import math
import time
import torch
import argparse
#######################my import###################################
import wandb
import datetime
import numpy as np
from raisimGymTorch.env.bin import MPC_DAgger_all
import raisimGymTorch.algo.DAgger_all.module as module
from raisimGymTorch.env.envs.MPC_DAgger_all.my_helper import MyHelper
from raisimGymTorch.mpc.mpc_controller_DAgger_all import control
from raisimGymTorch.mpc.mpc_controller_DAgger_all.control import Controller
from raisimGymTorch.mpc.mpc_controller_DAgger_all.openloop_gait_generator import OpenloopGaitGenerator
from raisimGymTorch.mpc.mpc_controller_DAgger_all.openloop_gait_generator_vec import OpenloopGaitGenerator_vec
###################################################################


# configuration
weight_path_arg = '/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/data/MPC_DAgger_all/2022-11-22-23-36-46/full_90000.pt'
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
env = VecEnv(MPC_DAgger_all.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

################# my wandb #####################################
wandb.init(project="MPC_guide",group="1129",job_type="test",name="MPC-DAgger_all",notes="MPC-DAgger_all"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
################################################################

# Constants
env_obs_dim = env.num_obs
env_act_dim = env.num_acts
sim_time_step = cfg['environment']['simulation_dt']
num_env = cfg['environment']['num_envs']

rl_obs_dim = 4+12+3+3+1+1+4 # foot contact, joint pose, body vel, body ang vel, height, body z angle,phase in full cyle
rl_act_dim = 12*2 # joint pos, joint torque

ctrl_time_step = cfg['environment']['control_dt']
ctrl_obs_dim = 26 # body orientation(4), foot contact(4), joint pos(12), body lin vel(3), body ang vel(3)
ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD

num_threads = cfg['environment']['num_threads']
num_env = cfg['environment']['num_envs']
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * num_env
record = cfg["record_video"]

my_helper = MyHelper(env,cfg,None,rl_obs_dim,rl_act_dim,n_steps, \
                    control._STANCE_DURATION_SECONDS,control._DUTY_FACTOR,control._INIT_PHASE_FULL_CYCLE,control._INIT_LEG_STATE)

weight_path = args.weight
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'

if weight_path == "":
    print("Can't find trained weight, please provide a trained weight with --weight switch\n")

# simulation
else:
    print("Loaded weight from {}\n".format(weight_path))    
    done_sum = 0
    average_dones = 0.
    start_step_id = 0
    print("Visualizing and evaluating the policy: ", weight_path)

    # load actor
    loaded_graph = module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, rl_obs_dim, rl_act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

    # load sampler
    loaded_sampler = module.MultivariateGaussianDiagonalCovariance(rl_act_dim,env.num_envs,1.0,MPC_DAgger_all.NormalSampler(rl_act_dim),cfg['seed'])
    loaded_sampler.load_state_dict(torch.load(weight_path)['actor_distribution_state_dict'])

    # load normalizing scale
    # env.load_scaling(weight_dir, int(iteration_number))

    # actual steps    
    my_helper.on_visualize(update=iteration_number,record=record,istest=True)
    reset_time = np.zeros((num_env,1))
    env.reset()
    gait_generator = []
    init_raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
    init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
    init_joint_angle = init_ctrl_obs['jointPos']
    # controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)
    # for i in range(num_env):
    #     gait_generator.append(OpenloopGaitGenerator(                            
    #                         stance_duration=control._STANCE_DURATION_SECONDS,
    #                         duty_factor=control._DUTY_FACTOR,
    #                         initial_leg_phase=control._INIT_PHASE_FULL_CYCLE,
    #                         initial_leg_state=control._INIT_LEG_STATE)
    #                         )
    gait_generator_vec = OpenloopGaitGenerator_vec(
                            num_env=num_env,
                            stance_duration=control._STANCE_DURATION_SECONDS,
                            duty_factor=control._DUTY_FACTOR,
                            initial_leg_phase=control._INIT_PHASE_FULL_CYCLE,
                            initial_leg_state=[0,1,1,0])
    simulation_steps = 10*n_steps
    for step in range(simulation_steps):
        # get observations
        frame_start = time.time()
        raw_obs,_ = env.observe(update_statistics=False, isnoise = False)
        normed_obs = my_helper.change_normed_obs(raw_obs)
        phase_in_full_cycle = my_helper.get_phase_in_full_cycle(ctrl_time_step*step-reset_time) # (num_env,4)
        rl_obs = MyHelper.get_rl_obs(normed_obs,phase_in_full_cycle,num_env) # get rl obs
        ctrl_obs = MyHelper.get_ctrl_obs(raw_obs) # get control obs
        # u_mpc = controller.get_mpc_contact_force(ctrl_obs)
        # mpc_action = controller.control_with_conactForce(u_mpc,ctrl_obs,step).reshape(-1,ctrl_act_dim)
        gait_generator_vec.update(ctrl_time_step*step-reset_time)
        # for i in range(num_env):
        #     gait_generator[i].update(ctrl_time_step*step-reset_time[i],[False,False,False,False])
        # next step
        rl_action = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
        rl_action_control = my_helper.process_rl_action(rl_action) # unnormalize
        behavior_action = rl_action_control
        # alpha = 1-int(iteration_number)/100000
        # behavior_action = alpha*mpc_action[:,:24]+(1-alpha)*rl_action_control
        env_act = np.zeros((num_env,48))
        env_act[:,:24] = behavior_action

        # env_act = my_helper.append_gains(env_act,gait_generator)
        env_act = my_helper.append_gains_vec(env_act,gait_generator_vec)
        # env_act[:,24:] = mpc_action[:,24:]

        env_act = env_act.astype(np.float32)
        reward, dones = env.step(env_act,step)
        for i in range(num_env):
            if dones[i]:
                init_raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
                init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
                init_joint_angle = init_ctrl_obs['jointPos']
                # controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)
                reset_time[i] = ctrl_time_step*(step+1)
                # gait_generator[i].reset(0.0)
                gait_generator_vec.reset_indiv(0.0,i)
        frame_end = time.time()
        wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
        if wait_time > 0.:
            time.sleep(wait_time)
        if np.any(dones) or step == simulation_steps - 1:
            # ############################################3 my comment ################################################
            print('----------------------------------------------------')
            print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * cfg['environment']['control_dt'])))
            print('----------------------------------------------------\n')
            # #########################################################################################################
            start_step_id = step + 1

        # if step%10 == 0:
        #     step_data = {"step":step}
        #     loss = np.mean((mpc_action_normed[:,:24]-rl_action)**2)
        #     step_data["loss"] = loss
        #     wandb.log(step_data)
    my_helper.off_visualize(record=record)

    ######################## my record to wandb ################################################
    # time.sleep(20)
    # wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name, caption="test", fps=4, format="mp4")})
    ############################################################################################
    print("Finished at the maximum visualization steps")