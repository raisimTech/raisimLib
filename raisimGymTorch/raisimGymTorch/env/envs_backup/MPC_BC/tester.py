from turtle import update
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import MPC_BC
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo_BC.module as module
import os
import math
import time
import torch
import argparse
#######################my import###################################
import wandb
import datetime
import numpy as np
from raisimGymTorch.mpc.mpc_controller_BC.control import Controller
from raisimGymTorch.env.envs.MPC_BC.my_helper import MyHelper
###################################################################


# configuration
weight_path_arg = '/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/data/MPC_BC/2022-11-08-18-48-55/full_400.pt'
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
env = VecEnv(MPC_BC.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

################# my wandb #####################################
wandb.init(project="MPC_guide",group="1108",job_type="test",name="MPC-BC",notes="MPC with BC loss"
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

    # load actor
    loaded_graph = module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, rl_obs_dim, rl_act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

    # load sampler
    loaded_sampler = module.MultivariateGaussianDiagonalCovariance(rl_act_dim,env.num_envs,1.0,MPC_BC.NormalSampler(rl_act_dim),cfg['seed'])
    loaded_sampler.load_state_dict(torch.load(weight_path)['actor_distribution_state_dict'])

    # load normalizing scale
    # env.load_scaling(weight_dir, int(iteration_number))

    # actual steps
    my_helper.on_visualize(record=True,update=iteration_number,istest=True)
    controller = my_helper.reset_env_controller()
    #simulate
    for step in range(n_steps):
        with torch.no_grad():
            frame_start = time.time()
            raw_obs,_ = env.observe(update_statistics=False,isnoise=False)
            normed_obs = my_helper.change_normed_obs(raw_obs)
            desired_leg_state = MyHelper.change_foot_contact_range(controller.get_desired_leg_state()) # get desired leg state for rl obs
            rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state) # get rl obs
            ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)

            u_rl = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
            u_rl_unnormalized = my_helper.unnormalize(u_rl)
            
            u_rl_unnormalized = u_rl_unnormalized.reshape(-1,4,3)
            env_act = controller.control_with_conactForce(u_rl_unnormalized,ctrl_obs,step)
            reward, dones = env.step(env_act,step)
            for i in range(num_env):
                if dones[i]:
                    init_raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
                    init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
                    init_joint_angle = init_ctrl_obs['jointPos']
                    controller.reset(cfg['environment']['control_dt']*(step+1),init_joint_angle,i)
            frame_end = time.time()
            wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
            if wait_time > 0.:
                time.sleep(wait_time)
            if step%10 ==0:
                ## collect performance info
                step_data = {"step":step+n_steps}
                # rewards
                reward_data = {}
                rewardinfo = env.getRewardinfo()
                for i in range(len(rewardinfo)):
                    for key in rewardinfo[i]:
                        if key in reward_data:
                            reward_data[key].append(rewardinfo[i][key])
                        else:
                            reward_data[key] = []
                for key in reward_data:
                    step_data[key+"(reward)"] = np.mean((np.array(reward_data[key])))
                # velocities
                step_data["forward_velocity"] = np.mean(ctrl_obs["baseLinVel"][:,0])
      
                # log stepwise data
                wandb.log(step_data)
    my_helper.off_visualize(record=True)



    # compare with MPC
    my_helper.on_visualize(record=False,update=iteration_number,istest=True)
    controller = my_helper.reset_env_controller()
    if cfg['environment']['render']:
        my_helper.compare_with_mpc(update=iteration_number,controller=controller)
    my_helper.off_visualize(record=False)


    env.reset()
    init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
    init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
    init_joint_angle = init_ctrl_obs['jointPos']
    controller = Controller(init_joint_angle,ctrl_time_step,ctrl_act_dim,num_env)

    for step in range(total_steps):
        frame_start = time.time()

        raw_obs, _ = env.observe(update_statistics=False, isnoise=False)
        normed_obs = my_helper.change_normed_obs(raw_obs)
        desired_leg_state = controller.get_desired_leg_state()
        desired_leg_state = MyHelper.change_foot_contact_range(desired_leg_state)
        rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state)
        ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)
        
        u_rl = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
        u_rl_unnormalized = my_helper.unnormalize(u_rl)
        u_mpc = controller.get_mpc_contact_force(ctrl_obs).reshape(-1,rl_act_dim)
        u_mpc_normed = my_helper.normalize_target(u_mpc)

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

        
        u_rl = u_rl.reshape(-1,4,3)
        u_rl_unnormalized = u_rl_unnormalized.reshape(-1,4,3)
        u_mpc = u_mpc.reshape(-1,4,3)
        env_act = controller.control_with_conactForce(u_rl_unnormalized,ctrl_obs,step)
        # env_act = controller.control_with_conactForce(u_mpc,ctrl_obs,step)
        # env_act = controller.control_with_conactForce(0.5*u_mpc+0.5*u_rl,ctrl_obs,step)
        reward, dones = env.step(env_act)

        for i in range(num_env):
            if dones[i]:
                init_raw_obs, init_normed_obs = env.observe(update_statistics=False, isnoise=False)
                init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
                init_joint_angle = init_ctrl_obs['jointPos']
                controller.reset(ctrl_time_step*(step+1),init_joint_angle,i)
        step_data = {"step":step}
        for key in contact_forces:
                step_data[key] = contact_forces[key]
        for key in u_rl_contact_forces:
            step_data[key] = u_rl_contact_forces[key]
        for key in u_rl_contact_forces_normed:
            step_data[key] = u_rl_contact_forces_normed[key]
        step_data["loss"] = np.mean((u_rl.reshape(-1,rl_act_dim)-u_mpc_normed)**2)
        wandb.log(step_data)
        # frame_end = time.time()
        # wait_time = cfg['environment']['control_dt'] - (frame_end - frame_start)
        # if wait_time > 0.:
        #     time.sleep(wait_time)
        # else:
        #     print("Calculation delayed about %fs. It might make it look slower than its control"%(-wait_time))

        if dones or step == total_steps - 1:
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