from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import rsg_anymal
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo.module as ppo_module
import os
import math
import time
import torch
import argparse
#######################my import###################################
import wandb
import datetime
import numpy as np
###################################################################


# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimGymTorch/data/anymal_locomotion/2022-08-13-00-39-08/full_600.pt')
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1

################# my wandb #####################################
wandb.init(project="0801",group="debbugging_code",job_type="test",name="debugging_velocity_graph",notes="implemented code that shows velocity in wandb graph"
            ,config=cfg,save_code=True)
################################################################

env = VecEnv(rsg_anymal.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

###########################3 my save environment.hpp ###################################
wandb.save(task_path+"/Environment.hpp")
wandb.save(task_path+"/cfg.yaml")
#########################################################################################

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts

weight_path = args.weight
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'

if weight_path == "":
    print("Can't find trained weight, please provide a trained weight with --weight switch\n")
else:
    print("Loaded weight from {}\n".format(weight_path))
    start = time.time()
    env.reset()
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.
    n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
    total_steps = n_steps * 1
    start_step_id = 0

    print("Visualizing and evaluating the policy: ", weight_path)
    loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, ob_dim, act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

    env.load_scaling(weight_dir, int(iteration_number))
    env.turn_on_visualization()
    ################### my video record ####################################################
    vid_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_test"+'.mp4'
    env.start_video_recording(vid_name)
    ########################################################################################

    # max_steps = 1000000
    max_steps = 1000 ## 15 secs
    for step in range(max_steps):
        time.sleep(0.01)
        obs = env.observe(False)
        action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())
        reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
        reward_ll_sum = reward_ll_sum + reward_ll[0]
        ############3#######################3 my check velocity ##############################################3
        # if not dones and step%5==0:
        #     # #record reward
        #     wandb.log({"average_reward":reward_ll_sum / (step + 1 - start_step_id)})
        #     # record velocity
        #     obs_raw = env.observe(False,True)
        #     wandb.log({"forward_velocity": obs_raw[0][16], "velocity": np.linalg.norm(obs_raw[0][16:19]) })
        #     wandb.log({"yawing_velocity": abs(obs_raw[0][21])})
        ######################################################################################################3
        if dones or step == max_steps - 1:
            # ############################################3 my comment ################################################
            print('----------------------------------------------------')
            print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(reward_ll_sum / (step + 1 - start_step_id))))
            print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * 0.01)))
            print('----------------------------------------------------\n')
            # #########################################################################################################
            start_step_id = step + 1
            reward_ll_sum = 0.0
            
    env.turn_off_visualization()
    env.reset()
    env.stop_video_recording()
    ######################## my record to wandb ################################################
    time.sleep(20)
    wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name, caption="test", fps=4, format="mp4")})
    ############################################################################################
    print("Finished at the maximum visualization steps")
