from turtle import update
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import MPC_BC_baseline
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
from raisimGymTorch.mpc_BC_baseline import control
import raisimpy as raisim
###################################################################

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1

################# my wandb #####################################
wandb.init(project="MPC_guide",group="1017",job_type="test_recording",name="MPC_bc",notes="mpc with bc loss"
            ,config=cfg,save_code=True, mode = cfg['wandb_mode'])
################################################################

a1_urdf_file = "/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/rsc/a1/urdf/a1.urdf"
world = raisim.World()
world.setTimeStep(cfg['environment']['simulation_dt'])
server = raisim.RaisimServer(world)
ground = world.addGround()
server.launchServer(8080)
a1_obj = world.addArticulatedSystem(a1_urdf_file)
a1_obj.setName("a1")
a1_nominal_joint_config = np.array([0.0, 0.0, 0.27, 1, 0.0, 0.0, 0.0,
                                        0.0, 0.9, -1.8,        0.0, 0.9, -1.8,
                                        0.0, 0.9, -1.8,       0.0, 0.9, -1.8])

a1_obj.setGeneralizedCoordinate(a1_nominal_joint_config)

recorded_config = np.loadtxt(os.path.join(task_path+"record_config.csv"))

# recording
record_start = time.time()
vid_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_test"+'.mp4'
server.startRecordingVideo(vid_name)

for i in range(recorded_config.shape[0]):
    a1_obj.setGeneralizedCoordinate(recorded_config[i])
    time.sleep(cfg['environment']['control_dt'])

# recording end and reset
server.stopRecordingVideo()
######################## my record to wandb ################################################
time.sleep(20)
wandb.log({"video": wandb.Video("/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/raisimUnity/linux/Screenshot/"+vid_name, caption="test", fps=4, format="mp4")})
############################################################################################
