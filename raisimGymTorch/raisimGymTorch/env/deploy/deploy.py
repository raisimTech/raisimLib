import torch
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
import argparse
import numpy as np
import time
import os
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin.rsg_go1 import NormalSampler
from robot import Robot
import torch.nn as nn
from sine_generator import sine_generator

robot = Robot()




parser = argparse.ArgumentParser()
# parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
parser.add_argument('-u', '--update', help='update times', type=int, default=120)
parser.add_argument('-p', '--cfg_path', help='where to find the path', type=str, default=None)
# parser.add_argument('-b', '--load_best', help='load best file in last train', type=bool, default=False)
args = parser.parse_args()
# mode = args.mode
weight_path = args.weight
weight_path = './data/full_152.pt'
if weight_path!='' and args.cfg_path == None:
    cfg_path = os.path.split(weight_path)[0] +'/cfg.yaml'
else:
    cfg_path = args.cfg_path
# load_best = args.load_best
# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path

# config
if cfg_path is not None:
    cfg = YAML().load(open(cfg_path, 'r'))
else:
    cfg = YAML().load(open( + "/cfg.yaml", 'r'))


def update_dim(robot):
    """
    update the dimentions with robot
    make the mid-plugin to get the info using python

    params:
        ob_dim: contain the basic info ,which is same with the ob_dim when training
        act_dim: contains the action_dim, the number of motors to control

    """
    global act_dim, ob_dim
    act_dim = robot.act_dim()
    ob_dim = robot.ob_dim()


act_dim = 0
ob_dim = 0
update_dim(robot)
print(cfg['architecture']['policy_net'], ob_dim, act_dim)
actor = ppo_module.MLP(cfg['architecture']['policy_net'], nn.LeakyReLU, ob_dim, act_dim)



def reset(robot):
    """
    reset the A1 robot with reset the basic info
    robot: to be implemented by c++
    todo
    """

    pass

def load_model(weight_path, actor):
    a = input('Are you sure to load this model and run? Y or N')
    if a.lower() == 'y':
        checkpoint = torch.load(weight_path)
        actor.architecture.load_state_dict(checkpoint['actor_architecture_state_dict'])
        # actor.distribution.load_state_dict(checkpoint['actor_distribution_state_dict'])
        # critic.architecture.load_state_dict(checkpoint['critic_architecture_state_dict'])
        # optimizer.load_state_dict(checkpoint['optimizer_state_dict'])


def init():
    weight_path = './data/full_152.pt'
    reset(robot)
    load_model(weight_path, actor)


def cal_limit(act :np.array):
    """
    get the limit and norm the act to the limit
    param:
        act: actions to be clip into the limits
    """
    return act

def act_concate(act:np.array, sine:np.array) -> np.array:
    """
    cal the act to the limit and add it with the sine

    """
    sine = np.array(sine)
    act = cal_limit(act)
    return act + sine

def run():
    """
    run the ob-act
    """
    cnt = 0
    angle_list = [0 for x in range(12)]
    while True:
        print('running func')
        if not robot.alive():
            break

        ob = robot.observe()

        act = actor.architecture(torch.from_numpy(ob).cpu()).cpu().detach().numpy()

        angle_list = sine_generator(angle_list, cnt, cfg['environment']['schedule'], cfg['environment']['angle_rate'])

        act = act_concate(act, angle_list)

        robot.take_action(act)

        time.sleep(0.1)

        cnt += 1

if __name__=='__main__':
    init()

    run()

