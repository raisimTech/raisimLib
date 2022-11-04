import gym
import os

from ruamel.yaml import YAML, dump, RoundTripDumper
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from raisimGymTorch.env.bin import rsg_anymal
from raisimGymTorch.stable_baselines3.RaisimSbGymVecEnv import RaisimSbGymVecEnv as VecEnv


# Parallel environments
# directories
stb_path = os.path.dirname(os.path.realpath(__file__))
rsc_path = stb_path + "/../../../rsc"
task_path = stb_path + "/../env/envs/rsg_anymal"

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
env = VecEnv(rsg_anymal.RaisimGymEnv(rsc_path, dump(cfg['environment'], Dumper=RoundTripDumper)), normalize_ob=True)
obs = env.reset()

n_steps = int(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
model = PPO(MlpPolicy, env,
            n_steps=n_steps,
            verbose=1,
            batch_size=int(n_steps*env.num_envs/4),
            n_epochs=4)

model.learn(total_timesteps=250000000)
