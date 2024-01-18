from math import sin,pi
import numpy as np
# from onnx_deploy import rad_deg, deg_rad
from raisimGymTorch.deploy_log.draw_map import Drawer
def deg_rad(x):
    if isinstance(x, list):
        return [deg_rad(a) for a in x]
    else:
        return x / 180 * pi

def rad_deg(x):
    if isinstance(x, list):
        return [rad_deg(a) for a in x]
    else:
        return x / pi * 180


low = [-46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5]
upp = [46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5]
low = deg_rad(low)
upp = deg_rad(upp)
bund = [upp[i] - low[i] for i in range(12)]
def transfer(act_gen, sine, k, history_act = None) ->np.array:
    """
    clip, lerp, generate the final action
    params: act_gen: [12 * (-1, 1)]  np.array
    params: sine: [12 * rad_target]  np.array
    """
    act_gen = act_gen * bund  # - +
    act_gen = np.clip(act_gen, low, upp)
    if history_act is not None:
        kk = 0.9
        act_gen = act_gen * (1-kk) + history_act * kk
        act_gen = np.clip(act_gen, low, upp)
    if act_gen.shape[0] == 1:
        act_gen = act_gen[0]
    action = act_gen * k + sine
    # print(np.abs(act_gen).max(), sine.max())
    action = np.clip(action, low, upp)
    return action


def get_last_position(obs):
    if isinstance(obs, list):
        x = [obs[-2 * i] for i in range(1, 13)]
        x.reverse()
        return x
    elif isinstance(obs, np.ndarray):
        x = []
        for i in range(1, 13):
            x.append(obs[:, -2*i])
        x.reverse()
        return np.stack(x, axis=1)

if __name__=='__main__':
    a = np.random.rand(2,30)
    print(a)
    print(get_last_position(a).shape)
