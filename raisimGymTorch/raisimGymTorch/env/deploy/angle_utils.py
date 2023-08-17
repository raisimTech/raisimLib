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
def sine_generator(angle_list, idx, T, rate=1):
    base1= 0.523
    base3= 0.0
    base2 = -2 * base1
    ang = abs(sin( float(idx) / T  * pi)) * rate

    idx_base = 0

    if (int(idx/T) % 2 ) == 1:
        angle_list[idx_base+1] = ang + base1
        angle_list[idx_base+2] = -2 * ang + base2

        angle_list[idx_base+4] = base1
        angle_list[idx_base+5] = base2

        angle_list[idx_base+7] = base1
        angle_list[idx_base+8] = base2
        angle_list[idx_base+10] = ang + base1
        angle_list[idx_base+11] = - 2 * ang + base2
    else:
        angle_list[idx_base+1] = base1
        angle_list[idx_base+2] = base2

        angle_list[idx_base+10] = base1
        angle_list[idx_base+11] = base2
        angle_list[idx_base+4] = ang + base1
        angle_list[idx_base+5] = -2 * ang + base2

        angle_list[idx_base+7] = ang + base1
        angle_list[idx_base+8] = -2 * ang + base2
    angle_list[idx_base+0] = base3
    angle_list[idx_base+3] = base3
    angle_list[idx_base+6] = base3
    angle_list[idx_base+9] = base3
    return angle_list

def transfer(act_gen, sine, k, history_act = None) ->np.array:
    """
    clip, lerp, generate the final action
    params: act_gen: [12 * (-1, 1)]  np.array
    params: sine: [12 * rad_target]  np.array
    """
    # act_gen = (act_gen + 1) /2 # 0-1
    act_gen = act_gen * bund  # - +
    # print(act_gen)
    act_gen = np.clip(act_gen, low, upp)
    if history_act is not None:
        kk = 0.9
        # act_gen = (1-kk) * act_gen
        act_gen = act_gen * (1-kk) + history_act * kk
        act_gen = np.clip(act_gen, low, upp)
    if act_gen.shape[0] == 1:
        act_gen = act_gen[0]
    action = act_gen * k + sine
    # print(np.abs(act_gen).max(), sine.max())
    action = np.clip(action, low, upp)
    return action

def transfer_f(act_gen, sine, k, history_act = None) ->np.array:
    """
    clip, lerp, generate the final action
    params: act_gen: [12 * (-1, 1)]  np.array
    params: sine: [12 * rad_target]  np.array
    """
    # act_gen = (act_gen + 1) /2 # 0-1
    act_gen = act_gen * bund / 2  # - +
    # print(act_gen)
    act_gen = np.clip(act_gen, low, upp)
    if history_act is not None:
        kk = 1-k
        # act_gen = (1-kk) * act_gen
        # print(f'his1 {history_act}\n act_gen{act_gen}')
        history_act = act_gen * (1-kk) + history_act * kk
        history_act = np.clip(history_act, low, upp)
        # print(f'his2 {history_act}')

    if history_act.shape[0] == 1:
        tmp = history_act[0]
    else:
        tmp = history_act
    action = tmp * 0.2 + sine
    # print(np.abs(act_gen).max(), sine.max())
    action = np.clip(action, low, upp)
    # print(f'bound{bund}\n act_gen{act_gen}\n history_act{history_act}\n sine{sine} \naction{action}\nk{k} ')
    # input('1')
    return action, history_act

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
