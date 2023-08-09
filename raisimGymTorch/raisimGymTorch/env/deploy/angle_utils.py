from math import sin,pi
import numpy as np
# from onnx_deploy import rad_deg, deg_rad
def deg_rad(x):
    if isinstance(x, list):
        return [deg_rad(a) for a in x]
    else:
        return x / 180 * pi

def rad_deg(x):
    if isinstance(x, list):
        return [rad_deg(a) for a in x]
    else:
        return x * pi / 180


low = [-46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5]
upp = [46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5]
low = deg_rad(low)
upp = deg_rad(upp)
def sine_generator(angle_list, idx, T, rate=1):
    base1= 0.82
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

def transfer(act_gen, sine, k) ->np.array:
    """
    clip, lerp, generate the final action
    params: act_gen: [12 * (-1, 1)]  np.array
    params: sine: [12 * rad_target]  np.array
    """
    if act_gen.shape[0] == 1:
        act_gen = act_gen[0]
    action = act_gen * k + sine * (1-k)
    action = np.clip(action, low, upp)
    return action

if __name__=='__main__':
    angle_list = [0 for x in range(12)]

    sine_generator(angle_list, 2, 40, 0.3)
    angle_list = np.array(angle_list)
    act = transfer(np.random.random((100,12)), angle_list, 0.3)
    print('sine:', angle_list)
    print('low:', low)
    print('upp:', upp)
    print(act.shape, type(act))
