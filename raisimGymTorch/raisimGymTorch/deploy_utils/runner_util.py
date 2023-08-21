import numpy as np
from math import cos,pi
def lerp_np(a, b, c):
    return a* (1-c) + b*c
def add_list_np(act_gen, sine, history,kb):
    kk = 0.9
    kf = 1
    kb = kb
    history = history*kk + (1-kk) * act_gen
    ans = np.clip(kb*history + kf * sine, -1, 1)
    ans = (ans + 1) /2  # 100 * 12
    ans = lerp_np(low_np, upp_np, ans)

    return ans, history
def ang_trans(lower, upper, x):
    """
    trans the rate x to the angle
    """
    if isinstance(lower, list):
        return [ang_trans(lower[i], upper[i], x[i]) for i in range(len(lower))]
    else:
        x = (x+1)/2
        return x * upper + (1-x) * lower
def norm(lower, upper, x):
    # print(lower, upper, x)
    # assert abs((x - lower) / (upper - lower) ) <= 1, f"{abs((x - lower) / (upper - lower) )} {x, upper, lower} "
    return (x - lower) / (upper - lower)
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
def deg_normalize(lower, upper, x):
    """
        make angle to rate from -1 -- 1
    """
    if isinstance(lower, list):
        ans = []
        for a,b,c in zip(lower, upper, x):
            ans.append(deg_normalize(a,b,c))
        return ans
    else:
        return 2 * norm(lower, upper, x) - 1
        # return 1 - 2* norm(lower, upper,x)
low = [-46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5]
upp = [46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5]
tha1,tha2 = 30, 30
u0 = [0, tha2, -2*tha2, 0 ,tha1, -tha1 * 2, 0 , tha2, -2*tha2, 0, tha1,-2*tha1]

low_np = np.array(low)
upp_np = np.array(upp)
u0 = deg_normalize(low, upp, u0)
u0_ang = ang_trans(low, upp, u0)

def sine_gene_pt(idx, T, rate):
    # print(idx)
    if isinstance(idx, np.ndarray) or isinstance(idx, list):
        mat = np.zeros((len(idx), 12))
        i_set = set(idx)
        idd = {}
        for i in i_set:
            idd[i] = sine_gene_pt(i, T, rate)
        for i in range(len(idx)):
            mat[i] = idd[idx[i]]
        return mat
    if not isinstance(idx, list):
        angle_list = [0 for i in range(12)]
        if idx >= 2 * T:
            idx = 0
        dh = rate #:w


        if idx >= 0 and idx <= T:
            tp0 =idx - 0
            y1 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            y2 = 0
            # y2 = y1
        elif idx>T and idx<=2*T:
            tp0 =idx - T
            y2 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            y1 = 0
            # y1 = y2
        # idx = idx % T
        # y1 = dh  * 10 * (-cos(pi * 2  *idx  /T) + 1)
        # y2  = deg_rad(60)
        angle_list[0] = 0
        angle_list[1] = y1
        angle_list[2] = -2 * y1
        angle_list[3] = 0
        angle_list[4] = y2
        angle_list[5] = -2 * y2
        angle_list[6] = 0
        angle_list[7] = y2
        angle_list[8] = -2 * y2
        angle_list[9] = 0
        angle_list[10] = y1
        angle_list[11] = -2 * y1

        angle_list = [deg_normalize(low[i], upp[i], angle_list[i] + u0_ang[i]) for i in range(12)]

        return angle_list
    else:
        ans = []
        for i in idx:
            ans.append(sine_gene_pt(i, T, rate))
        return ans
def list_pt(act_gen, idx, T):
    ans = []
    for i in range(act_gen.shape[0]):
        ans.append(run_model_with_pt_input(act_gen[i, :], idx[i], T))
    ans = np.vstack(ans).astype(np.float32)
    return  ans
def run_model_with_pt_input_modify(act_gen, idx, T, history, kb, rate):
    # act_gen = np.clip(act_gen- 1, -1, 1)
    if isinstance(idx, list):
        idx = np.array(idx)
        idx = idx%(2*T)
    else:
        idx = idx % (2 * T)
    # act_gen = np.zeros_like(act_gen)
    sine = sine_gene_pt(idx, T, rate)
    ans, history = add_list_np(act_gen, sine, history, kb)
    ans = ans / 180 * 3.14

    return ans.astype(np.float32), history.astype(np.float32)

if __name__=='__main__':
    print(run_model_with_pt_input_modify(np.zeros((1,12)), 1, 50, np.zeros((1,12))))