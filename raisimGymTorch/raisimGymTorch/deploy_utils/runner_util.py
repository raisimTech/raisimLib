import math

import numpy as np
from math import cos,pi
def lerp_np(a, b, c):
    return a* (1-c) + b*c
def add_list_np(act_gen, sine, history,kb):
    kk = 0.95
    kf = 1
    # kb = np.array([0.15 ,0., 0.] * 4)
    # print(act_gen)
    # kb = np.array([0.2,0.1, 0.1] * 4  )
    kb = np.array([0.2] *12  )
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
        return x * 180 / pi
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

def rad_normalize(lower, upper, x):
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
abss = 8
tha1,tha2 = 41 + abss ,  41 + abss
for_r = 0
u0 = [0, tha2, -2*tha2 +2 * abss , 0 ,tha1, -tha1 * 2 +2*abss  , 0 , tha2 + for_r , -2*tha2 - 2 * for_r   +2*abss, 0, tha1 + for_r  ,-2*tha1 -2 * for_r +2*abss]

low_np = np.array(low)
upp_np = np.array(upp)
u0_rad = deg_rad(u0)

low_rad = low_np / 180 * 3.14
upp_rad = upp_np / 180 * 3.14

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
        # assert dh == 0.6
        ds = 0.
        H = 0.3
        # print(idx)
        if idx >= 0 and idx <= T:
            tp0 =idx - 0
            y1 = dh * 0.05 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            y2 = 0
            x1 =ds * (-0.05 + 0.1 * tp0 /T)
            x2 = ds * (0.05 - 0.1 * tp0 / T)
            # y11 = 0

        elif idx>T and idx<=2*T:
            tp0 =idx - T
            y2 = dh * 0.05 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            y1 = 0
            x1 =ds * (0.05 - 0.1 * tp0 /T)
            x2 = ds * (-0.05 + 0.1 * tp0 / T)
            # y1 = y2 /2
            # y1 = -y2
            # y1 = y2
        # idx = idx % T
        # y1 = dh  * 10 * (-cos(pi * 2  *idx  /T) + 1)
        # y2  = deg_rad(60)
        # y1 = 2 * y1
        # y2 = 2 * y2

        # ct0 = math.atan(- x1 / (H-y1))
        # L0 = math.sqrt((H-y1) **2 + x1 ** 2 )
        # ct1 = math.acos(L0 / 0.4)
        #
        # c1 = ct0 + ct1
        # c2 = -2 * ct1
        #
        # act0 = math.atan(-x2/(H-y2))
        # aL0 = math.sqrt((H-y2)  ** 2 + x2 **2 )
        # act1 = math.acos(aL0 / 0.4  )
        #
        # ac1 = act0 + act1
        # ac2 = -2 * act1
        #
        #
        # angle_list[0] = 0
        # angle_list[1] = c1
        # angle_list[2] = c2
        # angle_list[3] = 0
        # angle_list[4] = ac1
        # angle_list[5] = ac2
        # angle_list[6] = 0
        # angle_list[7] = ac1
        # angle_list[8] = ac2
        # angle_list[9] = 0
        # angle_list[10] = c1
        # angle_list[11] = c2



        ct0 = math.atan(-x1 / (H - y1))
        L0 = math.sqrt((H - y1) * (H - y1) + x1 * x1)
        ct1 = math.acos(L0 / 0.4)
        c1 = ct0 + ct1
        c2 = -2 * ct1

        act0 = math.atan(-x2 / (H - y2))
        aL0 = math.sqrt((H - y2) * (H - y2) + x2 * x2)
        act1 = math.acos(aL0 / 0.4)
        ac1 = act0 + act1
        ac2 = -2 * act1
        abss = 8
        angle_list[0] = 0
        angle_list[1] = ac1 + deg_rad(abss)
        angle_list[2] = ac2
        angle_list[3] = 0
        angle_list[4] = c1 + deg_rad(abss)
        angle_list[5] = c2
        angle_list[6] = 0
        angle_list[7] = ac1 + deg_rad(abss)
        angle_list[8] = ac2
        angle_list[9] = 0
        angle_list[10] = c1 + deg_rad(abss)
        angle_list[11] = c2


        angle_list = rad_deg(angle_list)
        # print(angle_list)
        angle_list = [deg_normalize(low[i], upp[i], angle_list[i] ) for i in range(12)]
        # angle_list = [rad_normalize(low_rad[i] , upp_rad[i], angle_list[i] + u0_rad[i]) for i in range(12)]
        return angle_list
    else:
        ans = []
        for i in idx:
            ans.append(sine_gene_pt(i, T, rate))
        return ans

def sine_gene_pt_step(idx, T, rate):
    # print(idx)
    if isinstance(idx, np.ndarray) or isinstance(idx, list):
        mat = np.zeros((len(idx), 12))
        i_set = set(idx)
        idd = {}
        for i in i_set:
            idd[i] = sine_gene_pt_step(i, T, rate)
        for i in range(len(idx)):
            mat[i] = idd[idx[i]]
        return mat
    if not isinstance(idx, list):
        angle_list = [0 for i in range(12)]
        if idx >= 2 * T:
            idx = 0
        dh = 0.6 #:w


        if idx >= 0 and idx <= T:
            tp0 =idx - 0
            y1 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            # y11 = 0.7 * dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            # y11 = 0

            y2 = 0
            # y22 = -y11 * 0.5
            # y2 = y1 /2
            # y2 = -y1
            # y2 = y1
            # y1 = 1.5 * y1
        elif idx>T and idx<=2*T:
            tp0 =idx - T
            y2 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            # y22 = 0.7 * dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            # y11 = 0

            y1=0
            # y11 = -y2 * 0.5
            # y1 = y2 /2
            # y1 = -y2
            # y1 = y2
        # idx = idx % T
        # y1 = dh  * 10 * (-cos(pi * 2  *idx  /T) + 1)
        # y2  = deg_rad(60)
        # y1 = 2 * y1
        # y2 = 2 * y2
        angle_list[0] = 0
        angle_list[1] = y1
        angle_list[2] = -2 * y1
        angle_list[3] = 0
        angle_list[4] = y2
        angle_list[5] =  -2 * y2
        angle_list[6] = 0
        angle_list[7] = y2
        angle_list[8] =  -2 * y2
        angle_list[9] = 0
        angle_list[10] = y1
        angle_list[11] = -2 * y1

        angle_list = [deg_normalize(low[i], upp[i], angle_list[i] + u0_ang[i]) for i in range(12)]

        return angle_list
    else:
        ans = []
        for i in idx:
            ans.append(sine_gene_pt_step(i, T, rate))
        return ans
def list_pt(act_gen, idx, T):
    ans = []
    for i in range(act_gen.shape[0]):
        ans.append(run_model_with_pt_input(act_gen[i, :], idx[i], T))
    ans = np.vstack(ans).astype(np.float32)
    return  ans

def step_reset(act_gen, idx, T, history, kb, rate):
    # act_gen = np.clip(act_gen- 1, -1, 1)
    if isinstance(idx, list):
        idx = np.array(idx)
        idx = idx % (2 * T)
    else:
        idx = idx % (2 * T)
    act_gen = np.zeros_like(act_gen)
    # print(act_gen.mean())
    sine = sine_gene_pt_step(idx, T, rate)
    ans, history = add_list_np(act_gen, sine, history, kb)
    ans = ans / 180 * 3.14
    # ans = np.array([0] + ans.tolist()[0])
    return ans.astype(np.float32), history.astype(np.float32)


def run_model_with_pt_input_modify(act_gen, idx, T, history, kb, rate):
    # act_gen = np.clip(act_gen- 1, -1, 1)
    # qq = 1 if (idx // (10 * T)) % 2 == 0 else -1
    # print(qq)
    if isinstance(idx, list):
        idx = np.array(idx)
        idx = idx%(2*T)
    else:
        idx = idx % (2 * T)
    #
    act_gen[:, 0] = act_gen[:, 6+0]
    act_gen[:, 1] = act_gen[:, 6+1]
    act_gen[:, 2] = act_gen[:, 6+2]
    act_gen[:, 3] = act_gen[:, 6+3]
    act_gen[:, 4] = act_gen[:, 6+4]
    act_gen[:, 5] = act_gen[:, 6+5]
    # act_gen[:, 10] = act_gen[:, 3+1]
    # act_gen[:, 11] = act_gen[:, 3+2]
    # act_gen[:, 7] = act_gen[:, 3+1]
    # act_gen[:, 8] = act_gen[:, 3+2]

    # act_gen = np.zeros_like(act_gen)
    # print(act_gen.mean())
    sine = sine_gene_pt(idx, T, rate)
    ans, history = add_list_np(act_gen, sine, history, kb)
    ans = ans / 180 * 3.14
    # ans = np.array([qq] + ans.tolist()[0])
    # print(ans.astype(np.float32))
    return ans.astype(np.float32), history.astype(np.float32)

if __name__=='__main__':
    # print(run_model_with_pt_input_modify(np.zeros((1,12)), 0, 30, np.zeros((1,12)), 0.15, 0.6))
    # print(step_reset(np.zeros((1,12)), 0, 30, np.zeros((1,12)), 0.15, 0.6))
    for i in range(100):
        print(run_model_with_pt_input_modify(np.zeros((1,12)), i ,30, np.zeros((1,12)),0.15,0.6))