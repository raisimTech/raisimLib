import time

import onnxruntime as ort
# import onnx_file
# from mlprodict.onnxrt import OnnxInference
from math import cos,pi
import numpy as np
from raisimGymTorch.deploy_utils.angle_utils import deg_rad, rad_deg
from raisimGymTorch.deploy_log.draw_map import Drawer
# 加载模型



def ang_trans(lower, upper, x):
    """
    trans the rate x to the angle
    """
    if isinstance(lower, list):
        return [ang_trans(lower[i], upper[i], x[i]) for i in range(len(lower))]
    else:
        x = (x+1)/2 #todo for test
        return x * upper + (1-x) * lower

def norm(lower, upper, x):
    # print(lower, upper, x)
    # assert abs((x - lower) / (upper - lower) ) <= 1, f"{abs((x - lower) / (upper - lower) )} {x, upper, lower} "
    return (x - lower) / (upper - lower)

def rad_normalize(lower, upper,x):
    if isinstance(lower, list):
        ans = []
        for a,b,c in zip(lower, upper, x):
            ans.append(rad_normalize(a,b,c))
        return ans
    else:
        ang1 = x * 180 / pi
        ang1 = norm(lower, upper, ang1)
        return 2*ang1 - 1

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
        return 2 * norm(lower, upper, x) - 1 #todo for test
        # return 1 - 2* norm(lower, upper,x)


low = [-46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5, -46, -60, -154.5]
upp = [46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5, 46, 240, -52.5]
tha1,tha2 = 60,  30
u0 = [0, tha2, -2*tha2, 0 ,tha1, -tha1 * 2, 0 , tha2, -2*tha2, 0, tha1,-2*tha1]

low_np = np.array(low)
upp_np = np.array(upp)
# u0_rad = deg_rad(u0)
# u0 = [0] * 12 # todo sure ? u0 = 0
# low_rad = deg_rad(low)
# upp_rad = deg_rad(upp)
u0 = deg_normalize(low, upp, u0)
u0_ang = ang_trans(low, upp, u0)
# u0_rad = rad_normalize(low_rad, upp_rad, u0_rad)
# u0 = [-deg_normalize(low, upp, u0)[i] if deg_normalize(low, upp, u0)[i]!=0 else 0 for i in range(12)]
# print(u0)
history_u = [0] * 12
# history_u_rad = [0] * 12/
# history_u = u0.copy()
model = ort.InferenceSession('/env/deploy_utils/onnx_file/2203.onnx')

idx_local = 0

# 准备输入
input_name = model.get_inputs()[0].name
# print(model.get_outputs())
output_name = model.get_outputs()[2].name
print(input_name, output_name)
# input_data = np.random.rand(1, 34).astype(np.float32)

# def sine_gene_test_qua(idx, T):
#     angle_list = [0 for i in range(12)]
#     if idx >= 2 * T:
#         idx = 0
#     dh = 1.2 # todo for test
#     if idx >= 0 and idx <= T:
#         tp0 =idx - 0
#         y1 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
#         y2 = 0
#     elif idx>T and idx<=2*T:
#         tp0 =idx - T
#         y2 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
#         y1 = 0
#
#
#     angle_list[0] = 0
#     angle_list[1] = y1
#     angle_list[2] = -2 * y1
#     angle_list[3] = 0
#     angle_list[4] = y2
#     angle_list[5] = -2 * y2
#     angle_list[6] = 0
#     angle_list[7] = y1
#     angle_list[8] = -2 * y1
#     angle_list[9] = 0
#     angle_list[10] = y2
#     angle_list[11] = -2 * y2
#     angle_list = [deg_normalize(low[i], upp[i], angle_list[i] + ang_trans(low[i], upp[i], u0[i])) for i in range(12)]
#     return angle_list


def sine_gene(idx, T):
    # print(idx)
    if isinstance(idx, np.ndarray) or isinstance(idx, list):
        mat = np.zeros((len(idx), 12))
        i_set = set(idx)
        idd = {}
        for i in i_set:
            idd[i] = sine_gene(i, 40)
        for i in range(len(idx)):
            mat[i] = idd[idx[i]]
        return mat
    if not isinstance(idx, list):
        angle_list = [0 for i in range(12)]
        if idx >= 2 * T:
            idx = 0
        dh = 1.6 # todo for test

        if idx >= 0 and idx <= T:
            tp0 =idx - 0
            y1 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            y2 = 0
            y2 = y1
        elif idx>T and idx<=2*T:
            tp0 =idx - T
            y2 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
            y1 = 0
            y1 = y2


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
        # print("ang_list ", angle_list)
        angle_list = [deg_normalize(low[i], upp[i], angle_list[i] + ang_trans(low[i], upp[i], u0[i]) ) for i in range(12)]
        # print("ang_list ", angle_list)
        return angle_list
    else:
        ans = []
        for i in idx:
            ans.append(sine_gene(i, T))
        return ans

def sine_gene_pt(idx, T):
    # print(idx)
    if isinstance(idx, np.ndarray) or isinstance(idx, list):
        mat = np.zeros((len(idx), 12))
        i_set = set(idx)
        idd = {}
        for i in i_set:
            idd[i] = sine_gene_pt(i, 40)
        for i in range(len(idx)):
            mat[i] = idd[idx[i]]
        return mat
    if not isinstance(idx, list):
        angle_list = [0 for i in range(12)]
        if idx >= 2 * T:
            idx = 0
        dh =-0.5#:w


        # if idx >= 0 and idx <= T:
        #     tp0 =idx - 0
        #     y1 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
        #     y2 = 0
        #     # y2 = y1
        # elif idx>T and idx<=2*T:
        #     tp0 =idx - T
        #     y2 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
        #     y1 = 0
            # y1 = y2
        idx = idx % T
        y1 = dh  * 10 * (-cos(pi * 2  *idx  /T) + 1)
        y2  = deg_rad(60)
        angle_list[0] = 0
        angle_list[1] = y1
        angle_list[2] = -2 * y1
        angle_list[3] = 0
        angle_list[4] = y2
        angle_list[5] = -2 * y2
        angle_list[6] = 0
        angle_list[7] = y1
        angle_list[8] = -2 * y1
        angle_list[9] = 0
        angle_list[10] = y2
        angle_list[11] = -2 * y2

        angle_list = [deg_normalize(low[i], upp[i], angle_list[i] + u0_ang[i]) for i in range(12)]

        return angle_list
    else:
        ans = []
        for i in idx:
            ans.append(sine_gene_pt(i, T))
        return ans


def rate_to_act(lower, upper, rate):
    if isinstance(lower, list):
        return [rate_to_act(a,b,c) for a,b,c in zip(lower,upper, rate)]
    else:
        norm_for_act = lambda a: (a+1)*0.5
        lerp = lambda a, b, c: a*(1-c) + b * c
        # print("11111 ", lower, upper, rate, lerp(lower, upper, norm_for_act(rate)))

        return lerp(lower, upper, norm_for_act(rate))

def clip(a,b,c):
    if isinstance(a, np.ndarray):
        return np.clip(a,b,c)

    if a <= b :
        a = b
    if a >= c:
        a = c
    return a

def add_list(act_gen, sine, history=None):
    if isinstance(act_gen, np.ndarray):
        assert act_gen.shape[0] == 12
    else:
        assert len(act_gen) == 12
    kk = 0.9
    kf = 1
    kb = 0.4
    if history is None:
        global history_u

        # print(history_u, act_gen)
        history_u = [history_u[i] * kk + (1-kk) * act_gen[i] for i in range(12)] # todo the act_gen[i] is to big
        # print("11111", history_u, act_gen)

        ans = [rate_to_act(low[i], upp[i], clip(kb * history_u[i] + kf * sine[i], -1, 1)) for i in range(12)]
        ans = [deg_rad(x) for x in ans]
        return ans
    else:
        history = [history[i] * kk + (1-kk) * act_gen[i] for i in range(12)] # todo the act_gen[i] is to big
        ans = [rate_to_act(low[i], upp[i], clip(kb * history[i] + kf * sine[i], -1, 1)) for i in range(12)]
        ans = [deg_rad(x) for x in ans]
        return ans, history

def lerp_np(a, b, c):
    return a* (1-c) + b*c

def add_list_np(act_gen, sine, history):
    kk = 0.9
    kf = 1
    kb = 0.2
    history = history*kk + (1-kk) * act_gen
    ans = np.clip(kb*history + kf * sine, -1, 1)
    ans = (ans + 1) /2  # 100 * 12
    ans = lerp_np(low_np, upp_np, ans)

    return ans, history


def run_model_with_pt_input_modify(act_gen, idx, T, history):
    act_gen = np.clip(act_gen- 1, -1, 1)
    if isinstance(idx, list):
        idx = np.array(idx)
        idx = idx%(2*T)
    else:
        idx = idx % (2 * T)

    sine = sine_gene_pt(idx, T)
    ans, history = add_list_np(act_gen, sine, history)
    ans = ans / 180 * 3.14

    return ans.astype(np.float32), history.astype(np.float32)



def list_pt(act_gen, idx, T):
    ans = []
    for i in range(act_gen.shape[0]):
        ans.append(run_model_with_pt_input(act_gen[i, :], idx[i], T))
    ans = np.vstack(ans).astype(np.float32)
    return  ans

def run_model(observation, idx, T, save_gen=None):
    # todo idx should return 0 when fall
    idx = idx % (2 * T)
    # print(observation.shape)
    # obs = rad_deg(observation)
    observation[:, [0,1]] = observation[:, [1,0]]
    observation[:, [1]] = - observation[:, [1]]

    # observation[:,[2,3,4]] =  rad_deg(observation[:, [2,3,4]])
    # observation[:,[2,3,4]] =  observation[:, [2,3,4]]
    observation[:,[2,3,4]] =  observation[:, [3,4,2]]
    # observation[:,[2,3,4]] =  rad_deg(observation[:, [3,4,2]])
    observation[:, [3, 4]] = - observation[:, [3, 4]]
    # observation[:, [4]
    if observation.shape[1] == 29:
        offset = 0
    elif observation.shape[1] == 26:
        offset = 3
    else:
        offset = 5
    # print('offset' ,offset)
    be_obe =[5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28]
    aft_obs = [23,24,25,26,27,28,17,18,19,20,21,22,11,12,13,14,15,16,5,6,7,8,9,10]
    be_obe = [i -offset for i in be_obe]
    aft_obs=[i -offset for i in aft_obs]
    # print(be_obe, aft_obs)
    observation[: ,be_obe]=\
    observation[: ,aft_obs]


    act_gen = model.run([output_name], input_feed={input_name: observation})
    act_gen = act_gen[0]

    # minu = np.array([0, 0, deg_rad(30), 0, deg_rad(-60), 0] * 4)  # 6
    # observation[:, -24:] -= minu

    act_gen = np.zeros((1,12))
    if save_gen is not None:
        save_gen.add_list(act_gen[0])

    sine = sine_gene(idx, T)
    ans = []
    for act in act_gen:
        ans.append(add_list(act, sine))
    ans = np.array(ans).astype(np.float32)
    ans[:, [0,1,2,3,4,5,6,7,8,9,10,11]] = \
    ans[:, [9,10,11,6,7,8,3,4,5,0,1,2]]
    ans = [ans]

    return ans, observation


def run_model1(observation):
    return model.run([output_name], input_feed={input_name: observation})

if __name__=="__main__":

    z = []
    minn = 100
    minnn =[]
    for i in range(100):
        # x = run_model(np.zeros((1,26)).astype(np.float32), i, 40)[0][0][0]
        x = add_list(np.zeros((12)), sine_gene(i, 40))
        if i == 0:
            print(x)
        if x[1] <= minn:
            print(x[1], minn)
            minn = x[1]
            minnn =x
        z.append(x)
    z = np.vstack(z)
    print(minnn)
    x= [i for i in range(z.shape[0])]
    import matplotlib.pyplot as plt
    plt.plot(x, z)
    plt.show()



def test_run_model():
    ans = run_model(np.zeros((1, 34)).astype(np.float32), 10, 50)
    ans1 = run_model1(np.zeros((1, 34)).astype(np.float32))
    assert ans1[0] == ans[0]

def test_norm():
    assert deg_normalize(-10, 10, -10) == -1