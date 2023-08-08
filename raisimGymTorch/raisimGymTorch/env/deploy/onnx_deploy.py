import onnxruntime as ort
import onnx
# from mlprodict.onnxrt import OnnxInference
from math import cos,pi
import numpy as np
# 加载模型

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
def ang_trans(lower, upper, x):
    """
    trans the rate x to the angle between lower and upper
    """
    x = (x+1)/2 #todo for test
    return x * upper + (1-x) * lower

def norm(lower, upper, x):
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
u0 = [0, 30, -60,0, 30, -60,0, 30, -60,0, 30, -60]
u0 = deg_normalize(low, upp, u0)
# u0 = [-deg_normalize(low, upp, u0)[i] if deg_normalize(low, upp, u0)[i]!=0 else 0 for i in range(12)]
# print(u0)
history_u = u0
model = ort.InferenceSession('/home/lr-2002/code/raisimLib/raisimGymTorch/raisimGymTorch/env/deploy/new.onnx')

idx_local = 0

# 准备输入
input_name = model.get_inputs()[0].name
# print(model.get_outputs())
output_name = model.get_outputs()[2].name
print(input_name, output_name)
# input_data = np.random.rand(1, 34).astype(np.float32)

def sine_gene_test_qua(idx, T):
    angle_list = [0 for i in range(12)]
    if idx >= 2 * T:
        idx = 0
    dh = 1 # todo for test
    if idx >= 0 and idx <= T:
        tp0 =idx - 0
        y1 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
        y2 = 0
    elif idx>T and idx<=2*T:
        tp0 =idx - T
        y2 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
        y1 = 0


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
    angle_list = [deg_normalize(low[i], upp[i], angle_list[i] + ang_trans(low[i], upp[i], u0[i])) for i in range(12)]
    return angle_list


def sine_gene(idx, T):
    angle_list = [0 for i in range(12)]
    if idx >= 2 * T:
        idx = 0
    dh = 2 # todo for test
    if idx >= 0 and idx <= T:
        tp0 =idx - 0
        y1 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
        y2 = 0
    elif idx>T and idx<=2*T:
        tp0 =idx - T
        y2 = dh * 10 * (-cos(pi * 2 * tp0 / T) + 1 ) / 2
        y1 = 0


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
    angle_list = [deg_normalize(low[i], upp[i], angle_list[i] + ang_trans(low[i], upp[i], u0[i])) for i in range(12)]
    return angle_list

def rate_to_act(lower, upper, rate):
    if isinstance(lower, list):
        return [rate_to_act(a,b,c) for a,b,c in zip(lower,upper, rate)]
    else:
        norm_for_act = lambda a: (a+1)*0.5
        lerp = lambda a, b, c: a*(1-c) + b * c
        return lerp(lower, upper, norm_for_act(rate))

def add_list(act_gen, sine):
    global history_u
    kk = 0.9
    kf = 1
    kb = 0.5
    history_u = [history_u[i] * kk + (1-kk) * act_gen[i] for i in range(12)] # todo the act_gen[i] is to big
    clip = lambda a, b, c: np.clip(a, b, c)


    # f0 = lambda a: a
    # f1 = lambda a, b, c: a

    ans = [rate_to_act(low[i], upp[i], clip(kb * history_u[i] + kf * sine[i], -1, 1)) for i in range(12)]
    ans = [deg_rad(x) for x in ans]
    # todo swap the ans
    # for i in range(6):
    #     tmp = ans[i+6]
    #     ans[i+6] = ans[i]
    #     ans[i] = tmp
    return ans


def run_model(observation, idx, T):
    # todo idx should return 0 when fall
    # idx = idx % T
    act_gen = model.run([output_name], input_feed={input_name: observation})
    act_gen = act_gen[0]
    act_gen = np.zeros((1,12))
    # [array([[]])]
    sine = sine_gene(idx, T)
    sine = sine_gene_test_qua(idx, T)
    # sine = [0 for x in range(12)]
    # print(sine,act_gen)
    ans = []
    for act in act_gen:
        ans.append(add_list(act, sine))

    ans = [np.array(ans).astype(np.float32)]

    return ans


def run_model1(observation):
    return model.run([output_name], input_feed={input_name: observation})

# 运行模型

# outputs = model.run([output_name], input_feed={input_name: input_data})
# print(outputs)
# x = run_model(input_data)
# print(x)
if __name__=="__main__":
    # test_exp = np.zeros((1,34))
    # test_exp1 = np.zeros((1,34))
    # for i in range(12):
    #     test_exp[0][i * 2 + 10] = -u0[i]
    #     test_exp1[0][i * 2 + 10] = u0[i]
    # # print(test_exp)
    #
    # ans = run_model(test_exp.astype(np.float32),0, 50)
    # ans1 = run_model(test_exp1.astype(np.float32),0, 50)
    # # ans1 = run_model(np.zeros((1,34)).astype(np.float32))
    # print(ans)
    # print(ans1)
    # print(u0)

    a = [ang_trans(low[i], upp[i], u0[i]) for i in range(12)]
    print(a)
    print(u0)
    uu = rate_to_act(low, upp, u0)
    uu = deg_rad(uu)
    print(uu)
    # norm_for_act = lambda a: (a + 1) * 0.5
    # lerp = lambda a, b, c: a * (1 - c) + b * c
    # print(deg_normalize(-10,10,6))
    # print(norm_for_act(deg_normalize(-10,10,6)))
    # print(lerp(-10, 10,norm_for_act(deg_normalize(-10,10,6)) ))

    # import matplotlib.pyplot as plt
    # x = [x for x in range(200)]
    # y = [sine_gene(a ,50) for a in x]
    # y =np.array(y).transpose()
    #
    # for i in range(12):
    #
    #     plt.plot(x,y[i])
    # plt.show()

"""
def test_rad_deg():
    assert rad_deg(1.57)-90 <=1
def test_rad_deg_list():
    assert isinstance(rad_deg([1, 2,3]), list)

def test_deg_rad():
    assert deg_rad(90) - 1.57 <= 0.05

def test_deg_rad_list():
    assert isinstance(deg_rad([1, 2, 3]), list)

"""
# def test_sine_gene():
#     for i in range(0, 200, 10):
#         a = sine_gene(i, 50)
#         print(a)
#         assert max(a) <= pi and min(a) >= 0

# todo
"""
todo
1. on action 的范围
2. 似乎是角度问题？应该是范围需要考虑是 角度还是弧度


老师的输入是deg or rad
what's my output


using deg to get the rate 
and feed the rate to the env

"""


def test_run_model():
    ans = run_model(np.zeros((1, 34)).astype(np.float32), 10, 50)
    ans1 = run_model1(np.zeros((1, 34)).astype(np.float32))
    assert ans1[0] == ans[0]

def test_norm():
    assert deg_normalize(-10, 10, -10) == -1