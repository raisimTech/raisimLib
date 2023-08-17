import random

import matplotlib.pyplot as plt
import numpy as np
import time
map_mapping = []
class Drawer:
    def __init__(self, name, path=None):
        self.name = name
        self.path = name+'.png' if path == None else path+name+'.png'
        self.draw_list = []

    def add_map_list(self, l):
        self.draw_list.append(l.copy())

    def draw(self):
        plt.figure(figsize=(10,10))
        now = np.stack(self.draw_list)
        now = now.transpose()
        le = now.shape[1]
        color = plt.cm.rainbow(np.linspace(0, 1, now.shape[0]))
        x = [i for i in range(le)]
        print(le, now.shape)
        print(now)
        for i, c in enumerate(color):
            plt.plot(x, now[i], c=c)
        plt.savefig(self.name)

        plt.show()

class DynDrawer:
    def __init__(self):
        plt.ion()
        plt.figure(1)
        self.cnt = 0

    def append_data(self,data):
        plt.plot(self.cnt, data, '.')
        self.cnt+=1
        plt.pause(0.01)

    def end(self):
        plt.savefig('./test.png')




if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import numpy as np
    import time
    from math import *

    # plt.ion()  # 开启interactive mode 成功的关键函数
    # plt.figure(1)
    # t = [0]
    # t_now = 0
    # m = [sin(t_now)]
    # mm = [0]
    # for i in range(200):
    #     # plt.clf() # 清空画布上的所有内容。此处不能调用此函数，不然之前画出的点，将会被清空。
    #     t_now = i * 0.1
    #     """
    #     由于第次只画一个点，所以此处有两种方式，第一种plot函数中的样式选
    #     为点'.'、'o'、'*'都可以，就是不能为线段'-'。因为一条线段需要两
    #     个点才能确定。第二种方法是scatter函数，也即画点。
    #     """
    #     plt.plot(t_now, sin(t_now), '.')  # 第次对画布添加一个点，覆盖式的。
    #
    #     plt.plot(t_now, cos(t_now), 'o')
    #
    #     plt.pause(0.01)
    # plt.savefig('test.png')
    dyn = DynDrawer()
    for i in range(100):
        dyn.append_data(0.01)