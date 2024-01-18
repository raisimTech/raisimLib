import numpy as np
class Robot:
    def __init__(self):
        self.ob_dims = 29
        self.act_dims = 12
        self.position = [0, 0.5233, -1.0466] * 4



    def observe(self):
        return  np.array(
            [[0. ,        0.       ,  0. ,        0.   ,      0.  ,       0.,
              0. ,        0.5233   ,  0., - 1.0466    , 0.      ,   0.,
              0. ,        0.5235988,  0., - 1.0471976 , 0.      ,   0.,
              0. ,        0.5233   ,  0., - 1.0466    , 0.      ,   0.,
              0. ,        0.5235988,  0., - 1.0471976 , 0.]]
        ).astype(np.float32)

    def ob_dim(self):
        return self.ob_dims

    def act_dim(self):
        return self.act_dims

    def take_action(self,action):
        pass

    def init_motor(self,act):
        pass

def init_position(a,b):
    pass

a1 = Robot()