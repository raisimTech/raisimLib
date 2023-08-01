import numpy as np
class Robot():
    def __init__(self):
        self.robot = None
        self.ob_dims = self.robot.ob_dims if self.robot is not None else 34
        self.act_dims = self.robot.act_dims if self.robot is not None else 12
        pass


    def connect(self):
        """
        connect to A1 with high level first
        """

        pass

    def ob_dim(self) -> int:
        return self.ob_dims

    def act_dim(self) -> int:
        return self.act_dims

    def alive(self) -> bool:
        return True

    def observe(self) -> np.ndarray:
        return np.random.rand(1, 34).astype(np.float32)
        pass

    def take_action(self, act):
        """
        upd recv and send
        """
        pass

    def reset(self):
        """
        how to implement
        maybe to be implement with stand
        """
        self.robot.reset()



