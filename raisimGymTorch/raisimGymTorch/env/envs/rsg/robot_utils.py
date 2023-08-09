from unitree_api import robot as rbt
from unitree_deploy.sine_generator import sine_generator
a1 = rbt.Robot()

# todo test this after sleep
def init_position(position, timing = 100):
    a1.go_position(position, timing)

def init_robot():
    a1.observe()
    act = a1.position
    a1.init_motor(act)


if __name__=='__main__':
    init_robot()

    init_position(position=sine_generator(0, 40, 0.14))
