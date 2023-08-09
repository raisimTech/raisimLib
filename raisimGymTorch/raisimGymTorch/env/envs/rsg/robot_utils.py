from unitree_api import robot as rbt
from unitree_deploy.sine_generator import sine_generator
import signal
import sys
a1 = rbt.Robot()

# todo test this after sleep
def init_position(position, timing = 100):
    a1.go_position(position, timing)
    a1.observe()

def quit_robot(robot):
    robot.back_safe()
    print('Task finished')
    sys.exit(0)

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    quit_robot(a1)

signal.signal(signal.SIGINT, signal_handler)
def init_robot(dt):
    a1.dt = dt
    print(a1.observe())

    act = a1.position
    # print(act)
    a1.init_motor(act)


if __name__=='__main__':
    init_robot(0.01)
    print(a1.observe())
    init_position(position=sine_generator(0, 40, 0.14).tolist())
    #
    while True:
        # print(a1.observe())
        a1.hold_on()
