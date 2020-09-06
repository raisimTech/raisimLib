#!/usr/bin/env python
"""Anymal example in raisimLib.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

References:
    - [1] https://github.com/leggedrobotics/raisimLib/blob/master/examples/anymal.cpp
    - [2] raisimLib: https://github.com/leggedrobotics/raisimLib
    - [3] raisimOgre: https://github.com/leggedrobotics/raisimOgre
"""

__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"


import os
import time
import numpy as np
import raisimpy as raisim


sim = raisim.World()
sim.set_time_step(0.002)
checker_board = sim.add_ground()

joint_config = np.array([0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8])
joint_velocity_target = np.zeros(18)
joint_state = np.zeros(18)
joint_vel = np.zeros(18)
joint_p_gain = np.zeros(18)
joint_d_gain = np.zeros(18)
position = np.zeros(3)
orientation = np.zeros([3, 3])

joint_p_gain[-12:] = 200.
joint_d_gain[-12:] = 10.

anymal = sim.add_articulated_system(os.path.dirname(os.path.abspath(__file__)) + "/../rsc/ANYmal/anymal.urdf")
anymal.set_states(joint_config, joint_vel)
anymal.set_control_mode(raisim.ControlMode.PD_PLUS_FEEDFORWARD_TORQUE)
anymal.set_pd_gains(joint_p_gain, joint_d_gain)
anymal.set_pd_targets(joint_config, joint_velocity_target)

n = 1000000
start = time.time()
print("starting...")
for i in range(n):
    sim.integrate()

end = time.time()
print("The simulation of anymal is running at: {:.2f} kHz\n".format(n / (end - start) / 1.e3))

