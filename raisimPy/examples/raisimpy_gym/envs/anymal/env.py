#!/usr/bin/env python
"""ANYmal RL environment using Gym and raisimpy.

The environment is the Python translation (using the `raisimpy` library) of the one described in [1].

References:
    - [1] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/env/ANYmal/Environment.hpp
"""

import numpy as np

import raisimpy as raisim

from raisimpy_gym.envs.anymal import ANYMAL_RESOURCE_DIRECTORY
from raisimpy_gym.envs.raisim_gym_env import RaisimGymEnv, keyboard_interrupt
from raisimpy_gym.envs import reward_logger
from raisimpy_gym.envs.vis_setup_callback import setup_callback


__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"


def get_rotation_matrix_from_quaternion(q):
    """
    Get rotation matrix from the given quaternion.

    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]

    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    w, x, y, z = q
    rot = np.array([[2 * (w**2 + x**2) - 1, 2 * (x*y - w*z), 2 * (x*z + w*y)],
                    [2 * (x*y + w*z), 2 * (w**2 + y**2) - 1, 2*(y*z - w*x)],
                    [2 * (x*z - w*y), 2 * (y*z + w*x), 2 * (w**2 + z**2) - 1]])
    return rot


class AnymalEnv(RaisimGymEnv):
    r"""Anymal environment

    This is a Python translation of [1].

    observation space = [ height                                                      n =  1, si =  0
                          z-axis in world frame expressed in body frame (R_b.row(2))  n =  3, si =  1
                          joint angles,                                               n = 12, si =  4
                          body Linear velocities,                                     n =  3, si = 16
                          body Angular velocities,                                    n =  3, si = 19
                          joint velocities,                                           n = 12, si = 22 ] total 34

    References:
        - [1] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/env/ANYmal/Environment.hpp
    """

    def __init__(self, config, resource_directory=ANYMAL_RESOURCE_DIRECTORY, visualizable=False):
        super(AnymalEnv, self).__init__(config, resource_directory, visualizable)

        # set variables
        self.distribution = lambda: np.random.normal(0.0, 0.2)

        # add objects
        self.robot = self.world.add_articulated_system(self.resource_directory + "anymal.urdf")
        self.robot.set_control_mode(raisim.ControlMode.PD_PLUS_FEEDFORWARD_TORQUE)
        self.ground = self.world.add_ground()
        self.world.set_erp(0., 0.)

        # get robot data (gc = generalized coordinates, gv = generalized velocities)
        self.gc_dim = self.robot.get_generalized_coordinate_dim()
        self.gv_dim = self.robot.get_dof()
        self.num_joints = 12

        # initialize containers
        self.gc, self.gc_init = np.zeros(self.gc_dim), np.zeros(self.gc_dim)
        self.gv, self.gv_init = np.zeros(self.gv_dim), np.zeros(self.gv_dim)
        self.torques = np.zeros(self.gv_dim)
        self.p_targets, self.v_targets = np.zeros(self.gc_dim), np.zeros(self.gv_dim)
        self.p_target12 = np.zeros(self.num_joints)

        # this is the nominal configuration of anymal
        self.gc_init = np.array([0, 0, .54, 1, 0, 0, 0, .03, .4, -.8, -.03, .4, -.8, .03, -.4, .8, -.03, -.4, .8])

        # set PD gains
        self.joint_p_gains, self.joint_d_gains = np.zeros(self.gv_dim), np.zeros(self.gv_dim)
        self.joint_p_gains[-self.num_joints:] = 40.
        self.joint_d_gains[-self.num_joints:] = 1.
        self.robot.set_pd_gains(self.joint_p_gains, self.joint_d_gains)
        self.robot.set_generalized_forces(np.zeros(self.gv_dim))

        # MUST BE DONE FOR ALL ENVIRONMENTS
        self.ob_dim = 34  # convention described on top
        self.action_dim = self.num_joints

        # action and observation scaling
        self.action_mean = self.gc_init[-self.num_joints:]
        self.action_std = 0.6 * np.ones(self.action_dim)

        self.ob_mean = np.array([0.44,  # average height
                                 0.0, 0.0, 0.0,  # gravity axis 3
                                 *self.gc_init[-self.num_joints:],  # joint position 12
                                 *np.zeros(6),  # body linear/angular velocity
                                 *np.zeros(self.num_joints)])  # joint velocity history

        self.ob_std = np.array([0.12,  # average height
                                *np.ones(3) * 0.7,  # gravity axes angles
                                *np.ones(12),  # joint angles
                                *np.ones(3) * 2.0,  # body linear velocity
                                *np.ones(3) * 4.0,  # body angular velocity
                                *np.ones(12) * 10.0,  # joint velocities
                                ])

        # reward coefficients
        self.forward_vel_reward_coeff = float(self.config['environment']['forwardVelRewardCoeff'])
        self.torque_reward_coeff = float(self.config['environment']['torqueRewardCoeff'])
        raisim.gui.reward_logger.init(["forwardVelReward", "torqueReward"])

        # indices of links that should not make contact with ground
        self.foot_indices = set([self.robot.get_body_index(name)
                                 for name in ['LF_SHANK', 'RF_SHANK', 'LH_SHANK', 'RH_SHANK']])

        # visualize
        if self.visualizable:
            vis = raisim.OgreVis.get()

            # these methods must be called before initApp
            vis.set_world(self.world)
            vis.set_window_size(1280, 720)
            vis.set_default_callbacks()
            vis.set_setup_callback(setup_callback)
            vis.set_anti_aliasing(2)

            # starts visualizer thread
            vis.init_app()

            self.robot_visual = vis.create_graphical_object(self.robot, name="ANYmal")
            vis.create_graphical_object(self.ground, dimension=20, name="floor", material="checkerboard_green")
            self.desired_fps = 60.
            vis.set_desired_fps(self.desired_fps)

        # define other variables
        self.forward_vel_reward, self.torque_reward, self.total_reward = 0., 0., 0.
        self.terminal_reward_coeff, self.done = -10., False
        self.ob_double, self.ob_scaled = np.zeros(self.ob_dim), np.zeros(self.ob_dim)
        self.body_linear_vel, self.body_angular_vel = np.zeros(3), np.zeros(3)

    def init(self):
        pass

    def reset(self):
        self.robot.set_states(self.gc_init, self.gv_init)
        self.update_observation()
        if self.visualizable:
            raisim.gui.reward_logger.clean()
        return self.ob_scaled

    @keyboard_interrupt
    def step(self, action):
        # action scaling
        self.p_target12 = np.array(action, dtype=np.float64)
        self.p_target12 *= self.action_std
        self.p_target12 += self.action_mean
        self.p_targets[-self.num_joints:] = self.p_target12

        # set actions
        self.robot.set_pd_targets(self.p_targets, self.v_targets)

        loop_count = int(self.control_dt / self.simulation_dt + 1.e-10)
        vis_decimation = int(1. / (self.desired_fps * self.simulation_dt) + 1.e-10)

        # update world
        for i in range(loop_count):
            self.world.integrate()

            if self.visualizable and self.visualize_this_step and (self.visualization_counter % vis_decimation == 0):
                vis = raisim.OgreVis.get()
                vis.render_one_frame()

            self.visualization_counter += 1

        # update observation
        self.update_observation()

        # update if episode is over or not
        self.is_terminal_state()

        # update reward
        self.update_reward()

        # update extra info
        self.update_extra_info()

        # visualization
        if self.visualize_this_step:
            raisim.gui.reward_logger.log("torqueReward", self.torque_reward)
            raisim.gui.reward_logger.log("forwardVelReward", self.forward_vel_reward)

            # reset camera
            vis = raisim.OgreVis.get()

            vis.select(self.robot_visual[0], False)
            vis.get_camera_man().set_yaw_pitch_dist(3.14, -1.3, 3, track=True)

        return self.ob_scaled, self.total_reward, self.done, self.extra_info

    def update_observation(self):
        self.gc, self.gv = self.robot.get_states()
        self.ob_double, self.ob_scaled = np.zeros(self.ob_dim), np.zeros(self.ob_dim)

        # body height
        self.ob_double[0] = self.gc[2]

        # body orientation
        quat = self.gc[3:7]
        rot = get_rotation_matrix_from_quaternion(quat)
        self.ob_double[1:4] = rot[:, 2]  # numpy is row-major while Eigen is column-major

        # joint angles
        self.ob_double[4:16] = self.gc[-12:]

        # body (linear and angular) velocities
        self.body_linear_vel = rot.dot(self.gv[:3])
        self.body_angular_vel = rot.dot(self.gv[3:6])
        self.ob_double[16:19] = self.body_linear_vel
        self.ob_double[19:22] = self.body_angular_vel

        # joint velocities
        self.ob_double[-12:] = self.gv[-12:]
        self.ob_scaled = np.asarray((self.ob_double - self.ob_mean) / self.ob_std, dtype=np.float)

        return self.ob_scaled

    def observe(self):
        return self.ob_scaled

    def update_reward(self):
        self.torque_reward = self.torque_reward_coeff * np.linalg.norm(self.robot.get_generalized_forces())**2
        self.forward_vel_reward = self.forward_vel_reward_coeff * self.body_linear_vel[0]
        self.total_reward = self.torque_reward + self.forward_vel_reward

        if self.done:
            self.total_reward += self.terminal_reward_coeff

        return self.total_reward

    def update_extra_info(self):
        self.extra_info["forward vel reward"] = self.forward_vel_reward
        self.extra_info["base height"] = self.gc[2]
        return self.extra_info

    def is_terminal_state(self):
        # if the contact body is not the foot, the episode is over
        self.done = False
        for contact in self.robot.get_contacts():
            if contact.get_local_body_index() not in self.foot_indices:
                self.done = True

        return self.done

    def set_seed(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

    def close(self):
        if not self.already_closed:
            if self.visualizable:
                vis = raisim.OgreVis.get()
                vis.close_app()
            self.already_closed = True

    def __del__(self):
        self.close()


# Quick test
if __name__ == '__main__':
    from itertools import count
    env = AnymalEnv(config='default_cfg.yaml', visualizable=True)

    env.reset()
    for t in count():
        action = np.random.normal(0.0, 0.01, size=12)
        obs, reward, done, info = env.step(action)
        print("reward: {}".format(reward))

        # if t == 200:
        #     print("End")
        #     break

