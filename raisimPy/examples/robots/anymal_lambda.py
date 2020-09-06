#!/usr/bin/env python
"""ANYmal example using the visualizer.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

Note that in this example we use a decorator on the control function which is given to the `set_control_callback`,
instead of a callable class as done in `anymal.py`. The rest of the code is the same.

References:
    - [1] https://github.com/leggedrobotics/raisimOgre/blob/master/examples/src/robots/anymal.cpp
    - [2] raisimLib: https://github.com/leggedrobotics/raisimLib
    - [3] raisimOgre: https://github.com/leggedrobotics/raisimOgre
"""

__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"

import os
import numpy as np
import raisimpy as raisim


def normalize(array):
    return np.asarray(array) / np.linalg.norm(array)


def setup_callback():
    vis = raisim.OgreVis.get()

    # light
    light = vis.get_light()
    light.set_diffuse_color(1, 1, 1)
    light.set_cast_shadows(True)
    light.set_direction(normalize([-3., -3., -0.5]))
    vis.set_camera_speed(300)

    # load textures
    vis.add_resource_directory(vis.get_resource_dir() + "/material/checkerboard")
    vis.load_material("checkerboard.material")

    # shadow setting
    manager = vis.get_scene_manager()
    manager.set_shadow_technique(raisim.ogre.ShadowTechnique.SHADOWTYPE_TEXTURE_ADDITIVE)
    manager.set_shadow_texture_settings(2048, 3)

    # scale related settings!! Please adapt it depending on your map size
    # beyond this distance, shadow disappears
    manager.set_shadow_far_distance(10)
    # size of contact points and contact forces
    vis.set_contact_visual_object_size(0.03, 0.6)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(5)


if __name__ == '__main__':
    # create raisim world
    world = raisim.World()
    world.set_time_step(0.0025)

    vis = raisim.OgreVis.get()

    # these methods must be called before initApp
    vis.set_world(world)
    vis.set_window_size(1800, 1200)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(2)

    # starts visualizer thread
    vis.init_app()

    # create raisim objects
    ground = world.add_ground()
    ground.set_name("checkerboard")

    anymals = []

    # create visualizer objects
    vis.create_graphical_object(ground, dimension=20, name="floor", material="checkerboard_green")

    # ANYmal joint PD controller
    joint_nominal_config = np.zeros(19)
    joint_velocity_target = np.zeros(18)
    joint_state = np.zeros(18)
    joint_force = np.zeros(18)
    joint_p_gain = np.zeros(18)
    joint_d_gain = np.zeros(18)

    joint_p_gain[-12:] = 200.
    joint_d_gain[-12:] = 10.

    N = 1
    anymal_urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../rsc/ANYmal/anymal.urdf"

    for i in range(N):
        for j in range(N):
            anymals.append(world.add_articulated_system(anymal_urdf_path))
            vis.create_graphical_object(anymals[-1], name="ANYmal" + str(i) + "X" + str(j))
            anymals[-1].set_generalized_coordinates([2.*i, j, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8,
                                                     -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8])
            anymals[-1].set_generalized_forces(np.zeros(anymals[-1].get_dof()))
            anymals[-1].set_control_mode(raisim.ControlMode.PD_PLUS_FEEDFORWARD_TORQUE)
            anymals[-1].set_pd_gains(joint_p_gain, joint_d_gain)
            anymals[-1].set_name("anymal" + str(j + i*N))

    distribution = lambda: np.random.normal(0.0, 0.2)
    anymals[-1].print_body_names_in_order()

    def decorate():
        def controller():
            controller.decimation += 1

            if controller.decimation % 2500 == 0:
                for i in range(N):
                    for j in range(N):
                        anymals[i * N + j].set_generalized_coordinates([2. * i, j, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                                                        -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03,
                                                                        -0.4, 0.8])
            if controller.decimation % 50 != 0:
                return

            # ANYmal joint PD controller
            joint_nominal_config = np.array([0, 0, 0, 0, 0, 0, 0, 0.03, 0.3, -.6, -0.03, 0.3, -.6, 0.03, -0.3, .6,
                                             -0.03, -0.3, .6])
            joint_velocity_target = np.zeros(18)

            for i in range(N):
                for j in range(N):
                    joint_config = joint_nominal_config + distribution()
                    anymals[i * N + j].set_pd_targets(joint_config, joint_velocity_target)

        setattr(controller, "decimation", 0)
        return controller  # return the controller function

    vis.set_control_callback(decorate())

    # set camera
    camera = vis.get_camera_man().get_camera()
    camera.set_position(N, -2 - N, 1.5 + N)
    camera.pitch(1.)

    # run the app
    vis.run()

    # terminate
    vis.close_app()
