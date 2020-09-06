#!/usr/bin/env python
"""Chain example using the visualizer.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

References:
    - [1] https://github.com/leggedrobotics/raisimOgre/blob/master/examples/src/robots/chain.cpp
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
    vis.get_light_node().set_direction(normalize([-3., -3., -0.5]))
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
    vis.set_contact_visual_object_size(0.03, 0.2)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(5)


if __name__ == '__main__':
    # create raisim world
    world = raisim.World()
    world.set_time_step(0.001)
    world.set_erp(world.get_time_step() * 0.1, world.get_time_step() * 0.1)

    vis = raisim.OgreVis.get()

    # these methods must be called before initApp
    vis.set_world(world)
    vis.set_window_size(1800, 900)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(8)

    # init
    vis.init_app()

    # create raisim objects
    ground = world.add_ground()
    chain = world.add_articulated_system(os.path.dirname(os.path.abspath(__file__)) + "/../rsc/chain/robot.urdf")

    # create visualizer objects
    vis.create_graphical_object(ground, dimension=50, name="floor", material="default")
    chain_graphics = vis.create_graphical_object(chain, name="chain")

    vis.select(chain_graphics[0])
    vis.get_camera_man().set_yaw_pitch_dist(0., -np.pi / 4., 0.5)
    chain.set_generalized_forces(np.zeros(chain.get_dof()))
    chain.set_control_mode(raisim.ControlMode.FORCE_AND_TORQUE)

    # run the app
    vis.run()

    # terminate
    vis.close_app()
