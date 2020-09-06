#!/usr/bin/env python
"""Heightmap example using the visualizer.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

References:
    - [1] https://github.com/leggedrobotics/raisimOgre/blob/master/examples/src/robots/heightMap.cpp
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
    light.set_direction(normalize([1., 1., -1.]))

    vis.set_camera_speed(300)

    # load tetxture
    vis.add_resource_directory(vis.get_resource_dir() + "/material/gravel")
    vis.load_material("gravel.material")

    # shadow setting
    manager = vis.get_scene_manager()
    manager.set_shadow_technique(raisim.ogre.ShadowTechnique.SHADOWTYPE_TEXTURE_ADDITIVE)
    manager.set_shadow_texture_settings(2048, 3)

    # beyond this distance, shadow disappears
    manager.set_shadow_far_distance(10)
    # size of contact points and contact forces
    vis.set_contact_visual_object_size(0.03, 0.4)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(5)


if __name__ == '__main__':
    # create raisim world
    world = raisim.World()
    world.set_time_step(0.002)

    # these methods must be called before initApp
    vis = raisim.OgreVis.get()
    vis.set_world(world)
    vis.set_window_size(1800, 1200)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(8)
    raisim.gui.manual_stepping = True

    # init
    vis.init_app()

    # create raisim objects
    sphere1 = world.add_sphere(radius=0.1, mass=10)
    sphere2 = world.add_sphere(radius=0.1, mass=10)
    anymal = world.add_articulated_system(os.path.dirname(os.path.abspath(__file__)) + "/../rsc/ANYmal/anymal.urdf")
    sphere1.set_position(0, 0, 5)
    sphere2.set_position(0.5, 0, 3)
    world.set_erp(world.get_time_step() * .1, world.get_time_step() * .1)

    # create heightmap
    terrain_properties = raisim.TerrainProperties()
    terrain_properties.frequency = 0.2
    terrain_properties.z_scale = 3.0
    terrain_properties.x_size = 20.0
    terrain_properties.y_size = 20.0
    terrain_properties.x_samples = 50
    terrain_properties.y_samples = 50
    terrain_properties.fractal_octaves = 3
    terrain_properties.fractal_lacunarity = 2.0
    terrain_properties.fractal_gain = 0.25
    heightmap = world.add_heightmap(x_center=0., y_center=0., terrain_properties=terrain_properties)

    # create visualizer objects
    vis.create_graphical_object(heightmap, name="terrain", material="default")
    vis.create_graphical_object(sphere1, name="sphere1", material="gravel")
    vis.create_graphical_object(sphere2, name="sphere2", material="default")

    # ANYmal joint PD controller
    joint_nominal_config = np.array([0, 0, 0, 0, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03,
                                     -0.4, 0.8])
    joint_velocity_target = np.zeros(18)
    joint_state = np.zeros(18)
    joint_vel = np.zeros(18)
    joint_force = np.zeros(18)
    joint_p_gain = np.zeros(18)
    joint_d_gain = np.zeros(18)

    joint_p_gain[-12:] = 200.
    joint_d_gain[-12:] = 10.

    anymal_graphics = vis.create_graphical_object(anymal, name="ANYmal")
    anymal.set_generalized_coordinates([0, 0, 3.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03,
                                        -0.4, 0.8, -0.03, -0.4, 0.8])

    vis.select(anymal_graphics[0], False)
    vis.get_camera_man().set_yaw_pitch_dist(0., -np.pi/4., 1)
    anymal.set_generalized_forces(np.zeros(anymal.get_dof()))
    anymal.set_control_mode(raisim.ControlMode.PD_PLUS_FEEDFORWARD_TORQUE)
    anymal.set_pd_gains(joint_p_gain, joint_d_gain)
    anymal.set_pd_targets(joint_nominal_config, joint_velocity_target)

    # run the app
    vis.run()

    # terminate
    vis.close_app()
