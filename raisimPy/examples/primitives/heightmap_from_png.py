#!/usr/bin/env python
"""Heightmap from PNG example using the visualizer.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

References:
    - [1] https://github.com/leggedrobotics/raisimOgre/blob/master/examples/src/primitives/heightMapFromPng.cpp
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


def get_quaternion_from_axis_angle(angle, axis):
    """Get the quaternion associated from the axis/angle representation.

    Args:
        angle (float): angle.
        axis (np.array[float[3]], list/tuple of 3 float): 3d axis vector.

    Returns:
        np.array[float[4]]: quaternion [w,x,y,z]
    """
    axis = np.asarray(axis)
    w = np.cos(angle / 2.)
    x, y, z = np.sin(angle / 2.) * axis
    return np.array([w, x, y, z])


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

    # shadow setting
    manager = vis.get_scene_manager()
    manager.set_shadow_technique(raisim.ogre.ShadowTechnique.SHADOWTYPE_TEXTURE_ADDITIVE)
    manager.set_shadow_texture_settings(2048, 3)

    # beyond this distance, shadow disappears
    manager.set_shadow_far_distance(10)
    # size of contact points and contact forces
    vis.set_contact_visual_object_size(0.025, 0.4)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(5)

    # skybox
    quat = get_quaternion_from_axis_angle(angle=np.pi/2, axis=[1., 0., 0.])
    manager.set_skybox(enable=True, material_name="Examples/StormySkyBox", distance=500, render_queue=True,
                       orientation=quat)


if __name__ == '__main__':
    # create raisim world
    world = raisim.World()
    world.set_time_step(0.002)
    world.set_erp(0.003, 0.)

    # start visualizer thread

    # these methods must be called before initApp
    vis = raisim.OgreVis.get()
    vis.set_world(world)
    vis.set_window_size(1200, 600)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(8)

    # init
    vis.init_app()

    # create raisim objects
    sphere1 = world.add_sphere(radius=0.1, mass=10)
    sphere2 = world.add_sphere(radius=0.1, mass=10)

    # the floating base orientation is initialized to identity and all joints and positions are initialized to 1
    resource_path = os.path.dirname(os.path.abspath(__file__)) + "/../rsc/"
    anymal = world.add_articulated_system(resource_path + "ANYmal/anymal.urdf")

    # load heightmap from a png file
    heightmap = world.add_heightmap(resource_path + "heightMap/zurichHeightMap.png", x_center=0, y_center=0,
                                    x_size=100, y_size=100, height_scale=0.0005, height_offset=-10)

    vis.create_graphical_object(sphere1, "sphere1", "gravel")
    vis.create_graphical_object(sphere2, "sphere2", "default")
    vis.create_graphical_object(heightmap, "floor", "default")
    anymal_visual = vis.create_graphical_object(anymal, "anymal")

    sphere1.set_position(0, 0, 5)
    sphere2.set_position(0.5, 0, 3)

    # set camera
    vis.select(anymal_visual[0])
    vis.get_camera_man().set_yaw_pitch_dist(0., -1., 3)

    # run the app
    vis.run()

    # terminate
    vis.close_app()
