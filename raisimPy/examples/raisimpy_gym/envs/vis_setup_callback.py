#!/usr/bin/env python
"""visualizer setup callback.

This is the Python version (using the ``raisimpy`` library) of the ``visSetupCallback.hpp`` file as it can be found in
[1].

References:
    - [1] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/env/ANYmal/visSetupCallback.hpp
"""

import numpy as np
import raisimpy as raisim

__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)"]
__license__ = "MIT"


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
    manager.set_shadow_far_distance(3)
    # size of contact points and contact forces
    vis.set_contact_visual_object_size(0.03, 0.2)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(5)
