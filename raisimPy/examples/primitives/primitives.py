#!/usr/bin/env python
"""Primitives example using the visualizer.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

References:
    - [1] https://github.com/leggedrobotics/raisimOgre/blob/master/examples/src/primitives/primitives.cpp
    - [2] raisimLib: https://github.com/leggedrobotics/raisimLib
    - [3] raisimOgre: https://github.com/leggedrobotics/raisimOgre
"""

__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"


import raisimpy as raisim


def setup_callback():
    vis = raisim.OgreVis.get()

    # light
    light = vis.get_light()
    light.set_diffuse_color(1, 1, 1)
    light.set_cast_shadows(True)
    vis.get_light_node().set_position(3, 3, 3)

    # load textures
    vis.add_resource_directory(vis.get_resource_dir() + "/material/gravel")
    vis.load_material("gravel.material")

    vis.add_resource_directory(vis.get_resource_dir() + "/material/checkerboard")
    vis.load_material("checkerboard.material")

    # shadow setting
    manager = vis.get_scene_manager()
    manager.set_shadow_technique(raisim.ogre.ShadowTechnique.SHADOWTYPE_TEXTURE_ADDITIVE)
    manager.set_shadow_texture_settings(2048, 3)

    # scale related settings!! Please adapt it depending on your map size
    # beyond this distance, shadow disappears
    manager.set_shadow_far_distance(60)
    # size of contact points and contact forces
    vis.set_contact_visual_object_size(0.1, 3.0)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(10)


if __name__ == '__main__':
    # create raisim world
    world = raisim.World()
    world.set_time_step(0.003)
    world.set_erp(world.get_time_step(), world.get_time_step())

    # start visualizer thread

    # these methods must be called before initApp
    vis = raisim.OgreVis.get()
    vis.set_world(world)
    vis.set_window_size(1800, 1000)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(2)

    # init
    vis.init_app()

    # vis = raisim.OgreVis(world)  # You could have called this instead of writing lines [65-75]

    # create raisim objects
    ground = world.add_ground()

    cubes, spheres, capsules, cylinders = [], [], [], []

    N = 6
    for i in range(N):
        for j in range(N):
            for k in range(N):
                number = str(i) + str(j) + str(k)
                res = (i + j + k) % 4

                if res == 0:
                    cubes.append(world.add_box(1, 1, 1, 1))
                    vis.create_graphical_object(cubes[-1], name="cube" + number, material="red")
                    ob = cubes[-1]
                elif res == 1:
                    spheres.append(world.add_sphere(0.5, 1))
                    vis.create_graphical_object(spheres[-1], name="sphere" + number, material="green")
                    ob = spheres[-1]
                elif res == 2:
                    capsules.append(world.add_capsule(0.5, 1., 1))
                    vis.create_graphical_object(capsules[-1], name="capsule" + number, material="blue")
                    ob = capsules[-1]
                else:
                    cylinders.append(world.add_cylinder(0.5, 0.5, 1))
                    vis.create_graphical_object(cylinders[-1], name="cylinder" + number, material="default")
                    ob = cylinders[-1]

                ob.set_position(-N + 2. * i, -N + 2. * j, N * 2. + 2. * k)

    # create visualizer objects
    vis.create_graphical_object(ground, dimension=20, name="floor", material="default")

    # set camera
    camera = vis.get_camera_man().get_camera()
    camera.set_position(0, -N * 3.5, N * 1.5)
    camera.pitch(1.2)

    # run the app
    vis.run()

    # terminate
    vis.close_app()
