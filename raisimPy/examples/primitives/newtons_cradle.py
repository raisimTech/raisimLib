#!/usr/bin/env python
"""Newtons Cradle example using the visualizer.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

References:
    - [1] https://github.com/leggedrobotics/raisimOgre/blob/master/examples/src/primitives/newtonsCradle.cpp
    - [2] raisimLib: https://github.com/leggedrobotics/raisimLib
    - [3] raisimOgre: https://github.com/leggedrobotics/raisimOgre
"""

__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"


import numpy as np
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
    world.set_time_step(0.001)
    world.set_erp(0.5, 0.)

    # start visualizer thread

    # these methods must be called before initApp
    vis = raisim.OgreVis.get()
    vis.set_world(world)
    vis.set_window_size(1800, 1000)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(2)
    raisim.gui.manual_stepping = True

    # init
    vis.init_app()

    # create raisim objects
    ground = world.add_ground()

    # simulator
    world.set_material_pair_properties(material1="steel", material2="steel", friction=0.1, restitution=1., threshold=0.)

    pins, visual_pins = [], []
    for i in range(4):
        pin = world.add_sphere(radius=0.1, mass=0.8)
        pin.set_position(i * 0.3, 0., 3.)
        pin.set_body_type(raisim.BodyType.STATIC)
        visual_pin = vis.create_graphical_object(pin, name="pin" + str(i + 1), material="lawn_green")
        pins.append(pin)
        visual_pins.append(visual_pin)

    balls = []
    for i, material in enumerate(["dark_red", "gray", "green", "orange"]):
        ball = world.add_sphere(radius=0.1498, mass=0.8, material="steel")
        ball.set_position(i * 0.3, 0., 1.)
        vis.create_graphical_object(ball, "ball" + str(i + 1), material)
        balls.append(ball)

    balls[-1].set_position(2.9, 0., 3.)

    wires = []
    for i in range(4):
        wire = world.add_stiff_wire(object1=pins[i], local_idx1=0, pos_body1=[0., 0., 0.],
                                    object2=balls[i], local_idx2=0, pos_body2=np.zeros(3), length=2.)
        vis.create_graphical_object(wire, name="wire" + str(i + 1), material="red")

    # create visualizer objects
    vis.create_graphical_object(ground, dimension=20, name="floor", material="default")

    # set camera
    camera = vis.get_camera_man().get_camera()
    camera.set_position(0, 3.5, 1.5)
    camera.pitch(1.2)
    vis.select(visual_pins[0][0])  # highlight selected object

    # run the app
    vis.run()

    # terminate
    vis.close_app()
