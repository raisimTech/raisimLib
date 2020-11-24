import os
import numpy as np
import raisimpy as raisim
import time

raisim.World.set_license_file(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
anymal_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/anymal/urdf/anymal.urdf"
laikago_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/laikago/laikago.urdf"
atlas_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/atlas/robot.urdf"
monkey_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/monkey/monkey.obj"
dummy_inertia = np.zeros([3, 3])
np.fill_diagonal(dummy_inertia, 0.1)

world = raisim.World()
world.set_time_step(0.001)
server = raisim.RaisimServer(world)
ground = world.add_ground()

anymal = world.add_articulated_system(anymal_urdf_file)
anymal.set_name("anymal")
anymal_nominal_joint_config = np.array([0, -1.5, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8,
                                        -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8])
anymal.set_generalized_coordinates(anymal_nominal_joint_config)
anymal.set_pd_gains(200*np.ones([18]), np.ones([18]))
anymal.set_pd_targets(anymal_nominal_joint_config, np.zeros([18]))

laikago = world.add_articulated_system(laikago_urdf_file)
laikago.set_name("laikago")
laikago_nominal_joint_config = np.array([0, 1.5, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1,
                                         0.00, 0.5, -1, 0, 0.5, -0.7])
laikago.set_generalized_coordinates(laikago_nominal_joint_config)
laikago.set_pd_gains(200*np.ones([18]), np.ones([18]))
laikago.set_pd_targets(laikago_nominal_joint_config, np.zeros([18]))

atlas = world.add_articulated_system(atlas_urdf_file)
atlas.set_name("atlas")
atlas_nominal_joint_config = np.zeros(atlas.get_generalized_coordinate_dim())
atlas_nominal_joint_config[2] = 1.5
atlas_nominal_joint_config[3] = 1
atlas.set_generalized_coordinates(atlas_nominal_joint_config)


server.launch_server(8080)

for i in range(5):
    for j in range(5):
        object_type = (i + j*6) % 5

        if object_type == 0:
            obj = world.add_mesh(monkey_file, 5.0, dummy_inertia, np.array([0, 0, 0]), 0.3)
        elif object_type == 1:
            obj = world.add_cylinder(0.2, 0.3, 2.0)
        elif object_type == 2:
            obj = world.add_capsule(0.2, 0.3, 2.0)
        elif object_type == 3:
            obj = world.add_box(0.4, 0.4, 0.4, 2.0)
        else:
            obj = world.add_sphere(0.3, 2.0)

        obj.set_position(i-2.5, j-2.5, 5)

time.sleep(2)
world.integrate1()

### get dynamic properties
# mass matrix
mass_matrix = anymal.get_mass_matrix()

# non-linear term (gravity+coriolis)
non_linearities = anymal.get_non_linearities()

for i in range(50000):
    world.integrate()

server.kill_server()
