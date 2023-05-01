##########################################################
RaiSim World Configuration File
##########################################################

We provide a few examples `here <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts>`_.

The following describes the RaiSim world configuration xml convention.
The (optional) tag means that the element is optional given the parent.
If the element is not marked (optional), it must exist given that the parent exist.
The (multiple) tag means that there can be multiple elements for the same parent.

1. raisim: Top most node.
    1. <attribute> ``version`` : Describes the version of RaiSim that created the configuration file. The file might be read by different version.
    2. <child> (optional) ``material`` : For more information and examples, check out `here <https://raisim.com/sections/MaterialSystem.html>`_.
        1. <child> (optional) ``default`` : If it doesn't exist, the default parameters are as described `here <https://raisim.com/sections/MaterialSystem.html>`_.
            1. <attribute> ``friction`` [float]
            2. <attribute> ``restitution`` [float]
            3. <attribute> ``restitution_threshold`` [float]
        2. <child> (optional, multiple) ``pair_prop``
            1. <attribute> ``name1`` [string]
            2. <attribute> ``name2`` [string]
            3. <attribute> ``friction`` [float]
            4. <attribute> ``restitution`` [float]
            5. <attribute> ``restitution_threshold`` [float]
    3. <child> (optional) ``gravity``
        1. <attribute> ``value`` [Vec<3>]: default={0, 0, -9.81}
    4. <child> (optional) ``timestep``
        1. <attribute> ``value`` [float]: default = 0.005
    5. <child> (optional) ``erp`` : For experts only. It is a spring and damper term to the error dynamics.
        1. <attribute> ``erp`` : spring term [float]
        2. <attribute> ``erp2`` : damping term [float]
    6. <child> ``objects`` : described in "Object XML Description"
    7. <child> ``wire`` : `DESCRIPTION <https://raisim.com/sections/Constraints.html>`_, `EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/wire/newtonsCradle.xml>`_
        1. <attribute> ``name`` [string]: wire name
        2. <attribute> ``type`` [string]: wire type. "stiff", "compliant", or "custom"
        3. <attribute> ``stretch_type`` [string]: stretch type
        4. <child> ``object1`` and ``object2`` : the object to which the wire is attached
            First option
            1. <attribute> ``local_index`` [int]: The body index for the wire attachment. 0 for singleBodyObject.
            2. <attribute> ``pos`` [Vec<3>]: The position in the local body frame to where the wire is attached.
            Second option, for articulated system only
            1. <attribute> ``frame`` [string]: The name of the frame to where the wire is attached.

Object XML Description
----------------------------

Collision groups and masks can be written as ``collision[1]`` or ``collision[1|4|6]``.
The first one represents the collision group 1.
The second one is a collision mask that collides with the collision group 1, 4 and 6.
The collision mask ``collision[-1]`` means that it can collide with any collision group (note that the collision is AND logic and two mask and group pairs should agree).

sphere
^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``mass`` [float]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia`` : <attribute> ``xx`` [float], ``xy`` [float], ``xz`` [float], ``yy`` [float], ``yz`` [float], ``zz`` [float]

2. <child> dim: <attribute> radius[float]

3. <child> state: <attribute> ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

capsule and cylinder
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) collision_group[uint64_t], (optional, default=-1) collision_mask[uint64_t], (optional) appearance[string], (optional) body_type[string]: one of {dynamic, kinematic, static}, (optional) name[string], mass[float]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia`` : <attribute> ``xx`` [float], ``xy`` [float], ``xz`` [float], ``yy`` [float], ``yz`` [float], ``zz`` [float]

2. <child> dim: <attribute> ``radius`` [float], ``height`` [float]

3. <child> state: <attribute> ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

box
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``mass`` [float]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia`` : <attribute> ``xx`` [float], ``xy`` [float], ``xz`` [float], ``yy`` [float], ``yz`` [float], ``zz`` [float]

2. <child> dim: <attribute> ``x`` [float], ``y`` [float], ``z`` [float]

3. <child> state: <attribute> ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

compound
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``com`` [Vec<3>], ``mass`` [float]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia``
    **attributes**: ``xx`` [float], ``xy`` [float], ``xz`` [float], ``yy`` [float], ``yz`` [float], ``zz`` [float]

2. <child> ``children``
        Common attributes of the children: ``appearance`` [string]
    1. <child> (optional, multiple) ``sphere``
        1. <child> dim
            1. <attribute> ``radius`` [float]
        2. <attribute> (optional, default=default) ``material``
    2. <child> (optional, multiple) ``cylinder``
        1. <child> ``dim``
            1. <attribute> radius[float]
            2. <attribute> height[float]
        2. <attribute> (optional, default=default) ``material``
    3. <child> (optional, multiple) ``capsule``
        1. <child> ``dim``
            1. <attribute> ``radius`` [float]
            2. <attribute> ``height`` [float]
        2. <attribute> (optional, default=default) ``material``
    4. <child> (optional, multiple) ``box``
        1. <child> ``dim``
            1. <attribute> ``x`` [float]
            2. <attribute> ``y`` [float]
            3. <attribute> ``z`` [float]
        2. <attribute> (optional, default=default) ``material``

3. <child> state
    **attributes**: ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

mesh
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``mass`` [float], ``file_name`` [string], ``com`` [Vec<3>], ``scale`` [Vec<3>]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia``
    **attributes**: ``xx`` [float], ``xy`` [float], ``xz`` [float], ``yy`` [float], ``yz`` [float], ``zz`` [float]

2. <child> ``state``
    **attributes**: ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

ground
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/material/material.xml>`_

**attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``name`` [string], (optional, default=0) ``height`` [float]

heightmap
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts/heightMaps>`_

**Options**

1. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``x_sample`` [size_t], ``y_sample`` [size_t], ``x_size`` [float], ``y_size`` [float], ``center_x`` [float], ``center_y`` [float], ``height`` [std::vector<float>]

2. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``x_sample`` [size_t], ``y_sample`` [size_t], ``x_size`` [float], ``y_size`` [float], ``center_x`` [float], ``center_y`` [float], ``z_scale`` [float], ``z_offset`` [float], ``png`` [string]

3. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``center_x`` [float], ``center_y`` [float], ``text`` [string]

4. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``x_sample`` [size_t], ``y_sample`` [size_t], ``x_size`` [float], ``y_size`` [float], ``center_x`` [float], ``center_y`` [float]
    1. <child> ``terrain_properties``
        **attributes**: ``z_scale`` [float], ``fractal_octaves`` [size_t], ``fractal_lacunarity`` [float], ``fractal_gain`` [float], ``step_size`` [float], ``frequency`` [float], ``seed`` [size_t]

articulated_system
^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/heightMaps/heightMapUsingPng.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``name`` [string], (optional, default=the URDF directory) ``res_dir`` [string], ``urdf_path`` [string]

1. <child> ``state``
    **attributes**: ``qpos`` [VecDyn], (optional, default=zeros) ``qvel`` [VecDyn]


Configuration Template
----------------------------
Configuration templates can be useful to systematically create a world.
An example can be found `here <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts/templatedWorld/templatedWorld.xml>`_.

You can include other configuration files using tag ``include``.
This can be useful if you have subworlds and want to create multiple combinations of them.

You can denote parameters prefixed by ``@@``.
Variables can be either specified in the script using ``params``.
An example can be found `here <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts/templatedWorld/templatedWorld.xml>`_.
This can be useful if you want to change the world at runtime or create multiple versions of the configuration file.

You can use tag ``array`` to create **forloop** in the configuration file.
An example can be found in `here <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts/templatedWorld/spheres.xml>`_.
You have to give attribute ``idx``, ``start``, ``end``, and ``increment``.
The last three should be integers.

You can also write **math expressions** inside the configuration file.
An example can be found in the same file.
You have to use ``{}`` to encapsulate the equation part.
You can use simple binary functions such as ``sin``, ``cos``, ``exp``, and ``log``.
You have to use ``()`` for functions and closing expressions just like in C++.

Objects can have an attribute ``exist``.
This allows you to remove objects at runtime using parameters.
An example file can be found `here <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts/templatedWorld/spheres.xml>`_.
