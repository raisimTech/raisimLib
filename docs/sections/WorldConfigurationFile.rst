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
            1. <attribute> ``friction`` [double]
            2. <attribute> ``restitution`` [double]
            3. <attribute> ``restitution_threshold`` [double]
        2. <child> (optional, multiple) ``pair_prop``
            1. <attribute> ``name1`` [string]
            2. <attribute> ``name2`` [string]
            3. <attribute> ``friction`` [double]
            4. <attribute> ``restitution`` [double]
            5. <attribute> ``restitution_threshold`` [double]
    3. <child> (optional) ``gravity``
        1. <attribute> ``value`` [Vec<3>]: default={0, 0, -9.81}
    4. <child> (optional) ``timestep``
        1. <attribute> ``value`` [double]: default = 0.005
    5. <child> (optional) ``erp`` : For experts only. It is a spring and damper term to the error dynamics.
        1. <attribute> ``erp`` : spring term [double]
        2. <attribute> ``erp2`` : damping term [double]
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

sphere
^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``mass`` [double]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia`` : <attribute> ``xx`` [double], ``xy`` [double], ``xz`` [double], ``yy`` [double], ``yz`` [double], ``zz`` [double]

2. <child> dim: <attribute> radius[double]

3. <child> state: <attribute> ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

capsule and cylinder
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) collision_group[uint64_t], (optional, default=-1) collision_mask[uint64_t], (optional) appearance[string], (optional) body_type[string]: one of {dynamic, kinematic, static}, (optional) name[string], mass[double]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia`` : <attribute> ``xx`` [double], ``xy`` [double], ``xz`` [double], ``yy`` [double], ``yz`` [double], ``zz`` [double]

2. <child> dim: <attribute> ``radius`` [double], ``height`` [double]

3. <child> state: <attribute> ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

box
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``mass`` [double]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia`` : <attribute> ``xx`` [double], ``xy`` [double], ``xz`` [double], ``yy`` [double], ``yz`` [double], ``zz`` [double]

2. <child> dim: <attribute> ``x`` [double], ``y`` [double], ``z`` [double]

3. <child> state: <attribute> ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

compound
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``com`` [Vec<3>], ``mass`` [double]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia``
    **attributes**: ``xx`` [double], ``xy`` [double], ``xz`` [double], ``yy`` [double], ``yz`` [double], ``zz`` [double]

2. <child> ``children``
        Common attributes of the children: ``appearance`` [string]
    1. <child> (optional, multiple) ``sphere``
        1. <child> dim
            1. <attribute> ``radius`` [double]
        2. <attribute> (optional, default=default) ``material``
    2. <child> (optional, multiple) ``cylinder``
        1. <child> ``dim``
            1. <attribute> radius[double]
            2. <attribute> height[double]
        2. <attribute> (optional, default=default) ``material``
    3. <child> (optional, multiple) ``capsule``
        1. <child> ``dim``
            1. <attribute> ``radius`` [double]
            2. <attribute> ``height`` [double]
        2. <attribute> (optional, default=default) ``material``
    4. <child> (optional, multiple) ``box``
        1. <child> ``dim``
            1. <attribute> ``x`` [double]
            2. <attribute> ``y`` [double]
            3. <attribute> ``z`` [double]
        2. <attribute> (optional, default=default) ``material``

3. <child> state
    **attributes**: ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

mesh
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/objects/SingleBodies.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``body_type`` [string]: one of {dynamic, kinematic, static}, (optional) ``name`` [string], ``mass`` [double], ``file_name`` [string], ``com`` [Vec<3>], ``scale`` [Vec<3>]

1. <child> (optional, default=From geometry assuming uniform density) ``inertia``
    **attributes**: ``xx`` [double], ``xy`` [double], ``xz`` [double], ``yy`` [double], ``yz`` [double], ``zz`` [double]

2. <child> ``state``
    **attributes**: ``pos`` [Vec<3>], (optional, default=1,0,0,0) ``quat`` [Vec<4>], (optional, default=0,0,0) ``lin_vel`` [Vec<3>], (optional, default=0,0,0) ``ang_vel`` [Vec<3>]

ground
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/material/material.xml>`_

**attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional) ``name`` [string], (optional, default=0) ``height`` [double]

heightmap
^^^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts/heightMaps>`_

**Options**

1. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``x_sample`` [size_t], ``y_sample`` [size_t], ``x_size`` [double], ``y_size`` [double], ``center_x`` [double], ``center_y`` [double], ``height`` [std::vector<double>]

2. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``x_sample`` [size_t], ``y_sample`` [size_t], ``x_size`` [double], ``y_size`` [double], ``center_x`` [double], ``center_y`` [double], ``z_scale`` [double], ``z_offset`` [double], ``png`` [string]

3. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``center_x`` [double], ``center_y`` [double], ``text`` [string]

4. **attributes**: (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``appearance`` [string], (optional, default=default) ``material`` [string], (optional) ``name`` [string], ``x_sample`` [size_t], ``y_sample`` [size_t], ``x_size`` [double], ``y_size`` [double], ``center_x`` [double], ``center_y`` [double]
    1. <child> ``terrain_properties``
        **attributes**: ``z_scale`` [double], ``fractal_octaves`` [size_t], ``fractal_lacunarity`` [double], ``fractal_gain`` [double], ``step_size`` [double], ``frequency`` [double], ``seed`` [size_t]

articulated_system
^^^^^^^^^^^^^^^^^^^^^
`EXAMPLES <https://github.com/raisimTech/raisimLib/blob/master/rsc/xmlScripts/heightMaps/heightMapUsingPng.xml>`_

**attributes**: (optional, default=1) ``collision_group`` [uint64_t], (optional, default=-1) ``collision_mask`` [uint64_t], (optional) ``name`` [string], (optional, default=the URDF directory) ``res_dir`` [string], ``urdf_path`` [string]

1. <child> ``state``
    **attributes**: ``qpos`` [VecDyn], (optional, default=zeros) ``qvel`` [VecDyn]