#############################
World
#############################
:code:`raisim::World` class creates/manages all resources.
All objects defined in the same Wolrd class instance can collide with each other unless otherwise their collision mask and group explicitly disables the collision ().

There are two ways to generate the World instance (i.e., two constuctors).
The first way is to load an raisim world configuration file, which is in a form of an XML file.
The second way is to generate world dynamically in code.
You can also mix the two ways, by loading an XML file and dynamically adding objects.

RaiSim World Configuration File Convention
=============================================

We provide a few examples `here <https://github.com/raisimTech/raisimLib/tree/master/rsc/xmlScripts>`_.

The following describes the RaiSim world configuration xml convention.
The (optional) tag means that the element is optional given the parent.
If the element is not marked (optional), it must exist given that the parent exist.
The (multiple) tag means that there can be multiple elements for the same parent.

1. raisim
    1. <attribute> version: Describes the version of RaiSim that created the configuration file. The file might be read by different version.
    2. <child> (optional) material: For more information and examples, check out `here <https://raisim.com/sections/MaterialSystem.html>`_.
        1. <child> (optional) default: If it doesn't exist, the default parameters are as described `here <https://raisim.com/sections/MaterialSystem.html>`_.
            1. <attribute> friction
            2. <attribute> restitution
            3. <attribute> restitution_threshold
        2. <child> (optional, multiple) pair_prop
            1. <attribute> name1
            2. <attribute> name2
            3. <attribute> friction
            4. <attribute> restitution
            5. <attribute> restitution_threshold
    3. <child> (optional) gravity: default={0, 0, -9.81}
        1. <attribute> value
    4. <child> (optional) timestep: default = 0.005
        1. <attribute> value
    5. <child> (optional) erp: For experts only. It is a spring and damper term to the error dynamics.
        1. <attribute> erp: spring term
        2. <attribute> erp2: damping term
    6. <child> objects
        1. <child> TYPE_OF_OBJECT, can be one of "sphere", "box", "cylinder", "cone", "capsule", "mesh", "ground", "compound", "heightmap", "articulated_system"
            1. <attribute> (optional, for all objects) collision_group: default=1
            2. <attribute> (optional, for all objects) collision_mask: default=-1
            3. <attribute> (for sphere, capsule, cylinder, box, mesh, compound) mass
            4. <child> (for sphere, capsule, cylinder, box, mesh, compound) inertia
                1. <attribute> xx
                2. <attribute> xy
                3. <attribute> xz
                4. <attribute> yy
                5. <attribute> yz
                6. <attribute> zz
            5. <attribute> (optional, for sphere, capsule, cylinder, box, mesh, compound) body_type, can be one of "dynamic", "kinematic", "static". For more information, check out `this link <https://raisim.com/sections/Object.html#body-types>`_.
            6. <attribute> (optional) material: default="default"
            7. <child> (for sphere, capsule, cylinder, box) dim
                1. <attribute> (for sphere, capsule, cylinder) radius
                2. <attribute> (for capsule, cylinder) height
                3. <attribute> (for box) x
                4. <attribute> (for box) y
                5. <attribute> (for box) z
            8. <attribute> (for mesh) file_
    7. <child> constraints: NOT IMPLEMENTED YET



Adding New Objects
============================
To add a new object of a shape X, a method named :code:`addX` is used.
For example, to add a sphere

.. code-block:: c

  raisim::World world;
  auto sphere = world.addSphere(0.5, 1.0);

Here :code:`sphere` is a pointer to the internal resource.
It can be used to access or to modify the internal variables.

There are three hidden arguments to all object-creation methods: :code:`material`, :code:`collisionGroup` and :code:`collisionMask`.
Descriptions of the collision varaibles are given in "Collision and Contact" chapter.
:code:`material` argument specifies the material which governs contact dynamics.
It is further explained in "Material System" chapter.

The list of objects is given in "Object" chapter.

Once an object is added, a name can be set as below

.. code-block:: c

  sphere.setName("ball");

A pointer to an object with a specific name can be retrieved as below

.. code-block:: c

  auto ball = world.getObject("ball");

An object might contain multiple bodies (i.e., articulated system).
To designate each body, **local index** can be used.
To keep the interface consistent, many methods ask for the local index even for simgle body objects.
In a single body object case, local index arguments are ignored and users can simply put 0 to comply with the AIP.

Changing Simulation Parameters
================================

The following paramters can be changed using the world API

* **Time step**

RaiSim uses a fixed time step. The time step obtained and modified using :code:`getTimeStep` and :code:`setTimeStep` method.

API
=========

.. doxygenclass:: raisim::World
   :members: