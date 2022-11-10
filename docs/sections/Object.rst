#############################
Objects
#############################

Body types
===============

There are three available body types.

1. ``DYNAMIC``: can have a velocity, has a finite mass
2. ``KINEMATIC``: can have a velocity, has an infinite mass (e.g., conveyor belt)
3. ``STATIC``: cannot have a velocity, has an infinite mass (e.g., wall)

SingleBodyObjects can be of any type.
ArticulatedSystems can only be DYNAMIC except the fixed base which can be STATIC.

You can get/set the body type using

* ``setBodyType(BodyType type)``
* ``getBodyType()`` or ``getBodyType(body_index)``

``getBodyType()`` for ArticulatedSystems always returns DYNAMIC.

Name
===============

All objects can be named.
These names are used by visualizers.
`raisim::World` has a functionality to retrieve an object by name.
Here is an example.

.. code:: c

    auto sphere = world.addSphere(1,1);
    sphere->setName("sphere");
    std::string name = sphere->getName();
    auto same_sphere = world.getObject("sphere");

Types
===============

All objects can be specified as either a SingleBodyObject or an ArticulatedSystem.


.. toctree::
   :maxdepth: 2

   ArticulatedSystem
   SingleBodyObjects

API
=========

.. doxygenclass:: raisim::Object
   :members: