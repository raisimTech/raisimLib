CustomLengthConstraint
==========================

``CustomLengthConstraint`` allows users to apply any kind of custom constraints.
Users simply set the tension between two points using ``setTension()``.

An example of this type of constraints is shown below

.. code-block:: c

  auto wire7 = world.addCustomWire(pin7, 0, {0,0,0}, anymalB, 0, {0., 0, 0}, 2.0);
  wire7->setTension(400);

The following code will results in

.. image:: ../image/customWire.gif

You can find a runnable example in `here <https://github.com/raisimTech/raisimLib/blob/master/examples/src/server/newtonsCradle.cpp>`_.


API
----------------

.. doxygenclass:: raisim::CustomLengthConstraint
   :members: