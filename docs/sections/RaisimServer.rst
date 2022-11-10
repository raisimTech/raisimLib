#############################
Raisim Server
#############################

RaisimServer serializes ``raisim::World`` and streams the data to clients via tcp/ip.
We provide the raisimUnity (`doc <https://raisim.com/sections/RaisimUnity.html>`_) client, which visualizes a ``raisim::World``.
The basic usage is described in the `doc <https://raisim.com/sections/RaisimUnity.html#how-to-use-raisimunity>`_.

Other than just visualizing a ``raisim::World``, ``raisim::RaisimServer`` can visualize additional visual objects.
An example can be found `here <https://github.com/raisimTech/raisimLib/blob/master/examples/src/server/visualObjects.cpp>`_.
The example will be displayed as following

.. image:: ../image/visuals.gif


RaisimServer API
=========================

.. doxygenclass:: raisim::RaisimServer
   :members:

Visuals API
=========================

.. doxygenstruct:: raisim::Visuals
   :members:

Polyline API
=========================

.. doxygenstruct:: raisim::PolyLine
   :members:

ArticulatedSystemVisual API
============================

.. doxygenstruct:: raisim::ArticulatedSystemVisual
   :members: