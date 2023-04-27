#############################
Change Log
#############################
Here we only describe the changes to the RaiSim library.
Other peripheral codes are not tracked here.

**v1.1.7**
========================================
Inverse dynamics pipeline using the recursive newton euler algorithm added.
This allows users to compute the constrained forces and torques at the joints.
IMU implementation added (check ``sensors`` example).
Raisim is compiled with clang15

**v1.1.6**
========================================
The contact solver has been improved.
Both the accuracy and speed has been improved.
New macro ``RS_TIMED_LOOP(us)`` hass been added.
Please check the examples.

**v1.1.4**
=========================================
This version contains significant improvements on the visualizers: RaisimUnity and RaisimUnreal.

* The new server does not reinitialize when a new object is added (try ``examples/src/server/balls` example to see the difference).
* New ``Map`` feature on `RaisimUnreal <https://raisim.com/sections/RaisimUnreal.html>`_.


**v1.1**
========================================
There have been huge changes in this version.
The biggest change is in the articulated system (AS) solver.
Please read the "Performance" section of this documentation if you are interested.
Note that the new AS solver is analytically equivalent to the old solver.

There have been a few non-backward-compatible changes as well.
They were unavoidable to keep RaiSim simpler and more understandable.
``PerObjectContact`` class is gone and it is replaced by a simple ``std::vector`` in ``Object.hpp``.
``ArticulatedSystem::getNonlinearities`` now takes gravity argument (due to reasons described in the "Performance" section).
But if you are using for RL, the chance is that you are not even going to notice any of the changes.

RaiSim is now compiled by Clang++-11.
GCC could not keep up with the runtime performance.
This should not affect users in any way but maybe there unpredictable negative impacts.
Post any issues to the GitHub issues.

