#############################
Sensors
#############################
**The implementations of sensor modules are highly experimental at this stage.**
**This documentation is only for the internal alpha testers.**

This module provides a way to specify sensor properties using a URDF-like format.
We provide a few implementation examples for depth camera, RGB camera, and IMU.
Please note that the API of this module will likely change in the future.

We use RaiSim to interface with the real hardware.
This makes sense because we want to use the same code to control both the simulated and real robot.
This sensor package is also designed for real robots as well.
This might confuse some people but it makes the code simpler and shorter.


How to attach a sensor to a link
----------------------------------
Before further explanations, we clarify some terms that will be used throughout this documentations.
**"sensor"** is a part which outputs a single type of information. For examples, RGB cameras, depth cameras, gyroscope, etc.
**"sensor_set"** is a set of sensors contained in a single link.
For examples, Intel Realsense (it contains imu+rgb+depth), IMU (gyro and accelerometer).
However, a sensor_set can contain only a single sensor.

Create a link for a sensor_set and give it a ``sensor`` attribute as

.. code-block:: xml

    <link name="realsense_d435" sensor="realsense435.xml"/>

The "realsense435.xml" file should specify all necessary details of the sensor.
It should be stored in the same directory as the URDF file.
If it is not found, raisim will search the following directories in order: ``[urdf_dir]/sensor``, ``[urdf_dir]/sensors``, ``[urdf_dir]/..`` and ``[urdf_dir]/../sensors``.

Any example of an sensor xml file can be found in ``rsc/anymal_c/sensors``.
A more formal xml definitions will be uploaded once the module is expanded and tested enough.

