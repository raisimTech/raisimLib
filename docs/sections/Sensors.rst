#############################
Sensors
#############################

This module provides a way to specify sensor properties using a URDF-like format.
We provide a few implementation examples for depth camera, RGB camera, and IMU.

We use RaiSim to interface with the real hardware.
This makes sense because we want to use the same code to control both the simulated and real robot.
This sensor package is also designed for real robots as well.
This might confuse some people but it makes the code simpler and shorter.


How to attach a sensor to a link
====================================
Before further explanations, we clarify some terms that will be used throughout this documentations.
**"sensor"** is a part which outputs a single type of information. For examples, RGB cameras, depth cameras, gyroscope, etc.
**"sensor_set"** is a set of sensors contained in a single link.
For examples, Intel Realsense (it contains imu+rgb+depth) and IMU (gyro and accelerometer) are "sensor_set".
A sensor_set can contain only a single sensor.

Create a link for a sensor_set and give it a ``sensor`` attribute as

.. code-block:: xml

    <link name="realsense_d435" sensor="realsense435.xml"/>

The `realsense435.xml <https://github.com/raisimTech/raisimLib/blob/master/rsc/anymal_c/sensors/realsense435.xml>`_ file should specify all necessary details of the sensor.
It should be stored in the same directory as the URDF file.
If it is not found, raisim will search the following directories in order: ``[urdf_dir]/sensor``, ``[urdf_dir]/sensors``, ``[urdf_dir]/..`` and ``[urdf_dir]/../sensors``.
An example URDF file can be found `here <https://github.com/raisimTech/raisimLib/blob/master/rsc/anymal_c/urdf/anymal_sensored.urdf>`_.


Update
====================================
The sensor update method can be set using ``raisim::Sensor::setMeasurementSource``.
It can be either updated using raisim (``raisim::Sensor::MeasurementSource::RAISIM``) or the visualizer (``raisim::Sensor::MeasurementSource::VISUALIZER``).
``RGBSensor`` can only be updated using the RaisimUnreal.
``DepthSensor`` can be updated either raisim or the RaisimUnreal.
``InertialMeasurementUnit`` can only be updated using raisim.

The update is performed at the specified frequency (i.e., ``update_rate`` in the sensor xml).

Example
====================================
https://github.com/raisimTech/raisimLib/blob/master/examples/src/server/sensors.cpp

Parent Class API
=====================

.. doxygenclass:: raisim::Sensor
   :members:

Depth Camera API
=====================

.. doxygenclass:: raisim::DepthCamera
   :members:

RGB Camera API
=====================

.. doxygenclass:: raisim::RGBCamera
   :members:

Inertial Measurement Unit API
======================================

.. doxygenclass:: raisim::InertialMeasurementUnit
   :members: