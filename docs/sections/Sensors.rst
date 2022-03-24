#############################
Sensors
#############################
**The implementations of sensor modules are highly experimental at this stage.**

This module provides a way to specify sensor properties using a URDF-like format.
We provide a few implementation examples for depth and RGB cameras.
Please note that the API of this module will likely change in the future.

At KAIST, we use RaiSim to interface with the real hardware.
This makes sense because we want to use the same code to control both the simulated and real robot.
This sensor package is also designed for real robots as well.
This might confuse some people but it makes the code simpler and shorter.

How to attach a sensor to a link
----------------------------------

Before further explanations, we clarify some terms that will be used throughout this documentations.
**"sensor"** is a part which outputs a single type of information. For examples, RGB cameras, depth cameras, gyroscope, etc.
**"sensor_set"** is a set of sensors contained in a single link. For examples, Intel Realsense(it contains imu+rgb+depth), IMU (gyro and accelerometer).
sensor_set is a link.
You can specify

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