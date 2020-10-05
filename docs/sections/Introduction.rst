#############################
Introduction
#############################

RaiSim is a physics engine developed by RaiSim Tech Inc.
It is designed to provide both the accuracy and speed for simulating robotic systems.
However, it is a generic rigid-body simulator and can simulate any rigid body very efficiently.

Why RaiSim?

* The speed is benchmarked against other popular physics engine. (`[1] <https://github.com/leggedrobotics/SimBenchmark>`_)
* The accuracy of RaiSim has been demonstrated in a number of papers (`[2] <https://robotics.sciencemag.org/content/4/26/eaau5872/tab-article-info>`_, `[3] <https://arxiv.org/pdf/1901.07517.pdf>`_)
* Easiest C++ simulation library to learn/use
* A minimum number of dependencies (only on STL and Eigen)

Visualizer
=====================
There are two options available for visualization.

- **raisimUnity**

    * binaries included in the ``raisimUnity`` directory
    * easy to use but less features
    * runs as a seperate process
    * works well in both windows and Linux (will be supported in Mac soon)
    * source code available `here <https://github.com/raisimTech/raisimUnity>`_ under the MIT license. Written in C#

- **raisimOgre**

    * harder to use but rich in features
    * works well in Linux. Should work in other two major OS's but not well tested
    * easy to customize. Only C++ code
    * source code available `here <https://github.com/raisimTech/raisimogre>`_ under the MIT license


System Requirements
=====================

- **Linux**

    * We recommend ubuntu 16.04, 18.04 and 20.04 but RaiSim might work on other distributions

- **Windows 10**

    * Visual Studio 2019

- **Mac**

    * (WILL BE AVAILABLE SOON)The binaries also contain AVX2 instructions.

Example code
===================
Here is an example of an RaiSim application code.

.. code-block:: c

  #include “raisim/World.hpp”
  #include "raisim/RaisimServer.hpp"

  int main() {
    raisim::World::setActivationKey(PATH_TO_THE_ACTIVATION_KEY);
    raisim::World world;
    auto anymal = world.addArticulatedSystem(PATH_TO_URDF);
    auto ball = world.addSphere(1, 1);
    auto ground = world.addGround();
    world.setTimeStep(0.002);

    /// launch raisim server for visualization. Can be visualized on raisimUnity
    raisim::RaisimServer server(&world);
    server.launchServer();

    while (1) {
      raisim::MSLEEP(2);
      server.integrateWorldThreadSafe();
    }

    server.killServer();
  }

Here is a cmake file to compile above application code.

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.10)
  project(raisim_examples LANGUAGES CXX)

  find_package(raisim CONFIG REQUIRED)
  find_package(Eigen3 REQUIRED)
  
  include_directories (${EIGEN3_INCLUDE_DIRS})

  add_executable(APP_NAME ${file_name})
  target_link_libraries(APP_NAME PUBLIC raisim::raisim pthread)
  target_include_directories(APP_NAME PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

A working version can be found here (RAISIM_EXAMPLE_).

.. _RAISIM_EXAMPLE: https://github.com/raisimTech/raisimExample

Included Software
======================

raisimUnity
---------------
A visualizer for RaiSim.
An executable is provided so you don't have to install anything.
If you want to modify it, feel free to fork it at https://github.com/raisimTech/raisimUnity

raisimOgre is not a recommended visualizer anymore because it is very tricky to make it work for both MATLAB and Python.
However, if you prefer it, it still works and you can clone it at

raisimPy
---------------
A python wrapper for RaiSim. It was initially developed by Delhaisse, Brian and Rozo, Leonel (https://github.com/robotlearn/raisimpy)

raisimMatlab
---------------
A MATLAB wrapper for RaiSim. It is currently minimalistic. We also provide the source so that you can add a method that you need in the cpp file.
