#############################
Introduction
#############################

RaiSim is a physics engine developed by RaiSim Tech Inc.
It is designed to provide both the accuracy and speed for simulating robotic systems.
However, it is a generic rigid-body simulator and can simulate any rigid body very efficiently.

**Why RaiSim?**

* The speed is benchmarked against other popular physics engine. (`[1] <https://github.com/leggedrobotics/SimBenchmark>`_)
* The accuracy of RaiSim has been demonstrated in a number of papers (`[2] <https://robotics.sciencemag.org/content/4/26/eaau5872/tab-article-info>`_, `[3] <https://arxiv.org/pdf/1901.07517.pdf>`_, `[4] <https://robotics.sciencemag.org/content/5/47/eabc5986>`_,  `[5] <https://arxiv.org/abs/1909.08399>`_,  `[6] <https://arxiv.org/abs/2011.08811>`_)
* Easiest C++ simulation library to learn/use
* A minimum number of dependencies (only on STL and Eigen)

Visualizer
=====================
There are three options available for visualization.
Most of the examples included in this repo requires raisimUnity to visualize.
You have to run raisimUnity before running the examples.

RaisimUnity
***************

.. image:: ../image/raisimUnity4.png
  :alt: raisimUnity
  :width: 600

*  **If this doesn't work on your linux machine, try raisimUnityOpengl, which is a stripped-down version**
*  binaries included in the ``raisimUnity`` directory
*  easy to use but less features
*  runs on a separate process
*  works well in Linux, Mac and Windows

RaisimUnreal - Alpha
*********************

.. image:: ../image/raisimUnreal1.png
  :alt: raisimUnity
  :width: 600

*  Currently available in Widnows and Linux
*  Not well tested yet
*  Easy to use.
*  Can visualize graphs and bar charts
*  runs on a separate process
*  Closed source

RaisimOgre
************

.. image:: ../image/raisimOgre.png
  :alt: raisimOgre
  :width: 420

*  harder to use but rich in features
*  works well in Linux. Should work in other two major OS's but not tested
*  easy to customize. Only C++ code
*  source code available `here <https://github.com/raisimTech/raisimogre>`_ under the MIT license


Comparison between RaisimUnity and RaisimUnreal
==================================================

Here is a comparison between RaisimUnity and RaisimUnreal.

*  **Graphics Quality (my personal opinion)**

   *  **RaisimUnity**: 7/10.
   *  **RaisimUnreal**: 10/10 (Somehow the quality is lower in Linux. This might be due to a driver issue.).

*  **Compatibility**

   *  **RaisimUnity**: There is an alternate Opengl version for compatibility because many old or integrated GPU's do not have a proper vulkan-supporting GPU driver on Linux.
   *  **RaisimUnreal**: Not tested at all. I need your feedback on this. If you find issues, please post it on the github issue.

*  **GPU utilization (based on a few examples on my pc)**

   *  **RaisimUnity**: 90%.
   *  **RaisimUnreal**: 98% (it runs much faster in Windows. It seems like the Unreal Engine is not working well on Linux. In addition, the weather preset affects the performance significantly).

*  **GPU memory usage (with raisim examples)**

   *  **RaisimUnity**: ~2Gb.
   *  **RaisimUnreal**: ~2Gb.

*  **Mesh loading time**

   *  **RaisimUnity**: Very fast.
   *  **RaisimUnreal**: Terribly slow! It also lacks a mechanism for instancing a mesh. This means that if you have 100 robots in the scene, it will create the same meshes 100 times.

*  **Support**

   *  **RaisimUnity**: Will be supported and bug-fixed.
   *  **RaisimUnreal**: New visualization developments will be on raisimUnreal as it seems like a more promising option. Because Unreal is using C++, It is much easier to develop Simulation API (not just the visualization) as well. This might come sometime this year (2022).

*  **Graphs (time series and bar charts)**

   *  **RaisimUnity**: none.
   *  **RaisimUnreal**: RaisimUnreal uses Kantan Chart to visualize user created graphs from the server. It can be a convenient way to visualize the training process. Check the ``examples/atlas.cpp`` for usage.

*  **Vidoes Recording**

   *  **RaisimUnity**: Works in Linux.
   *  **RaisimUnreal**: Will be implemented soon.


System Requirements
=====================
- **Linux**
    * We recommend ubuntu 16.04, 18.04 and 20.04 but RaiSim might work on other distributions. Works only on a X86 CPU with an AVX2 instruction set.

- **Windows 10**
    * Visual Studio 2019. X86 CPU only.

- **Mac**
    * Latest version. Requires the AVX2 instruction set. Works on both M1 (Apple ARM) and X86.

Example code
===================
Here is an example of an RaiSim application code. You can visualize it using either raisimUnity or raisimUnreal.

.. code-block:: c

  #include “raisim/World.hpp”
  #include "raisim/RaisimServer.hpp"

  int main() {
    raisim::World::setActivationKey("PATH_TO_THE_ACTIVATION_KEY");
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