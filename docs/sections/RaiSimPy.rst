#############################
RaisimPy
#############################

RaisimPy is a nearly complete Python wrapper for raisim.
It was initiated by Brian Delhaisse and now RaiSim Tech Inc. is continuing the development.
RaisimPy is distributed under the MIT license (but it depends on raisim).
The install instructions are given `here <https://raisim.com/sections/Installation.html>`_.

The syntax of raisimPy is nearly identical to that of the C++ API except that it takes numpy arrays instead of Eigen vectors.
Therefore, we do not provide a separate documentation for the python API.

You can check the difference between RaiSim C++ and RaiSimPy in the following example.

.. tabs::
  .. group-tab:: C++

    .. code-block:: c

        #include "raisim/World.hpp"
        #include "raisim/RaisimServer.hpp"

        int main(int argc, char* argv[]) {
          auto binaryPath = raisim::Path::setFromArgv(argv[0]);
          raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

          /// create raisim world
          raisim::World world;

          /// create objects
          auto ground = world.addGround();

          /// launch raisim server
          raisim::RaisimServer server(&world);
          server.launchServer();

          auto visSphere = server.addVisualSphere("v_sphere", 1.0, 1, 1, 1, 1);
          auto visBox = server.addVisualBox("v_box", 1, 1, 1, 1, 1, 1, 1);
          auto visCylinder = server.addVisualCylinder("v_cylinder", 1, 1, 0, 1, 0, 1);
          auto visCapsule = server.addVisualCapsule("v_capsule", 1, 0.5, 0, 0, 1, 1);
          auto anymalB = server.addVisualArticulatedSystem("v_anymal", binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf");

          visSphere->setPosition(2,0,0);
          visCylinder->setPosition(0,2,0);
          visCapsule->setPosition(2,2,0);
          Eigen::VectorXd gc(19);
          gc << 0, 0, 3.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
          anymalB->setGeneralizedCoordinate(gc);
          anymalB->color = {0.5,0.0,0.0,0.5};
          auto lines = server.addVisualPolyLine("lines");
          lines->color = {0,0,1,1};

          for( int i = 0; i < 100; i++)
            lines->points.push_back({sin(i*0.1), cos(i*0.1), i*0.01});

          size_t counter = 0;
          while (1) {
            counter++;
            visBox->color[2] = double((counter)%255+1)/256.;
            visBox->setBoxSize(double((counter)%255+1)/256.+0.01, 1, 1);
            visSphere->color[1] = double((counter)%255+1)/256.;
            raisim::MSLEEP(2);

            lines->color[2] = double((counter)%255+1)/256.;
            lines->color[0] = 1. - lines->color[2];
        //    server.integrateWorldThreadSafe();
          }

          server.killServer();
        }


  .. group-tab:: Python

    .. code-block:: python

        import os
        import numpy as np
        import raisimpy as raisim
        import math
        import time

        raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
        world = raisim.World()
        ground = world.addGround()

        # launch raisim server
        server = raisim.RaisimServer(world)
        server.launchServer(8080)

        visSphere = server.addVisualSphere("v_sphere", 1, 1, 1, 1, 1)
        visBox = server.addVisualBox("v_box", 1, 1, 1, 1, 1, 1, 1)
        visCylinder = server.addVisualCylinder("v_cylinder", 1, 1, 0, 1, 0, 1)
        visCapsule = server.addVisualCapsule("v_capsule", 1, 0.5, 0, 0, 1, 1)
        laikago = server.addVisualArticulatedSystem("v_laikago", os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/laikago/laikago.urdf")
        laikago.setGeneralizedCoordinate(np.array([0, 0, 3.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8]))
        laikago.setColor(0.5, 0.0, 0.0, 0.5)

        visSphere.setPosition(np.array([2, 0, 0]))
        visCylinder.setPosition(np.array([0, 2, 0]))
        visCapsule.setPosition(np.array([2, 2, 0]))

        lines = server.addVisualPolyLine("lines")
        lines.setColor(0, 0, 1, 1)

        for i in range(0, 100):
            lines.addPoint(np.array([math.sin(i * 0.1), math.cos(i * 0.1), i * 0.01]))

        counter = 0

        for i in range(500000):
            counter = counter + 1
            visBox.setColor(1, 1, (counter % 255 + 1) / 256., 1)
            visSphere.setColor(1, (counter % 255 + 1) / 256., 1, 1)
            lines.setColor(1 - (counter % 255 + 1) / 256., 1, (counter % 255 + 1) / 256., 1)
            visBox.setBoxSize((counter % 255 + 1) / 256. + 0.01, 1, 1)
            time.sleep(world.getTimeStep())

        server.killServer()

