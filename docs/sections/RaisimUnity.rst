#############################
RaiSim Unity
#############################

raisimUnity is a visualizer for RaiSim.
It communicates with RaiSim using a TCP/IP protocol.
Pre-built executables are provided in ``raisimUnity`` directory.

How to use raisimUnity
=========================

1. Write your simulated world in RaiSim, create a server and attach the world to the server.
An example is provided below.


.. code-block:: bash

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  while(1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();

2. Run RaiSimUnity executable (in the ``raisimUnity`` directory)

.. image:: ../image/raisimUnity1.png

.. image:: ../image/raisimUnity2.png

3. Run your simulation

.. image:: ../image/raisimUnity3.png

4. If the IP and port number matches, raisimUnity will try to connect automatically.
You can disable auto-connection by unchecking the ``auto-connect`` checkbox.

.. image:: ../image/raisimUnity4.png

5. A quick summary

.. image:: ../image/RSUinstruction1.png

.. image:: ../image/RSUinstruction2.png

.. image:: ../image/RSUinstruction3.png
