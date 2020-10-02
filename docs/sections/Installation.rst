#############################
Installation
#############################

Setup
========

Please install/save everything locally to prevent any conflicts with other libraries. We will assume that you have a single workspace where you save all repos related to raisim. Here we introduce two variables

* ``WORKSPACE``: workspace where you clone your git repos (e.g., ~/raisim_workspace)
* ``LOCAL_INSTALL``: install directory where you install exported cmake libraries (e.g., ~/raisim_build)

Dependencies
============

RaiSim depends on the following open-source libraries

* *Eigen3* (:code:`sudo apt-get install libeigen3-dev`)
* cmake > 3.10 (Technically it is not a dependency but it makes the installation easier)

RaiSim includes many open-source libraries. See the COPYING file for the full list.

RaiSim Install
===============

RaiSim is installed using cmake. The following options are available

* ``RAISIM_EXAMPLE`` : Compile C++ RaiSim examples
* ``RAISIM_MATLAB`` : Compile raisimMatlab (compiled binary is also provided). You need MATLAB for this option
* ``RAISIM_PY`` : Compile raisimPy. The python version can be set as ``-DPYTHON_EXECUTABLE:FILEPATH=$LOCATION_OF_THE_PYTHON_EXECUTABLE``

Example install in Linux

.. code-block:: bash

    cd $WORKSPACE/raisimLib
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON
    make install -j4

.. note::
    To use RaiSimUnity in Linux, you need to install ``minizip``, ``ffmpeg`` and ``vulkan``.
    To install vulkan, follow this link https://linuxconfig.org/install-and-test-vulkan-on-linux


    To install ``minizip`` and ``ffmpeg``,

    .. code-block:: bash

        sudo apt install minizip ffmpeg

    To conveniently use raisim and raisimPy, add the following lines to your ``~/.bashrc`` file

    .. code-block:: bash

        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/<WHERE-YOU-HAVE-INSTALLED-RAISIM>/lib
        export PYTHONPATH=$PYTHONPATH:~/<WHERE-YOU-HAVE-INSTALLED-RAISIM>/lib





