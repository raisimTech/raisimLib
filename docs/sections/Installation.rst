#############################
Installation
#############################

Setup
========

Please install/save everything locally to prevent any conflicts with other libraries. We will assume that you have a single workspace where you save all repos related to raisim. Here we introduce two variables

* ``WORKSPACE``: workspace where you clone your git repos (e.g., ~/raisim_workspace)
* ``LOCAL_INSTALL``: install directory where you install exported cmake libraries (e.g., ~/raisim_build)

You can add them to your environment variables or simply replace them by the path you want in the below instructions.

Dependencies
============

RaiSim depends on the following open-source libraries

* *Eigen3* (:code:`sudo apt-get install libeigen3-dev`)
* cmake > 3.10 (Technically it is not a dependency but it makes the installation easier)
* For windows users, visual studio 2019 (we only support 2019 but it will probably work with 2015 or above)

RaiSim includes many open-source libraries. See the COPYING file for the full list.

RaiSim Install
===============

Clone raisim from https://github.com/raisimTech/raisimlib.

RaiSim is installed using cmake. The following options are available

* ``RAISIM_EXAMPLE`` : Compile C++ RaiSim examples
* ``RAISIM_MATLAB`` : Compile raisimMatlab (compiled binary is also provided). You need MATLAB for this option
* ``RAISIM_PY`` : Compile raisimPy. The desired python version can be set by ``-DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)")`` in Linux or Mac. In Linux, you need to install python library using ``sudo apt install libpython<YOUR_PYTHON_VERSION>-dev`` to build a python package.

You can generate build files using CMake as following

.. tabs::
  .. group-tab:: Linux or Mac

    Replace ``$(python3 -c "import sys; print(sys.executable)")`` to specify which python executable to use.

    .. code-block:: c

      cd $WORKSPACE/raisimLib
      mkdir build
      cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)")
      make install -j4

  .. group-tab:: Windows

    We recommend using the CMake GUI.
    After generating the build files with the CMake GUI, you can build and install RaiSim using Visual Studio.
    Alternatively, you can do it as below in Windows Powershell

    .. code-block::

      cd $WORKSPACE/raisimLib
      mkdir build
      cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=<THE-PATH-TO-THE-PYTHON-EXE>
      cmake --build . --target install --config Release

To use RaiSim more conveniently, you have to let your linker know where you installed RaiSim

.. tabs::
  .. group-tab:: Linux

    Add the following lines to your ``~/.bashrc`` file

    .. code-block:: bash

        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<WHERE-YOU-HAVE-INSTALLED-RAISIM>/lib
        export PYTHONPATH=$PYTHONPATH:<WHERE-YOU-HAVE-INSTALLED-RAISIM>/lib

    Do not use ``~`` to reference your home directory. Write the full path

  .. group-tab:: Mac

    Add the following lines to your ``.zshrc`` file

    .. code-block:: bash

        export DYLD_LIBRARY_PATH=DYLD_LIBRARY_PATH:<WHERE-YOU-HAVE-INSTALLED-RAISIM>/lib
        export PYTHONPATH=$PYTHONPATH:<WHERE-YOU-HAVE-INSTALLED-RAISIM>/lib

    Do not use ``~`` to reference your home directory. Write the full path

  .. group-tab:: Windows
    Add the installation directory to your `Path` environment variable.
    You can do that by following these steps

    1. “Edit the system environment variables”

    2. In "Advanced tab", "Environment Variables"

    3. Click "Path" variable in the "System variables" list and click "edit"

    4. Append the install directory

.. note::
    **For Linux users**
    To use (vulkan version) RaiSimUnity in Linux, you need to install ``minizip``, ``ffmpeg`` and ``vulkan``.
    To install vulkan, follow this link https://linuxconfig.org/install-and-test-vulkan-on-linux

    To install ``minizip`` and ``ffmpeg``,

    .. code-block:: bash

        sudo apt install minizip ffmpeg

    If you still cannot raisimUnity, this probably means that your driver does not support vulkan so well.
    In that case, you should use raisimUnityOpengl.
    It only supports minimalistic graphics.

Activation Key
================

Rename the activation key that you received by email to ``activation.raisim``.
Save that file in ``<YOUR-HOME-DIR>/.raisim``.
In Linux and Mac, this is ``/home/<YOUR-USERNAME>/.raisim``.
In Windows, this is ``C:\Users\<YOUR-USERNAME>\.raisim`` (You might not be using ``C`` as your home directory).

RaiSim will check the path you set by ``raisim::World::setActivationKey()``.
If the file is not found, it will search in the user directory, where you saved your ``activation.raisim`` file.

Examples
===============

The built examples are stored in ``examples`` directory.
In Windows, use powershell to run the examples, instead of manually clicking the icons.
If you made a mistake during installation, it will give you an error message.

Make sure that you run raisimUnity executable in ``raisimUnity/<OS>/RaiSimUnity`` before you run the examples.