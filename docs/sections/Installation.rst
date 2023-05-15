#############################
Installation
#############################

Setup
========

Please install/save everything locally to prevent any conflicts with other libraries.
We will assume that you have a single workspace where you save all repos related to raisim.
Here we introduce two variables

* ``WORKSPACE``: workspace where you clone your git repos (e.g., ~/raisim_workspace)
* ``LOCAL_INSTALL``: install directory where you install exported cmake libraries (e.g., ~/raisim_build)

You can add them to your environment variables or simply replace them by the path you want in the following instructions on this page.

Dependencies
============

You have to install the following packages before using raisim

* eigen library
* cmake > 3.10
* For windows users, visual studio 2019 (we only support 2019 but it will probably work with 2015 or above). **Make sure that you install the C++ module as well by checking the corresponding checkbox during install**.

RaiSim includes many open-source libraries. See the COPYING file for the full list.
Source code of Eigen and Pybind11 are included in the ``thirdParty`` directory.

RaiSim Install
===============

Clone raisim from https://github.com/raisimTech/raisimlib.

The cloned repo is already a set of installed raisim packages.
Under ``raisimLib/raisim/<OS-TYPE>``, you will find the installed cmake packages.

To use raisim in your project, you can simply add the cmake package path and the shared library path to their corresponding environment variables as following


.. tabs::
  .. group-tab:: Linux
    Add the following lines to your ``~/.bashrc`` file

    .. code-block:: bash

        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WORKSPACE/raisim/linux/lib
        export PYTHONPATH=$PYTHONPATH:$WORKSPACE/raisim/linux/lib

    This allow the linux linker to find the raisim shared libraries.
    To let your project CMake know where to find raisim, simply pass an argument ``-DCMAKE_PREFIX_PATH=$WORKSPACE/raisim/linux``.

  .. group-tab:: Mac
    Add the following lines to your ``.zshrc`` file

    .. code-block:: bash

        export DYLD_LIBRARY_PATH=DYLD_LIBRARY_PATH:$WORKSPACE/raisim/mac/lib
        export PYTHONPATH=$PYTHONPATH:$WORKSPACE/raisim/mac/lib

    If you use an M1 or M2 based Mac, use ``m1`` directory instead of ``mac``.
    Do not use ``~`` to reference your home directory.
    Write the full path.
    Now the Mac linker knows where to find the raisim shared libraries.
    To let your project CMake know where to find raisim, simply pass an argument ``-DCMAKE_PREFIX_PATH=$WORKSPACE/raisim/mac``.
    Again, use ``m1`` if you use an Apple silicon.


  .. group-tab:: Windows
    Add the win32 cmake package directory to your `Path` environment variable.
    You can do that by following these steps

    1. “Edit the system environment variables”
    2. In "Advanced tab", "Environment Variables"
    3. Click "Path" variable in the "System variables" list and click "edit"
    4. Append ``$WORKSPACE/raisim/win32/``
    5. To let your project know where raisim is, simply pass an argument ``-DCMAKE_PREFIX_PATH=$WORKSPACE/raisim/win32``. (Note that we no longer have separate directories for release and debug builds. They are combined in to a single one.)


Building RaiSim Examples and Installing RaisimPy
====================================================

You can use the ``CMakeLists.txt`` in the ``raisimLib`` directory to build raisim examples and other modules.
The following options are available

* ``RAISIM_EXAMPLE`` : Compile C++ RaiSim examples
* ``RAISIM_MATLAB`` : Compile raisimMatlab (compiled binary is also provided). You need MATLAB for this option
* ``RAISIM_PY`` : Compile raisimPy. When installed, it will copy raisimpy files to your python site-packages directory

You can generate build files using CMake as following

To build raisimPy for the correct python version, activate your conda environment before calling cmake.

.. tabs::
  .. group-tab:: Linux or Mac

    .. code-block:: c

      cd $WORKSPACE/raisimLib
      mkdir build
      cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON
      make install -j4

  .. group-tab:: Windows

    We recommend using the CMake GUI.
    After generating the build files with the CMake GUI, you can build and install RaiSim using Visual Studio.
    Alternatively, you can do it as below in Windows Powershell

    .. code-block::

      cd $WORKSPACE/raisimLib
      mkdir build
      cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON
      cmake --build . --target install --config Release


.. note::
    **For Linux users**
    To use (vulkan version) RaiSimUnity in Linux, you need to install ``minizip``, ``ffmpeg`` and ``vulkan``.
    If you are going to use raisimGym, install the recommended version of gpu driver by pytorch.
    If not, install the latest version.

    To install ``minizip`` and ``ffmpeg``,

    .. code-block:: bash

        sudo apt install minizip ffmpeg

    Vulkan installation depends on your OS distribution.
    You can easily find instructions online.

    Ubuntu 22.04 users should run the following command before running RaisimUnity.

    .. code-block:: bash

        sudo ln -s /usr/lib/x86_64-linux-gnu/libdl.so.2 /usr/lib/x86_64-linux-gnu/libdl.so

    For other versions of Ubuntu, the symlink is provided by Ubuntu automatically.

    If you still cannot use raisimUnity, this probably means that your driver does not support vulkan.
    In that case, you should use raisimUnityOpengl.
    It only supports minimalistic graphics.

Activation Key
================

Rename the activation key that you received by email to ``activation.raisim``.
Save that file in ``<YOUR-HOME-DIR>/.raisim``.
In Linux and Mac, this is ``/home/<YOUR-USERNAME>/.raisim``.
In Windows, this is ``C:\Users\<YOUR-USERNAME>\.raisim`` (You might not be using ``C`` as your home directory).

RaiSim will also check the path you set by ``raisim::World::setActivationKey()``.
If the file is not found, it will search in the user directory, where you saved your ``activation.raisim`` file.

Examples
===============

The built examples are stored in ``examples`` directory.
In Windows, use powershell to run the examples, instead of manually clicking the icons.
If you made a mistake during installation, it will give you an error message.

Make sure that you run raisimUnity executable in ``raisimUnity/<OS>/RaiSimUnity`` before you run the examples.