RaiSimPy: Python wrapper for RaiSim
===================================

This folder contains python wrappers around RaiSim (``raisimLib`` and ``raisimOgre``) using ``pybind11``.

Small parts of the wrappers were inspired by the code given in the ``raisimGym/raisim_gym/env/`` folder. 
If you use these wrappers, please acknowledge their contribution as well by citing [1-4].

The following wrappers have been tested on Ubuntu 16.04 with Python 3.5 and 2.7. Other platforms and Python 
versions might work as well but we didn't test them yet.


How to use the wrappers?
~~~~~~~~~~~~~~~~~~~~~~~~

In order to use the wrappers, you will have to install at least
`raisimLib <https://github.com/leggedrobotics/raisimLib>`_ and
`raisimOgre <https://github.com/leggedrobotics/raisimOgre>`_. You will also have to install
`pybind11 <https://pybind11.readthedocs.io/en/stable/>`_ as we use this to wrap the C++ code.

If you followed the installation procedure of ``raisimLib`` and/or ``raisimOgre``, you will have the two following
environment variables defined:

- WORKSPACE: workspace where you clone your git repos (e.g., ~/raisim_workspace)
- LOCAL_BUILD: build directory where you install exported cmake libraries (e.g., ~/raisim_build)

First, clone this repository:

.. code-block:: bash

    cd $WORKSPACE
    git clone https://github.com/robotlearn/raisimpy
    cd raisimpy

Then, compile the python wrappers from the ``raisimpy`` folder by typing:

.. code-block:: bash

    mkdir build && cd build
    cmake -DPYBIND11_PYTHON_VERSION=$PYTHON_VERSION -DCMAKE_PREFIX_PATH=$LOCAL_BUILD -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD ..
    make -j4
    make install

where ``$PYTHON_VERSION`` is the Python version you wish to use (e.g. ``PYTHON_VERSION=3.5``).

Now, you just need to ``export PYTHONPATH=$PYTHONPATH:$LOCAL_BUILD/lib`` to be able to access the python library. You can 
add this ``export`` line in your ``bashrc``.

Once it has been compiled, you can access to the Python library ``raisimpy`` in your code with:

.. code-block:: python

    import raisimpy as raisim

    print(dir(raisim))


We mostly follow the naming convention defined in ``raisimLib`` and ``raisimOgre``, however we follow the PEP8 guideline.
Thus, a C++ method like:

.. code-block:: cpp

    getComPosition()

becomes

.. code-block:: python

    get_com_position()


Note that in the original ``raisimLib``, the authors sometimes use their own defined data types for vectors and
matrices (such as ``Vec<n>``, ``Mat<n,m>``, ``VecDyn``, ``MatDyn``, etc). The Ogre library used for the 
visualization also uses its own defined data types (such as ``Vector3``, ``Matrix3``, ``Quaternion``, etc). 
When using ``raisimpy``, these datatypes are automatically converted (back and forth) to numpy arrays as this 
is the standard in Python. We also follow the convention that if an attribute is a python list or std::vector, 
we add the suffix 's' at the end of the attribute, and we write the full name of the variables (i.e. without 
using diminutives), such as:

.. code-block:: cpp

    Body b;
    std::vector<Shape::Type> shapes = b.colshape;

in C++, becomes in Python:

.. code-block:: python

    b = Body()
    shapes = b.collision_shapes  # no diminutives (colshape --> collision_shape), and added the 's' suffix to specify it is a list.


The code has been documented, so you can always see it using ``help`` or ``?``, for instance:

.. code-block:: python

    import raisimpy as raisim
    
    world = raisim.World()
    world.add_sphere?
    # or
    help(world.add_sphere)


Examples
~~~~~~~~

Here is the C++ example that was provided in the README in [2]:

.. code-block:: cpp

    #include "raisim/World.hpp"

    int main() {
        raisim::World world;
        auto anymal = world.addArticulatedSystem("pathToURDF"); // initialized to zero angles and identity orientation. Use setState() for a specific initial condition
        auto ball = world.addSphere(1, 1); // radius and mass
        auto ground = world.addGround();

        world.setTimeStep(0.002);
        world.integrate();
    }

This becomes in Python:

.. code-block:: python

    import raisimpy as raisim

    world = raisim.World()
    anymal = world.add_articulated_system("path_to_urdf")
    ball = world.add_sphere(radius=1, mass=1)
    ground = world.add_ground()

    world.set_time_step(0.002)
    world.integrate()


Other examples can be found in the ``examples`` folder, which are the sames as the ones that you can find in the
``examples`` folders in ``raisimLib`` [2], ``raisimOgre`` [3], and ``raisimGym`` [4]. I will add the other examples
as soon as I have the time.


References
~~~~~~~~~~

- [1] "Per-contact iteration method for solving contact dynamics", Hwangbo et al., 2018
- [2] raisimLib: https://github.com/leggedrobotics/raisimLib
- [3] raisimOgre: https://github.com/leggedrobotics/raisimOgre
- [4] raisimGym: https://github.com/leggedrobotics/raisimGym
- [5] pybind11: https://pybind11.readthedocs.io/en/stable/


Troubleshooting
~~~~~~~~~~~~~~~

- ``fatal error: Eigen/*: No such file or directory``
    - If you have Eigen3 installed on your system, you probably have to replace all the ``#include <Eigen/*>`` by
      ``#include <eigen3/Eigen/*>``. You can create symlinks to solve this issue:

    .. code-block:: bash

        cd /usr/local/include
        sudo ln -sf eigen3/Eigen Eigen
        sudo ln -sf eigen3/unsupported unsupported

    or you can replace the ``#include <Eigen/*>`` by ``#include <eigen3/Eigen/*>``.

- Segmentation fault. This is probably an oversight on my part, the error is probably due to some poor management 
  of pointers and memory allocation. E.g. an object has been deleted from the Python side but the C++ side is also 
  trying to delete it. Just open an issue on Github, and I will fix it as soon as I can. If you fixed it, you can 
  submit a pull request.

- Error about adress mapping while running RL examples, like the following one:

    .. code-block:: bash

        Signal: Segmentation fault (11)
        Signal code: Address not mapped (1)
        Failing at address: xxxxx

    this seems to be caused by TensorFlow, but I have to investigate deeper what causes that error.


LICENSE
~~~~~~~

The following software is distributed under the `MIT <https://choosealicense.com/licenses/mit/>`_ License, 
however the RaiSim software is under the End-User License Agreement that you can find 
`here <https://github.com/leggedrobotics/raisimLib/blob/master/LICENSE.md>`_.


Citation
~~~~~~~~

If the code presented here was useful to you, we would appreciate if you could cite the original authors:

.. code-block:: latex

    @article{hwangbo2018per,
        title={Per-contact iteration method for solving contact dynamics},
        author={Hwangbo, Jemin and Lee, Joonho and Hutter, Marco},
        journal={IEEE Robotics and Automation Letters},
        volume={3},
        number={2},
        pages={895--902},
        year={2018},
        publisher={IEEE}
    }


If you still have some space in your paper for the references, you can add the following citation:

.. code-block:: latex

    @misc{delhaisse2019raisimpy
        author = {Delhaisse, Brian and Rozo, Leonel},
    	title = {RaiSimPy: A Python wrapper for RaiSim},
    	howpublished = {\url{https://github.com/robotlearn/raisimpy}},
    	year=2019,
	}

Otherwise, you can just add me in the acknowledgements ;)

If you use ``raisimpy`` through the `pyrobolearn <https://github.com/robotlearn/pyrobolearn>`_ framework (this is an
ongoing work), you can cite this last one instead (but you still have to cite the authors of RaiSim).

