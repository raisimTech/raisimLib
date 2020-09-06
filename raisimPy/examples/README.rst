Examples
========

This folder contains examples that are the same as the ones that can be found in
`raisimLib <https://github.com/leggedrobotics/raisimLib>`_, `raisimOgre <https://github.com/leggedrobotics/raisimOgre>`_,
and `raisimGym <https://github.com/leggedrobotics/raisimGym>`_. The examples however use the `raisimpy` library
(which are wrappers around ``raisimLib`` and ``raisimOgre``) and are purely written in Python.

For the ``raisimpy_gym`` examples, you will first have to install them before using them by typing from this folder:

.. code-block:: bash

     pip install -e .

This will install notably ``gym`` and ``stable_baselines``. If you don't want to mess up with your local Python
environment, it is always a good idea to create a virtual environment, as such:

.. code-block:: bash

    # install pip and virtualenv
    sudo apt install python3-pip
    sudo pip install virtualenv

    # create virtual environment
    virtualenv -p /usr/bin/python<version> <virtualenv_name>

    # activate the virtual environment
    source <virtualenv_name>/bin/activate

where ``<version>`` is the python version you want to use (e.g. ``3.5``), and ``<virtualenv_name>`` is a name of your
choice for the virtual environment (e.g. ``py3.5``).

Once the environment has been activated, you can then type the command "``pip install -e .``".

To deactivate the virtual environment, just type:

.. code-block:: bash

    deactivate
