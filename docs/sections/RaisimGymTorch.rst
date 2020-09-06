#############################
RaisimGymTorch
#############################

How does it work?
=============================
RaiSimGymTorch wraps a c++ environment (i.e., ENVIRONMENT.hpp) as a python library using Pybind11.
When you call ``python3 setup.py develop``, all environments under ``raisimGymTorch/raisimGymTorch/env/envs`` are compiled.
The compiled libraries are stored in ``raisimGymTorch/raisimGymTorch/env/bin``.
Then you can import your environment to your python code.
For example, the anymal locomotion example can be imported as ``from raisimGymTorch.env.bin import rsg_anymal``

All the rest happens in Python.
Your launch file (e.g., ``runner.py``) can be customized for your need.



How to add a custom environment?
===================================
You can add your environment in ``raisimGymTorch/raisimGymTorch/env/envs``.
If you want to keep your source file somewhere else, then add a symlink to it in ``raisimGymTorch/raisimGymTorch/env/envs``.


Code structure
===================================
``ENVIRONMENT`` class is where you define the dynamics, reward, termination condition and so on.
This class inherits from "RaisimGymEnv.hpp", which add basic functionalities to the environment such as ``setSimulationTimeStep``, ``setControlTimeStep``, ``getObDim`` and so on.
If ``RaisimGymEnv`` is not general enough for you, you can also make ``ENVIRONMENT`` independent from ``RaisimGymEnv``.

``RaisimGymEnv`` is wrapped by ``VectorizedEnvironment``, which parallelizes the environment.
You can consider it similar to ``VectorEnv`` in OpenAI Baselines.
But RaisimGym parallelization happens in C++, which make it much faster.

''raisim_gym.cpp`` is a Pybind11 wrapping of ``VectorizedEnvironment``.
It simply defines the interface functions.

Finally, RaisimGymVecEnv is a python class that wraps a python library created from ``raisim_gym.cpp``.


