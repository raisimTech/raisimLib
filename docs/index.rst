#############################
RaiSim
#############################

RaiSim is a cross-platform multi-body physics engine for robotics and AI.
It fully supports **Linux**, **Mac Os**, and **Windows**.
RaiSim is closed-source and is distributed under a few different types of license. Please check License section for details.

Examples
========

.. image:: image/demo_robots.gif
  :alt: RaiSimPy demo (robots.py)
  :width: 600
**RaisimPy example** in ``raisimPy/examples/robots.py``

.. image:: image/skeleton.bmp
  :alt: Skeleton img from S. K at KAIST
  :width: 600
**Biomechanical simulation**, created by Young-Jun Koo, PhD and Seungbum Koo, PhD at Musculoskeletal BioDynamics Lab, KAIST.
The geometric model is created using the Full-body musculoskeletal model in Rajagopal et al. (2016).

.. image:: image/huskyScan.gif
  :alt: husky
  :width: 600
**Ray Test example with Husky and Velodyne**

.. image:: image/anymals.png
  :alt: anymals
  :width: 600
**ANYmal B and C robots**, by ANYbotics AG.

.. toctree::
   :maxdepth: 1
   :caption: Before using RaiSim

   sections/License
   sections/Acknowledgement

.. toctree::
   :maxdepth: 1
   :caption: RaiSim C++

   sections/Introduction
   sections/Installation
   sections/ConventionsAndNotations
   sections/WorldSystem
   sections/Object
   sections/Contact
   sections/MaterialSystem
   sections/HeightMap
   sections/Constraints
   sections/RayTest

.. toctree::
   :maxdepth: 1
   :caption: RaiSimMatlab

   sections/RaiSimMatlabIntroduction
   sections/RaiSimMatlabExample

.. toctree::
   :maxdepth: 1
   :caption: RaiSimPy

   sections/RaiSimPyIntroduction
   sections/RaiSimPyExample

.. toctree::
   :maxdepth: 1
   :caption: Related Software

   sections/RaisimGymTorch
   sections/RaisimUnity
