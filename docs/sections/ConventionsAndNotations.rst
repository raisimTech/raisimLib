#############################
Conventions and Notations
#############################


Kinematics
===============

1. A quaternion is defined as :math:`\mathbf{q} = [w, x, y, z]^T`. :math:`[x, y, z]^T` is the (scaled) axis of rotation and :math:`w` is cosine of half of the angle.
2. Unless otherwise stated explicitly, every quantities in RaiSim is expressed in the world frame and relative to the world frame.
3. The world frame has zero acceleration and zero velocity
4. For articulated systems, the base angular velocity is defined in the world frame (many other simulators use the base frame). For details, read :ref:`articulated_systems`


We represent a position vector in a Cartesian coordinate as :math:`^Wr`, where :math:`^W` is the frame in which the vector is expressed.
In this document, :math:`W` represents the **world frame** which is an inertial frame (zero acceleration).
Unless otherwise stated explicitly, all vector quanties in RaiSim are expressed in the world frame.

A position vector can be expressed in any frame. To express the vector in a different frame, we use a linear transformation as shown below

.. math::

  \begin{eqnarray}
    ^Br&= {}^{BW\!}R ^Wr,\\
    ^Wr&= {}^{WB\!}R ^Br.
  \end{eqnarray}

In this document, symbol :math:`^B` represents the **body frame**, which is assigned to every bodies in RaiSim. 
:math:`^{WB}R` is called **body rotation matrix**. 
**Note that the body rotation matrix maps a vector expressed in the body frame to a vector expressed in the world frame**