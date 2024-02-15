#############################
Math Classes
#############################

RaiSim incorporates its dedicated math library, located within the `raisim/math` directory.
It is important to note that while the RaiSim math library is available, it is not recommended for direct use by users.
However, several RaiSim methods generate a RaiSim math object as output.
In these situations, conversion to an Eigen object is required.
To facilitate this conversion process, RaiSim provides a convenient method within the Mat class: the ``e()`` method, which returns the corresponding ``Eigen::Map``.
An example demonstrating this conversion is provided below.

.. code-block:: c

  /// convertsion from a raisim object to an Eigen object (i.e., Eigen::Map)
  raisim::Mat<3,3> matrix33RaiSim;
  matrix33RaiSim.setIdentity();
  Eigen::Matrix3d matrix33Eigen = matrix33RaiSim.e();
  std::cout<<"matrix33Eigen\n"<<matrix33Eigen<<std::endl; // pretty std output formatting is not implemented yet. So we use Eigen's

  /// use a raisim object as if it is an Eigen object
  Eigen::Vector3d vector3Eigen1, vector3Eigen2;
  vector3Eigen1.setConstant(1);
  vector3Eigen2 = matrix33RaiSim.e() * vector3Eigen1;
  std::cout<<"vector3Eigen2\n"<<vector3Eigen2<<std::endl;
