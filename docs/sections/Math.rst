#############################
Math Classes
#############################

RaiSim provides its own math library.
All files related to the math library can be found in `raisim/math`.
I do not recommend using RaiSim math library to users.
But many RaiSim methods return a RaiSim math object.
In such a case, you have to convert it to an Eigen object.
RaiSim includes a convenient conversion method in the Mat class: ``e()`` method returns the corresponding ``Eigen::Map``.
Here is an example.

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
