// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.001);

  auto ANYmal = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal_c_nolimit.urdf");
  auto ground = world.addGround();

  /// anymalC joint PD controller
  Eigen::VectorXd jointNominalConfig(ANYmal->getGeneralizedCoordinateDim()), jointVelocityTarget(ANYmal->getDOF());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  for (int i=0; i<ANYmal->getDOF(); i++)
    jointVelocityTarget[i] = (i+4)*0.01;

  Eigen::VectorXd jointPgain(ANYmal->getDOF()), jointDgain(ANYmal->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  ANYmal->setGeneralizedCoordinate(jointNominalConfig);
  ANYmal->setGeneralizedForce(Eigen::VectorXd::Zero(ANYmal->getDOF()));
  ANYmal->setPdGains(jointPgain, jointDgain);
  ANYmal->setPdTarget(jointNominalConfig, jointVelocityTarget);
  ANYmal->setName("anymalC");

  /// this allows inverse dynamics computation
  ANYmal->setComputeInverseDynamics(true);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(ANYmal);
  Eigen::VectorXd torqueFromInverseDynamics(ANYmal->getDOF());

  for (int i=0; i<8000; i++) {
    RS_TIMED_LOOP(1e3)
    std::vector<Eigen::Vector3d> axes(ANYmal->getDOF()-6);

    /// joint axis should be stored before integration because it changes after integration
    for (int j=0; j<ANYmal->getDOF()-6; j++)
      axes[j] = ANYmal->getJointAxis(j+1).e();

    ANYmal->setExternalForce(3, {0,0,-0.1}, {1.5, 0, 0});
    ANYmal->setExternalForce(2, {0,0,-0.1}, {1.5, 0, 0});
    ANYmal->setExternalForce(0, {0.1,0,-0.1}, {1.5, 0, 10.});

    server.integrateWorldThreadSafe();

    /// retrieve force/torque acting at joints
    torqueFromInverseDynamics.head(3) = ANYmal->getForceAtJointInWorldFrame(0).e();
    torqueFromInverseDynamics.segment<3>(3) = ANYmal->getTorqueAtJointInWorldFrame(0).e();

    for (size_t j=1; j<ANYmal->getDOF()-5; j++)
      torqueFromInverseDynamics(j+5) = ANYmal->getTorqueAtJointInWorldFrame(j).dot(axes[j-1]);

    /// They do not match perfectly if you use Raisim PD controller because it is an implicit PD controller.
    std::cout<<"torqueFromInverseDynamics      "<<torqueFromInverseDynamics.transpose()<<std::endl;
    std::cout<<"generalized force that you set "<<ANYmal->getGeneralizedForce().e().transpose()<<std::endl;
  }

  server.killServer();
}
