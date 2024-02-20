// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  world.addGround();
  auto minitaur = world.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/minitaur/minitaur.urdf");
  minitaur->setName("minitaur");

/// anymalC joint PD controller
  Eigen::VectorXd jointNominalConfig(minitaur->getGeneralizedCoordinateDim()), jointVelocityTarget(minitaur->getDOF());
  jointNominalConfig << 0,0,0.35, 0,0,1,0,
      -M_PI_2,-2.2,-M_PI_2,-2.2,
      -M_PI_2,-2.2,-M_PI_2,-2.2,
      -M_PI_2,-2.2,-M_PI_2,-2.2,
      -M_PI_2,-2.2,-M_PI_2,-2.2;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(minitaur->getDOF()), jointDgain(minitaur->getDOF());
  /// we set the gains of the unactuated joints to zero
  jointPgain << 0,0,0, 0,0,0,
      20,0.,20,0.,
      20,0.,20,0.,
      20,0.,20,0.,
      20,0.,20,0.;
  jointDgain << 0,0,0, 0,0,0,
      0.1,0.,0.1,0.,
      0.1,0.,0.1,0.,
      0.1,0.,0.1,0.,
      0.1,0.,0.1,0.;

  minitaur->setGeneralizedCoordinate(jointNominalConfig);
  minitaur->setGeneralizedForce(Eigen::VectorXd::Zero(minitaur->getDOF()));
  minitaur->setPdGains(jointPgain, jointDgain);
  minitaur->setPdTarget(jointNominalConfig, jointVelocityTarget);
  minitaur->setName("minitaur");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(minitaur);

  for (int i=0; i<2000000; i++) {
    RS_TIMED_LOOP(1e6*world.getTimeStep())
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
