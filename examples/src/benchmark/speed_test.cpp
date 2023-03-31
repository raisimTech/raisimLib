// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include "benchmarkCommon.hpp"
#include <raisim/World.hpp>
#include <Eigen/Dense>
#include "raisim/RaisimServer.hpp"


int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  int loopN = 1000000;
  std::chrono::steady_clock::time_point begin, end;

  raisim::World world;
  auto checkerBoard = world.addGround();

  Eigen::VectorXd jointConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointVel(18), jointPgain(18), jointDgain(18);

  jointPgain.setZero();
  jointPgain.tail(12).setConstant(200.0);

  jointDgain.setZero();
  jointDgain.tail(12).setConstant(10.0);

  jointVelocityTarget.setZero();

  jointConfig << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  auto anymal = world.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/anymal/urdf/anymal.urdf");
  anymal->setState(jointConfig, jointVel);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setPdTarget(jointConfig, jointVelocityTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
//  anymal->setComputeInverseDynamics(true);

  begin = std::chrono::steady_clock::now();

  for (int i = 0; i < loopN; i++) {
    world.integrate();
  }

  end = std::chrono::steady_clock::now();

  raisim::print_timediff("ANYmal, 4 contacts", loopN, begin, end);

  world.removeObject(checkerBoard);
  begin = std::chrono::steady_clock::now();

  for (int i = 0; i < loopN; i++) {
    world.integrate();
  }

  end = std::chrono::steady_clock::now();

  raisim::print_timediff("ANYmal, 0 contact", loopN, begin, end);

  /////////////////////////// atlas ////////////////////////////////////
  raisim::World world2;
  world2.setTimeStep(0.001);

  loopN = 100000;

  /// create objects
  auto ground2 = world2.addGround();
  auto atlas = world2.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/atlas/robot.urdf");
  atlas->setGeneralizedCoordinate(
      {0,               0,       1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  atlas->setGeneralizedForce(Eigen::VectorXd::Zero(atlas->getDOF()));

  size_t contactSize = 0;
  begin = std::chrono::steady_clock::now();

  for (int i = 0; i < loopN; i++) {
    world2.integrate();
    contactSize += world2.getContactProblem()->size();
  }

  end = std::chrono::steady_clock::now();

  raisim::print_timediff("Atlas", loopN, begin, end);
  std::cout<<"average number of contacts "<<double(contactSize) / loopN<<std::endl;

  world2.removeObject(ground2);
  begin = std::chrono::steady_clock::now();
  contactSize = 0;
  for (int i = 0; i < loopN; i++) {
    world2.integrate();
    contactSize += world2.getContactProblem()->size();
  }

  end = std::chrono::steady_clock::now();

  raisim::print_timediff("Atlas, no ground", loopN, begin, end);
  std::cout<<"average number of contacts "<<double(contactSize) / loopN<<std::endl;

  //////////////////////////////////// chain 10 /////////////////////////////////////////////////////
  raisim::World world3;
  world3.setTimeStep(0.001);

  loopN = 100000;

  /// create objects
  auto chain10 = world3.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/chain/robot_springed_10.urdf");
  std::cout<<"chain10 dof is "<<chain10->getDOF()<<std::endl;

  begin = std::chrono::steady_clock::now();

  for (int i = 0; i < loopN; i++)
    world3.integrate();

  end = std::chrono::steady_clock::now();

  raisim::print_timediff("Chain10 (30 dof)", loopN, begin, end);

  raisim::World world4;
  world4.setTimeStep(0.001);

  loopN = 100000;

  /// create objects
  auto chain20 = world4.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/chain/robot_springed_20.urdf");
  std::cout<<"chain 20 dof is "<<chain20->getDOF()<<std::endl;

  begin = std::chrono::steady_clock::now();

  for (int i = 0; i < loopN; i++)
    world4.integrate();

  end = std::chrono::steady_clock::now();

  raisim::print_timediff("Chain20 (60 dof)", loopN, begin, end);
}