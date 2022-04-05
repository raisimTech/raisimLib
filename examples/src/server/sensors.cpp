// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif


int main(int argc, char **argv) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const int loopN = 200000000;
  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  raisim::World world;
  raisim::RaisimServer server(&world);

  auto checkerBoard = world.addGround(0.0, "glass");

  Eigen::VectorXd jointConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointVel(18), jointPgain(18), jointDgain(18);

  jointPgain.setZero();
  jointPgain.tail(12).setConstant(200.0);

  jointDgain.setZero();
  jointDgain.tail(12).setConstant(10.0);

  jointVelocityTarget.setZero();

  jointConfig << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  auto anymal = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal.urdf");
  anymal->setState(jointConfig, jointVel);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setPdTarget(jointConfig, jointVelocityTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  auto depthSensor = anymal->getSensor<raisim::DepthCamera>("depth_camera_front_camera_parent/depth");
  depthSensor->setDataType(raisim::DepthCamera::DepthCameraProperties::DataType::COORDINATE);

  server.launchServer();
  std::vector<raisim::Visuals*> scans;
  for(int i=0; i<depthSensor->getProperties().width; i++)
    for(int j=0; j<depthSensor->getProperties().height; j++)
      scans.push_back(server.addVisualBox("box" + std::to_string(i) + "/" + std::to_string(j), 0.03, 0.03, 0.03, 1, 0, 0));

  for (int k = 0; k < loopN; k++) {
    world.integrate();
    raisim::MSLEEP(world.getTimeStep() * 1000);
    depthSensor->update(world);
    auto& pos = depthSensor->get3DPoints();
    for(int i=0; i<depthSensor->getProperties().width; i++)
      for(int j=0; j<depthSensor->getProperties().height; j++) {
        size_t id = i * depthSensor->getProperties().height + j;
        scans[id]->setPosition(pos[id].e());
      }
  }

  server.killServer();
  return 0;
}