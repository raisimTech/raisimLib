//
// Created by jemin on 2022-07-21.
//

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char **argv) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const int loopN = 200000000;
  raisim::RaiSimMsg::setFatalCallback([](){throw;}); /// this will ensure that throw is called upon error

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

  auto anymal = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal_sensored.urdf");
  anymal->setState(jointConfig, jointVel);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setPdTarget(jointConfig, jointVelocityTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setName("Anymal");

  auto front_depthSensor = anymal->getSensor<raisim::DepthCamera>("depth_camera_front_camera_parent:depth");
  front_depthSensor->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);

  auto front_rgbCamera = anymal->getSensor<raisim::RGBCamera>("depth_camera_front_camera_parent:color");
  front_rgbCamera->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);

  auto rear_depthSensor = anymal->getSensor<raisim::DepthCamera>("depth_camera_rear_camera_parent:depth");
  rear_depthSensor->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);

  auto rear_rgbCamera = anymal->getSensor<raisim::RGBCamera>("depth_camera_rear_camera_parent:color");
  rear_rgbCamera->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);

  server.setupSocket();
  server.acceptConnection(2000.0);

  for (int k = 0; k < loopN; k++) {
//    RS_TIMED_LOOP(int(world.getTimeStep()*1e7))
//    for (int i = 0; i < 10; i++)
    world.integrate();

    if (server.waitForMessageFromClient(1.0)) {
      if (!server.processRequests()) {
        server.acceptConnection(2000.0);
      }
    }
  }

  server.closeConnection();
  return 0;
}