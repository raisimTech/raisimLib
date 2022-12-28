// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  raisim::TerrainProperties terrainProperties;
  terrainProperties.frequency = 0.2;
  terrainProperties.zScale = 2.0;
  terrainProperties.xSize = 70.0;
  terrainProperties.ySize = 70.0;
  terrainProperties.xSamples = 70;
  terrainProperties.ySamples = 70;
  terrainProperties.fractalOctaves = 3;
  terrainProperties.fractalLacunarity = 2.0;
  terrainProperties.fractalGain = 0.25;

  auto hm = world.addHeightMap(0.0, 0.0, terrainProperties);
  auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/husky/husky.urdf");
  robot->setName("smb");
  hm->setAppearance("soil2");
  Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim()), gv(robot->getDOF()), damping(robot->getDOF());
  gc.setZero(); gv.setZero();
  gc.segment<7>(0) << 0, 0, 2, 1, 0, 0, 0;
  robot->setGeneralizedCoordinate(gc);
  robot->setGeneralizedVelocity(gv);
  damping.setConstant(0);
  damping.tail(4).setConstant(1.);
  robot->setJointDamping(damping);

  /// launch raisim server
  raisim::RaisimServer server(&world);

  /// this method should be called before server launch
  auto scans = server.addInstancedVisuals("scan points",
                                          raisim::Shape::Box,
                                          {0.05, 0.05, 0.05},
                                          {1,0,0,1},
                                          {0,1,0,1});
  int scanSize1 = 40;
  int scanSize2 = 50;

  scans->resize(scanSize1*scanSize2);
  server.launchServer();
  server.focusOn(robot);

  Eigen::Vector3d direction;

  for(int time=0; time<1000000; time++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
    raisim::Vec<3> lidarPos; raisim::Mat<3,3> lidarOri;
    robot->getFramePosition("imu_joint", lidarPos);
    robot->getFrameOrientation("imu_joint", lidarOri);

    for(int i=0; i<scanSize1; i++) {
      for (int j = 0; j < scanSize2; j++) {
        const double yaw = j * M_PI / scanSize2 * 0.6 - 0.3 * M_PI;
        double pitch = -(i * 0.3/scanSize1) + 0.2;
        const double normInv = 1. / sqrt(pitch * pitch + 1);
        direction = {cos(yaw) * normInv, sin(yaw) * normInv, -pitch * normInv};
        Eigen::Vector3d rayDirection;
        rayDirection = lidarOri.e() * direction;
        auto &col = world.rayTest(lidarPos.e(), rayDirection, 30);
        if (col.size() > 0) {
          scans->setPosition(i * scanSize2 + j, col[0].getPosition());
          float length = (col[0].getPosition() - lidarPos.e()).norm();
          scans->setColorWeight(i * scanSize2 + j, std::min(length/15.f, 1.0f));
        }
        else
          scans->setPosition(i*scanSize2+j, {0, 0, 100});
      }
    }

    robot->setGeneralizedForce({0, 0, 0, 0, 0, 0, -20, -20, -20, -20});
    gc = robot->getGeneralizedCoordinate().e();

    if(fabs(gc[0])>35. || fabs(gc[1])>35.) {
      gc.segment<7>(0) << 0, 0, 2, 1, 0, 0, 0;
      gv.setRandom();
      robot->setState(gc, gv);
    }
  }

  server.killServer();
}
