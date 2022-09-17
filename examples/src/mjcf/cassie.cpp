// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
  timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif
  raisim::World world(binaryPath.getDirectory()+"\\rsc\\cassie\\cassie.xml");
  raisim::RaisimServer server(&world);
  auto torso = static_cast<raisim::ArticulatedSystem*>(world.getObject("cassie-pelvis"));
  torso->setGeneralizedCoordinate({ 0,0,1,1,0,0,0,
                                    0.0045, 0, 0.4973,                  //  left-hip-roll, left-hip-yaw, left-hip-pitch
                                    0.9785, -0.0164, 0.01787, -0.2049,  //  left-achilles-rod(unactuated, U) Quaternion
                                    -1.1997, 0, 1.4267, 0,               //  left-knee, left-shin(U), left-tarsus(U), left-heel-spring(U),
                                    -1.5244, 1.5244, -1.5968,
                                    -0.0045, 0, 0.4973,
                                    0.9786, 0.00386, -0.01524, -0.2051,
                                    -1.1997, 0, 1.4267, 0,
                                    -1.5244, 1.5244, -1.5968});            //  left-foot-crank(U), left-planar-rod(U), left-foot
  server.launchServer();
  for (int i=0; i<10000000; i++) {
    world.integrate();
    raisim::MSLEEP(1);
  }
  server.killServer();
}