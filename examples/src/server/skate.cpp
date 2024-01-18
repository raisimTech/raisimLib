//
// Created by lr-2002 on 23-8-18.
//
// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.01);

    /// create objects
    world.addGround();
    raisim::RaisimServer server(&world);
    server.launchServer();
    auto skate = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\skate\\skate.urdf");
    /// skate state
    Eigen::VectorXd jointNominalConfig(skate->getGeneralizedCoordinateDim());
    skate->setName("skate");
    jointNominalConfig.setZero();
//    jointNominalConfig[1] = 0.01;
    jointNominalConfig<< 0, 0, 0.11, 1 , 0, 0, 0 ,0 , 0;
    Eigen::VectorXd jointVelocityTarget(skate->getDOF() );
    jointVelocityTarget.setZero();
    skate->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    Eigen::VectorXd jointPgain(skate->getDOF()), jointDgain(skate->getDOF());
    jointPgain.tail(2).setConstant(80);
    jointDgain.tail(2).setConstant(4);

    skate->setGeneralizedCoordinate(jointNominalConfig);
    skate->setGeneralizedForce(Eigen::VectorXd::Zero(skate->getDOF()));
    skate->setPdGains(jointPgain, jointDgain);
    skate->setPdTarget(jointNominalConfig, jointVelocityTarget);
    skate->setGeneralizedCoordinate(jointNominalConfig);
    std::cout << " position " << jointNominalConfig;
    /// launch raisim server

    std::cout<<"gra " << world.getGravity();
    std::cout << "\n mov ";
    auto na = skate->getMovableJointNames();

    server.focusOn(skate);
    for (auto i:na)
    {
        std::cout << i << std::endl;
    }
    for (int i=0; i<200000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
        jointNominalConfig.tail(2).setConstant( 0.02 * i);
        jointVelocityTarget.tail(2).setConstant(0);
        skate->setPdTarget(jointNominalConfig, jointVelocityTarget);
    }

    server.killServer();
}

