//
// Created by lr-2002 on 23-7-14.
//
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
float calculate_angle(float ang)
{
    return ang * 3.14 / 180;
}
void print_collision(raisim::Sphere* sphere)
{

    auto& contacts = sphere->getContacts();
    for (auto i : contacts)
    {
        std::cout<< i.getPosition() << std::endl;
    }
}
int main(int argc, char * argv[])
{
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

    // create raisim world
    raisim::World world;
    world.setTimeStep(0.01);
    auto ground = world.addGround();

    world.setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001);
    // launch server

    raisim::RaisimServer server(&world);
    server.launchServer();

    auto sphere = world.addSphere(0.4, 0.3, "steel");
    sphere->setName("sphere");
    auto leg = world.addArticulatedSystem(binaryPath.getDirectory()+"\\rsc\\leg\\leg2.urdf");
    leg->setName("leg");

//    Eigen::VectorXd gc(3);
//    gc << 0, 0, 0;
//    leg->setGeneralizedCoordinate(gc);


//    Eigen::VectorXd gc(3);
//    Eigen::VectorXd gv(3);
//    gc << calculate_angle(-50), 0, calculate_angle(-50);
//    gv << 0, -1, -1;
//    sphere->setPosition(0, 0, 0.4);
//    int cnt = 0;
//    auto foot_index = leg->getBodyIdx("r_2");
    while(1)
    {
//        cnt ++ ;
//        gc << calculate_angle(-50 + 0.8* cnt), 0, calculate_angle(-90);
//        print_collision(sphere);
//        std::cout<<calculate_angle(-50 + 0.8* cnt) <<" " << cnt <<  std::endl;
//        leg->setGeneralizedCoordinate(gc);
//        leg->setGeneralizedVelocity(gv);
//        leg->setGeneralizedForce(gc);
        server.integrateWorldThreadSafe();
        raisim::MSLEEP(10);
//        for(auto& contact : leg->getContacts())
//        {
//            if(contact.getPairContactIndexInPairObject() != raisim::BodyType::STATIC)
//            {
//                world.getObject(contact.getPairObjectIndex())->getContacts();
//            }
//        }
    }

    server.killServer();
}