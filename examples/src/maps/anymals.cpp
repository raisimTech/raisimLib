// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include <math.h>
#include <iostream>
#define  PI 3.1415926
int cnt1=0, cnt2=0;
void angle_generator(Eigen::VectorXd& angle_list, int idx, float T, float rate=1.f)
{
//    std::cout<< "size is "<<  angle_list.size() << std::endl;
    double base1= 0.4,  base3=0.03;
    double base2 = -2 * base1;
    double ang = abs(sin( float(idx) / T  * PI)) * rate;
    double ang2 = ang * 1.2;
    if (idx % int(T) == 1) cnt1++;
    int idx_base = 0;
//    std::cout<<idx_base+0 << " " << idx_base + 11 << std::endl;
//    jointNominalConfig <<  0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//    if(int(idx/(2*T)) %2 == 0 )
//    {
//        ang = ang;
//    }
//    else{
//        ang = 0;
//    }

    if ( (int(idx/T) % 2 ) == 1)
    {
        cnt2 ++;
        angle_list[idx_base+1] = ang2 + base1;
        angle_list[idx_base+2] = -2 * ang2 + base2;

        angle_list[idx_base+4] = base1;
        angle_list[idx_base+5] = base2;

        angle_list[idx_base+7] = - base1;
        angle_list[idx_base+8] = - base2;
        angle_list[idx_base+10] = -ang - base1;
        angle_list[idx_base+11] = 2 * ang - base2;
    }
    else
    {

        angle_list[idx_base+1] = base1;
        angle_list[idx_base+2] = base2;

        angle_list[idx_base+10] = - base1;
        angle_list[idx_base+11] = - base2;
        angle_list[idx_base+4] = ang2 + base1;
        angle_list[idx_base+5] = -2 * ang2 + base2;

        angle_list[idx_base+7] = -ang - base1;
        angle_list[idx_base+8] = 2 * ang - base2;
    }
    angle_list[idx_base+0] = base3;
    angle_list[idx_base+3] = -base3;
    angle_list[idx_base+6] = base3;
    angle_list[idx_base+9] = -base3;
//    std::cout<<cnt1 << " " << cnt2<<std::endl;
}

double cal(double low, double upp, double now){
    return (now + 1)/2  * (upp- low) + low;
}

double map_from_origin_to_limit(std::vector<raisim::Vec<2>>  join, Eigen::VectorXd& limit_list, Eigen::VectorXd& gen_list)
{
//    Eigen::VectorXd  limit_list;
//    join = join.tail(12);
    limit_list.setZero(join.size() - 6);
    std::cout<< "!------------" << gen_list.size()<< std::endl;
    int cnt = 0;
    int jp = 6;
    int kk = 0;
    for( auto i : join)
    {
        if(kk < jp){
            kk++;
            continue;
        }
        std::cout << i[1] << " " << i[0]<< std::endl;
        limit_list[cnt] = (cal(i[0] , i[1], gen_list[cnt]));
        cnt++;
    }
}

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.01);

  /// create objects
  auto ground = world.addGround(0, "gnd");
  ground->setAppearance("hidden");
  auto anymalC = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal.urdf");

  /// anymalC joint PD controller
  Eigen::VectorXd jointNominalConfig(anymalC->getGeneralizedCoordinateDim()), jointVelocityTarget(anymalC->getDOF());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(anymalC->getDOF()), jointDgain(anymalC->getDOF());
  jointPgain.tail(12).setConstant(200);
  jointDgain.tail(12).setConstant(4);
//  jointDgain[7] = 2;
//    jointDgain[10] = 2;
//    jointDgain[13] = 2;
//    jointDgain[16] = 2;

//    std::vector<size_t> footIndices_;
//
//    footIndices_.push_back(anymalC->getFrameIdxByName("LF_SHANK"));
//    footIndices_.push_back(anymalC->getFrameIdxByName("RF_SHANK"));
//    footIndices_.push_back(anymalC->getFrameIdxByName("LH_SHANK"));
//    footIndices_.push_back(anymalC->getFrameIdxByName("RH_SHANK"));

//    auto footframeindex = anymalC->getFrameIdxByName("foot")

  anymalC->setGeneralizedCoordinate(jointNominalConfig);
  anymalC->setGeneralizedForce(Eigen::VectorXd::Zero(anymalC->getDOF()));
  anymalC->setPdGains(jointPgain, jointDgain);
  anymalC->setName("anymalC");
//  std::vector<raisim::Vec<3>> foot_list;
//  for(auto i=0; i<footIndices_.size(); i++)
//  {
//      anymalC->getFramePosition(footIndices_[i], foot_list[i]);
//      std::cout<<"posi " << foot_list[i] <<std::endl;
//  }
  jointNominalConfig[1] = 1.0;

  /// friction example. uncomment it to see the effect
//  anymalB->getCollisionBody("LF_FOOT/0").setMaterial("LF_FOOT");
//  world.setMaterialPairProp("gnd", "LF_FOOT", 0.01, 0, 0);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.setMap("wheat");
  server.launchServer();
  server.focusOn(anymalC);

  /// graphs
  std::vector<std::string> jointNames = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                         "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

  raisim::VecDyn jc(12), jv(12), jf(12);
  Eigen::VectorXd angle_list(12);
  Eigen::Vector3d pose(0, 0, 3);
//  Eigen::Vector2d join;
  auto join = anymalC->getJointLimits();
  Eigen::VectorXd  limit_list;
  Eigen::VectorXd gene_angle;
  gene_angle.setZero(12);
    map_from_origin_to_limit(join, limit_list, gene_angle);
    std::cout<<std::endl << std::endl << limit_list<<std::endl;
//  for(auto i : limit_list)
//  {
//      std::cout<< i<<std::endl;
//  }


  for (int i=0; i<200000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
//    anymalC->setBasePos(pose);
    angle_generator(angle_list, i, 30.f, 0.3);

    jointNominalConfig.tail(12) = angle_list;
    if (i % 10 == 0) {

    }
      anymalC->setPdTarget(jointNominalConfig, jointVelocityTarget);
//      anymalC->setBasePos_e(pose);
//    anymalC->setGeneralizedCoordinate(jointNominalConfig);
  }

  server.killServer();
}
