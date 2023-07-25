//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include "Environment.hpp"
#include "VectorizedEnvironment.hpp"

int THREAD_COUNT = 1;

using namespace raisim;
#define  PI 3.1415926
//void angle_generator(Eigen::VectorXd& angle_list, int idx, float T, float rate=1.f)
//{
////    std::cout<< "size is "<<  angle_list.size() << std::endl;
//    double base1= 0.4, base2 = -0.8, base3=0.03;
//    double ang = abs(sin( float(idx) / T  * PI)) * rate;
//
//    int idx_base = 0;
////    std::cout<<idx_base+0 << " " << idx_base + 11 << std::endl;
////    jointNominalConfig <<  0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//    if ( (int(idx/T) % 2 ) == 1)
//    {
//
//        angle_list[idx_base+1] = ang + base1;
//        angle_list[idx_base+2] = -2 * ang + base2;
//
//        angle_list[idx_base+4] = base1;
//        angle_list[idx_base+5] = base2;
//
//        angle_list[idx_base+7] = - base1;
//        angle_list[idx_base+8] = - base2;
//        angle_list[idx_base+10] = -ang - base1;
//        angle_list[idx_base+11] = 2 * ang - base2;
//    }
//    else
//    {
//
//        angle_list[idx_base+1] = base1;
//        angle_list[idx_base+2] = base2;
//
//        angle_list[idx_base+10] = - base1;
//        angle_list[idx_base+11] = - base2;
//        angle_list[idx_base+4] = ang + base1;
//        angle_list[idx_base+5] = -2 * ang + base2;
//
//        angle_list[idx_base+7] = -ang - base1;
//        angle_list[idx_base+8] = 2 * ang - base2;
//    }
//    angle_list[idx_base+0] = base3;
//    angle_list[idx_base+3] = -base3;
//    angle_list[idx_base+6] = base3;
//    angle_list[idx_base+9] = -base3;
//
//}

void change_row(EigenRowMajorMat & matri, Eigen::VectorXd& vec, int idx)
{
    for(int i = 0 ;i < vec.size(); i++)
    {
        matri.row(idx)[i] = vec[i];
    }
}


int main(int argc, char *argv[]) {
  RSFATAL_IF(argc != 3, "got "<<argc<<" arguments. "<<"This executable takes three arguments: 1. resource directory, 2. configuration file")

  std::string resourceDir(argv[1]), cfgFile(argv[2]);
  std::ifstream myfile (cfgFile);
  std::string config_str, line;
  bool escape = false;

  while (std::getline(myfile, line)) {
    if(line == "environment:") {
      escape = true;
      while (std::getline(myfile, line)) {
        if(line.substr(0, 2) == "  ")
          config_str += line.substr(2) + "\n";
        else if (line[0] == '#')
          continue;
        else
          break;
      }
    }
    if(escape)
      break;
  }
  config_str.pop_back();
  VectorizedEnvironment<ENVIRONMENT> vecEnv(resourceDir, config_str);

  Yaml::Node config;
  Yaml::Parse(config, config_str);

  EigenRowMajorMat observation(config["num_envs"].template As<int>(), vecEnv.getObDim());
  EigenRowMajorMat action(config["num_envs"].template As<int>(), vecEnv.getActionDim());
  EigenVec reward(config["num_envs"].template As<int>(), 1);
  EigenBoolVec dones(config["num_envs"].template As<int>(), 1);
  action.setZero();
//  for(auto i:action.rows())
  std::cout<<action.size()<< std::endl;
    Eigen::VectorXd bb;
    bb.setZero(12);
//    std::cout<<bb <<std::endl;
//    bb.setOnes(12);
//    change_row(action,bb, 0);

//  angle_generator(bb, 0, 1000.f);
//  for(auto i=0; i< 100; i++)
//      change_row(action,bb,i);
  Eigen::Ref<EigenRowMajorMat> ob_ref(observation), action_ref(action);
  Eigen::Ref<EigenVec> reward_ref(reward);
  Eigen::Ref<EigenBoolVec> dones_ref(dones);

//    std::cout<<action_ref;
    bb.setZero();
  vecEnv.reset();
  for(auto i =0 ;i <=2000000; i ++)
  {
      angle_generator(bb, i, 80.f, 1);
//      bb[2] = 0;
std::cout<<"1"<<std::endl;
      for(auto j=0; j< 100; j++)
          change_row(action,bb,j);
//      sleep(/**/0.1);
      MSLEEP(10);
//      RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
      vecEnv.step(action_ref, reward_ref, dones_ref);
  }

  return 0;
}