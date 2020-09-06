//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include "Environment.hpp"
#include "VectorizedEnvironment.hpp"

using namespace raisim;

int main(int argc, char *argv[]) {
  RSFATAL_IF(argc != 4, "got "<<argc<<" arguments. "<<"This executable takes three arguments: 1. resource directory, 2. configuration file, 3. render or no_render to control rendering")

  std::string resourceDir(argv[1]), cfgFile(argv[2]);
  Yaml::Node config = YAML::LoadFile(cfgFile);
  std::stringstream config_str;

  if(std::string(argv[3]) == "no_render") {
    config["environment"]["render"] = false;
    std::cout<<argv[3]<<std::endl;
  } else {
    std::cout<<argv[3]<<std::endl;
    config["environment"]["render"] = true;
  }

  config_str << config["environment"];

  VectorizedEnvironment<ENVIRONMENT> vecEnv(resourceDir, config_str.str());
  vecEnv.init();

  EigenRowMajorMat observation(config["environment"]["num_envs"].as<int>(), vecEnv.getObDim());
  EigenRowMajorMat action(config["environment"]["num_envs"].as<int>(), vecEnv.getActionDim());
  EigenVec reward(config["environment"]["num_envs"].as<int>(), 1);
  EigenBoolVec dones(config["environment"]["num_envs"].as<int>(), 1);
  action.setZero();

  Eigen::Ref<EigenRowMajorMat> ob_ref(observation), action_ref(action);
  Eigen::Ref<EigenVec> reward_ref(reward);
  Eigen::Ref<EigenBoolVec> dones_ref(dones);

  vecEnv.reset();
  vecEnv.step(action_ref, reward_ref, dones_ref);

  return 0;
}