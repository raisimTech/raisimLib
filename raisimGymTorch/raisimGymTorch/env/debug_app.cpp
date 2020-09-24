//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include "Environment.hpp"
#include "VectorizedEnvironment.hpp"

using namespace raisim;

int main(int argc, char *argv[]) {
  RSFATAL_IF(argc != 3, "got "<<argc<<" arguments. "<<"This executable takes three arguments: 1. resource directory, 2. configuration file")

  std::string resourceDir(argv[1]), cfgFile(argv[2]);
  std::ifstream myfile (cfgFile);
  std::string config_str((std::istreambuf_iterator<char>(myfile)), std::istreambuf_iterator<char>(myfile));

  VectorizedEnvironment<ENVIRONMENT> vecEnv(resourceDir, config_str);
  vecEnv.init();

  Yaml::Node config;
  Yaml::Parse(config, cfgFile);

  EigenRowMajorMat observation(config["environment"]["num_envs"].As<int>(), vecEnv.getObDim());
  EigenRowMajorMat action(config["environment"]["num_envs"].As<int>(), vecEnv.getActionDim());
  EigenVec reward(config["environment"]["num_envs"].As<int>(), 1);
  EigenBoolVec dones(config["environment"]["num_envs"].As<int>(), 1);
  action.setZero();

  Eigen::Ref<EigenRowMajorMat> ob_ref(observation), action_ref(action);
  Eigen::Ref<EigenVec> reward_ref(reward);
  Eigen::Ref<EigenBoolVec> dones_ref(dones);

  vecEnv.reset();
  vecEnv.step(action_ref, reward_ref, dones_ref);

  return 0;
}