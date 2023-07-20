//
// Created by jemin on 20. 9. 22..
//

#ifndef _RAISIM_GYM_TORCH_RAISIMGYMTORCH_ENV_REWARD_HPP_
#define _RAISIM_GYM_TORCH_RAISIMGYMTORCH_ENV_REWARD_HPP_

#include <initializer_list>
#include <string>
#include <map>
#include "Yaml.hpp"


namespace raisim {

struct RewardElement {
  float coefficient;
  float reward;
  float integral;
};

class Reward {
 public:
  Reward (std::initializer_list<std::string> names) {
    for(auto& nm: names)
      rewards_[nm] = raisim::RewardElement();
  }

  Reward () = default;

  void initializeFromConfigurationFile(const Yaml::Node& cfg) {
    for(auto rw = cfg.Begin(); rw != cfg.End(); rw++) {
      rewards_[(*rw).first] = raisim::RewardElement();
      RSFATAL_IF((*rw).second.IsNone() || (*rw).second["coeff"].IsNone(),
                 "Node " + (*rw).first + " or its coefficient doesn't exist");
      rewards_[(*rw).first].coefficient = (*rw).second["coeff"].template As<float>();
    }
  }

  const float& operator [] (const std::string& name) {
    return rewards_[name].reward;
  }

  void record (const std::string& name, float reward, bool accumulate = false) {
    RSFATAL_IF(rewards_.find(name) == rewards_.end(), name<<" was not found in the configuration file")
    RSISNAN_MSG(reward, name<<" is nan")

    if(!accumulate)
      rewards_[name].reward = 0.f;
    rewards_[name].reward += reward * rewards_[name].coefficient;
    rewards_[name].integral += rewards_[name].reward;
  }

  float sum() {
    float sum = 0.f;
    for(auto& rw: rewards_)
      sum += rw.second.reward;

    return sum;
  }

  void setZero() {
    for(auto& rw: rewards_)
      rw.second.reward = 0.f;
  }

  void reset() {
    for(auto& rw: rewards_) {
      rw.second.integral = 0.f;
      rw.second.reward = 0.f;
    }
  }

  const std::map<std::string, float>& getStdMapOfRewardIntegral() {
    for(auto& rw: rewards_)
      costSum_[rw.first] = rw.second.integral;

    return costSum_;
  }

  const std::map<std::string, float>& getStdMap() {
    for(auto& rw: rewards_)
      rewardMap_[rw.first] = rw.second.reward;

    return rewardMap_;
  }

 private:
  std::map<std::string, raisim::RewardElement> rewards_;
  std::map<std::string, float> costSum_;
  std::map<std::string, float> rewardMap_;
};

}  // namespace raisim

#endif //_RAISIM_GYM_TORCH_RAISIMGYMTORCH_ENV_REWARD_HPP_
