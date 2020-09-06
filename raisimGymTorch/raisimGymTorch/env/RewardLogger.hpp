//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef _RAISIM_GYM_REWARDLOGGER_HPP
#define _RAISIM_GYM_REWARDLOGGER_HPP

#include <unordered_map>

namespace raisim {

class RewardLogger {

  class RewardTerm {
   public:
    RewardTerm() = default;

    void clean() {
      sum = 0.;
      count = 0;
      values.clear();
    }

    void log(double value) {
      values.push_back(value);
      sum += value;
      count++;
    }

    double sum = 0.;
    int count = 0;
    std::vector<double> values;
  };

 public:
  RewardLogger() = default;

  void init(std::initializer_list<std::string> rewardTermNames) {
    for (auto &item : rewardTermNames)
      rewardTerms_[item] = RewardTerm();
  }

  void log(const std::string &termName, double value) {
    rewardTerms_[termName].log(value);
  }

  const std::unordered_map<std::string, RewardTerm>& getRewardTerms() const {
    return rewardTerms_;
  }

  void clean() {
    for (auto& item : rewardTerms_)
      item.second.clean();
  }

 private:
  std::unordered_map<std::string, RewardTerm> rewardTerms_;
};

};
#endif //_RAISIM_GYM_REWARDLOGGER_HPP
