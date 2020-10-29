//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAI_RAICONTACT2_HPP
#define RAI_RAICONTACT2_HPP

#include "raisim/helper.hpp"
#include <vector>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <raisim/Materials.hpp>
#include "raisim/math.hpp"
#include "raisim/object/Object.hpp"

namespace raisim {

class Object;

namespace contact {

class Single3DContactProblem {
 public:
  Single3DContactProblem(const MaterialPairProperties &mat, double x, double y, double z, double depth_in);
  Single3DContactProblem() = default;
  void checkRank();

  raisim::Vec<3> imp_i, tau_i, position_W;
  raisim::Mat<3, 3> MappInv_i;
  raisim::Mat<3, 3> MappInvWODel_i;
  raisim::Mat<3, 3> Mapp_i;
  raisim::Mat<2, 2> Mapp_iInv22, Mapp_i22;
  Mat<3,2> basisMat;
  raisim::Mat<3, 2> MappInv_red;
  std::vector<raisim::Mat<3, 3>> MappInv_j;
  std::vector<Vec<3> *> imp_j;
  double mu, n2_mu, muinv, negMuSquared, coeffRes, bounceThres, bouceVel=0., Mapp_iInv11, impact_vel=0., depth=0., tempE=0.;
  Object *obA = nullptr, *obB = nullptr;
  int rank = 3;
  union {
    size_t pointIdA;
    size_t jointId;
  };
  size_t pointIdB;
  bool atLeastOneWithoutDel = false;
};

typedef std::vector<contact::Single3DContactProblem, AlignedAllocator<contact::Single3DContactProblem, 32>>
    ContactProblems;

class BisectionContactSolver {

 public:

  struct SolverConfiguration {
    double alpha_init = 1.0;
    double alpha_low = 1.0;
    double alpha_decay = 1.0;
    double error_to_terminate = 1e-8;
    double erp = 0.3;
    double erp2 = 0.002;
    int maxIteration = 150;
  };

  explicit BisectionContactSolver() {
  }

  void solve(ContactProblems &contact);

  void updateConfig(SolverConfiguration config) { config_=config; }

  void setTimestep(double dt) {
    dt_ = dt;
    dtinv_ = 1/dt;
  }

  void setOrder(bool order) { order_ = order; }

  int getLoopCounter() const { return loopCounter_; }

  const std::vector<double>& getErrorHistory() const { return error_; }

  SolverConfiguration& getConfig() { return config_; }
  const SolverConfiguration& getConfig() const { return config_; }

 private:
  int loopCounter_ = 0;
  bool order_ = true;
  double dt_ = 0.01;
  double dtinv_ = 100.0;
  SolverConfiguration config_;
  std::vector<double> error_;
  static constexpr double impDebugThreshold=1e4;
};

} // contact
} // raisim


#endif //RAICONTACT2
