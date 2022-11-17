//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_COMPLIANTWIRE_HPP
#define RAISIM_COMPLIANTWIRE_HPP

#include "LengthConstraint.hpp"

namespace raisim {

class CompliantLengthConstraint : public LengthConstraint {
 public:
  CompliantLengthConstraint(Object* obj1, int localIdx1, Vec<3> pos1_b, Object* obj2, int localIdx2, Vec<3> pos2_b, double length, double stiffness);
  virtual ~CompliantLengthConstraint() override = default;
  double getStiffness() const { return stiffness_; }
  double getPotentialEnergy() const { return 0.5*stiffness_*stretch_*stretch_; }
  const Vec<3>& getTension() const { return tension_; }
 protected:
  void applyTension(contact::ContactProblems& contact_problems) final;
 private:
  double stiffness_;
  Vec<3> tension_, tension2_;
};

}

#endif //RAISIM_COMPLIANTWIRE_HPP
