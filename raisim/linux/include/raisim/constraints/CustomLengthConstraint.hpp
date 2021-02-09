//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_CONSTRAINTS_CUSTOMLENGTHCONSTRAINT_HPP_
#define RAISIM_INCLUDE_RAISIM_CONSTRAINTS_CUSTOMLENGTHCONSTRAINT_HPP_

#include "LengthConstraint.hpp"

namespace raisim {

class CustomLengthConstraint : public LengthConstraint {
 public:
  CustomLengthConstraint(Object* obj1, size_t localIdx1, Vec<3> pos1_b, Object* obj2, size_t localIdx2, Vec<3> pos2_b, double length);
  ~CustomLengthConstraint() = default;

  /**
   * set the tension of the wire
   * @param tension the applied tension. If this number is positive, then the force is acting such a way that brings the two objects together*/
  void setTension(double tension);

 protected:
  void applyTension(contact::ContactProblems& contact_problems) final;

 private:
  Vec<3> tension_, tension2_;
  double tensionScalar_ = 0.;
};

}

#endif //RAISIM_INCLUDE_RAISIM_CONSTRAINTS_CUSTOMLENGTHCONSTRAINT_HPP_
