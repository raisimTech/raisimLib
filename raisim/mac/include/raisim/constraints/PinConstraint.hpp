//
// Created by jemin on 7/3/21.
//

#ifndef RAISIM_CONSTRAINTS_PinConstraint_HPP_
#define RAISIM_CONSTRAINTS_PinConstraint_HPP_

#include <raisim/object/Object.hpp>

namespace raisim {

struct PinConstraintDefinition {
  std::string body1, body2;
  Vec<3> anchor;
};

class PinConstraint {
 public:
  PinConstraint(size_t localIdx1, Vec<3> pos1_b, size_t localIdx2, Vec<3> pos2_b);
  virtual ~PinConstraint() = default;
  Vec<3> pos1_b, pos2_b;
  Vec<3> pos1_w, pos2_w;
  size_t localIdx1, localIdx2;

 protected:
  void updateContactProblem(contact::ContactProblems& contact_problems);
};

}

#endif // RAISIM_CONSTRAINTS_ZEROLENGTHCONSTRAINT_HPP_
