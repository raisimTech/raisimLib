//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_CAPSULE_HPP
#define RAISIM_CAPSULE_HPP

#include "SingleBodyObject.hpp"

namespace raisim {

class Capsule: public SingleBodyObject {

  /// NOTE
  /// body origin of Capsule is C.O.M of Capsule
 public:
  Capsule(double radius, double height, double mass);

  double getRadius() const;
  double getHeight() const;

 protected:
  double radius_;
  double height_;

};

} // raisim

#endif //RAISIM_CAPSULE_HPP
