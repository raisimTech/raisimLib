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

  /**
   * returns the radius of the capsule
   * @return the radius of the capsule */
  double getRadius() const;

  /**
   * returns the height of the capsule
   * @return the height of the capsule */
  double getHeight() const;

 protected:
  double radius_;
  double height_;

};

} // raisim

#endif //RAISIM_CAPSULE_HPP
