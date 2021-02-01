//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_COLLISIONCYLINDER_HPP
#define RAISIM_COLLISIONCYLINDER_HPP

#include "SingleBodyObject.hpp"

namespace raisim {

class Cylinder : public SingleBodyObject {

  /// NOTE
  /// body frame origin of Cylinder is C.O.M of Cylinder
 public:
  Cylinder(double radius, double height, double mass);

  /**
   * returns the radius of the cylinder
   * @return the radius of the cylinder */
  double getRadius() const;

  /**
   * returns the height of the cylinder
   * @return the height of the cylinder */
  double getHeight() const;

 protected:

  double radius_;
  double height_;
};

} // raisim

#endif //RAISIM_COLLISIONCYLINDER_HPP
