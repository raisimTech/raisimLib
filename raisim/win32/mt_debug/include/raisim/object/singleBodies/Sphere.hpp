//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_RAICOLLISIONSPHERE_HPP
#define RAISIM_RAICOLLISIONSPHERE_HPP

#include "SingleBodyObject.hpp"

namespace raisim {

class Sphere : public SingleBodyObject {

  /// NOTE
  /// body frame origin of Sphere is C.O.M of Sphere
 public:
  explicit Sphere(double radius, double mass);

  /**
   * returns the radius of the sphere
   * @return the radius of the sphere */
  double getRadius() const;

 protected:
  double radius_;

};

} // raisim

#endif //RAISIM_RAICOLLISIONSPHERE_HPP
