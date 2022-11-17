//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_COLLISIONCONE_HPP
#define RAISIM_COLLISIONCONE_HPP

#include "SingleBodyObject.hpp"

namespace raisim {

class Cone : public SingleBodyObject {

  /// NOTE
  /// body origin of Cone is center of bottom
  /// C.O.M of Cone is 1/4 height point from bottom
  /// bullet origin of Cone is 1/2 height point from bottom
 public:
  Cone(double radius, double height, double mass);
  void setOrientation(const Eigen::Quaterniond& quaternion) override ;
  void setOrientation(double w, double x, double y, double z) override ;

  double getRadius() const;
  double getHeight() const;

 protected:

  double radius_;
  double height_;

};

} // raisim

#endif //RAISIM_COLLISIONCONE_HPP
