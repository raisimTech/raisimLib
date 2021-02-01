//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_COLLISIONCHECKERBOARD_HPP
#define RAISIM_COLLISIONCHECKERBOARD_HPP

#include <memory>

#include "raisim/object/singleBodies/SingleBodyObject.hpp"

namespace raisim {

class Ground : public SingleBodyObject {

 public:
  Ground(double height);

  /**
   * returns the height of the plane
   * @return the height */
  double getHeight() { return height_; }

 protected:
  double height_;

};

} // raisim

#endif //RAISIM_COLLISIONCHECKERBOARD_HPP
