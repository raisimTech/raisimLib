//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_COMPOUND_HPP
#define RAISIM_COMPOUND_HPP

#include "SingleBodyObject.hpp"
#include "raisim/math.hpp"
#include "ode/ode.h"

namespace raisim {

class Compound : public SingleBodyObject {
  friend class raisim::World;
 public:

  struct CompoundObjectChild {
    std::string childCompound; // not used currently
    ObjectType objectType;
    Vec<4> objectParam;
    std::string material;
    std::string appearance;
    Transformation trans;
  };

  Compound(const std::vector<CompoundObjectChild>& list, double mass, const Vec<3>& COM, const Mat<3,3>& inertia);

  /**
   * returns the children of the compound
   * @return the children of the compound */
  const std::vector<CompoundObjectChild>& getObjList () { return list_; };

  /**
   * returns the collision list of the compound
   * @return the collision list of the compound */
  const std::vector<dGeomID>& getCollisionObjectList () { return co; };

  void setBodyType(BodyType type) final;

  void destroyCollisionBodies(dSpaceID id) final;

 protected:

  std::vector<dGeomID> co;
  void updateCollision();
  std::vector<CompoundObjectChild> list_;
};


}
#endif //RAISIM_COMPOUND_HPP
