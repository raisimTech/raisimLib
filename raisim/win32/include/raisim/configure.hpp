//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_CONFIGURE_HPP
#define RAISIM_CONFIGURE_HPP

namespace raisim {

typedef uint64_t CollisionGroup;

#ifdef WIN32
#define RAISIM_STATIC_COLLISION_GROUP CollisionGroup(1)<<31
#else
#define RAISIM_STATIC_COLLISION_GROUP CollisionGroup(1)<<63
#endif

enum ObjectType : int { SPHERE = 0, BOX, CYLINDER, CONE, CAPSULE, MESH, HALFSPACE, COMPOUND, HEIGHTMAP, ARTICULATED_SYSTEM, UNRECOGNIZED };

inline bool isSingleBody(ObjectType type) {
  return type==SPHERE || type==BOX || type==CYLINDER || type==CAPSULE || type==MESH || type==COMPOUND || type==HALFSPACE || type==HEIGHTMAP;
}

inline bool isMovableBody(ObjectType type) {
  return type==SPHERE || type==BOX || type==CYLINDER || type==CAPSULE || type==MESH || type==COMPOUND || type==ARTICULATED_SYSTEM;
}

inline bool isMovableSingleBody(ObjectType type) {
  return type==SPHERE || type==BOX || type==CYLINDER || type==CAPSULE || type==MESH || type==COMPOUND;
}

inline bool isSingleCollisionBody(ObjectType type) {
  return type==SPHERE || type==BOX || type==CYLINDER || type==CAPSULE || type==MESH;
}

inline bool isPrimitiveCollisionBody(ObjectType type) {
  return type==SPHERE || type==BOX || type==CYLINDER || type==CAPSULE;
}

inline bool isMustHaveInertiaDefined(ObjectType type) {
  return type==MESH || type==COMPOUND;
}

inline ObjectType stringToObjectType(const std::string& typeName) {
  if(typeName == "sphere")
    return SPHERE;
  else if (typeName == "box")
    return BOX;
  else if (typeName == "cylinder")
    return CYLINDER;
  else if (typeName == "cone")
    return CONE;
  else if (typeName == "capsule")
    return CAPSULE;
  else if (typeName == "mesh")
    return MESH;
  else if (typeName == "ground")
    return HALFSPACE;
  else if (typeName == "compound")
    return COMPOUND;
  else if (typeName == "heightmap")
    return HEIGHTMAP;
  else if (typeName == "articulatedSystem")
    return ARTICULATED_SYSTEM;
  else if (typeName == "articulated_system")
    return ARTICULATED_SYSTEM;

  return ObjectType::UNRECOGNIZED;
}

inline std::string objectTypeToString(ObjectType type) {
  switch(type) {

    case SPHERE :
      return "sphere";

    case BOX :
      return "box";

    case CYLINDER:
      return "cylinder";

    case CONE:
      return "cone";

    case CAPSULE:
      return "capsule";

    case MESH:
      return "mesh";

    case HALFSPACE:
      return "ground";

    case COMPOUND:
      return "compound";

    case HEIGHTMAP:
      return "heightmap";

    case ARTICULATED_SYSTEM:
      return "articulatedSystem";

    case UNRECOGNIZED:
      return "unrecognized";
  }

  return "";
}

enum class BodyType : int {
  STATIC = 1,
  KINEMATIC = 2,
  DYNAMIC = 3
};

}

#endif //RAISIM_CONFIGURE_HPP
