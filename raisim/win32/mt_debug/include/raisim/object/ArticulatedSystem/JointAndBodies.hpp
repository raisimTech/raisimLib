//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_JOINTANDBODIES_HPP
#define RAISIM_JOINTANDBODIES_HPP

#include "ode/ode.h"
#include "raisim/math.hpp"
#include "raisim/object/singleBodies/Mesh.hpp"
#include <unordered_map>

namespace raisim {

class ArticulatedSystem;
class World;

namespace urdf {
class LoadFromURDF2;
}

/* shapes that raisim supports */
namespace Shape {
enum Type :
    int {
  Box = 0,
  Cylinder,
  Sphere,
  Mesh,
  Capsule,
  Cone, // cone is not currently supported
  Ground,
  NotShape
};

inline Shape::Type stringToShape(const std::string &shape) {
  if (shape == "mesh") {
    return Shape::Type::Mesh;
  } else if (shape == "sphere") {
    return Shape::Type::Sphere;
  } else if (shape == "capsule") {
    return Shape::Type::Capsule;
  } else if (shape == "cylinder") {
    return Shape::Type::Cylinder;
  } else if (shape == "box") {
    return Shape::Type::Box;
  } else if (shape == "plane") {
    return Shape::Type::Ground;
  }
  return Shape::Type::NotShape;
}

}

/* collision definition that is stored in Articulated System.
 CollisionBody stores how the collision body should be displayed as well. This only stores physics part */
struct CollisionDefinition {
  CollisionDefinition(const raisim::Mat<3, 3> &rotOffsetI, const raisim::Vec<3> &posOffsetI,
                      size_t localIdxI, dGeomID colObjI, std::string nameI) :
      rotOffset(rotOffsetI),
      posOffset(posOffsetI),
      localIdx(localIdxI),
      colObj(colObjI){
    colObj->name = nameI;
  }

 public:
  friend class raisim::ArticulatedSystem;
  friend class raisim::World;

 protected:
  CollisionDefinition() = default;

 public:
  void setMaterial(const std::string &material);
  const std::string &getMaterial();
  const dGeomID getCollisionObject() const { return colObj; }
  dGeomID getCollisionObject() { return colObj; }

  // check https://raisim.com/sections/Contact.html to know more about collision group and mask
  void setCollisionGroup(CollisionGroup group);
  void setCollisionMask(CollisionGroup mask);
  CollisionGroup getCollisionGroup() const;
  CollisionGroup getCollisionMask() const;

  raisim::Mat<3, 3> rotOffset;
  raisim::Vec<3> posOffset;
  size_t localIdx;
  dGeomID colObj;
};

typedef std::vector<CollisionDefinition> CollisionSet;

/// visualization definition created by a user or URDF
struct VisObject {

  /* to add primitive
   * vis_shape: choices, Box, Cylinder, Sphere, Capsule
   * vis_shapeParam: params associated with shape,
   *      for sphere: {radius},
   *      for Box: {lx, ly, lz},
   *      for Cylinder: {radius, height},
   *      for Capsule: {radius, height},
   * vis_origin: position of the visualized object
   * vis_rotMat: orientation of the visualized object
   * vis_color: color, should be ignored if material is specified,
   * vis_name: name of the visualized body,
   * vis_material: material */
  VisObject(Shape::Type vis_shape,
            const std::vector<double> &vis_shapeParam,
            const raisim::Vec<3> &vis_origin,
            const raisim::Mat<3, 3> &vis_rotMat,
            const raisim::Vec<4> &vis_color,
            const std::string &vis_name,
            const std::string &vis_material) :
      shape(vis_shape), visShapeParam(vis_shapeParam), offset(vis_origin), rot(vis_rotMat),
      color(vis_color), scale({1., 1., 1.}), material(vis_material), name(vis_name) {
    RSFATAL_IF(vis_shape == Shape::Type::Mesh, "This constructor is for primitive shapes")
  }

  /* to add mesh
   * vis_meshFile: file location relative to the resource directory of the articulated system,
   * vis_origin: position of the visualized object
   * vis_rotMat: orientation of the visualized object
   * vis_color: color, should be ignored if material is specified,
   * vis_scale: for mesh only, scaling size,
   * vis_name: name of the visualized body,
   * vis_material: material */
  VisObject(const std::string &vis_meshFile,
            const std::string &vis_name,
            const raisim::Vec<3> &vis_origin,
            const raisim::Mat<3, 3> &vis_rotMat,
            const raisim::Vec<4> &vis_color,
            const raisim::Vec<3> &vis_scale,
            const std::string &vis_material) :
      shape(Shape::Type::Mesh), offset(vis_origin), rot(vis_rotMat), color(vis_color),
      scale(vis_scale), fileName(vis_meshFile), material(vis_material), name(vis_name) {}

  /* vis_shape: choices, Box, Cylinder, Sphere, mesh, Capsule
   * vis_shapeParam: params associated with shape,
   *      for sphere: {radius},
   *      for Box: {lx, ly, lz},
   *      for Cylinder: {radius, height},
   *      for mesh: None
   *      for Capsule: {radius, height},
   * vis_origin: position of the visualized object
   * vis_rotMat: orientation of the visualized object
   * vis_color: color, should be ignored if material is specified,
   * vis_scale: for mesh only, scaling size,
   * vis_meshFile: file location relative to the resource directory of the articulated system,
   * vis_name: name of the visualized body,
   * vis_material: material */
  VisObject(Shape::Type vis_shape,
            const std::vector<double> &vis_shapeParam,
            const raisim::Vec<3> &vis_origin,
            const raisim::Mat<3, 3> &vis_rotMat,
            const raisim::Vec<4> &vis_color,
            const raisim::Vec<3> &vis_scale,
            const std::string &vis_meshFile,
            const std::string &vis_name,
            const std::string &vis_material) :
      shape(vis_shape), visShapeParam(vis_shapeParam), offset(vis_origin), rot(vis_rotMat),
      color(vis_color), scale(vis_scale), fileName(vis_meshFile), name(vis_name), material(vis_material) {
    RSFATAL_IF(shape == Shape::Type::Mesh && fileName.empty(), "Provide a valid mesh file name")
  }

  Shape::Type shape;
  std::vector<double> visShapeParam;
  raisim::Vec<3> offset;
  raisim::Mat<3, 3> rot;
  raisim::Vec<4> color;
  raisim::Vec<3> scale;
  std::string fileName;
  std::string name;
  size_t localIdx;
  std::string material;
};

class Joint {
 public:

  enum Type : int {
    FIXED = 0,
    REVOLUTE,
    PRISMATIC,
    SPHERICAL,
    FLOATING
  };

  Joint() {
    rot.setIdentity();
    limit.setZero();
    springMount.setZero();
    pos_P.setZero();
    axis = {0,0,1};
  }

  /* if upper and lower bounds of the limit are the same, the limit is ignored */
  Joint(const Vec<3> &joint_axis,
        const Vec<3> &joint_pos_P,
        const Mat<3, 3> &joint_rot,
        const Vec<2> &joint_limit,
        Type joint_type,
        const std::string &joint_name) :
      axis(joint_axis), pos_P(joint_pos_P), rot(joint_rot), limit(joint_limit), type(joint_type), name(joint_name) { }

  void jointAxis(std::initializer_list<double> a) {
    axis[0] = *(a.begin());
    axis[1] = *(a.begin() + 1);
    axis[2] = *(a.begin() + 2);
  }

  void jointPosition(std::initializer_list<double> p) {
    pos_P[0] = *(p.begin());
    pos_P[1] = *(p.begin() + 1);
    pos_P[2] = *(p.begin() + 2);
  }

  void jointPosition(const Vec<3> &p) {
    pos_P = p;
  }

  size_t getGcDim() {
    switch (type) {
      case FIXED:
        return 0;
      case REVOLUTE:
      case PRISMATIC:
        return 1;
      case SPHERICAL:
        return 4;
      case FLOATING:
        return 7;
    }
    return 0;
  }

  static Joint getFloatingBaseJoint() {
    Joint joint;
    joint.type = Joint::Type::FLOATING;
    return joint;
  }

  size_t getGvDim() {
    switch (type) {
      case FIXED:
        return 0;
      case REVOLUTE:
      case PRISMATIC:
        return 1;
      case SPHERICAL:
        return 3;
      case FLOATING:
        return 6;
    }
    return 0;
  }

  Vec<3> axis;
  Vec<3> pos_P;
  Mat<3, 3> rot;
  Vec<2> limit;
  Type type;
  double effort = 1e150;
  double damping = 0.0, friction = 0.0;
  double stiffness = 0.;
  double rotor_inertia = 0.;
  double velocity_limit = 0.;
  Vec<4> springMount;
  double jointRef = 0.;
  std::string name;
};

class CoordinateFrame {
 public:
  Mat<3, 3> orientation;
  Vec<3> position;
  size_t parentId, currentBodyId;
  std::string name; // name of the joint
  std::string parentName; // name of the parent body
  std::string bodyName; // name of the urdf link attached to the joint
  bool isChild =
      false; // child is the first body after movable joint. All fixed bodies attached to a child is not a child
  Joint::Type jointType; // type of the associated joint
};

struct CollisionBody {
  /* to add a primitive
   * col_shape: choices, Box, Cylinder, Sphere, mesh, Capsule
   * col_shapeParam: params associated with shape,
   *      for sphere: {radius},
   *      for Box: {lx, ly, lz},
   *      for Cylinder: {radius, height},
   *      for mesh: None
   *      for Capsule: {radius, height},
   * col_origin: position of the collision object
   * col_rotMat: orientation of the collision object
   * col_name: name of the visualized body,
   * col_material: collision material that defines contact physics
   * col_visualizedMaterial: how the collision body should be visualized */
  CollisionBody(Shape::Type col_shape,
                const std::vector<double> &col_shapeParam,
                const raisim::Vec<3> &col_origin,
                const raisim::Mat<3, 3> &col_rotMat,
                const std::string &col_name,
                const std::string &col_material,
                const std::string &col_visualizedMaterial) :
      shape(col_shape), shapeParam(col_shapeParam), offset(col_origin),
      rot(col_rotMat), name(col_name), materialName(col_material),
      collisionVisualizationMaterial(col_visualizedMaterial) {
    RSFATAL_IF(col_shape == Shape::Type::Mesh, "This constructor is for primitive shapes")
  }

  /* to add mesh
   * col_meshFile: file location relative to the resource directory of the articulated system,
   * col_origin: position of the collision object
   * col_rotMat: orientation of the collision object
   * col_scale: for mesh only, scaling size,
   * col_name: name of the visualized body,
   * col_material: collision material that defines contact physics
   * col_visualizedMaterial: how the collision body should be visualized */
  CollisionBody(const std::string &col_meshFileName,
                const raisim::Vec<3> &col_origin,
                const raisim::Mat<3, 3> &col_rotMat,
                const raisim::Vec<3> &col_meshScale,
                const std::string &col_name,
                const std::string &col_material,
                const std::string &col_visualizedMaterial) :
      shape(Shape::Type::Mesh), scale(col_meshScale), offset(col_origin),
      rot(col_rotMat), name(col_name), materialName(col_material),
      collisionVisualizationMaterial(col_visualizedMaterial), colMeshFileName(col_meshFileName) {}

  Shape::Type shape;
  raisim::Vec<3> scale;
  std::vector<double> shapeParam;
  raisim::Vec<3> offset;
  raisim::Mat<3, 3> rot;
  std::string name;
  std::string materialName; /// collision property
  std::string collisionVisualizationMaterial; /// collision property
  std::string colMeshFileName;
  CollisionGroup group = 1, mask = -1;

  /* col_shape: choices, Box, Cylinder, Sphere, mesh, Capsule
   * col_shapeParam: params associated with shape,
   *      for sphere: {radius},
   *      for Box: {lx, ly, lz},
   *      for Cylinder: {radius, height},
   *      for mesh: None
   *      for Capsule: {radius, height},
   * col_origin: position of the collision object
   * col_rotMat: orientation of the collision object
   * col_scale: for mesh only, scaling size,
   * col_name: name of the visualized body,
   * col_material: collision material that defines contact physics
   * col_visualizedMaterial: how the collision body should be visualized
   * col_meshFile: file location relative to the resource directory of the articulated system */
  CollisionBody(Shape::Type col_shape,
                const std::vector<double> &col_shapeParam,
                const raisim::Vec<3> &col_origin,
                const raisim::Mat<3, 3> &col_rotMat,
                const raisim::Vec<3> &col_meshScale,
                const std::string &col_name,
                const std::string &col_material,
                const std::string &col_visualizedMaterial,
                const std::string &col_meshFileName) :
      shape(col_shape), scale(col_meshScale), shapeParam(col_shapeParam), offset(col_origin),
      rot(col_rotMat), name(col_name), materialName(col_material),
      collisionVisualizationMaterial(col_visualizedMaterial), colMeshFileName(col_meshFileName) {}

};

inline void getInertialAssumingUniformDensity(Shape::Type shape,
                                       const std::vector<double> &shapeParam,
                                       const Mat<3,3>& rot,
                                       double density,
                                       double &mass,
                                       Mat<3, 3> &inertia) {
  Mat<3, 3> inertiaP; // about the principle axes
  inertiaP.setZero();
  double volume = 0.;

  if (shape == Shape::Type::Sphere) {
    volume = shapeParam[0] * shapeParam[0] * shapeParam[0] * M_PI * 4. / 3.;
  } else if (shape == Shape::Type::Box) {
    volume = shapeParam[0] * shapeParam[1] * shapeParam[2];
  } else if (shape == Shape::Type::Capsule) {
    volume = shapeParam[0] * shapeParam[0] * shapeParam[0] * M_PI * 4. / 3.
        + M_PI * shapeParam[0] * shapeParam[0] * shapeParam[1];
  } else if (shape == Shape::Type::Cylinder) {
    volume = M_PI * shapeParam[0] * shapeParam[0] * shapeParam[1];
  }

  mass = volume * density;

  if (shape == Shape::Type::Sphere) {
    double diagonal = 2. / 5. * mass * shapeParam[0] * shapeParam[0];
    inertiaP[0] = diagonal;
    inertiaP[4] = diagonal;
    inertiaP[8] = diagonal;
  } else if (shape == Shape::Type::Box) {
    inertiaP[0] = mass / 12.0 * (shapeParam[1] * shapeParam[1] + shapeParam[2] * shapeParam[2]);
    inertiaP[4] = mass / 12.0 * (shapeParam[0] * shapeParam[0] + shapeParam[2] * shapeParam[2]);
    inertiaP[8] = mass / 12.0 * (shapeParam[0] * shapeParam[0] + shapeParam[1] * shapeParam[1]);
  } else if (shape == Shape::Type::Capsule) {
    inertiaP[0] = mass / 12.0 * (3. * shapeParam[0] * shapeParam[0] + shapeParam[1] * shapeParam[1]);
    inertiaP[4] = mass / 12.0 * (3. * shapeParam[0] * shapeParam[0] + shapeParam[1] * shapeParam[1]);
    inertiaP[8] = mass / 2.0 * (shapeParam[0] * shapeParam[0]);
  } else if (shape == Shape::Type::Cylinder) {
    double sIner = 2. / 5. * mass * shapeParam[0] * shapeParam[0];
    double sMass = density * shapeParam[0] * shapeParam[0] * shapeParam[0] * M_PI * 4. / 3.;
    inertiaP[0] = mass / 12.0 * (3. * shapeParam[0] * shapeParam[0] + shapeParam[1] * shapeParam[1]) + sIner
        + sMass * shapeParam[1] * shapeParam[1] / 4.;
    inertiaP[4] = mass / 12.0 * (3. * shapeParam[0] * shapeParam[0] + shapeParam[1] * shapeParam[1]) + sIner
        + sMass * shapeParam[1] * shapeParam[1] / 4.;
    inertiaP[8] = mass / 2.0 * (shapeParam[0] * shapeParam[0]) + sIner;
  }

  inertia = rot * inertiaP * rot.transpose();
}

class Body {
  friend class ArticulatedSystem;

 public:

  Body() {
    mass_ = 0;
    inertia_.setZero();
    com_.setZero();
  }

  Body(double mass, const Mat<3, 3> &inertia, const Vec<3> &comPos) :
      mass_(mass), inertia_(inertia), com_(comPos) {}

  void setMass(double mass) { mass_ = mass; }

  double &getMass() { return mass_; }

  void setInertia(std::initializer_list<double> inertia) {
    inertia_[0] = *(inertia.begin());
    inertia_[1] = *(inertia.begin() + 1);
    inertia_[2] = *(inertia.begin() + 2);

    inertia_[3] = *(inertia.begin() + 1);
    inertia_[4] = *(inertia.begin() + 3);
    inertia_[5] = *(inertia.begin() + 4);

    inertia_[6] = *(inertia.begin() + 2);
    inertia_[7] = *(inertia.begin() + 4);
    inertia_[8] = *(inertia.begin() + 5);
  }

  void setInertia(const Vec<3> &inertia) {
    inertia_[0] = inertia[0];
    inertia_[1] = 0;
    inertia_[2] = 0;

    inertia_[3] = 0;
    inertia_[4] = inertia[1];
    inertia_[5] = 0;

    inertia_[6] = 0;
    inertia_[7] = 0;
    inertia_[8] = inertia[2];
  }

  void setInertia(const Vec<6> &inertia) {
    inertia_[0] = inertia[0];
    inertia_[1] = inertia[1];
    inertia_[2] = inertia[2];

    inertia_[3] = inertia[1];
    inertia_[4] = inertia[3];
    inertia_[5] = inertia[4];

    inertia_[6] = inertia[2];
    inertia_[7] = inertia[4];
    inertia_[8] = inertia[5];
  }

  void setInertiaMjcfOrder(const Vec<6> &inertia) {
    inertia_[0] = inertia[0];
    inertia_[4] = inertia[1];
    inertia_[8] = inertia[2];

    inertia_[1] = inertia[3];
    inertia_[2] = inertia[4];
    inertia_[5] = inertia[5];

    inertia_[3] = inertia[3];
    inertia_[6] = inertia[4];
    inertia_[7] = inertia[5];
  }

  void setZeroInertial() {
    com_.e().setZero();
    inertia_.e().setZero();
    mass_ = 0;
  }

  Mat<3, 3> &getInertia() { return inertia_; }

  void setCom(const Vec<3> &com) {
//    LOG_IF(FATAL, com.size() != 3) << "Provide 3 elements for inertia matrix";
    com_ = com;
  }

  Vec<3> &getCom() { return com_; }

  void clearColAndVis() {
    colObj.clear();
    visObj.clear();
  }

  void
  addCollisionObject(Shape::Type shape,
                     const std::vector<double> &param,
                     const raisim::Vec<3> &origin,
                     const raisim::Mat<3, 3> &rot,
                     const raisim::Vec<3> &scale,
                     const std::string &colName,
                     const std::string &materialName, /// collision property
                     const std::string &collisionVisualizedMaterial, /// collision property
                     const std::string &meshFileName,
                     CollisionGroup group = CollisionGroup(-1),
                     CollisionGroup mask = CollisionGroup(-1)) {
    colObj.emplace_back(shape,
                        param,
                        origin,
                        rot,
                        scale,
                        colName,
                        materialName,
                        collisionVisualizedMaterial,
                        meshFileName);
  }

  void addVisualObject(Shape::Type shape,
                       const std::vector<double> &shapeParam,
                       const raisim::Vec<3> &origin,
                       const raisim::Mat<3, 3> &rotMat,
                       const raisim::Vec<4> &color,
                       const raisim::Vec<3> &scale,
                       const std::string &meshFileNames,
                       const std::string &visName,
                       const std::string &material) {
    visObj.emplace_back(shape,
                        shapeParam,
                        origin,
                        rotMat,
                        color,
                        scale,
                        meshFileNames,
                        visName,
                        material);
  }

  std::vector<CollisionBody> colObj;
  std::vector<VisObject> visObj;
  double mass_;
  Mat<3, 3> inertia_;
  Vec<3> com_;
};

class Child {
 public:
  friend class raisim::ArticulatedSystem;
  friend class raisim::urdf::LoadFromURDF2;

  Child(const Body &body, const Joint &joint, const std::string &name) {
    Child::body = body;
    Child::joint = joint;
    Child::name = name;
  }

  Child() {}

  Body body;
  Joint joint;
  std::string name;

  size_t numberOfBodiesFromHere() {
    size_t nbody = 1;
    for (auto &ch : child)
      nbody += ch.numberOfBodiesFromHere();
    return nbody;
  }

  size_t numberOfDOFFromHere() {
    size_t dof = joint.getGvDim();
    for (auto &ch : child)
      dof += ch.numberOfDOFFromHere();
    return dof;
  };

  size_t numberOfGCFromHere() {
    size_t gcDim = joint.getGcDim();
    for (auto &ch : child)
      gcDim += ch.numberOfGCFromHere();
    return gcDim;
  };

  size_t jointIdx(std::string &nm, std::vector<std::string> &jointsNames) {
    for (size_t i = 0; i < jointsNames.size(); i++)
      if (nm == jointsNames[i]) return i;
    return -1;
  }

  void initCollisionBodies(CollisionSet &collect,
                           std::vector<VisObject> &visCollect,
                           std::vector<dTriMeshDataID> &mesh,
                           std::vector<std::vector<float>> &meshVert,
                           std::vector<std::vector<dTriIndex>> &meshIdx,
                           const std::string &resDir);
  void initVisuals(std::vector<VisObject> &collect);

  void consumeFixedBodies(std::vector<CoordinateFrame> &frameOfInterest);

  void processJointRef();

  void addChild(const Child &childNode) {
    if (childNode.joint.type == Joint::Type::FIXED) {
      fixedBodies.push_back(childNode);
    } else if (childNode.joint.type != Joint::Type::FLOATING) {
      child.push_back(childNode);
    } else {
      RSFATAL("Only the root can have a floating joint")
    }
  }

  std::vector<Child> child;
  std::vector<Child> fixedBodies;

 protected:
  size_t bodyIdx;
  size_t parentIdx;
  std::string parentBodyName;
  bool registered = false;
};

}

#endif //RAISIM_JOINTANDBODIES_HPP
