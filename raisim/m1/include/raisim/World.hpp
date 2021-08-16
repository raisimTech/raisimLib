//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_WORLD_HPP
#define RAISIM_WORLD_HPP

#include <memory>
#include <algorithm>

#include "raisim/helper.hpp"
#include "raisim/object/Object.hpp"
#include "raisim/object/singleBodies/SingleBodyObject.hpp"
#include "raisim/object/singleBodies/Sphere.hpp"
#include "raisim/object/singleBodies/Cylinder.hpp"
#include "raisim/object/singleBodies/Box.hpp"
#include "raisim/object/singleBodies/Capsule.hpp"
#include "raisim/object/singleBodies/Cone.hpp"
#include "raisim/object/singleBodies/Compound.hpp"
#include "raisim/constraints/StiffLengthConstraint.hpp"
#include "raisim/constraints/CustomLengthConstraint.hpp"
#include "raisim/constraints/CompliantLengthConstraint.hpp"
#include "raisim/object/terrain/Ground.hpp"
#include "raisim/object/terrain/HeightMap.hpp"
#include "raisim/Terrain.hpp"
#include "raisim/contact/BisectionContactSolver.hpp"
#include "raisim/object/ArticulatedSystem/ArticulatedSystem.hpp"
#include "raisim/rayCollision.hpp"
#include "raisim/Path.hpp"
#include "raisim/object/ArticulatedSystem/loaders.hpp"
#include "ode/collision.h"
#include "configure.hpp"

#ifdef RAISIM_STATIC_API
#undef RAISIM_STATIC_API
#endif

#ifdef WIN32
#ifdef RAISIM_STATIC_MEMBER_EXPORT
    #define RAISIM_STATIC_API __declspec(dllexport)
  #else
    #define RAISIM_STATIC_API __declspec(dllimport)
  #endif
#else
#define RAISIM_STATIC_API
#endif

namespace raisim {

/**
 * @param group Collision group ID
 * @return Collision group. Can also be used as a collision mask.
 */
static CollisionGroup COLLISION(unsigned int group) { return CollisionGroup(1) << group; }

class World {

  struct XmlObjectClass {
    ObjectType type;
    std::string name;
    std::string material;
    std::string filepath;
    std::string resDir;
    double mass = 0;
    CollisionGroup collisionGroup;
    CollisionGroup collisionMask;

    Vec<3> size = {0, 0, 0};
    Vec<3> com = {0, 0, 0,};
    bool useUniformityCOM = true;
    Mat<3,3> inertia;
    bool useUniformityInertia = true;
    std::vector<Compound::CompoundObjectChild> comChild;
  };

 public:

  /**
   * export the world to an xml config file, which can be loaded using a constructor
   * @param activationKey path to the license file */
  static void setActivationKey(const std::string& activationKey) { activationKey_ = activationKey; }

  /*!
    Create an empty world */
  explicit World();

  /*!
    Create an world as specified in the xml config file
  */
  explicit World(const std::string &configFile);

  /**
   * export the world to an xml config file, which can be loaded using a constructor
   * @param dir directory to save the xml file
   * @param fileName file name */
  void exportToXml(const std::string& dir, const std::string &fileName);

  World(const World &world) = delete;

  ~World();

  /** set the time step
   * @param dt the time step */
  void setTimeStep(double dt) {
    timeStep_ = dt;
    solver_.setTimestep(dt);
    for (auto &ob: objectList_) ob->updateTimeStep(dt);
  }

  /**
   * @return the time step */
  double getTimeStep() const { return timeStep_; }

  /**
   * @param radius radius
   * @param mass mass
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask".
   * @return pointer to the created box */
  Sphere *addSphere(double radius,
                    double mass,
                    const std::string &material = "default",
                    CollisionGroup collisionGroup = 1,
                    CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param xLength x dimension
   * @param yLength y dimension
   * @param zLength z dimension
   * @param mass mass
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask".
   * @return pointer to the created box */
  Box *addBox(double xLength,
              double yLength,
              double zLength,
              double mass,
              const std::string &material = "default",
              CollisionGroup collisionGroup = 1,
              CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param radius radius
   * @param height center-to-center distance
   * @param mass mass
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask".
   * @return pointer to the created cylinder */
  Cylinder *addCylinder(double radius,
                        double height,
                        double mass,
                        const std::string &material = "default",
                        CollisionGroup collisionGroup = 1,
                        CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param radius radius
   * @param height center-to-center distance
   * @param mass mass
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask".
   * @return pointer to the created capsule */
  Capsule *addCapsule(double radius,
                      double height,
                      double mass,
                      const std::string &material = "default",
                      CollisionGroup collisionGroup = 1,
                      CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param zHeight height of the terrain
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask". Note that collision group of a static object is CollisionGroup(1) << 61ul
   * @return pointer to the created ground */
  Ground *addGround(double zHeight = 0.0,
                    const std::string &material = "default",
                    CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param xSamples how many points along x axis
   * @param ySamples how many points along y axis
   * @param xSize x width of the height map
   * @param ySize y length of the height map
   * @param centerX x coordinate of the center of the height map
   * @param centerY y coordinate of the center of the height map
   * @param height a vector of doubles representing heights. the size should be xSample X ySamples
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @return pointer to the created height map */
  HeightMap *addHeightMap(size_t xSamples,
                          size_t ySamples,
                          double xSize,
                          double ySize,
                          double centerX,
                          double centerY,
                          const std::vector<double> &height,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = RAISIM_STATIC_COLLISION_GROUP,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param raisimHeightMapFileName the raisim text file which will be used to create the height map
   * @param centerX x coordinate of the center of the height map
   * @param centerY y coordinate of the center of the height map
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @return pointer to the created height map */
  HeightMap *addHeightMap(const std::string &raisimHeightMapFileName,
                          double centerX,
                          double centerY,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = RAISIM_STATIC_COLLISION_GROUP,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param pngFileName the png file which will be used to create the height map
   * @param centerX x coordinate of the center of the height map
   * @param centerY y coordinate of the center of the height map
   * @param xSize x width of the height map
   * @param ySize y length of the height map
   * @param heightScale a png file (if 8-bit) has pixel values from 0 to 255. This parameter scales the pixel values to the actual height
   * @param heightOffset height of the 0-value pixel
   * @param material material of the height map (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @return pointer to the created height map */
  HeightMap *addHeightMap(const std::string &pngFileName,
                          double centerX,
                          double centerY,
                          double xSize,
                          double ySize,
                          double heightScale,
                          double heightOffset,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = RAISIM_STATIC_COLLISION_GROUP,
                          CollisionGroup collisionMask = CollisionGroup(-1));

 /**
  * @param centerX x coordinate of the center of the height map
  * @param centerY y coordinate of the center of the height map
  * @param terrainProperties perlin noise parameters which will be used to create the height map
  * @param material material of the height map (which defines the contact dynamics)
  * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
  * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
  * @return pointer to the created height map */
  HeightMap *addHeightMap(double centerX,
                          double centerY,
                          TerrainProperties &terrainProperties,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = RAISIM_STATIC_COLLISION_GROUP,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param heightmapToBeCloned Another height map to be cloned
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @return pointer to the created height map */
  HeightMap *addHeightMap(const HeightMap* heightmapToBeCloned,
                          CollisionGroup collisionGroup = RAISIM_STATIC_COLLISION_GROUP,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * @param filePathOrURDFScript Path to urdf file or a URDF string. Depending on the contents of the string, RaiSim will interpret it as an xml string or a file path.
   * @param resPath Path to the resource directory. Leave it empty ("") if it is the urdf file directory
   * @param jointOrder this can be used to redefine the joint order. A child cannot precede its parent. Leave it empty ({}) to use the joint order defined in the URDF file.
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @param options Currently only support "doNotCollideWithParent"
   * @return pointer to the articulated system */
  ArticulatedSystem *addArticulatedSystem(const std::string &filePathOrURDFScript,
                                          const std::string &resPath = "",
                                          const std::vector<std::string> &jointOrder = {},
                                          CollisionGroup collisionGroup = 1,
                                          CollisionGroup collisionMask = CollisionGroup(-1),
                                          ArticulatedSystemOption options = ArticulatedSystemOption());

  /**
   * @param xmlFileTemplate xml template file.
   * @param params parameters for the xml file.
   * @param resPath Path to the resource directory. Leave it empty ("") if it is the urdf file directory
   * @param jointOrder this can be used to redefine the joint order. A child cannot precede its parent. Leave it empty ({}) to use the joint order defined in the URDF file.
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @param options Currently only support "doNotCollideWithParent"
   * @return pointer to the articulated system */
  ArticulatedSystem *addArticulatedSystem(const std::string &xmlFileTemplate,
                                          const std::unordered_map<std::string, std::string>& params,
                                          const std::string &resPath = "",
                                          const std::vector<std::string> &jointOrder = {},
                                          CollisionGroup collisionGroup = 1,
                                          CollisionGroup collisionMask = CollisionGroup(-1),
                                          ArticulatedSystemOption options = ArticulatedSystemOption());

  /**
   * This method programmatically creates an articulated system without an URDF file.
   * @param child an instance of Child class which has an articulated system structure.
   * @param resPath Path to the resource directory. Leave it empty ("") if it is the urdf file directory
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @param options Currently only support "doNotCollideWithParent"
   * @return pointer to the articulated system */
  ArticulatedSystem *addArticulatedSystem(const Child& child,
                                          const std::string &resPath = "",
                                          CollisionGroup collisionGroup = 1,
                                          CollisionGroup collisionMask = CollisionGroup(-1),
                                          ArticulatedSystemOption options = ArticulatedSystemOption());

  /**
   * Add a single body which is composed of multiple primitive collision shapes
   * @param children a vector of CompoundObjectChild which contains each primitive shape's position, orientation, material and shape parameters
   * @param mass mass of the composite body
   * @param COM center of the composite body
   * @param inertia inertia of the composite body
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @return pointer to the created compound object */
  Compound *addCompound(const std::vector<Compound::CompoundObjectChild> &children,
                        double mass,
                        Vec<3> COM,
                        const Mat<3, 3>& inertia,
                        CollisionGroup collisionGroup = 1,
                        CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * create mesh collision body. only the obj format is supported
   * @param meshFileInObjFormat obj file of the mesh
   * @param mass mass
   * @param inertia inertia
   * @param COM the center of the mass
   * @param scale rescale the mesh
   * @param material material of the mesh (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @return pointer to the created wire  */
  Mesh *addMesh(const std::string &meshFileInObjFormat,
                double mass,
                const Mat<3, 3> &inertia,
                const Vec<3> &COM,
                double scale = 1,
                const std::string &material = "",
                CollisionGroup collisionGroup = 1,
                CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * create mesh collision body. only the obj format is supported
   * @param meshToClone mesh to copy
   * @param material material of the mesh (which defines the contact dynamics)
   * @param collisionGroup read "Contact and Collision/ Collision Group and Mask"
   * @param collisionMask read "Contact and Collision/ Collision Group and Mask"
   * @return pointer to the created wire  */
  Mesh *addMesh(const Mesh* meshToClone,
                const std::string &material = "",
                CollisionGroup collisionGroup = 1,
                CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * Stiff unilateral constraint. It cannot push. It can only pull.
   * @param obj1 the first object the wire is attached to
   * @param localIdx1 the body index (0 for a SingleBodyObject) for the first object
   * @param pos1_b location of the cable attachment on the first object
   * @param obj2 the second object the wire is attached to
   * @param localIdx2 the body index (0 for a SingleBodyObject) for the second object
   * @param pos2_b location of the cable attachment on the second object
   * @param length length of the wire
   * @return pointer to the created wire  */
  StiffLengthConstraint *addStiffWire(Object *obj1,
                                      size_t localIdx1,
                                      Vec<3> pos1_b,
                                      Object *obj2,
                                      size_t localIdx2,
                                      Vec<3> pos2_b,
                                      double length);

  /**
   * soft unilateral constraint. It cannot push. It can only pull.
   * @param obj1 the first object the wire is attached to
   * @param localIdx1 the body index (0 for a SingleBodyObject) for the first object
   * @param pos1_b location of the cable attachment on the first object
   * @param obj2 the second object the wire is attached to
   * @param localIdx2 the body index (0 for a SingleBodyObject) for the second object
   * @param pos2_b location of the cable attachment on the second object
   * @param length length of the wire
   * @param stiffness stiffness of the wire
   * @return pointer to the created wire  */
  CompliantLengthConstraint *addCompliantWire(Object *obj1,
                                              int localIdx1,
                                              Vec<3> pos1_b,
                                              Object *obj2,
                                              int localIdx2,
                                              Vec<3> pos2_b,
                                              double length,
                                              double stiffness);

  /**
   * Custom wire that applies user-set tension between two points.
   * @param obj1 the first object the wire is attached to
   * @param localIdx1 the body index (0 for a SingleBodyObject) for the first object
   * @param pos1_b location of the cable attachment on the first object
   * @param obj2 the second object the wire is attached to
   * @param localIdx2 the body index (0 for a SingleBodyObject) for the second object
   * @param pos2_b location of the cable attachment on the second object
   * @param length length of the wire. You can use this to compute how much it stretched from a nominal length. It might not be necessary for some wire types.
   * @return pointer to the created wire  */
  CustomLengthConstraint *addCustomWire(Object *obj1,
                                        int localIdx1,
                                        Vec<3> pos1_b,
                                        Object *obj2,
                                        int localIdx2,
                                        Vec<3> pos2_b,
                                        double length);

  /**
   * @return object with the given name. returns nullptr if the object doesn't exist. The name can be set by Object::setName() */
  Object *getObject(const std::string &name);

  /**
   * @return object with the given index. This index can be retrieved by Object::getIndexInWorld() */
  Object* getObject(std::size_t worldIndex) { return objectList_[worldIndex]; }

  /**
   * @return returns a non-const vector of the objects */
  std::vector<Object*> &getObjList();

  /**
   * @return a constraint (e.g., wires) with the given name. returns nullptr if the object doesn't exist. The name can be set by Wire::setName(). This is equivalent to getWire(const std::string&) */
  Constraints *getConstraint(const std::string &name);

  /**
   * @return a wire with the given name. returns nullptr if the object doesn't exist. The name can be set by Wire::setName() */
  LengthConstraint *getWire(const std::string &name);

  /**
   * @returns the configuration number. this number is updated every time an object is added or removed */
  unsigned long getConfigurationNumber() { return objectConfiguration_; }

  /**
   * Returns the internal reference of the ray collision list
   * it contains the geoms (position, normal, object world/local id) and the number of intersections
   * This returns
   * @param[in] start The start position of the ray.
   * @param[in] direction The direction of the ray.
   * @param[in] length The length of the ray.
   * @param[in] closestOnly Only stores the first collision.
   * @param[in] collisionMask Collision mask to filter collisions. By default, it records collisions with all collision groups.
   * @return A reference to the internal container which contains all ray collisions. */
  const RayCollisionList& rayTest(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& direction,
                                  double length,
                                  bool closestOnly = true,
                                  CollisionGroup collisionMask = CollisionGroup(-1));

  /**
   * removes an object
   * @param obj object to be removed */
  void removeObject(Object *obj);

  /**
   * removes a wire (i.e., LengthConstraint)
   * @param wire the wire to be removed */
  void removeObject(LengthConstraint *wire);

  /**
   * integrate the world
   * It is equivalent to "integrate1(); integrate2();" */
  void integrate();

  /**
   * It performs
   *    1) deletion contacts from previous time step
   *    2) collision detection
   *    3) register contacts to each body
   *    4) calls "preContactSolverUpdate1()" of each object */
  void integrate1();

  /**
   * It performs
   *    1) calls "preContactSolverUpdate2()" of each body
   *    2) run collision solver
   *    3) calls "integrate" method of each object */
  void integrate2();

  /**
   * It performs
   *    1) calls "preContactSolverUpdate2()" of each body
   *    2) run collision solver
   *    3) calls "integrate" method of each object */
  const contact::ContactProblems *getContactProblem() const { return &contactProblems_; }

  void setGravity(const Vec<3> &gravity);

  /**
   * this deletes the existing material props and replace them with the argument
   * @param prop new material prop */
  void updateMaterialProp(const MaterialManager &prop);

  /**
   * Add a new material pair property. In RaiSim, material property is defined by the pair.
   * @param mat1 name of the first material (the order of mat1 and mat2 is not important)
   * @param mat2 name of the first material
   * @param friction the coefficient of friction
   * @param restitution the coefficient of restitution
   * @param resThreshold the minimum impact velocity to make the object bounce */
  void setMaterialPairProp(const std::string &mat1,
                           const std::string &mat2,
                           double friction,
                           double restitution,
                           double resThreshold);

  /**
   * Add a new material pair property. In RaiSim, material property is defined by the pair.
   * @param mat1 name of the first material (the order of mat1 and mat2 is not important)
   * @param mat2 name of the first material
   * @param friction the dynamic coefficient of friction
   * @param restitution the coefficient of restitution
   * @param resThreshold the minimum impact velocity to make the object bounce
   * @param staticFriction the static coefficient of friction
   * @param staticFrictionThresholdVelocity if the relative velocity of two points is bigger than this value, then the dynamic coefficient of friction is applied. Otherwise, the coefficient of friction is interpolated between the static and dynamic one proportional to the relative velocity.*/
  void setMaterialPairProp(const std::string &mat1,
                           const std::string &mat2,
                           double friction,
                           double restitution,
                           double resThreshold,
                           double staticFriction,
                           double staticFrictionThresholdVelocity);

  /**
   * this default material property is used if a material pair property is not defined for the specific collision
   * @param friction the coefficient of friction
   * @param restitution the coefficient of restitution
   * @param resThreshold the minimum impact velocity to make the object bounce */
  void setDefaultMaterial(double friction,
                          double restitution,
                          double resThreshold);

  /**
   * this default material property is used if a material pair property is not defined for the specific collision
   * @param friction the coefficient of friction
   * @param restitution the coefficient of restitution
   * @param resThreshold the minimum impact velocity to make the object bounce
   * @param staticFriction the static coefficient of friction
   * @param staticFrictionThresholdVelocity if the relative velocity of two points is bigger than this value, then the dynamic coefficient of friction is applied. Otherwise, the coefficient of friction is interpolated between the static and dynamic one proportional to the relative velocity.*/
  void setDefaultMaterial(double friction,
                          double restitution,
                          double resThreshold,
                          double staticFriction,
                          double staticFrictionThresholdVelocity);

  /**
   * @return gravitational acceleration of the world */
  const Vec<3> &getGravity() const { return gravity_; }

  /**
   * Changes the Error Reduction Parameter. It often has very minimalistic impact on simulation
   * @param erp spring constant between object. This constant is scaled by the apparent inertia so it has no well-defined physical meaning
   * @param erp2 damping constant between object. This constant is scaled by the apparent inertia so it has no well-defined physical meaning */
  void setERP(double erp, double erp2 = 0);

  /**
   * Changes the contact solver parameter.
   * For details, please check "Hwangbo, Jemin, Joonho Lee, and Marco Hutter. "Per-contact iteration method for solving contact dynamics." IEEE Robotics and Automation Letters 3.2 (2018): 895-902."
   * @param alpha_init how aggressive the solver is initially
   * @param alpha_min how aggressive the solver is after an infinite number of solver iterations
   * @param alpha_decay how fast alpha converges from alpha_init to alpha_min
   * @param threshold error threshold for termination
   * @param maxIter the maximum number of iterations allowed */
  void setContactSolverParam(double alpha_init, double alpha_min, double alpha_decay, int maxIter, double threshold);

  /**
   * @return the total integrated time (which is updated at every integrate2() call)*/
  double getWorldTime() const { return worldTime_; }

  /**
   * manually adjust the world time
   * @param time the world time */
  void setWorldTime(double time) { worldTime_ = time; }

  /**
   * @return a non-const ref of the contact solver. contact::BisectionContactSolver::setOrder(bool) can be used to make the solver deterministic */
  raisim::contact::BisectionContactSolver &getContactSolver() { return solver_; }

  /**
   * @return a const ref of the contact solver. Internal states can be retrieved using this method */
  const raisim::contact::BisectionContactSolver &getContactSolver() const { return solver_; }

  /**
   * get the config file if the world was created using a xml config file
   * @return the path to the xml config file */
  const std::string &getConfigFile() { return configFile_; };

  /**
   * get a vector wires in the world
   * @return a vector of unique_ptrs of wires
   */
  std::vector<std::unique_ptr<LengthConstraint>>& getWires () { return wire_; }

  /**
   * get the material pair properties. The order of materials does not matter.
   * @param[in] mat1 material name
   * @param[in] mat2 material name
   * @return material pair properties
   */
  const MaterialPairProperties& getMaterialPairProperties (const std::string& mat1, const std::string& mat2) const {
    return mat_.getMaterialPairProp(mat1, mat2); }

protected:
  void init();
  void contactProblemUpdate(Object *objectA);
  void addCollisionObject(dGeomID colObj,
                          size_t localIdx,
                          const std::string &material,
                          CollisionGroup collisionGroup,
                          CollisionGroup collisionMask);
  void loadRaiSimConfig(const std::string& configFile);
  raisim::SingleBodyObject* addMjcfGeom(const RaiSimTinyXmlWrapper& geom,
                                        const std::unordered_map<std::string, RaiSimTinyXmlWrapper>& defaults,
                                        const std::unordered_map<std::string, std::pair<std::string, Vec<3>>>& mesh,
                                        const mjcf::MjcfCompilerSetting& setting);
  void loadMjcf(const std::string& configFile);
  ArticulatedSystem* addArticulatedSystem(const RaiSimTinyXmlWrapper& node,
                                          const std::string &resPath,
                                          const std::unordered_map<std::string, RaiSimTinyXmlWrapper>& defaults,
                                          const std::unordered_map<std::string, std::pair<std::string, Vec<3>>>& mesh,
                                          const mjcf::MjcfCompilerSetting& setting,
                                          ArticulatedSystemOption options = ArticulatedSystemOption());
  void flattenCompoundClass(std::vector<Compound::CompoundObjectChild>& oc);

  dSpaceID collisionWorld_;
  std::pair<std::vector<dContactGeom>, int> contacts_;

  // simulation properties
  Vec<3> gravity_;

  // contact solver
  raisim::contact::BisectionContactSolver solver_;

  // list
  std::vector<Object *> objectList_;
  contact::ContactProblems contactProblems_;

  // constraints
  std::vector<std::unique_ptr<LengthConstraint>> wire_;

  MaterialManager mat_;
  MaterialPairProperties defaultMaterialProperty_;
  double timeStep_ = 0.005;
  double worldTime_ = 0.;

  std::string configFile_;

  // bookkeeping
  unsigned long stepsTaken_ = 0, objectConfiguration_ = 0;
  void updateObjConfiig() { objectConfiguration_++; }
  void updateStepCount() { stepsTaken_++; }

  // xml class
  std::unordered_map<std::string, XmlObjectClass> xmlObjectClasses;

  // ray test
  dGeomID ray_;
  RayCollisionList rayContact_;

  // the location of the license file
  RAISIM_STATIC_API static std::string activationKey_;
};

} // raisim

#endif //RAISIM_WORLD_HPP
