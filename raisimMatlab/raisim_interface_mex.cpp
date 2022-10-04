//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include <iostream>
#include <memory>

#include "mex.h"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#define MAKE_STR(x) _MAKE_STR(x)
#define _MAKE_STR(x) #x

namespace raisim {
using Mat33 = raisim::Mat<3, 3>;
using Vec3 = raisim::Vec<3>;
};  // namespace raisim

bool initialized = false;
#define DEFAULT_ARGS_N 1

void fatalIf(bool fatal, const std::string& msg) {
  if (fatal) mexErrMsgTxt(msg.c_str());
}

#define CHECK_IF_STRING(X, Y) \
if (!(nrhs>X)) mexErrMsgTxt((std::to_string(X) + "th argument is missing").c_str()); \
if (!mxIsChar(prhs[X])) mexErrMsgTxt((std::to_string(X) + std::string(Y)).c_str());

#define CHECK_IF_DOUBLE(X, Y) \
if (!(nrhs>X)) mexErrMsgTxt((std::to_string(X) + "th argument is missing").c_str()); \
if (!mxIsDouble(prhs[X])) mexErrMsgTxt((std::to_string(X) + std::string(Y)).c_str());

#define CHECK_CMD CHECK_IF_STRING(0, "The first argument is the type of command and must be a string")

#define GET_STRING(X, Y) \
    CHECK_IF_STRING(X, " th argument must be a string") \
    char Y##Char[256]; \
    if (mxGetString(prhs[X], Y##Char, sizeof(Y##Char)) >256) \
      mexErrMsgTxt("First input should be a command string less than 256 characters long."); \
    std::string Y(Y##Char);

#define GET_DOUBLE(X, Y) \
    CHECK_IF_DOUBLE(X, " th argument must be a double") \
    double Y = mxGetScalar(prhs[X]);                                           \

#define GET_COLOR(X, RED, GREEN, BLUE, ALPHA) \
  GET_DOUBLE(X, RED)                          \
  GET_DOUBLE(X + 1, GREEN)                    \
  GET_DOUBLE(X + 2, BLUE)                     \
  GET_DOUBLE(X + 3, ALPHA)

#define GETOBJ                                                              \
  CHECK_IF_STRING(1, "The second argument must be the name of the object"); \
  GET_STRING(1, objNameStr)                                                 \
  raisim::Object* obj = world_->getObject(objNameStr);                      \
  if(obj == nullptr)                                                        \
    mexErrMsgTxt("The object with the given name does not exist");

#define GET_AS \
  GETOBJ \
  fatalIf(obj->getObjectType() != raisim::ObjectType::ARTICULATED_SYSTEM, "the object is not an articulated system");\
  auto as = static_cast<raisim::ArticulatedSystem*>(obj);

std::unique_ptr<raisim::World> world_;
std::unique_ptr<raisim::RaisimServer> server_;

void quit() {
  if (server_) server_->killServer();
  world_.reset(nullptr);
  server_.reset(nullptr);
  initialized = false;
  mexUnlock();
}

#define CHECK_INPUT_SIZE(X) if(nrhs != X) printf("Expecting %d inputs. Received %d inputs", X, nrhs);

#define CHECK_OUTPUT_SIZE(X) if(nlhs != X) printf("Expecting %d outputs. Received %d outputs", X, nlhs);

// number of arguments, check names, etc
#define ADD_CHECK(X) \
  if (nrhs != 2 + X) \
    mexErrMsgTxt("To add an object, the following arguments are required: \"add<Type>\", \"name\", \"shape params\"(maybe more than one), \"mass\". \n For example \"raisim(\"addCylinder\", \"mySphereName\",1.5, 2). This returns 2kg sphere with the radius of 1.5 m "); \
  CHECK_IF_STRING(1, "The second argument must be the name of the object") \
  char objName[256];                                              \
  mxGetString(prhs[DEFAULT_ARGS_N], objName, sizeof(objName)); \
  std::string objNameStr(objName);                                \

#define ADD_SINGLE_BODY_CHECK(X)                                  \
  ADD_CHECK(X)                                                    \
  double mass =  mxGetScalar(prhs[1+X]);                       \

#define SINGLE_BODY_COMMON(X, Y)                                                  \
  if (nrhs != 2 + X)                                                              \
    mexErrMsgTxt(Y);                                                              \
  if (!mxIsChar(prhs[DEFAULT_ARGS_N]))                                         \
    mexErrMsgTxt("The second argument must be the name of the SingleBody object");\
  char objName[256];                                                              \
  mxGetString(prhs[DEFAULT_ARGS_N], objName, sizeof(objName));                 \
  std::string objNameStr(objName);                                                \
  auto* obj = static_cast<raisim::SingleBodyObject*>(world_->getObject(objNameStr)); \
  if(obj == nullptr) mexErrMsgTxt("The object doesn't exist");

#define SINGLE_BODY_GETTER(X) \
  else if (!strcmp(MAKE_STR(X), cmd)) {                                        \
    SINGLE_BODY_COMMON(0, "A Single body getter only takes the object name as an input. For example, raisim(\"getPosition\", \"MY_OBJECT_NAME\")") \
    auto param = obj->X();                                                \
    plhs[0] = mxCreateDoubleMatrix(param.rows(), param.cols(), mxREAL); \
    memcpy(mxGetPr(plhs[0]), param.data(), sizeof(double) * param.rows() * param.cols()); \
  }

#define GET_EIGEN_VEC_WITH_CHECK(POSITION, SIZE) \
  if (!(nrhs>POSITION)) mexErrMsgTxt((std::to_string(POSITION) + "th argument is missing").c_str()); \
  const mwSize* dim = mxGetDimensions(prhs[POSITION]);          \
  if (!mxIsDouble(prhs[POSITION])) mexErrMsgTxt((std::to_string(POSITION) + "th argument is not double array").c_str()); \
  if (std::max(dim[0], dim[1])!=SIZE) mexErrMsgTxt((std::to_string(POSITION) + "th argument should be " + std::to_string(SIZE) + " long. "+std::to_string(std::max(dim[0], dim[1])) +" given.").c_str()); \
  Eigen::Map<Eigen::Matrix<double, -1, 1>> vec(mxGetPr(prhs[POSITION]), std::max(dim[0], dim[1]), 1);\

#define GET_EIGEN_VEC_WITH_CHECK_AND_NAME(POSITION, SIZE, NAME) \
  if (!(nrhs>POSITION)) mexErrMsgTxt((std::to_string(POSITION) + "th argument is missing").c_str()); \
  const mwSize* dim = mxGetDimensions(prhs[POSITION]);          \
  if (!mxIsDouble(prhs[POSITION])) mexErrMsgTxt((std::to_string(POSITION) + "th argument is not double array").c_str()); \
  if (std::max(dim[0], dim[1])!=SIZE) mexErrMsgTxt((std::to_string(POSITION) + "th argument should be " + std::to_string(SIZE) + " long. "+std::to_string(std::max(dim[0], dim[1])) +" given.").c_str()); \
  Eigen::Map<Eigen::Matrix<double, -1, 1>> NAME(mxGetPr(prhs[POSITION]), std::max(dim[0], dim[1]), 1);\

#define GET_EIGEN_VEC(POSITION) \
  if (!(nrhs>POSITION)) mexErrMsgTxt((std::to_string(POSITION) + "th argument is missing").c_str()); \
  const mwSize* dim = mxGetDimensions(prhs[1 + DEFAULT_ARGS_N]);          \
  if (!mxIsDouble(prhs[POSITION])) mexErrMsgTxt((std::to_string(POSITION) + "th argument is not double array").c_str()); \
  Eigen::Map<Eigen::Matrix<double, -1, 1>> vec(mxGetPr(prhs[POSITION]), std::max(dim[0], dim[1]), 1);    \

#define SETTER(X)                                                              \
  else if (!strcmp(MAKE_STR(X), cmd)) {                                        \
    GET_EIGEN_VEC(2)                                                         \
    GET_AS                                                                     \
    as->X(vec);                                                              \
  }

#define RETURN_VEC(RETURN) \
    plhs[0] = mxCreateDoubleMatrix(RETURN.rows(), RETURN.cols(), mxREAL); \
    memcpy(mxGetPr(plhs[0]), RETURN.ptr(),                                \
           sizeof(double) * RETURN.rows() * RETURN.cols());               \
    return;

#define GETTER(X)                                                     \
  else if (!strcmp(MAKE_STR(X), cmd)) {                               \
    GET_AS                                                            \
    auto gc = as->X();                                                \
    plhs[0] = mxCreateDoubleMatrix(gc.rows(), gc.cols(), mxREAL); \
    memcpy(mxGetPr(plhs[0]), gc.ptr(),                            \
           sizeof(double) * gc.rows() * gc.cols());                   \
    return;                                                           \
  }

#define FRAME_GETTER(X, a)                                                  \
  else if (!strcmp(MAKE_STR(X), cmd)) {                                     \
    GET_AS                                                                  \
    a gc;                                                                   \
    char frameName[256];                                                    \
    mxGetString(prhs[1 + DEFAULT_ARGS_N], frameName, sizeof(frameName)); \
    size_t frameId = as->getFrameIdxByName(std::string(frameName));         \
    as->X(frameId, gc);                                                     \
    plhs[0] = mxCreateDoubleMatrix(gc.rows(), gc.cols(), mxREAL);       \
    memcpy(mxGetPr(plhs[0]), gc.ptr(),                                  \
           sizeof(double) * gc.rows() * gc.cols());                         \
    return;                                                                 \
  }
#define WORLD_NOARG(X) \
  else if (!strcmp(MAKE_STR(X), cmd)) { \
    CHECK_INPUT_SIZE(1) \
    CHECK_OUTPUT_SIZE(0) \
    world_->X(); \
  }

#define NOARG(X)                        \
  else if (!strcmp(MAKE_STR(X), cmd)) { \
    GET_AS                              \
    as->X();                            \
    return;                             \
  }

#define GETTERJACO(X)                                                       \
  else if (!strcmp(MAKE_STR(X), cmd)) {                                     \
    GET_AS                                                                  \
    if (nrhs < 2 + DEFAULT_ARGS_N) mexErrMsgTxt("need jaco index.");        \
    Eigen::MatrixXd jaco(3, as->getDOF());                                  \
    jaco.setZero();                                                         \
    char frameName[256];                                                    \
    mxGetString(prhs[1 + DEFAULT_ARGS_N], frameName, sizeof(frameName)); \
    size_t frameId = as->getFrameIdxByName(std::string(frameName));         \
    as->X(frameId, jaco);                                                   \
    plhs[0] = mxCreateDoubleMatrix(3, as->getDOF(), mxREAL);            \
    memcpy(mxGetPr(plhs[0]), jaco.data(),                               \
           sizeof(double) * jaco.rows() * jaco.cols());                     \
    return;                                                                 \
  }

void mexFunction(
  int nlhs, mxArray* plhs[], int nrhs,
  const mxArray* prhs[]) {

  // nlhs   number of expected outputs
  // plhs   array to be populated by outputs (DATA BACK TO MATLAB)
  // nrhs   number of inputs
  // prhs   array poplulated by inputs (DATA FROM MATLAB)

  // get the command string
  // mxGetString (from API) will convert the first input prhs[0] to char
  CHECK_CMD
  char cmd[256];
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)) > 64)
    mexErrMsgTxt("First input should be a command string less than 64 characters long.");

  if (cmd[0] == '$')
    mexErrMsgTxt(
        "Did you use double quotation mark (\") for the inputs? MATLAB string "
        "uses a single quotation mark (\')");

  if (!strcmp("init", cmd)) {
    // warn if other commands were ignored
    if (nrhs < 1 || nrhs > 3) mexErrMsgTxt("init: requires 1. (optional) xml file path 2. (optional) tcp/ip port number");
    if (nlhs > 0) mexErrMsgTxt("init does not output anything");

    quit();
    raisim::RaiSimMsg::setFatalCallback([]() { mexErrMsgTxt(""); });

    int portPosition;
    if (mxIsChar(prhs[1])) {
      GET_STRING(1, fileName)
      world_.reset(new raisim::World(fileName));
      portPosition = 2;
    } else {
      world_.reset(new raisim::World());
      portPosition = 1;
    }
    server_.reset(new raisim::RaisimServer(world_.get()));

    if (nrhs == portPosition + 1 && mxIsDouble(prhs[portPosition])) {
      std::cout << "port number: " << (int)mxGetScalar(prhs[portPosition]) << std::endl;
      server_->launchServer((int)mxGetScalar(prhs[portPosition]));
    } else {
      server_->launchServer(8080);
      std::cout << "port number: " << 8080 << std::endl;
    }

    // ensures that, when fatal error occurs, matlab doesn't crash
    // finish initialization
    mexAtExit(quit);
    mexLock();
    initialized = true;
    return;
  }

  if (!strcmp("initNoGraphics", cmd)) {
    // warn if other commands were ignored
    if (nrhs < 1 + DEFAULT_ARGS_N || nrhs > 2 + DEFAULT_ARGS_N)
      mexErrMsgTxt(
          "init: requires 1. xml file path 2. (optional) tcp/ip port number");
    if (nlhs > 0) mexErrMsgTxt("init does not output anything");

    if (!initialized) {
      raisim::RaiSimMsg::setFatalCallback([]() { mexErrMsgTxt(""); });
      GET_STRING(1, fileName)
      world_.reset(new raisim::World(fileName));

      // ensures that, when fatal error occurs, matlab doesn't crash
      // finish initialization
      mexAtExit(quit);
      mexLock();
      initialized = true;
    } else {
      mexWarnMsgTxt("init: the world is already initialized");
    }
  }

  else if (!strcmp("license", cmd)) {
    GET_STRING(1, license);
    raisim::World::setActivationKey(license);
  }


  // input from matlab requesting object deletion?
  else if (!strcmp("quit", cmd)) {
    std::cout << "quitting raisim. Memory is cleared " << std::endl;
    // warn if other commands were ignored
    if (nrhs != 0 + DEFAULT_ARGS_N)
      mexWarnMsgTxt("Quit does not require input.");

    // warn if other commands were ignored
    if (nlhs > 0) mexWarnMsgTxt("Quit does not output anything.");
    mexUnlock();
    quit();
  }

  else if (!world_) {
    mexErrMsgTxt("The world is not initialized.");
  }

  // integration
  else if (!strcmp("integrate", cmd)) {
    CHECK_INPUT_SIZE(1)
    CHECK_OUTPUT_SIZE(0)
    if(!server_)
      mexErrMsgTxt("no graphics option is chosen during init. Use the \"integrateNoGraphics\" method instead");

    server_->integrateWorldThreadSafe();
  }
  // integration
  else if (!strcmp("integrateNoGraphics", cmd)) {
    CHECK_INPUT_SIZE(1)
    CHECK_OUTPUT_SIZE(0)
    world_->integrate();
  }

  else if (!strcmp("getNonlinearities", cmd)) {
    CHECK_INPUT_SIZE(2)
    CHECK_OUTPUT_SIZE(1)
    GET_AS
    RETURN_VEC(as->getNonlinearities(world_->getGravity()))
  }

  WORLD_NOARG(integrate1)
  WORLD_NOARG(integrate2)

  SETTER(setGeneralizedForce)
  SETTER(setGeneralizedCoordinate)
  SETTER(setGeneralizedVelocity)
  SETTER(setPGains)
  SETTER(setDGains)
  SETTER(setPTarget)
  SETTER(setDTarget)

  GETTER(getGeneralizedCoordinate)
  GETTER(getGeneralizedVelocity)
  GETTER(getMassMatrix)

  FRAME_GETTER(getFramePosition, raisim::Vec3)
  FRAME_GETTER(getFrameAngularVelocity, raisim::Vec3)
  FRAME_GETTER(getFrameVelocity, raisim::Vec3)
  FRAME_GETTER(getFrameOrientation, raisim::Mat33)

  NOARG(printOutBodyNamesInOrder)
  NOARG(printOutFrameNamesInOrder)

  GETTERJACO(getDenseFrameJacobian)
  GETTERJACO(getDenseFrameRotationalJacobian)

  else if (!strcmp("getContactForcePositionsObject", cmd)) {
    GETOBJ
    size_t bodyIdx;
    CHECK_OUTPUT_SIZE(3)

    if(obj->getObjectType() == raisim::ARTICULATED_SYSTEM) {
      CHECK_INPUT_SIZE(3)

      char bodyName[256];
      mxGetString(prhs[DEFAULT_ARGS_N + 1], bodyName, sizeof(bodyName));
      auto as = static_cast<raisim::ArticulatedSystem *>(obj);
      bodyIdx = as->getBodyIdx(bodyName);
    } else {
      CHECK_INPUT_SIZE(2)
      bodyIdx = 0;
    }
    raisim::Vec<3> impulse;
    raisim::Vec<3> position;
    std::vector<raisim::Vec<3>> impulseLists;
    std::vector<raisim::Vec<3>> positionLists;
    std::vector<std::string> nameLists;

    // "vector<string>" convert to  matlab "cell" type
    impulse.setZero();
    for (auto& contact : obj->getContacts()) {
      if (contact.getlocalBodyIndex() == bodyIdx && !contact.skip()) {
        raisim::matvecmul(contact.getContactFrame(), contact.getImpulse(), impulse);
        if (!contact.isObjectA())
          impulse *= -1.;
        impulse /= world_->getTimeStep();
        impulseLists.push_back(impulse);
        positionLists.push_back(contact.getPosition());
        nameLists.push_back(world_->getObjList()[contact.getPairObjectIndex()]->getName());
      }
    }
    plhs[0] = mxCreateDoubleMatrix(3, impulseLists.size(), mxREAL);
    plhs[1] = mxCreateDoubleMatrix(3, impulseLists.size(), mxREAL);
    plhs[2] = mxCreateCellMatrix(impulseLists.size(), 1);
    for (size_t k = 0; k < impulseLists.size(); k++) {
      memcpy(mxGetPr(plhs[0]) + k * 3, impulseLists[k].ptr(),
             sizeof(double) * 3);
      memcpy(mxGetPr(plhs[1]) + k * 3, positionLists[k].ptr(),
             sizeof(double) * 3);
      mxArray* str = mxCreateString(nameLists[k].c_str());
      mxSetCell(plhs[2], k, mxDuplicateArray(str));
      mxDestroyArray(str);
    }
  }

  else if (!strcmp("addArticulatedSystem", cmd)) {
    ADD_CHECK(1)
    char fileName[256];
    mxGetString(prhs[2], fileName, sizeof(fileName));
    std::string fileNameStr(fileName);
    auto obj = world_->addArticulatedSystem(fileNameStr);
    obj->setName(objNameStr);
  }

  else if (!strcmp("addSphere", cmd)) {
    ADD_SINGLE_BODY_CHECK(2)
    auto obj = world_->addSphere(mxGetScalar(prhs[2]), mass);
    obj->setName(objNameStr);
  }

  else if (!strcmp("addCylinder", cmd)) {
    ADD_SINGLE_BODY_CHECK(3)
    auto obj = world_->addCylinder(mxGetScalar(prhs[2]), mxGetScalar(prhs[3]), mass);
    obj->setName(objNameStr);
  }

  else if (!strcmp("addCapsule", cmd)) {
    ADD_SINGLE_BODY_CHECK(3)
    auto obj = world_->addCapsule(mxGetScalar(prhs[2]), mxGetScalar(prhs[3]), mass);
    obj->setName(objNameStr);
  }

  else if (!strcmp("addBox", cmd)) {
    ADD_SINGLE_BODY_CHECK(4)
    auto obj = world_->addBox(mxGetScalar(prhs[2]), mxGetScalar(prhs[3]), mxGetScalar(prhs[4]), mass);
    obj->setName(objNameStr);
  }

  else if (!strcmp("setPosition", cmd)) {
    // setPosition, objectName, x, y, z
    SINGLE_BODY_COMMON(3, "setPosition should be called as following: raisim(\"setPosition\", \"OBJECT_NAME\", X_POS, Y_POS, Z_POS)")
    obj->setPosition(raisim::Vec<3>{mxGetScalar(prhs[2]), mxGetScalar(prhs[3]), mxGetScalar(prhs[4])});
    if (nlhs != 0) mexWarnMsgTxt("remove does not output anything.");
  }

  else if (!strcmp("setVelocity", cmd)) {
    // setPosition, objectName, x, y, z
    SINGLE_BODY_COMMON(6, "setVelocity should be called as following: raisim(\"setVelocity\", \"OBJECT_NAME\", X_Vel, Y_Vel, Z_Vel, Omega_X, Omega_Y, Omega_Z)")
    obj->setVelocity(raisim::Vec<3>{mxGetScalar(prhs[2]), mxGetScalar(prhs[3]), mxGetScalar(prhs[4])}, raisim::Vec<3>{mxGetScalar(prhs[5]), mxGetScalar(prhs[6]), mxGetScalar(prhs[7])});
    if (nlhs != 0) mexWarnMsgTxt("remove does not output anything.");
  }

  else if (!strcmp("setQuaternion", cmd)) {
    // setOrientation, objectName, w, x, y, z
    SINGLE_BODY_COMMON(4, "setOrientation should be called as following: raisim(\"setOrientation\", \"OBJECT_NAME\", W_QUAT, X_QUAT, Y_QUAT, Z_QUAT)")
    obj->setOrientation(raisim::Vec<4>{mxGetScalar(prhs[2]), mxGetScalar(prhs[3]), mxGetScalar(prhs[4]), mxGetScalar(prhs[5])});
    if (nlhs != 0) mexWarnMsgTxt("remove does not output anything.");
  }

  else if (!strcmp("setExternalForce", cmd)) {
    if(nrhs==4) {
      GET_AS
      GET_EIGEN_VEC_WITH_CHECK(3, 3)
      GET_STRING(2, FRAME_NAME)
      raisim::Vec3 vector;
      vector = vec;
      as->setExternalForce(FRAME_NAME, vector);
    } else if (nrhs==3) {
      GETOBJ
      GET_EIGEN_VEC_WITH_CHECK(2, 3)
      raisim::Vec3 vector;
      vector = vec;
      obj->setExternalForce(0, vector);
    }
  }

  else if (!strcmp("setExternalTorque", cmd)) {

  }

  SINGLE_BODY_GETTER(getPosition)
  SINGLE_BODY_GETTER(getQuaternion)

  else if (!strcmp("addVisualArticulatedSystem", cmd)) {
    GET_STRING(1, name)
    GET_STRING(2, filePath)
    GET_COLOR(3, r, g, b, a)

    server_->addVisualArticulatedSystem(name, filePath, r, g, b, a);
  }

  else if (!strcmp("updateVisualArticulatedSystem", cmd)) {
    GET_STRING(1, name)
    GET_EIGEN_VEC(2)
    GET_COLOR(3, r, g, b, a)
    auto* asv = server_->getVisualArticulatedSystem(name);
    asv->setGeneralizedCoordinate(vec);
    asv->setColor(r,g,b,a);
  }

  else if (!strcmp("addVisualSphere", cmd)) {
    GET_STRING(1, name)
    GET_DOUBLE(2, radius)
    GET_COLOR(3, r, g, b, a)
    server_->addVisualSphere(name, radius, r, g, b, a);
  }

  else if (!strcmp("addVisualCapsule", cmd)) {
    GET_STRING(1, name)
    GET_DOUBLE(2, radius)
    GET_DOUBLE(3, length)
    GET_COLOR(4, r, g, b, a)
    server_->addVisualCapsule(name, radius, length, r, g, b, a);
  }

  else if (!strcmp("addVisualCylinder", cmd)) {
    GET_STRING(1, name)
    GET_DOUBLE(2, radius)
    GET_DOUBLE(3, length)
    GET_COLOR(4, r, g, b, a)
    server_->addVisualCylinder(name, radius, length, r, g, b, a);
  }

  else if (!strcmp("addVisualBox", cmd)) {
    GET_STRING(1, name)
    GET_DOUBLE(2, x)
    GET_DOUBLE(3, y)
    GET_DOUBLE(4, z)
    GET_COLOR(5, r, g, b, a)
    server_->addVisualBox(name, x, y, z, r, g, b, a);
  }

  else if (!strcmp("updateVisualPose", cmd)) {
    GET_STRING(1, name)
    auto* vis = server_->getVisualObject(name);
    {
      GET_EIGEN_VEC_WITH_CHECK_AND_NAME(2, 3, position)
      vis->setPosition(position);
    }

    {
      GET_EIGEN_VEC_WITH_CHECK_AND_NAME(3, 4, quaternion)
      vis->setOrientation(quaternion);
    }

    GET_COLOR(4, r, g, b, a)
    vis->setColor(r,g,b,a);
  }

  else {
    mexErrMsgTxt("The first argument must be from the available command list. Check either raisim_interface_mex.cpp file or MATLAB examples.");
  }
}
