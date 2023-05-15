//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_RAISIMSERVER_HPP
#define RAISIM_RAISIMSERVER_HPP

#if defined __linux__ || __APPLE__
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#elif WIN32
#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#define RAISIM_SERVER_SOCKET_OPTION SO_REUSEADDR | SO_BROADCAST
#endif

#if __linux__
#define RAISIM_SERVER_SOCKET_OPTION SO_REUSEADDR | SO_REUSEPORT
#elif __APPLE__
#define RAISIM_SERVER_SOCKET_OPTION SO_REUSEPORT
#endif

#include <tinyxml_rai/tinyxml_rai.h>

#include <atomic>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <future>
#include <chrono>
#include <queue>
#include "raisim/server/Visuals.hpp"
#include "raisim/server/Charts.hpp"
#include "raisim/server/SerializationHelper.hpp"
#include "raisim/World.hpp"
#include "raisim/helper.hpp"
#include "raisim/object/ArticulatedSystem/JointAndBodies.hpp"
#include "raisim/sensors/Sensors.hpp"

namespace raisim {

class RaisimServer final {
 public:
  static constexpr int SEND_BUFFER_SIZE = 33554432;
  static constexpr int MAXIMUM_PACKET_SIZE = 32384;
  static constexpr int RECEIVE_BUFFER_SIZE = 33554432;

  enum ClientMessageType : int {
    REQUEST_UPDATE = 0,
    REQUEST_SENSOR_UPDATE
  };

  enum ServerMessageType : int {
    UPDATE_ALL = 0,
    No_MESSAGE
  };

  enum class ServerRequestType : int {
    NO_REQUEST = 0,
    START_RECORD_VIDEO = 1,
    STOP_RECORD_VIDEO = 2,
    FOCUS_ON_SPECIFIC_OBJECT = 3,
    SET_CAMERA_TO = 4,
    GET_SCREEN_SHOT = 5,
    SET_SCREEN_SIZE = 6
  };

  enum class ClientRequestType : int {
    CR_SPAWN_BOX = 0,
    CR_SPAWN_SPHERE,
    CR_SPAWN_CYLINDER,
    CR_SPAWN_CAPSULE,
    CR_SPAWN_HEIGHT_MAP,
    CR_SPAWN_MESH,
    CR_SPAWN_PLANE,
    CR_SPAWN_AS,
    CR_ATTACH_WIRE,
    CR_DRAG_OBJECT,
    CR_REMOVE_OBJECT,
    CR_SAVE_THE_WORLD
  };

  enum Status : int {
    STATUS_RENDERING = 0,
    STATUS_HIBERNATING = 1,
    STATUS_TERMINATING = 2
  };

  enum Masking : int {
    AS_VISUAL_OBJ = 0,
    AS_COL_OBJ,
    SB_OBJ,
    VIS_OBJ,
    CONSTRAINTS,
    CONTACT_FORCE,
    CONTACT_POINT,
    EXT_FORCE,
    EXT_TORQUE
  };

  /**
   * @param[in] world the world to visualize.
   * create a raisimSever for a world. */
  explicit RaisimServer(World *world) : world_(world) {
    receive_buffer.resize(RECEIVE_BUFFER_SIZE);
    send_buffer.resize(SEND_BUFFER_SIZE);
    memset(tempBuffer, 0, MAXIMUM_PACKET_SIZE * sizeof(char));
  }

  ~RaisimServer() = default;

  /**
   * Setup the port so that it can accept (acceptConnection) incoming connections
   */
  void setupSocket() {
    tryingToLock_ = false;

#if __linux__ || __APPLE__
    int opt = 1;
    addrlen = sizeof(address);

    // Creating socket file descriptor
    RSFATAL_IF((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0, "socket error, errno: " << errno)
    RSFATAL_IF(setsockopt(server_fd_, SOL_SOCKET, RAISIM_SERVER_SOCKET_OPTION,
                          (char *) &opt, sizeof(opt)) == -1, "setsockopt error, errno: " << errno)

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(raisimPort_);

    // Forcefully attaching socket to the port 8080
    RSFATAL_IF(bind(server_fd_, (struct sockaddr *) &address, sizeof(address)) < 0, "bind error, errno: " << errno)
    RSFATAL_IF(listen(server_fd_, 3) < 0, "listen error, errno: " << errno)

#elif WIN32
    WSADATA wsaData;
    int iResult;

    struct addrinfo *result = nullptr;
    struct addrinfo hints;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    RSFATAL_IF(result != 0, "WSAStartup failed with error: " << iResult)

    // holds address info for socket to connect to
    struct addrinfo *ptr = nullptr;

    int opt = 1000000;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    std::string portInString = std::to_string(raisimPort_);

    // Resolve the server address and port
    iResult = getaddrinfo(nullptr, portInString.c_str(), &hints, &result);
    if (iResult != 0) {
      printf("getaddrinfo failed with error: %d\n", iResult);
      WSACleanup();
      return;
    }

    // Create a SOCKET for connecting to server
    server_fd_ =
        int(socket(result->ai_family, result->ai_socktype, result->ai_protocol));
    if (server_fd_ == INVALID_SOCKET) {
      printf("socket failed with error: %ld\n", WSAGetLastError());
      WSACleanup();
      return;
    }

    setsockopt(server_fd_, SOL_SOCKET, SO_SNDBUF, (char *) &opt, sizeof(opt));
    setsockopt(server_fd_, SOL_SOCKET, SO_RCVBUF, (char *) &opt, sizeof(opt));

    // Setup the TCP listening socket
    iResult = bind(server_fd_, result->ai_addr, (int) result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
      printf("bind failed with error: %d\n", WSAGetLastError());
      closesocket(server_fd_);
      WSACleanup();
      return;
    }

    iResult = listen(server_fd_, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
      printf("listen failed with error: %d\n", WSAGetLastError());
      closesocket(server_fd_);
      WSACleanup();
      return;
    }
#endif
  }

  /**
   * @param[in] seconds the number of seconds to wait for a connection from a client
   * Accept a connection to the socket. Only one client can be connected at a time
   */
  void acceptConnection(int seconds) {
    if (waitForNewClients(seconds)) {
#if __linux__ || __APPLE__
      client_ = accept(server_fd_, (struct sockaddr *) &address, (socklen_t *) &addrlen);
      connected_ = client_ > -1;
#elif WIN32
      client_ = int(accept(server_fd_, nullptr, nullptr));
      connected_ = client_ != INVALID_SOCKET;
#endif
      RSWARN_IF(client_ < 0, "Accept failed, errno: " << errno)
      RSINFO_IF(client_ >= 0, "Connection to "<< client_<<" is established")

      clearScene();
    }
  }

  /**
   * Close the current connection from a client. For new connections, the port has to be setup again.
   */
  void closeConnection() const {
#if __linux__ || __APPLE__
    close(server_fd_);
#elif WIN32
    closesocket(server_fd_);
    WSACleanup();
#endif
  }

 private:

  inline void clearScene() {
    lockVisualizationServerMutex();
    for (auto ob : world_->getObjList()) ob->visualTag = 0u;
    for (auto& ob: world_->getWires()) ob->visualTag = 0u;
    for (auto& ob: visuals_) ob.second->visualTag = 0u;
    for (auto& ob: visualAs_) ob.second->obj.visualTag = 0u;
    for (auto& ob: instancedvisuals_) ob.second->visualTag = 0u;
    for (auto& ob: polyLines_) ob.second->visualTag = 0u;
    for (auto& ob: charts_) ob.second->visualTag = 0u;
    unlockVisualizationServerMutex();
  }

  inline void loop() {
    setupSocket();

    while (!terminateRequested_) {
      acceptConnection(2);

      while (connected_) {
        connected_ = processRequests() && !terminateRequested_;

        if (terminateRequested_)
          state_ = STATUS_TERMINATING;

        if (state_ == STATUS_HIBERNATING)
          std::this_thread::sleep_for(std::chrono::microseconds(100000));
      }
    }

    closeConnection();
    state_ = STATUS_RENDERING;
  }

 public:
  /**
   * @param[in] map name of the map to be loaded. This only works with RaisimUnreal. Currently, the following maps are available
   * "": Empty map. Supports weather and time.
   * "simple": Empty map. Simple sky for faster rendering.
   * "wheat": a flat wheat field
   * "dune": flat dune
   */
  inline void setMap(const std::string &map) {
    mapName_ = map;
  }

  /**
   * @param[in] port port number to stream
   * start spinning. */
  inline void launchServer(int port = 8080) {
    raisimPort_ = port;
    tryingToLock_ = false;

    threadResult_ = std::async(std::launch::async, [this] {
      serverThread_ = std::thread(&raisim::RaisimServer::loop, this);
      return true;
    });
  }

  /**
   * set the world mutex.
   * This will prevent visualization thread reading from the world (otherwise, there can be a segfault).
   * Integrate the world. */
  inline void integrateWorldThreadSafe() {
    serverMutex_.lock();
    applyInteractionForce();
    world_->integrate();
    serverMutex_.unlock();
    if (tryingToLock_)
      USLEEP(10);
  }

  /**
   * Apply interaction force, which is specified by the user in the visualizer (raisimUnreal).
   * This is automatically called in raisim::RaisimServer::integrateWorldThreadSafe. */
  inline void applyInteractionForce() {
    // interaction wire
    if (wireStiffness_ > 0 && hangingObjVisTag_ != 0) {
      Vec<3> normal, vel, force;
      auto iter = std::find_if(world_->getObjList().begin(), world_->getObjList().end(),
                               [&](const Object *o) { return o->visualTag == hangingObjVisTag_; });
      if (iter == world_->getObjList().end()) {
        hangingObjLocalId_ = -1;
      } else {
        interactingOb_ = *iter;
        interactingOb_->getPosition(hangingObjLocalId_, hangingObjLocalPos_, hangingObjPos_);
        interactingOb_->getVelocity(hangingObjLocalId_, hangingObjLocalPos_, vel);
        vecsub(hangingObjTargetPos_, hangingObjPos_, normal);
        double distance = normal.norm();
        if (distance > 1e-8) {
          normal *= 1. / distance;
          double axisVel = vecDot(vel, normal);
          force = wireStiffness_ * (distance - 0.5 * axisVel * world_->getTimeStep()) * normal * interactingOb_->getMass(hangingObjLocalId_);

          if (std::find(world_->getObjList().begin(), world_->getObjList().end(), interactingOb_)
              != world_->getObjList().end()) {
            interactingOb_->setConstraintForce(hangingObjLocalId_, hangingObjLocalPos_, force);
          }
        }
      }
    } else {
      hangingObjVisTag_ = 0;
    }
  }

  /**
   * hibernate the server. This will stop the server spinning. */
  inline void hibernate() {
    std::lock_guard<std::mutex> guard(serverMutex_);
    state_ = STATUS_HIBERNATING;
  }

  /**
   * wake up the server. Restart the server from hibernation */
  inline void wakeup() {
    std::lock_guard<std::mutex> guard(serverMutex_);
    state_ = STATUS_RENDERING;
  }

  /**
   * stop spinning the server and disconnect the client */
  inline void killServer() {
    terminateRequested_ = true;
    serverThread_.join();
    terminateRequested_ = false;
  }

  /**
   * lock the visualization mutex so that the server cannot read from the world */
  inline void lockVisualizationServerMutex() { serverMutex_.lock(); }

  /**
   * unlock the visualization mutex so that the server can read from the world */
  inline void unlockVisualizationServerMutex() {
    serverMutex_.unlock();
    if (tryingToLock_)
      USLEEP(10);
  }

  /**
   * @return boolean representing if the termination requested */
  inline bool isTerminateRequested() { return terminateRequested_; }

  /**
   * @param[in] name the name of the visual articulated system object
   * @param[in] urdfFile the path to the urdf file
   * @param[in] colorR the red value of the color (max=1)
   * @param[in] colorG the green value of the color (max=1)
   * @param[in] colorB the blue value of the color (max=1)
   * @param[in] colorA the alpha value of the color (max=1)
   * @return the articulated system visual pointer
   * add an articulated system without physics */
  inline ArticulatedSystemVisual *addVisualArticulatedSystem(const std::string &name,
                                                             const std::string &urdfFile,
                                                             double colorR = 0, double colorG = 0,
                                                             double colorB = 0, double colorA = 0) {
    if (visualAs_.find(name) != visualAs_.end()) RSFATAL("Duplicated visual object name: " + name)
    visualAs_[name] = new ArticulatedSystemVisual(urdfFile);
    visualAs_[name]->color = raisim::Vec<4>{colorR, colorG, colorB, colorA};
    visualAs_[name]->name = name;
    updateVisualConfig();
    return visualAs_[name];
  }

  /**
   * @param[in] as ArticulatedSystemVisual to be removed
   * remove a visualized articulated system */
  inline void removeVisualArticulatedSystem(ArticulatedSystemVisual *as) {
    auto it = visualAs_.begin();

    // Search for an element with value 2
    while (it != visualAs_.end()) {
      if (it->second == as)
        break;
      it++;
    }

    // Erase the element pointed by iterator it
    if (it != visualAs_.end())
      visualAs_.erase(it);

    delete as;
  }

  /**
   *
   * @param name
   * @param type
   * @param size
   * @param color1
   * @param color2
   * @return
   */
  inline InstancedVisuals *addInstancedVisuals(const std::string &name,
                                               Shape::Type type,
                                               const Vec<3> &size,
                                               const Vec<4> &color1,
                                               const Vec<4> &color2) {
    RSFATAL_IF(instancedvisuals_.find(name) != instancedvisuals_.end(), "Duplicated visual object name: " + name)
    updateVisualConfig();
    instancedvisuals_[name] = new InstancedVisuals(type, name, size, color1, color2);
    return instancedvisuals_[name];
  }

  /**
   * @param[in] name the name of the visual object
   * @param[in] radius radius of the sphere
   * @param[in] colorR the red value of the color (max=1)
   * @param[in] colorG the green value of the color (max=1)
   * @param[in] colorB the blue value of the color (max=1)
   * @param[in] colorA the alpha value of the color (max=1)
   * @param[in] material visualization material
   * @param[in] glow to glow or not (not supported)
   * @param[in] shadow to cast shadow or not (not supported)
   * @return the sphere pointer
   * add a sphere without physics */
  inline Visuals *addVisualSphere(const std::string &name, double radius,
                                  double colorR = 1, double colorG = 1,
                                  double colorB = 1, double colorA = 1,
                                  const std::string &material = "",
                                  bool glow = false, bool shadow = false) {
    if (visuals_.find(name) != visuals_.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    visuals_[name] = new Visuals();
    visuals_[name]->type = Shape::Sphere;
    visuals_[name]->name = name;
    visuals_[name]->size[0] = radius;
    visuals_[name]->color = {colorR, colorG, colorB, colorA};
    visuals_[name]->glow = glow;
    visuals_[name]->shadow = shadow;
    return visuals_[name];
  }

  /**
   * @param[in] name the name of the visual object
   * @param[in] xLength length of the box
   * @param[in] yLength width of the box
   * @param[in] zLength height of the box
   * @param[in] colorR the red value of the color (max=1)
   * @param[in] colorG the green value of the color (max=1)
   * @param[in] colorB the blue value of the color (max=1)
   * @param[in] colorA the alpha value of the color (max=1)
   * @param[in] material visualization material
   * @param[in] glow to glow or not (not supported)
   * @param[in] shadow to cast shadow or not (not supported)
   * @return the box pointer
   * add a box without physics */
  inline Visuals *addVisualBox(const std::string &name, double xLength,
                               double yLength, double zLength,
                               double colorR = 1, double colorG = 1,
                               double colorB = 1, double colorA = 1,
                               const std::string &material = "",
                               bool glow = false, bool shadow = false) {
    if (visuals_.find(name) != visuals_.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    visuals_[name] = new Visuals();
    visuals_[name]->type = Shape::Box;
    visuals_[name]->name = name;
    visuals_[name]->size[0] = xLength;
    visuals_[name]->size[1] = yLength;
    visuals_[name]->size[2] = zLength;
    visuals_[name]->color = {colorR, colorG, colorB, colorA};
    visuals_[name]->glow = glow;
    visuals_[name]->shadow = shadow;
    return visuals_[name];
  }

  /**
   * @param[in] name the name of the visual object
   * @param[in] radius radius of the cylinder
   * @param[in] length length of the cylinder
   * @param[in] colorR the red value of the color (max=1)
   * @param[in] colorG the green value of the color (max=1)
   * @param[in] colorB the blue value of the color (max=1)
   * @param[in] colorA the alpha value of the color (max=1)
   * @param[in] material visualization material
   * @param[in] glow to glow or not (not supported)
   * @param[in] shadow to cast shadow or not (not supported)
   * @return the cylinder pointer
   * add a cylinder without physics */
  inline Visuals *addVisualCylinder(const std::string &name, double radius,
                                    double length, double colorR = 1,
                                    double colorG = 1, double colorB = 1,
                                    double colorA = 1,
                                    const std::string &material = "",
                                    bool glow = false, bool shadow = false) {
    if (visuals_.find(name) != visuals_.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    visuals_[name] = new Visuals();
    visuals_[name]->type = Shape::Cylinder;
    visuals_[name]->name = name;
    visuals_[name]->size[0] = radius;
    visuals_[name]->size[1] = length;
    visuals_[name]->color = {colorR, colorG, colorB, colorA};
    visuals_[name]->glow = glow;
    visuals_[name]->shadow = shadow;
    return visuals_[name];
  }

  /**
   * @param[in] name the name of the visual object
   * @param[in] radius radius of the capsule
   * @param[in] length length of the capsule
   * @param[in] colorR the red value of the color (max=1)
   * @param[in] colorG the green value of the color (max=1)
   * @param[in] colorB the blue value of the color (max=1)
   * @param[in] colorA the alpha value of the color (max=1)
   * @param[in] material visualization material
   * @param[in] glow to glow or not (not supported)
   * @param[in] shadow to cast shadow or not (not supported)
   * @return the capsule pointer
   * add a capsule without physics */
  inline Visuals *addVisualCapsule(const std::string &name, double radius,
                                   double length, double colorR = 1,
                                   double colorG = 1, double colorB = 1,
                                   double colorA = 1,
                                   const std::string &material = "",
                                   bool glow = false, bool shadow = false) {
    if (visuals_.find(name) != visuals_.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    visuals_[name] = new Visuals();
    visuals_[name]->type = Shape::Capsule;
    visuals_[name]->name = name;
    visuals_[name]->size[0] = radius;
    visuals_[name]->size[1] = length;
    visuals_[name]->color = {colorR, colorG, colorB, colorA};
    visuals_[name]->glow = glow;
    visuals_[name]->shadow = shadow;
    return visuals_[name];
  }

  /**
   * @param[in] name the name of the visual mesh object
   * @param[in] file file name of the mesh
   * @param[in] scale scale of the mesh
   * @param[in] colorR the red value of the color   (max=1)
   * @param[in] colorG the green value of the color (max=1)
   * @param[in] colorB the blue value of the color  (max=1)
   * @param[in] colorA the alpha value of the color (max=1). Ignore color when negative
   * @param[in] glow to glow or not (not supported)
   * @param[in] shadow to cast shadow or not (not supported)
   * @return the mesh visual pointer
   * add a mesh without physics */
  inline Visuals *addVisualMesh(const std::string &name,
                                const std::string &file,
                                const Vec<3> &scale = {1, 1, 1},
                                double colorR = 0, double colorG = 0,
                                double colorB = 0, double colorA = -1,
                                bool glow = false, bool shadow = false) {
    if (visuals_.find(name) != visuals_.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    auto vm = new VisualMesh();
    vm->type = Shape::Mesh;
    vm->name = name;
    vm->meshFileName_ = file;
    vm->size = scale;
    vm->color = {colorR, colorG, colorB, colorA};
    vm->glow = glow;
    vm->shadow = shadow;
    visuals_[name] = vm;
    return visuals_[name];
  }

/**
 * @param[in] name the name of the visual mesh object
 * @param[in] file file name of the mesh
 * @param vertexArray array of vertices Should be a multiple of 3
 * @param colorArray array of colors in RGB. Should be a multiple of 3
 * @param indexArray array of triangle index
 * @param colorR red color value (if colorArray is empty)
 * @param colorG green color value (if colorArray is empty)
 * @param colorB blue color value (if colorArray is empty)
 * @param colorA alpha color value (if colorArray is empty)
 * @param glow to glow or not (not supported)
 * @param shadow to cast shadow or not (not supported)
 * @return the mesh visual pointer (VisualMesh struct)
 */
  inline VisualMesh *addVisualMesh(const std::string &name,
                                   const std::vector<float>& vertexArray,
                                   const std::vector<uint8_t>& colorArray,
                                   const std::vector<int32_t>& indexArray,
                                   double colorR = 0, double colorG = 0,
                                   double colorB = 0, double colorA = 1,
                                   bool glow = false, bool shadow = false) {
    RSFATAL_IF(visuals_.find(name) != visuals_.end(), "Duplicated visual object name: " + name)
    RSFATAL_IF(vertexArray.size() % 3 != 0, "The number of vertex elements should be a multiple of 3")
    RSFATAL_IF(colorArray.size() % 3 != 0, "The number of color elements should be a multiple of 3")
    RSFATAL_IF(indexArray.size() % 3 != 0, "The number of index elements should be a multiple of 3")
    RSFATAL_IF(colorArray.size() != vertexArray.size(), "The number of the vertex elements and the number of the color elements should be the same")

    updateVisualConfig();
    auto vm = new VisualMesh();
    vm->type = Shape::Mesh;
    vm->name = name;
    vm->vertexArray_ = vertexArray;
    vm->colorArray_ = colorArray;
    vm->indexArray_ = indexArray;
    vm->color = {colorR, colorG, colorB, colorA};
    vm->size = {1, 1, 1, 1};
    vm->glow = glow;
    vm->shadow = shadow;
    visuals_[name] = vm;
    return vm;
  }

  /**
   * @param[in] name the name of the visual mesh object
   * @param[in] radius radius of the arrow
   * @param[in] height height of the arrow
   * @param[in] colorR the red value of the color   (max=1)
   * @param[in] colorG the green value of the color (max=1)
   * @param[in] colorB the blue value of the color  (max=1)
   * @param[in] colorA the alpha value of the color (max=1)
   * @param[in] glow to glow or not (not supported)
   * @param[in] shadow to cast shadow or not (not supported)
   * @return the visual pointer
   * add an arrow without physics */
  inline Visuals *addVisualArrow(const std::string &name,
                                 double radius, double height,
                                 double colorR = 0, double colorG = 0,
                                 double colorB = 0, double colorA = -1,
                                 bool glow = false, bool shadow = false) {
    if (visuals_.find(name) != visuals_.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    visuals_[name] = new Visuals();
    visuals_[name]->type = Shape::Arrow;
    visuals_[name]->name = name;
    visuals_[name]->size[0] = radius;
    visuals_[name]->size[1] = height;
    visuals_[name]->color = {colorR, colorG, colorB, colorA};
    visuals_[name]->glow = glow;
    visuals_[name]->shadow = shadow;
    return visuals_[name];
  }

  /**
   * @param[in] name the name of the polyline
   * @return the polyline pointer
   * add a polyline without physics */
  inline PolyLine *addVisualPolyLine(const std::string &name) {
    RSFATAL_IF(polyLines_.find(name) != polyLines_.end(), "Duplicated polyline object name: " + name)
    polyLines_[name] = new PolyLine();
    polyLines_[name]->name = name;
    return polyLines_[name];
  }

  /**
   * @param[in] name the name of the polyline
   * get visualized polyline */
  inline PolyLine *getVisualPolyLine(const std::string &name) {
    RSFATAL_IF(polyLines_.find(name) == polyLines_.end(), name + " doesn't exist")
    return polyLines_[name];
  }

  /**
   * @param[in] name the name of the visual articulated system
   * get visualized articulated system */
  inline ArticulatedSystemVisual *getVisualArticulatedSystem(const std::string &name) {
    RSFATAL_IF(visualAs_.find(name) == visualAs_.end(), name + " doesn't exist")
    return visualAs_[name];
  }

  /**
   * @param[in] name the name of the polyline to be removed
   * remove an existing polyline */
  inline void removeVisualPolyLine(const std::string &name) {
    if (polyLines_.find(name) == polyLines_.end()) RSFATAL("Visual polyline with name \"" + name + "\" doesn't exist.")
    updateVisualConfig();
    delete polyLines_[name];
    polyLines_.erase(name);
  }

  /**
   * @param[in] name the name of the visual object to be retrieved
   * @return visual object with a specified name
   * retrieve a visual object with a specified name */
  inline Visuals *getVisualObject(const std::string &name) {
    if (visuals_.find(name) == visuals_.end()) RSFATAL(
        "Visual object with name \"" + name + "\" doesn't exist.")
    return visuals_[name];
  }

  /**
   * @param[in] name the name of the visual object to be removed
   * remove an existing visual object */
  inline void removeVisualObject(const std::string &name) {
    if (visuals_.find(name) == visuals_.end()) RSFATAL(
        "Visual object with name \"" + name + "\" doesn't exist.")
    updateVisualConfig();
    delete visuals_[name];
    visuals_.erase(name);
  }

  /**
   * @param[in] videoName name of the video file to be saved. The videoName must be a valid file name (e.g., no spaces, ending in .mp4)
   * start recording video. RaisimUnity only supports video recording in linux */
  inline void startRecordingVideo(const std::string &videoName) {
    serverRequest_.push_back(ServerRequestType::START_RECORD_VIDEO);
    videoName_ = videoName;
  }

  /**
   * stop recording video */
  inline void stopRecordingVideo() {
    serverRequest_.push_back(ServerRequestType::STOP_RECORD_VIDEO);
  }

 private:

 public:
  /**
   * @param[in] pos the position of the camera
   * @param[in] lookAt the forward direction of the camera (the up direction is always z-axis)
   * set the camera to a specified position */
  void setCameraPositionAndLookAt(const Eigen::Vector3d &pos, const Eigen::Vector3d &lookAt) {
    serverRequest_.push_back(ServerRequestType::SET_CAMERA_TO);
    position_ = pos;
    lookAt_ = lookAt;
  }

  /**
   * @param[in] obj the object to look at
   * move the camera to look at the specified object */
  void focusOn(raisim::Object *obj) {
    RSFATAL_IF(obj == nullptr, "object does not exist.")
    serverRequest_.push_back(ServerRequestType::FOCUS_ON_SPECIFIC_OBJECT);
    toBeFocused_ = obj;
  }

  /**
   * @return if a client is connected to a server
   * check if a client is connected to a server */
  bool isConnected() const { return connected_; }

  /**
   * @param[in] seconds the number of seconds to wait for the client
   * @return true if there is a message from the client
   * This method checks if there is a message from the client. It waits a specified time for a message.
   * If this method is not used, the application will stop if there is no message. */
  inline bool waitForMessageFromClient(int seconds) {
#if defined __linux__ || __APPLE__
    struct pollfd fds[1];
    int ret;
    fds[0].fd = client_;
    fds[0].events = POLLIN;
    ret = poll(fds, 1, seconds * 1000);
    if ( ret == 0) {
      RSWARN("The client failed to respond in "<<seconds<<" seconds. Looking for a new client");
      return false;
    } else if( ret == -1 ) {
      RSWARN("The client error. Failed to communicate.");
      return false;
    }
    return true;
#elif WIN32
    fd_set fds;
    int n;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(client_, &fds);
    tv.tv_sec = 20;
    tv.tv_usec = 100000;
    n = select(server_fd_ + 1, &fds, NULL, NULL, &tv);
    if (n == 0) {
      RSWARN("The client failed to respond in " << seconds << " seconds. Looking for a new client");
      return false;
    } else if (n == -1) {
      RSWARN("The client error. Failed to communicate.");
      return false;
    }
    return true;
#endif
  }

  inline bool waitForMessageToClient(int seconds) {
#if defined __linux__ || __APPLE__
    struct pollfd fds[2];
    fds[0].fd = client_;
    fds[0].events = POLLOUT;
    int ret = poll(fds, 2, seconds * 1000);
    if ( ret == 0) {
      RSWARN("The client failed to respond in " << seconds << " seconds. Looking for a new client");
      return false;
    } else if( ret == -1 ) {
      RSWARN("The client error. Failed to communicate.");
      return false;
    }
    return true;
#elif WIN32
    fd_set fds;
    int n;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(client_, &fds);
    tv.tv_sec = 20;
    tv.tv_usec = 100000;
    n = select(server_fd_ + 1, NULL, &fds, NULL, &tv);
    if (n == 0) {
      RSWARN("The client failed to respond in " << seconds << " seconds. Looking for a new client");
      return false;
    } else if (n == -1) {
      RSWARN("The client error. Failed to communicate.");
      return false;
    }
    return true;
#endif
  }

  /**
   * Synchronous update method.
   * Receive a request from the client, process it and return the requested data to the client.
   * The method return false if 1) the client failed to respond 2) the client protocol version is different 3) the client refused to receive the data 4) the client did not send the sensor data in time
   * @return if succeeded or not.
   */
  inline bool processRequests() {
    using namespace server;
    ClientMessageType type;
    if (!receiveData(10)) return false;

    int clientVersion;
    rData_ = get(rData_, &clientVersion);
    rData_ = get(rData_, &type, &objectId_);

    data_ = set(&send_buffer[0] + sizeof(int), version_);
    if (clientVersion == version_) {
      // get client request
      int clientRequestSize;
      ClientRequestType requestType;
      rData_ = get(rData_, &clientRequestSize);
      wireStiffness_ = 0.;
      tryingToLock_ = true;
      lockVisualizationServerMutex();
      tryingToLock_ = false;

      for (int i=0; i<clientRequestSize; i++) {
        rData_ = get(rData_, &requestType);
        switch (requestType) {
          case ClientRequestType::CR_ATTACH_WIRE: {
            rData_ = get(rData_, &hangingObjVisTag_, &hangingObjLocalId_, &hangingObjPos_);

            // hanging rope
            auto &obList = world_->getObjList();
            auto iter = std::find_if(obList.begin(), obList.end(),
                                     [&](const Object *o) { return o->visualTag == hangingObjVisTag_; });
            if (iter != obList.end()) {
              interactingOb_ = *iter;
              if (interactingOb_->getObjectType() == ObjectType::ARTICULATED_SYSTEM) {
                auto as = dynamic_cast<ArticulatedSystem *>(interactingOb_);
                as->getPositionInBodyCoordinate(hangingObjLocalId_, hangingObjPos_, hangingObjLocalPos_);
              } else {
                auto sob = dynamic_cast<SingleBodyObject *>(interactingOb_);
                Vec<3> sobPos;
                Mat<3, 3> sobRot;
                sob->getPosition(sobPos);
                sob->getOrientation(0, sobRot);
                hangingObjLocalPos_ = sobRot.transpose() * (hangingObjPos_ - sobPos);
              }
            } else {
              hangingObjLocalId_ = -1;
            }
          }
            break;
          case ClientRequestType::CR_DRAG_OBJECT: {
            rData_ = getInFloat(rData_, &wireStiffness_);
            rData_ = get(rData_, &hangingObjTargetPos_);
          }
            break;

          case ClientRequestType::CR_REMOVE_OBJECT: {
            uint32_t id;
            rData_ = get(rData_, &id);
            auto &obList = world_->getObjList();
            auto iter = std::find_if(obList.begin(), obList.end(),
                                     [&](const Object *o) { return o->visualTag == id; });
            if (iter != obList.end()) {
              auto ob = *iter;
              world_->removeObject(ob);
            } else {
              auto &wireList = world_->getWires();
              LengthConstraint *w = nullptr;
              for (int j = 0; j < wireList.size(); j++) {
                if (wireList[j]->visualTag == id) {
                  w = wireList[j].get();
                  break;
                }
              }
              if (w)
                world_->removeObject(w);
            }
          }
            break;
          case ClientRequestType::CR_SPAWN_BOX:
          case ClientRequestType::CR_SPAWN_SPHERE:
          case ClientRequestType::CR_SPAWN_CYLINDER:
          case ClientRequestType::CR_SPAWN_CAPSULE:
          case ClientRequestType::CR_SPAWN_HEIGHT_MAP:
          case ClientRequestType::CR_SPAWN_MESH:
          case ClientRequestType::CR_SPAWN_PLANE:
          case ClientRequestType::CR_SPAWN_AS: {
            std::string name, appearance, file;
            float mass;
            int bodyType;
            Vec<3> pos, linVel, angVel;
            Vec<6> size;
            Vec<4> quat;
            rData_ = get(rData_, &name, &appearance, &mass);
            rData_ = getInFloat(rData_, &size, &pos, &linVel, &angVel, &quat);
            rData_ = get(rData_, &bodyType, &file);

            if (requestType == ClientRequestType::CR_SPAWN_HEIGHT_MAP ||
                requestType == ClientRequestType::CR_SPAWN_MESH ||
                requestType == ClientRequestType::CR_SPAWN_AS) {
              if (!raisim::fileExists(file)) {
                RSWARN("file \""<<file<<"\" does not exist. Ignoring the spawning commnad.")
                continue;
              }
            }

            if (int(requestType) < 4 || requestType == ClientRequestType::CR_SPAWN_MESH) {
              raisim::SingleBodyObject* sob = nullptr;

              if (requestType == ClientRequestType::CR_SPAWN_BOX) {
                sob = world_->addBox(size[0], size[1], size[2], mass);
              } else if (requestType == ClientRequestType::CR_SPAWN_SPHERE) {
                sob = world_->addSphere(size[0], mass);
              } else if (requestType == ClientRequestType::CR_SPAWN_CYLINDER) {
                sob = world_->addCylinder(size[0], size[1], mass);
              } else if (requestType == ClientRequestType::CR_SPAWN_CAPSULE) {
                sob = world_->addCapsule(size[0], size[1], mass);
              } else if (requestType == ClientRequestType::CR_SPAWN_MESH) {
                sob = world_->addMesh(file, mass);
              }

              if (sob) {
                sob->setPosition(pos);
                sob->setOrientation(quat);
                sob->setLinearVelocity(linVel);
                sob->setAngularVelocity(angVel);
                sob->setName(name);
                sob->setAppearance(appearance);
                if (bodyType == 0) {
                  sob->setBodyType(BodyType::DYNAMIC);
                } else if (bodyType == 1) {
                  sob->setBodyType(BodyType::KINEMATIC);
                } else if (bodyType == 2) {
                  sob->setBodyType(BodyType::STATIC);
                } else {
                  RSFATAL("Unknown body!")
                }
              }
            } else if (requestType == ClientRequestType::CR_SPAWN_PLANE) {
              auto ground = world_->addGround(size[0]);
              ground->setName(name);
            } else if (requestType == ClientRequestType::CR_SPAWN_HEIGHT_MAP) {
              auto hm = world_->addHeightMap(file, size[0], size[1], size[2], size[3], size[4], size[5]);
              hm->setName(name);
            } else if (requestType == ClientRequestType::CR_SPAWN_AS) {
              auto as = world_->addArticulatedSystem(file);
              as->setName(name);
              as->setBasePos(pos);
              as->setBaseOrientation(quat);
            }
          }
            break;

          case ClientRequestType::CR_SAVE_THE_WORLD: {
            std::string path;
            rData_ = get(rData_, &path);
            world_->exportToXml(path);
          }
            break;
          default:
            break;
        }
      }

      // set server request
      char* toBeFocusedPtr = nullptr;
      data_ = set(data_, state_);
      data_ = set(data_, (int32_t) serverRequest_.size());
      for (const auto &sr: serverRequest_) {
        data_ = set(data_, (int) sr);

        switch (sr) {
          case ServerRequestType::NO_REQUEST:
          case ServerRequestType::STOP_RECORD_VIDEO:
          case ServerRequestType::GET_SCREEN_SHOT:
            break;

          case ServerRequestType::START_RECORD_VIDEO:
            data_ = set(data_, videoName_);
            break;

          case ServerRequestType::SET_CAMERA_TO:
            data_ = set(data_, position_, lookAt_);
            break;

          case ServerRequestType::FOCUS_ON_SPECIFIC_OBJECT:
            toBeFocusedPtr = data_;
            data_ = set(data_, toBeFocused_->visualTag);
            break;

          case ServerRequestType::SET_SCREEN_SIZE:
            data_ = set(data_, screenShotWidth_, screenShotHeight_);
            break;
        }
      }

      serverRequest_.clear();

      if (state_ != Status::STATUS_HIBERNATING) update();

      /// reassign the vis tag because it was reset in the update
      if (toBeFocusedPtr)
        set(toBeFocusedPtr, toBeFocused_->visualTag);

      unlockVisualizationServerMutex();
    } else {
      RSWARN("Version mismatch. Raisim protocol version: "<<version_<<", Visualizer protocol version: "<<clientVersion)
      return false;
    }

    if (!sendData())
      return false;

    if (needsSensorUpdate_) {
      if (!receiveData(5))
        return false;

      /// send dummy data to let visualizer know that receive is done
      data_ = set(&send_buffer[0] + sizeof(int), version_);
      sendData();

      updateSensorMeasurements();
      needsSensorUpdate_ = false;
    }

    return state_ == STATUS_RENDERING || state_ == STATUS_HIBERNATING;
  }

  /**
   * wait for a new client
   * @param seconds how long to wait for a new client
   * @return if a new client was found or not
   */
  inline bool waitForNewClients(int seconds) {
    fd_set sdset;
    struct timeval tv;
    tv.tv_sec = seconds;
    tv.tv_usec = 0;
    FD_ZERO(&sdset);
    FD_SET(server_fd_, &sdset);
    return select(server_fd_ + 1, &sdset, nullptr, nullptr, &tv) > 0;
  }

  /**
   * Saves the screenshot (the directory is chosen by the visualizer)
   */
  void requestSaveScreenshot() {
    std::lock_guard<std::mutex> guard(serverMutex_);
    serverRequest_.push_back(ServerRequestType::GET_SCREEN_SHOT);
  }

 private:

  static inline std::string colorToString(const raisim::Vec<4> &vec) {
    std::string str;
    for (int i = 0; i < vec.size() - 1; i++) {
      str += std::to_string(vec[i]) + ", ";
    }
    str += std::to_string(vec[vec.size() - 1]);
    return str;
  }

  inline void serializeAS(ArticulatedSystem* as, bool initialized, const raisim::Vec<4>& colorOverride) {
    using namespace server;
    data_ = set(data_, (int32_t) (as->getVisOb().size() + as->getVisColOb().size()));
    if (!initialized) data_ = set(data_, as->name_);

    for (int i = 0; i < 2; i++) {
      std::vector<VisObject> *visVec;
      if (i == 0) visVec = &as->getVisOb();
      else visVec = &as->getVisColOb();

      for (int j = 0; j < visVec->size(); j++) {
        if (!initialized) {
          data_ = set(data_, visVec->at(j).shape);
          if (visVec->at(j).shape == Shape::Mesh) {
            data_ = set(data_, int32_t(1), visVec->at(j).fileName);
            data_ = set(data_, as->getResourceDir());
          }
          data_ = set(data_, i, int32_t(visVec->at(j).localIdx));
        }

        if (visVec->at(j).shape == Shape::Mesh)
          data_ = set(data_, int(false));

        if (colorOverride[3] < 0.001)
          data_ = set(data_, colorToString(visVec->at(j).color));
        else
          data_ = set(data_, colorToString(colorOverride));

        if (visVec->at(j).shape == Shape::Mesh)
          data_ = setInFloat(data_, visVec->at(j).scale, 0.);
        else
          data_ = setInFloat(data_, visVec->at(j).visShapeParam);

        Vec<3> pos, offsetInWorld, posOffset;
        Vec<4> quat;
        Mat<3, 3> oriOffset, bodyRotation, rot;
        as->getPosition(visVec->at(j).localIdx, pos);
        as->getOrientation(visVec->at(j).localIdx, bodyRotation);

        if (i == 0) {
          posOffset = visVec->at(j).offset;
          oriOffset = visVec->at(j).rot;
        } else {
          posOffset = as->getCollisionBodies()[j].posOffset;
          oriOffset = as->getCollisionBodies()[j].rotOffset;
        }

        matvecmul(bodyRotation, posOffset, offsetInWorld);
        matmul(bodyRotation, oriOffset, rot);
        raisim::rotMatToQuat(rot, quat);
        pos = pos + offsetInWorld;
        data_ = setInFloat(data_, pos, quat);
      }
    }

    data_ = set(data_, (int32_t) as->getSensors().size());

    // add sensors to be updated
    for (auto &sensor: as->getSensors()) {
      if (!initialized) data_ = sensor.second->serializeProp(data_);

      data_ = set(data_, sensor.second->getMeasurementSource());

      if (sensor.second->getMeasurementSource() == Sensor::MeasurementSource::VISUALIZER &&
          sensor.second->getUpdateTimeStamp() + 1. / sensor.second->getUpdateRate()
              < world_->getWorldTime() + 1e-10) {
        sensor.second->setUpdateTimeStamp(world_->getWorldTime());
        data_ = set(data_, true);
        needsSensorUpdate_ = true;
      } else {
        data_ = set(data_, false);
      }

      sensor.second->updatePose();
      Vec<4> quat;
      auto &pos = sensor.second->getPosition();
      auto &rot = sensor.second->getOrientation();
      rotMatToQuat(rot, quat);
      data_ = setInFloat(data_, pos, quat);

      if (sensor.second->getMeasurementSource() != Sensor::MeasurementSource::VISUALIZER)
        data_ = sensor.second->serializeMeasurements(data_);
    }
  }

  inline void update() {
    using namespace server;
    auto &objList = world_->getObjList();
    data_ = set(data_, ServerMessageType::UPDATE_ALL);
    data_ = set(data_, (double) world_->getWorldTime());
    data_ = set(data_, mapName_);
    data_ = set(data_, (uint32_t) (world_->getConfigurationNumber() + visualConfiguration_));
    data_ = set(data_, (uint32_t) (objList.size() +
        visuals_.size() +
        instancedvisuals_.size() +
        visualAs_.size() +
        polyLines_.size() +
        world_->getWires().size()));

    /// UniqueVisualTag/IsInitialized/Name/IsVisual/ObjectType/Instanced/
    /// VisualSize/{IfNotInitialized:[VisualType/VisualDescription/Masking]/
    /// Appearance/Size/Pose}/
    /// SensorSize/{SensorDescription}
    for (auto *ob: objList) {
      // set gc
      bool initialized = ob->visualTag != 0;
      if (!initialized) ob->visualTag = visTagCounter++;
      data_ = set(data_, ob->visualTag, initialized, ob->getObjectType(), false);
      if (ob->getObjectType() == ObjectType::ARTICULATED_SYSTEM) {
        auto as = dynamic_cast<ArticulatedSystem *>(ob);
        Vec<4> colorOverride;
        colorOverride.setZero();
        serializeAS(as, initialized, colorOverride);
      } else if (ob->getObjectType() == ObjectType::COMPOUND) {
        auto *com = dynamic_cast<Compound *>(ob);
        data_ = set(data_, (int32_t) (com->getObjList().size()));
        if (!initialized ) data_ = set(data_, ob->name_);

        for (auto &vob: dynamic_cast<Compound *>(ob)->getObjList()) {
          if (!initialized) {
            switch (vob.objectType) {
              case BOX:
                data_ = set(data_, Shape::Box);
                break;
              case CAPSULE:
                data_ = set(data_, Shape::Capsule);
                break;
              case CYLINDER:
                data_ = set(data_, Shape::Cylinder);
                break;
              case SPHERE:
                data_ = set(data_, Shape::Sphere);
                break;
              default:
                break;
            }
            data_ = set(data_, Masking::SB_OBJ, int32_t(0));
          }
          data_ = set(data_, vob.appearance);
          data_ = setInFloat(data_, vob.objectParam);

          Vec<3> pos, center;
          com->getPosition(center);
          matvecmul(com->getRotationMatrix(), vob.trans.pos, pos);
          Mat<3, 3> rot;
          matmul(com->getRotationMatrix(), vob.trans.rot, rot);
          Vec<4> quat;
          raisim::rotMatToQuat(rot, quat);
          pos = pos + center;
          data_ = setInFloat(data_, pos, quat);
        }
        data_ = set(data_, (int32_t) 0);
      } else {
        data_ = set(data_, (int32_t) 1);

        if (!initialized) {
          data_ = set(data_, ob->name_);
          switch (ob->getObjectType()) {
            case SPHERE:
              data_ = set(data_, Shape::Sphere);
              break;
            case BOX:
              data_ = set(data_, Shape::Box);
              break;
            case CYLINDER:
              data_ = set(data_, Shape::Cylinder);
              break;
            case CONE:
              break;
            case CAPSULE:
              data_ = set(data_, Shape::Capsule);
              break;
            case HALFSPACE:
              data_ = set(data_, Shape::Ground);
              break;
            case MESH:
              data_ = set(data_, Shape::Mesh);
              data_ = set(data_, int32_t(1), dynamic_cast<Mesh *>(ob)->getMeshFileName());
              data_ = set(data_, std::string());
              break;
            case HEIGHTMAP: {
              auto hm = dynamic_cast<HeightMap *>(ob);
              data_ = set(data_, Shape::HeightMap);
              data_ = setInFloat(data_, hm->getCenterX(), hm->getCenterY(), hm->getXSize(), hm->getYSize());
              data_ = set(data_, (int32_t) hm->getXSamples(), (int32_t) hm->getYSamples());
              data_ = setInFloat(data_, hm->getHeightVector());
              data_ = set(data_, hm->getColorMap());
            }
            case COMPOUND:
            case ARTICULATED_SYSTEM:
            case UNRECOGNIZED:
              break;
          }
          data_ = set(data_, Masking::SB_OBJ, int32_t(0));
        }
        auto *sob = dynamic_cast<SingleBodyObject *>(ob);
        auto tempAdd = data_;

        // if heightmap, check if update is necessary
        if (ob->getObjectType() == ObjectType::HEIGHTMAP) {
          auto hm = dynamic_cast<HeightMap *>(ob);
          data_ = set(data_, int(hm->isUpdated()));
          if (hm->isUpdated()) {
            data_ = setInFloat(data_, hm->getCenterX(), hm->getCenterY(), hm->getXSize(), hm->getYSize());
            data_ = set(data_, (int32_t) hm->getXSamples(), (int32_t) hm->getYSamples());
            data_ = setInFloat(data_, hm->getHeightVector());
            data_ = set(data_, hm->getColorMap());
          }
        } else if (ob->getObjectType() == ObjectType::MESH)
          data_ = set(data_, int(false));

        data_ = set(data_, sob->getAppearance());

        switch (ob->getObjectType()) {
          case SPHERE:
            data_ = setInFloat(data_, dynamic_cast<Sphere *>(ob)->getRadius(), 0., 0., 0.);
            break;
          case BOX:
            data_ = setInFloat(data_, dynamic_cast<Box *>(ob)->getDim(), 0.);
            break;
          case CYLINDER:
            data_ = setInFloat(data_,
                               dynamic_cast<Cylinder *>(ob)->getRadius(),
                               dynamic_cast<Cylinder *>(ob)->getHeight(),
                               0.,
                               0.);
            break;
          case CONE:
            break;
          case CAPSULE:
            data_ = setInFloat(data_,
                               dynamic_cast<Capsule *>(ob)->getRadius(),
                               dynamic_cast<Capsule *>(ob)->getHeight(),
                               0.,
                               0.);
            break;
          case HALFSPACE:
            data_ = setInFloat(data_, dynamic_cast<Ground *>(ob)->getHeight(), 0., 0., 0.);
            break;
          case HEIGHTMAP:
            data_ = set(data_, 0.f, 0.f, 0.f, 0.f);
            break;
          case MESH: {
            auto scale = float(dynamic_cast<Mesh *>(ob)->getScale());
            data_ = set(data_, scale, scale, scale, 0.f);
          }
            break;
          case COMPOUND:
          case ARTICULATED_SYSTEM:
          case UNRECOGNIZED:
            break;
        }

        Vec<3> pos;
        Vec<4> quat;
        sob->getPosition(pos);
        sob->getQuaternion(quat);
        data_ = setInFloat(data_, pos, quat);
        data_ = set(data_, (int32_t) 0);
      }
    }

    for (auto &sw: world_->getWires()) {
      bool initialized = sw->visualTag != 0;
      if (!initialized) sw->visualTag = visTagCounter++;
      data_ = set(data_, sw->visualTag, initialized, int32_t(-1), false, (int32_t)1);
      if (!initialized) data_ = set(data_, sw->name_, Shape::SingleLine, Masking::CONSTRAINTS, int32_t(0));
      data_ = set(data_, colorToString(sw->getColor()));
      Vec<3> pos, diff, diff_norm;
      Vec<4> quat;
      Mat<3,3> rot;
      diff = sw->getP2() - sw->getP1();
      diff_norm = diff * (1./diff.norm());
      pos = (sw->getP1() + sw->getP2()) / 2.0;
      raisim::zaxisToRotMat(diff_norm, rot);
      raisim::rotMatToQuat(rot, quat);
      data_ = set(data_, float(sw->getVisualizationWidth()), float(sw->getVisualizationWidth()), (float)diff.norm(), 0.f);
      data_ = setInFloat(data_, pos, quat);
      data_ = set(data_, (int32_t) 0);
    }

    // single visuals
    for (auto &ob: visuals_) {
      auto *vo = ob.second;
      bool initialized = vo->visualTag != 0;
      if (!initialized) vo->visualTag = visTagCounter++;
      Vec<3> pos = vo->getPosition();
      Vec<4> quat = vo->getOrientation();
      data_ = set(data_, vo->visualTag, initialized, int32_t(-1), false, int32_t(1));
      if (!initialized) {
        data_ = set(data_, vo->name, vo->type);
        if (vo->type == Shape::Mesh) {
          auto vm = reinterpret_cast<VisualMesh*>(vo);

          if (vm->meshFileName_.empty())
            data_ = set(data_, int32_t(0), vm->vertexArray_, vm->indexArray_, vm->colorArray_);
          else
            data_ = set(data_, int32_t(1), vm->meshFileName_, std::string());
        }
        data_ = set(data_, Masking::VIS_OBJ, int32_t(0));
      }

      if (vo->type == Shape::Mesh) {
        auto vm = reinterpret_cast<VisualMesh *>(vo);
        data_ = set(data_, int(vm->isUpdated()));
        if (vm->isUpdated())
          data_ = set(data_, vm->vertexArray_, vm->colorArray_);
      }

      data_ = set(data_, colorToString(vo->color));
      data_ = setInFloat(data_, vo->size, pos, quat);
      data_ = set(data_, (int32_t) 0);
    }

    // ArticulatedSystemVisuals
    for (auto &vis: visualAs_) {
      auto *ob = &vis.second->obj;
      bool initialized = ob->visualTag != 0;
      if (!initialized) ob->visualTag = visTagCounter++;
      data_ = set(data_, ob->visualTag, initialized, ob->getObjectType(), false);
      serializeAS(ob, initialized, vis.second->color);
    }

    // instanced visuals
    for (auto &iv: instancedvisuals_) {
      auto *v = iv.second;
      bool initialized = v->visualTag != 0;
      if (!initialized) v->visualTag = visTagCounter++;
      data_ = set(data_, v->visualTag, initialized, int32_t(-1), true, (int32_t) v->count());
      if (!initialized) data_ = set(data_, v->name, v->type, Masking::VIS_OBJ);
      data_ = setInFloat(data_, v->color1, v->color2);
      for (size_t i = 0; i < v->count(); i++) {
        data_ = set(data_, v->data[i].colorWeight);
        data_ = setInFloat(data_, v->data[i].scale, v->data[i].pos, v->data[i].quat);
      }
    }

    // polylines
    for (auto &pl: polyLines_) {
      auto *ptr = pl.second;
      bool initialized = ptr->visualTag != 0;
      if (!initialized) ptr->visualTag = visTagCounter++;
      data_ = set(data_, ptr->visualTag, initialized, int32_t(-1), true, (int32_t) (ptr->points.size()-1));
      if (!initialized) data_ = set(data_, ptr->name, Shape::PolyLine, Masking::VIS_OBJ);
      data_ = setInFloat(data_, ptr->color, ptr->color);
      RSFATAL_IF(ptr->points.size() < 2, "polyline point size should be greater than 1")
      for (int i=0; i<ptr->points.size() - 1; i++) {
        Vec<3> pos, diff, norm_diff;
        Vec<4> quat;
        Mat<3,3> rot;
        pos = (ptr->points[i] + ptr->points[i+1]) / 2.0;
        diff = ptr->points[i+1] - ptr->points[i];
        norm_diff = diff / diff.norm();
        raisim::zaxisToRotMat(norm_diff, rot);
        raisim::rotMatToQuat(rot, quat);
        data_ = setInFloat(data_, 0., ptr->width, ptr->width, diff.norm());
        data_ = setInFloat(data_, pos, quat);
      }
    }

    // External forces
    int32_t numExtForce = 0;
    int32_t numExtTorque = 0;

    for (auto *ob: world_->getObjList()) {
      numExtForce += int32_t(ob->getExternalForce().size());
      numExtTorque += int32_t(ob->getExternalTorque().size());
    }

    data_ = set(data_, numExtForce);
    for (auto *ob: world_->getObjList()) {
      for (size_t extNum = 0; extNum < ob->getExternalForce().size(); extNum++) {
        data_ = setInFloat(data_, ob->getExternalForcePosition()[extNum]);
        data_ = setInFloat(data_, ob->getExternalForce()[extNum]);
      }
    }

    data_ = set(data_, numExtTorque);
    for (auto *ob: world_->getObjList()) {
      for (size_t extNum = 0; extNum < ob->getExternalTorque().size(); extNum++) {
        data_ = setInFloat(data_, ob->getExternalTorquePosition()[extNum]);
        data_ = setInFloat(data_, ob->getExternalTorque()[extNum]);
      }
    }

    auto *contactList = world_->getContactProblem();
    int32_t contactIncrement = 0;

    auto contactSizeLocation = data_;
    data_ = set(data_, contactIncrement); // reserving space

    /// contact position
    for (auto *obj: world_->getObjList()) {
      for (auto &contact: obj->getContacts()) {
        if (!contact.isObjectA() && contact.getPairObjectBodyType()==BodyType::DYNAMIC)
          continue;

        contactIncrement++;
        // contact points
        data_ = setInFloat(data_, contact.getPosition());

        // contact forces
        auto impulseB = contact.getImpulse();
        auto contactFrame = contact.getContactFrame();

        Vec<3> impulseW;
        raisim::matTransposevecmul(contactFrame, impulseB, impulseW);
        impulseW /= world_->getTimeStep();
        data_ = setInFloat(data_, impulseW);
      }
    }
    set(contactSizeLocation, contactIncrement);

    // object information
    auto found = std::find_if(world_->getObjList().begin(), world_->getObjList().end(),
                              [this](const Object * ptr) { return ptr->visualTag == this->objectId_; });

    if (found != world_->getObjList().end()) {
      auto obSelected = *found;
      if (obSelected->getObjectType() == ObjectType::ARTICULATED_SYSTEM) {
        data_ = set(data_, int32_t(1));
        auto *as = reinterpret_cast<ArticulatedSystem *>(obSelected);
        data_ = set(data_, int32_t(as->getGeneralizedCoordinateDim()));
        data_ = set(data_, int32_t(as->getDOF()));
        data_ = set(data_, int32_t(as->getMovableJointNames().size()));
        data_ = set(data_, int32_t(as->getFrames().size()));

        for (int i = 0; i < as->getGeneralizedCoordinateDim(); i++)
          data_ = set(data_, float(as->getGeneralizedCoordinate()[i]));

        for (int i = 0; i < as->getDOF(); i++)
          data_ = set(data_, float(as->getGeneralizedVelocity()[i]));

        for (int i = 0; i < as->getMovableJointNames().size(); i++) {
          data_ = set(data_, as->getMovableJointNames()[i]);
          int j = i;
          if (as->getJointType(0) == Joint::Type::FIXED)
            j = i + 1;

          Vec<3> vec;
          data_ = set(data_, int(as->getJointType(j)));
          as->getPosition(j, {0, 0, 0}, vec);
          data_ = setInFloat(data_, vec);
          vec = as->getJointAxis(j);
          data_ = setInFloat(data_, vec);
        }

        Vec<3> pos;
        Vec<4> quat;
        Mat<3, 3> rot;
        for (int i = 0; i < as->getFrames().size(); i++) {
          data_ = set(data_, as->getFrames()[i].name);
          as->getFramePosition(i, pos);
          as->getFrameOrientation(i, rot);
          rotMatToQuat(rot, quat);
          data_ = setInFloat(data_, pos, quat);
        }

        // COM position
        if (as->getJointType(0) == Joint::FLOATING) {
          data_ = set(data_, int32_t(0));
          data_ = setInFloat(data_, as->getCOM());
        } else
          data_ = set(data_, int32_t(1));

      } else {
        data_ = set(data_, int32_t(0));
        auto *sb = reinterpret_cast<SingleBodyObject *>(obSelected);
        data_ = set(data_, int32_t(7));
        data_ = set(data_, int32_t(6));
        data_ = set(data_, int32_t(1));
        data_ = set(data_, int32_t(0));
        auto pos = sb->getPosition();
        auto quat = sb->getQuaternion();
        data_ = setInFloat(data_, pos, quat, sb->getLinearVelocity(), sb->getAngularVelocity());
        data_ = set(data_, std::string("ROOT"));
        data_ = set(data_, int(Joint::Type::FLOATING));
        data_ = setInFloat(data_, pos, pos);
        data_ = set(data_, int32_t(0));
        data_ = setInFloat(data_, pos);
      }
    } else {
      data_ = set(data_, int32_t(-1));
    }

    // charts
    data_ = set(data_, (int32_t) (charts_.size()));
    for (auto c: charts_) {
      bool initialized = c.second->visualTag != 0;
      if (!initialized) c.second->visualTag = visTagCounter++;

      data_ = set(data_, (int32_t) c.second->getType(), initialized, c.second->visualTag);
      if (!initialized)
        data_ = c.second->initialize(data_);

      data_ = c.second->serialize(data_);
    }
  }

  inline bool receiveData(int seconds) {
    using namespace server;
    int totalDataSize = RECEIVE_BUFFER_SIZE, totalReceivedDataSize = 0, currentReceivedDataSize;

    while (totalDataSize > totalReceivedDataSize) {
      if (waitForMessageFromClient(seconds)) {
        currentReceivedDataSize =
            recv(client_, &receive_buffer[0] + totalReceivedDataSize, RECEIVE_BUFFER_SIZE - totalReceivedDataSize, 0);
        if (currentReceivedDataSize == -1) return false;
      } else {
        RSWARN("Lost connection to the client. Trying to find a new client...")
        return false;
      }

      if (totalDataSize == RECEIVE_BUFFER_SIZE)
        rData_ = get(&receive_buffer[0], &totalDataSize);

      if (currentReceivedDataSize <= 0) return false;

      totalReceivedDataSize += currentReceivedDataSize;
    }

    return true;
  }

  inline bool sendData() {
    using namespace server;
    int dataSize = int(data_ - &send_buffer[0]);
    int totalSentBytes = 0;
    int currentlySentBytes = 0;
    set(&send_buffer[0], dataSize);

    while (dataSize > totalSentBytes)
      if (waitForMessageToClient(1)) {
        currentlySentBytes = send(client_, &send_buffer[0] + totalSentBytes, dataSize - totalSentBytes, 0);
        totalSentBytes += currentlySentBytes;
        if (currentlySentBytes == -1) return false;
      } else
        return false;

    return true;
  }

  inline bool updateSensorMeasurements() {
    using namespace server;
    std::lock_guard<std::mutex> guard(serverMutex_);
    ClientMessageType cMsgType;
    int nSensors;
    rData_ = get(rData_, &cMsgType, &nSensors);

    if (cMsgType != ClientMessageType::REQUEST_SENSOR_UPDATE) return false;

    auto &obList = world_->getObjList();

    for (int i = 0; i < nSensors; i++) {
      uint32_t visualTag;
      std::string name;
      Sensor::Type type;
      rData_ = get(rData_, &visualTag, &type, &name);
      ArticulatedSystem *as = dynamic_cast<ArticulatedSystem*>(*std::find_if(obList.begin(), obList.end(),
                                                                             [visualTag](const Object* i){ return i->visualTag == visualTag; }));
      auto sensor = as->getSensors()[name];

      if (type == Sensor::Type::RGB) {
        int width, height;
        rData_ = get(rData_, &width, &height);
        auto &img = std::static_pointer_cast<RGBCamera>(sensor)->getImageBuffer();
        RSFATAL_IF(width * height * 4 != img.size(), "Image size mismatch. Sensor module not working properly")
        rData_ = getN(rData_, img.data(), width * height * 4);
      } else if (type == Sensor::Type::DEPTH) {
        int width, height;
        rData_ = get(rData_, &width, &height);
        auto &depthArray = std::static_pointer_cast<DepthCamera>(sensor)->getDepthArray();
        RSFATAL_IF(width * height != depthArray.size(), "Image size mismatch. Sensor module not working properly")
        rData_ = getN(rData_, depthArray.data(), width * height);
      }

    }

    return true;
  }

  char *data_, *rData_;
  bool needsSensorUpdate_ = false;
  World *world_;
  std::vector<char> receive_buffer, send_buffer;
  bool connected_ = false;
  char tempBuffer[MAXIMUM_PACKET_SIZE];
  int state_ = STATUS_RENDERING;
  std::vector<ServerRequestType> serverRequest_;
  Object* toBeFocused_;
  std::string videoName_;
  uint32_t objectId_;
  std::atomic<bool> terminateRequested_ = {false};
  int client_;
  int server_fd_;
  sockaddr_in address;
  int addrlen;
  std::thread serverThread_;
  std::future<bool> threadResult_;
  std::string mapName_;

  std::mutex serverMutex_;
  std::atomic_bool tryingToLock_;

  int32_t visualConfiguration_ = 0;
  void updateVisualConfig() { visualConfiguration_++; }

  int raisimPort_ = 8080;
  Eigen::Vector3d position_, lookAt_;
  int screenShotWidth_, screenShotHeight_;

  // version
  constexpr static int version_ = 10016;

  // visual tag counter
  uint32_t visTagCounter = 30;

  // hanging object
  uint32_t hangingObjVisTag_ = 0;
  Object* interactingOb_;
  double wireStiffness_ = 0.;
  int hangingObjLocalId_ = -1;
  Vec<3> hangingObjPos_, hangingObjLocalPos_, hangingObjTargetPos_;

  std::unordered_map<std::string, Visuals *> visuals_;
  std::unordered_map<std::string, InstancedVisuals *> instancedvisuals_;
  std::unordered_map<std::string, PolyLine *> polyLines_;
  std::unordered_map<std::string, ArticulatedSystemVisual *> visualAs_;
  std::map<std::string, Chart *> charts_;

 public:
  /**
   * Only works with RaisimUnreal. Please read the "atlas" example to see how it works.
   * @param[in] title title of the chart
   * @param[in] names name of the data curves to be plotted
   * @param[in] xAxis title of the x-axis
   * @param[in] yAxis title of the y-axis
   * @return pointer to the created Time Series Graph */
  inline TimeSeriesGraph *addTimeSeriesGraph(std::string title,
                                             std::vector<std::string> names,
                                             std::string xAxis,
                                             std::string yAxis) {
    RSFATAL_IF(charts_.find(title) != charts_.end(), "A chart named " << title << "already exists")
    auto chart = new TimeSeriesGraph(std::ref(title), std::ref(names), std::ref(xAxis), std::ref(yAxis));
    charts_[title] = chart;
    return chart;
  }

  /**
    * Only works with RaisimUnreal. Please read the "atlas" example to see how it works.
    * @param[in] title title of the chart
    * @param[in] names name of the data histogram to be plotted
    * @return pointer to the created Bar Chart */
  inline BarChart *addBarChart(std::string title, std::vector<std::string> names) {
    RSFATAL_IF(charts_.find(title) != charts_.end(), "A chart named " << title << "already exists")
    auto chart = new BarChart(std::ref(title), std::ref(names));
    charts_[title] = chart;
    return chart;
  }
};

}  // namespace raisim

#endif  // RAISIM_RAISIMSERVER_HPP
