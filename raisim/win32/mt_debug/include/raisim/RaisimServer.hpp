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
#include "SerializationHelper.hpp"
#include "raisim/World.hpp"
#include "raisim/helper.hpp"
#include "raisim/object/ArticulatedSystem/JointAndBodies.hpp"

namespace raisim {


struct PolyLine {
  std::string name;
  Vec<4> color = {1, 1, 1, 1};
  std::vector<Vec<3>> points;
  double width = 0.01;

  /**
   * @param[in] r red value of the color (max=1).
   * @param[in] g green value of the color (max=1).
   * @param[in] b blue value of the color (max=1).
   * @param[in] a alpha value of the color (max=1).
   * set the color of the polyline. */
  void setColor(double r, double g, double b, double a) { color = {r, g, b, a}; }

  /**
   * @param[in] point new polyline point.
   * append a new point to the polyline. */
  void addPoint(const Eigen::Vector3d &point) { points.push_back(point); }

  /**
   * clear all polyline points. */
  void clearPoints() { points.clear(); }
};

struct ArticulatedSystemVisual {
  ArticulatedSystemVisual(const std::string &urdfFile) : obj (urdfFile) {
    color.setZero();
  }

  ~ArticulatedSystemVisual() = default;

  /**
   * @param[in] r red value (max=1)
   * @param[in] g green value (max=1)
   * @param[in] b blue value (max=1)
   * @param[in] a alpha value (max=1)
   * set color. if the alpha value is 0, it uses the original color defined in the mesh file */
  void setColor(double r, double g, double b, double a) {
    color = {r,g,b,a};
  }

  /**
   * @param[in] gc the generalized coordinate
   * set the configuration of the visualized articulated system */
  void setGeneralizedCoordinate(const Eigen::VectorXd& gc) {
    obj.setGeneralizedCoordinate(gc);
  }

  raisim::Vec<4> color;
  ArticulatedSystem obj;
};

struct Visuals {
  enum VisualType : int {
    VisualSphere = 0,
    VisualBox,
    VisualCylinder,
    VisualCapsule,
    VisualMesh,
    VisualArrow
  };

  VisualType type;
  std::string name;
  std::string material;
  std::string meshFileName;
  bool glow = true;
  bool shadow = false;

  // {r, g, b, a}
  Vec<4> color = {1, 1, 1, 1};

  /*
   * sphere     {radius, 0, 0}
   * box        {xlength, ylength, zlength}
   * cylinder   {radius, length, 0}
   * capsule    {radius, length, 0}
   * mesh       {xscale, yscale, zscale}
   */
  Vec<3> size = {0, 0, 0};

  /**
   * @param[in] radius the raidus of the sphere.
   * set size of the sphere. */
  void setSphereSize(double radius) { size[0] = radius; }

  /**
   * @param[in] x length.
   * @param[in] y width.
   * @param[in] z height.
   * set size of the box. */
  void setBoxSize(double x, double y, double z) { size = {x, y, z}; }

  /**
   * @param[in] radius the raidus of the cylinder.
   * @param[in] height the height of the cylinder.
   * set size of the cylinder. */
  void setCylinderSize(double radius, double height) { size = {radius, height, 1.}; }

  /**
   * @param[in] radius the raidus of the capsule.
   * @param[in] height the height of the capsule.
   * set size of the capsule. */
  void setCapsuleSize(double radius, double height) { size = {radius, height, 1.}; }

  /**
   * @param[in] x x coordinate of the visual object.
   * @param[in] y y coordinate of the visual object.
   * @param[in] z z coordinate of the visual object.
   * set the position of the visual object. */
  void setPosition(double x, double y, double z) { position = {x, y, z}; }

  /**
   * @param[in] w angle part of the quaternion.
   * @param[in] x scaled x coordinate of the rotation axis.
   * @param[in] y scaled y coordinate of the rotation axis.
   * @param[in] z scaled z coordinate of the rotation axis.
   * set the orientation of the visual object. */
  void setOrientation(double w, double x, double y, double z) { quaternion = {w, x, y, z}; }

  /**
   * @param[in] pos position of the visual object in Eigen::Vector3d.
   * set the position of the visual object. */
  void setPosition(const Eigen::Vector3d &pos) { position = pos; }

  /**
   * @param[in] ori quaternion of the visual object in Eigen::Vector4d.
   * set the orientation of the visual object. */
  void setOrientation(const Eigen::Vector4d &ori) { quaternion = ori; }

  /**
   * @param[in] r red value of the color (max=1).
   * @param[in] g green value of the color (max=1).
   * @param[in] b blue value of the color (max=1).
   * @param[in] a alpha value of the color (max=1).
   * set the color of the visual object. */
  void setColor(double r, double g, double b, double a) { color = {r, g, b, a}; }

  /**
   * @return the position of the visual object.
   * get the position of the visual object. */
  Eigen::Vector3d getPosition() { return position.e(); }

  /**
   * @return the orientation of the visual object.
   * get the orientation of the visual object. */
  Eigen::Vector4d getOrientation() { return quaternion.e(); }

 private:
  Vec<3> position = {0, 0, 0};
  Vec<4> quaternion = {1, 0, 0, 0};
};


class RaisimServer final {
 public:
  static constexpr int SEND_BUFFER_SIZE = 33554432;
  static constexpr int MAXIMUM_PACKET_SIZE = 1024;
  static constexpr int FOOTER_SIZE = sizeof(char);
  static constexpr int RECEIVE_BUFFER_SIZE = 1024;

  enum ClientMessageType : int {
    REQUEST_OBJECT_POSITION = 0,
    REQUEST_INITIALIZATION,
    REQUEST_RESOURCE,  // request mesh, texture. etc files
    REQUEST_CHANGE_REALTIME_FACTOR,
    REQUEST_CONTACT_SOLVER_DETAILS,
    REQUEST_PAUSE,
    REQUEST_RESUME,
    REQUEST_CONTACT_INFOS,
    REQUEST_CONFIG_XML,
    REQUEST_INITIALIZE_VISUALS,
    REQUEST_VISUAL_POSITION,
    REQUEST_SERVER_STATUS
  };

  enum ServerMessageType : int {
    INITIALIZATION = 0,
    OBJECT_POSITION_UPDATE = 1,
    STATUS = 2,
    NO_MESSAGE = 3,
    CONTACT_INFO_UPDATE = 4,
    CONFIG_XML = 5,
    VISUAL_INITILIZATION = 6,
    VISUAL_POSITION_UPDATE = 7
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

  enum Status : int {
    STATUS_RENDERING = 0,
    STATUS_HIBERNATING = 1,
    STATUS_TERMINATING = 2
  };

  /**
   * @param[in] world the world to visualize.
   * create a raisimSever for a world. */
  explicit RaisimServer(World *world) : world_(world) {
    receive_buffer.resize(RECEIVE_BUFFER_SIZE);
    send_buffer.resize(SEND_BUFFER_SIZE);
    memset(tempBuffer, 0, MAXIMUM_PACKET_SIZE * sizeof(char));
  }

  ~RaisimServer() {
//    for (auto &vis : _visualObjects)
//      delete vis.second;
//
//    for (auto &vis : _polyLines)
//      delete vis.second;
//
//    for (auto &vis : _visualArticulatedSystem)
//      delete vis.second;
  }

 private:
#if __linux__ || __APPLE__
  inline void loop() {
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    RSFATAL_IF((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0, "socket error: " << strerror(errno))
    RSFATAL_IF(setsockopt(server_fd, SOL_SOCKET, RAISIM_SERVER_SOCKET_OPTION,
                          (char *) &opt, sizeof(opt)), "setsockopt error: "<< strerror(errno))

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(raisimPort_);

    // Forcefully attaching socket to the port 8080
    RSFATAL_IF(bind(server_fd, (struct sockaddr *) &address, sizeof(address)) < 0, "bind error: " << strerror(errno))
    RSFATAL_IF(listen(server_fd, 3) < 0, "listen error: " << strerror(errno))

    while (!terminateRequested_) {
      if (waitForReadEvent(2.0)) {
        RSFATAL_IF((client_ = accept(server_fd, (struct sockaddr *) &address,
                                     (socklen_t *) &addrlen)) < 0, "accept failed")
        connected_ = true;
      }

      while (connected_) {
        if (terminateRequested_) {
          state_ = STATUS_TERMINATING;
          connected_ = false;
        }

        if (!processRequests())
          connected_ = false;

        if (state_ == STATUS_HIBERNATING)
          usleep(100000);
      }
    }
    close(server_fd);
    state_ = STATUS_RENDERING;
  }
#elif WIN32
  inline void loop() {
    WSADATA wsaData;
    int iResult;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    RSFATAL_IF(result != 0, "WSAStartup failed with error: " << iResult)

    // holds address info for socket to connect to
    struct addrinfo *ptr = NULL;

    int opt = 1000000;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    std::string portInString = std::to_string(raisimPort_);

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, portInString.c_str(), &hints, &result);
    if (iResult != 0) {
      printf("getaddrinfo failed with error: %d\n", iResult);
      WSACleanup();
      return;
    }

    // Create a SOCKET for connecting to server
    server_fd =
        int(socket(result->ai_family, result->ai_socktype, result->ai_protocol));
    if (server_fd == INVALID_SOCKET) {
      printf("socket failed with error: %ld\n", WSAGetLastError());
      WSACleanup();
      return;
    }

    setsockopt(server_fd, SOL_SOCKET, SO_SNDBUF, (char *)&opt, sizeof(opt));
    setsockopt(server_fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt, sizeof(opt));

    // Setup the TCP listening socket
    iResult = bind(server_fd, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
      printf("bind failed with error: %d\n", WSAGetLastError());
      closesocket(server_fd);
      WSACleanup();
      return;
    }

    iResult = listen(server_fd, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
      printf("listen failed with error: %d\n", WSAGetLastError());
      closesocket(server_fd);
      WSACleanup();
      return;
    }

    while (!terminateRequested_) {
      if (waitForReadEvent(2.0)) {
        RSFATAL_IF((client_ = int(accept(server_fd, NULL, NULL))) < 0,
                   "accept failed")
        connected_ = true;
      }

      std::chrono::steady_clock::time_point lastChecked, current;
      lastChecked = std::chrono::steady_clock::now();

      while (connected_) {
        if (terminateRequested_) {
          state_ = STATUS_TERMINATING;
          connected_ = false;
        }

        if (connected_ && !processRequests())
          connected_ = false;

        if (state_ == STATUS_HIBERNATING)
          std::this_thread::sleep_for(std::chrono::microseconds(100000));
      }
    }
    closesocket(server_fd);
    WSACleanup();
    state_ = STATUS_RENDERING;
  }
#endif

 public:
  /**
   * @param[in] port port number to stream
   * start spinning. */
  inline void launchServer(int port = 8080) {
    raisimPort_ = port;

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
    std::lock_guard<std::mutex> guard(serverMutex_);
    world_->integrate();
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
  inline void unlockVisualizationServerMutex() { serverMutex_.unlock(); }

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
    if (_visualArticulatedSystem.find(name) != _visualArticulatedSystem.end()) RSFATAL("Duplicated visual object name: " + name)
    _visualArticulatedSystem[name] = new ArticulatedSystemVisual(urdfFile);
    _visualArticulatedSystem[name]->color = raisim::Vec<4>{colorR, colorG, colorB, colorA};
    updateVisualConfig();
    return _visualArticulatedSystem[name];
  }

  /**
   * @param[in] as ArticulatedSystemVisual to be removed
   * remove a visualized articulated system */
  inline void removeVisualArticulatedSystem(ArticulatedSystemVisual* as) {
    auto it = _visualArticulatedSystem.begin();

    // Search for an element with value 2
    while(it != _visualArticulatedSystem.end()) {
      if(it->second == as)
        break;
      it++;
    }

    // Erase the element pointed by iterator it
    if (it != _visualArticulatedSystem.end())
      _visualArticulatedSystem.erase(it);

    delete as;
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
    if (_visualObjects.find(name) != _visualObjects.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();
    _visualObjects[name]->type = Visuals::VisualType::VisualSphere;
    _visualObjects[name]->name = name;
    _visualObjects[name]->size[0] = radius;
    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
    _visualObjects[name]->glow = glow;
    _visualObjects[name]->shadow = shadow;
    return _visualObjects[name];
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
    if (_visualObjects.find(name) != _visualObjects.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();
    _visualObjects[name]->type = Visuals::VisualType::VisualBox;
    _visualObjects[name]->name = name;
    _visualObjects[name]->size[0] = xLength;
    _visualObjects[name]->size[1] = yLength;
    _visualObjects[name]->size[2] = zLength;
    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
    _visualObjects[name]->glow = glow;
    _visualObjects[name]->shadow = shadow;
    return _visualObjects[name];
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
    if (_visualObjects.find(name) != _visualObjects.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();
    _visualObjects[name]->type = Visuals::VisualType::VisualCylinder;
    _visualObjects[name]->name = name;
    _visualObjects[name]->size[0] = radius;
    _visualObjects[name]->size[1] = length;
    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
    _visualObjects[name]->glow = glow;
    _visualObjects[name]->shadow = shadow;
    return _visualObjects[name];
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
    if (_visualObjects.find(name) != _visualObjects.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();
    _visualObjects[name]->type = Visuals::VisualType::VisualCapsule;
    _visualObjects[name]->name = name;
    _visualObjects[name]->size[0] = radius;
    _visualObjects[name]->size[1] = length;
    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
    _visualObjects[name]->glow = glow;
    _visualObjects[name]->shadow = shadow;
    return _visualObjects[name];
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
    if (_visualObjects.find(name) != _visualObjects.end()) RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();
    _visualObjects[name]->type = Visuals::VisualType::VisualMesh;
    _visualObjects[name]->name = name;
    _visualObjects[name]->meshFileName = file;
    _visualObjects[name]->size = scale;
    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
    _visualObjects[name]->glow = glow;
    _visualObjects[name]->shadow = shadow;
    return _visualObjects[name];
  }

// will be added soon
//  /**
//   * @param[in] name the name of the visual mesh object
//   * @param[in] radius radius of the arrow
//   * @param[in] height height of the arrow
//   * @param[in] colorR the red value of the color   (max=1)
//   * @param[in] colorG the green value of the color (max=1)
//   * @param[in] colorB the blue value of the color  (max=1)
//   * @param[in] colorA the alpha value of the color (max=1)
//   * @param[in] glow to glow or not (not supported)
//   * @param[in] shadow to cast shadow or not (not supported)
//   * @return the visual pointer
//   * add an arrow without physics */
//  inline Visuals *addVisualArrow(const std::string &name,
//                                 double radius, double height,
//                                 double colorR = 0, double colorG = 0,
//                                 double colorB = 0, double colorA = -1,
//                                 bool glow = false, bool shadow = false) {
//    if (_visualObjects.find(name) != _visualObjects.end()) RSFATAL("Duplicated visual object name: " + name)
//    updateVisualConfig();
//    _visualObjects[name] = new Visuals();
//    _visualObjects[name]->type = Visuals::VisualType::VisualArrow;
//    _visualObjects[name]->name = name;
//    _visualObjects[name]->size[0] = radius;
//    _visualObjects[name]->size[1] = height;
//    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
//    _visualObjects[name]->glow = glow;
//    _visualObjects[name]->shadow = shadow;
//    return _visualObjects[name];
//  }

  /**
   * @param[in] name the name of the polyline
   * @return the polyline pointer
   * add a polyline without physics */
  inline PolyLine *addVisualPolyLine(const std::string &name) {
    RSFATAL_IF(_polyLines.find(name) != _polyLines.end(), "Duplicated polyline object name: " + name)
    _polyLines[name] = new PolyLine();
    _polyLines[name]->name = name;
    return _polyLines[name];
  }

  /**
   * @param[in] name the name of the polyline
   * get visualized polyline */
  inline PolyLine *getVisualPolyLine(const std::string &name) {
    RSFATAL_IF(_polyLines.find(name) == _polyLines.end(), name + " doesn't exist")
    return _polyLines[name];
  }

  /**
   * @param[in] name the name of the visual articulated system
   * get visualized articulated system */
  inline ArticulatedSystemVisual *getVisualArticulatedSystem(const std::string &name) {
    RSFATAL_IF(_visualArticulatedSystem.find(name) == _visualArticulatedSystem.end(), name + " doesn't exist")
    return _visualArticulatedSystem[name];
  }

  /**
   * @param[in] name the name of the polyline to be removed
   * remove an existing polyline */
  inline void removeVisualPolyLine(const std::string &name) {
    if (_polyLines.find(name) == _polyLines.end()) RSFATAL("Visual polyline with name \"" + name + "\" doesn't exist.")
    updateVisualConfig();
    delete _polyLines[name];
    _polyLines.erase(name);
  }

  /**
   * @param[in] name the name of the visual object to be retrieved
   * @return visual object with a specified name
   * retrieve a visual object with a specified name */
  inline Visuals *getVisualObject(const std::string &name) {
    if (_visualObjects.find(name) == _visualObjects.end()) RSFATAL(
        "Visual object with name \"" + name + "\" doesn't exist.")
    return _visualObjects[name];
  }

  /**
   * @param[in] name the name of the visual object to be removed
   * remove an existing visual object */
  inline void removeVisualObject(const std::string &name) {
    if (_visualObjects.find(name) == _visualObjects.end()) RSFATAL(
        "Visual object with name \"" + name + "\" doesn't exist.")
    updateVisualConfig();
    delete _visualObjects[name];
    _visualObjects.erase(name);
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

  /** NOT WORKING YET
   * request for screenshot */
//  inline const std::vector<char>& getScreenShot() {
//    serverRequest_.push_back(ServerRequestType::GET_SCREEN_SHOT);
//    while(!screenShotReady_)
//      usleep(3000);
//
//    screenShotReady_ = false;
//    return screenShot_;
//  }

  /** NOT WORKING YET
   * change the screen size */
//  inline void setScreenSize(int width, int height) {
//    serverRequest_.push_back(ServerRequestType::SET_SCREEN_SIZE);
//    screenShotWidth_ = width;
//    screenShotHeight_ = height;
//  }

 private:
  inline bool waitForReadEvent(int timeout) {
    fd_set sdset;
    struct timeval tv;
    tv.tv_sec = timeout;
    tv.tv_usec = 0;
    FD_ZERO(&sdset);
    FD_SET(server_fd, &sdset);
    return select(server_fd + 1, &sdset, nullptr, nullptr, &tv) > 0;
  }

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
    focusedObjectName_ = obj->getName();
  }

  /**
   * @return if a client is connected to a server
   * check if a client is connected to a server */
  bool isConnected() const { return connected_; }

 private:
  inline bool hasPendingData(int seconds) {
    fd_set fds ;
    int n ;
    struct timeval tv ;
    FD_ZERO(&fds) ;
    FD_SET(client_, &fds) ;
    tv.tv_sec = 20 ;
    tv.tv_usec = 100000 ;
    n = select ( server_fd+1, &fds, NULL, NULL, &tv ) ;
    if ( n == 0) {
      RSWARN("The client failed to respond in 20 seconds. Looking for a new client");
      return false;
    } else if( n == -1 ) {
      RSWARN("The client error. Failed to communicate.");
      return false;
    }
    return true;
  }

  inline bool processRequests() {
    using namespace server;
    data_ = &send_buffer[0];
    ClientMessageType type;
    int recv_size = 0;

    if (hasPendingData(20))
      recv_size = recv(client_, &receive_buffer[0], MAXIMUM_PACKET_SIZE, 0);
    else
      return false;

    if (recv_size <= 0)
      return false;

    int clientVersion;
    auto data = get(&receive_buffer[0], &clientVersion);
    data_ = set(data_, version_);

    if (clientVersion == version_) {
      data = get(data, &type);
      data = get(data, &objectId_);
      int requestSize;
      data = get(data, &requestSize);
      if(requestSize == 1) {
        data = get(data, &screenShotWidth_);
        data = get(data, &screenShotHeight_);
        int dataSize = screenShotWidth_*screenShotHeight_*3;
        if(screenShot_.size() != dataSize) {
          screenShot_.resize(dataSize);
        }
        data = getN(data, &screenShot_[0], dataSize);
        screenShotReady_ = true;
      }

      data_ = set(data_, state_);

      // set server request
      data_ = set(data_, (uint64_t) serverRequest_.size());
      for (const auto &sr : serverRequest_) {
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
            data_ = setN(data_, position_.data(), 3);
            data_ = setN(data_, lookAt_.data(), 3);
            break;

          case ServerRequestType::FOCUS_ON_SPECIFIC_OBJECT:
            data_ = set(data_, focusedObjectName_);
            break;

          case ServerRequestType::SET_SCREEN_SIZE:
            data_ = set(data_, screenShotWidth_, screenShotHeight_);
            break;
        }
      }

      serverRequest_.clear();

      lockVisualizationServerMutex();

      if (state_ != Status::STATUS_HIBERNATING) {
        switch (type) {
          case REQUEST_OBJECT_POSITION:
            serializeWorld();
            break;

          case REQUEST_INITIALIZATION:
            serializeObjects();
            serializeVisuals();
            serializeWorld();
            break;

          case REQUEST_CHANGE_REALTIME_FACTOR:
            changeRealTimeFactor();
            break;

          case REQUEST_CONTACT_INFOS:
            serializeContacts();
            break;

          case REQUEST_CONFIG_XML:
            break;

          case REQUEST_INITIALIZE_VISUALS:
          case REQUEST_VISUAL_POSITION:
            break;

          case REQUEST_SERVER_STATUS:
            return false;

          default:
            break;
        }
      }
      unlockVisualizationServerMutex();
    } else {
      RSWARN("Version mismatch. Make sure you have the correct visualizer version")
    }

    bool eom = false;
    char *startPtr = &send_buffer[0];
    while (!eom) {
      int sentBytes;
      if (data_ - startPtr > MAXIMUM_PACKET_SIZE) {
        memcpy(&tempBuffer[0], startPtr, MAXIMUM_PACKET_SIZE - FOOTER_SIZE);
        tempBuffer[MAXIMUM_PACKET_SIZE - FOOTER_SIZE] = 'c';
        sentBytes = send(client_, &tempBuffer[0], MAXIMUM_PACKET_SIZE, 0);
        startPtr += MAXIMUM_PACKET_SIZE - FOOTER_SIZE;
      } else {
        memcpy(&tempBuffer[0], startPtr, data_ - startPtr);
        tempBuffer[MAXIMUM_PACKET_SIZE - FOOTER_SIZE] = 'e';
        sentBytes = send(client_, &tempBuffer[0], MAXIMUM_PACKET_SIZE, 0);
        eom = true;
      }
      if (sentBytes < 0) return false;
    }

    return state_ == STATUS_RENDERING || state_ == STATUS_HIBERNATING;
  }

  inline void serializeWorld() {
    using namespace server;
    auto &objList = world_->getObjList();
    data_ = set(data_, ServerMessageType::OBJECT_POSITION_UPDATE);
    data_ = set(data_, (uint64_t) world_->getConfigurationNumber() + visualConfiguration_);
    data_ = set(data_, (uint64_t) (objList.size()));

    for (auto *ob : objList) {
      // set gc
      if (ob->getObjectType() == ObjectType::ARTICULATED_SYSTEM) {
        data_ = set(data_, (uint64_t) (dynamic_cast<ArticulatedSystem *>(ob)->getVisOb().size() +
            dynamic_cast<ArticulatedSystem *>(ob)->getVisColOb().size()));

        for (uint64_t i = 0; i < 2; i++) {
          std::vector<VisObject> *visOb;
          if (i == 0)
            visOb = &(dynamic_cast<ArticulatedSystem *>(ob)->getVisOb());
          else
            visOb = &(dynamic_cast<ArticulatedSystem *>(ob)->getVisColOb());

          for (uint64_t k = 0; k < (*visOb).size(); k++) {
            auto &vob = (*visOb)[k];
            std::string name = std::to_string(ob->getIndexInWorld()) +
                "/" + std::to_string(i) + "/" +
                std::to_string(k);
            data_ = set(data_, name);

            Vec<3> pos, offsetInWorld;
            Vec<4> quat;
            Mat<3, 3> bodyRotation, rot;
            ob->getPosition(vob.localIdx, pos);
            ob->getOrientation(vob.localIdx, bodyRotation);
            matvecmul(bodyRotation, vob.offset, offsetInWorld);
            matmul(bodyRotation, vob.rot, rot);
            raisim::rotMatToQuat(rot, quat);
            pos = pos + offsetInWorld;
            data_ = set(data_, pos, quat);
          }
        }
      } else if (ob->getObjectType() == ObjectType::COMPOUND) {
        auto *com = dynamic_cast<Compound *>(ob);
        data_ = set(data_, (uint64_t) (com->getObjList().size()));
        for (uint64_t k = 0; k < com->getObjList().size(); k++) {
          std::string name = std::to_string(ob->getIndexInWorld()) + "/" + std::to_string(k);
          data_ = set(data_, name);
          Vec<3> pos, center;
          com->getPosition(center);
          matvecmul(com->getRotationMatrix(), com->getObjList()[k].trans.pos, pos);
          Mat<3, 3> rot;
          matmul(com->getRotationMatrix(), com->getObjList()[k].trans.rot, rot);
          Vec<4> quat;
          raisim::rotMatToQuat(rot, quat);
          pos = pos + center;
          data_ = set(data_, pos, quat);
        }
      } else {
        data_ = set(data_, (uint64_t) (1));
        Vec<3> pos;
        Vec<4> quat;
        std::string name = std::to_string(ob->getIndexInWorld());
        data_ = set(data_, name);
        dynamic_cast<SingleBodyObject *>(ob)->getPosition(pos);
        dynamic_cast<SingleBodyObject *>(ob)->getQuaternion(quat);
        data_ = set(data_, pos, quat);
      }
    }

    // visuals
    data_ = set(data_, (uint64_t) (_visualObjects.size() + _visualArticulatedSystem.size()));
    data_ = set(data_, (uint64_t) (_visualObjects.size()));
    data_ = set(data_, (uint64_t) (_visualArticulatedSystem.size()));

    for (auto &kAndVo : _visualObjects) {
      auto *vo = kAndVo.second;
      Vec<3> pos = vo->getPosition();
      Vec<4> quat = vo->getOrientation();
      data_ = set(data_, vo->name, pos, quat, vo->type, vo->color, vo->size);
    }

    // ArticulatedSystemVisuals
    for (auto &vis : _visualArticulatedSystem) {
      auto *ob = &vis.second->obj;
      data_ = set(data_, vis.second->color);
      data_ = set(data_, (uint64_t) ob->getVisOb().size() + ob->getVisColOb().size());

      for (uint64_t i = 0; i < 2; i++) {
        std::vector<VisObject> *visOb;
        if (i == 0)
          visOb = &(ob->getVisOb());
        else
          visOb = &(ob->getVisColOb());

        for (uint64_t k = 0; k < (*visOb).size(); k++) {
          auto &vob = (*visOb)[k];
          std::string name = vis.first + "/" + std::to_string(i) + "/" + std::to_string(k);
          data_ = set(data_, name);

          Vec<3> pos, offsetInWorld;
          Vec<4> quat;
          Mat<3, 3> bodyRotation, rot;

          ob->getPosition(vob.localIdx, pos);
          ob->getOrientation(vob.localIdx, bodyRotation);
          matvecmul(bodyRotation, vob.offset, offsetInWorld);
          matmul(bodyRotation, vob.rot, rot);
          pos = pos + offsetInWorld;
          raisim::rotMatToQuat(rot, quat);
          data_ = set(data_, pos);
          data_ = set(data_, quat);
        }
      }
    }

    // polylines
    data_ = set(data_, (uint64_t) (_polyLines.size()));
    for (auto &pl : _polyLines) {
      auto *ptr = pl.second;
      data_ = set(data_, ptr->name, ptr->color, ptr->width, (uint64_t) (ptr->points.size()));
      for (auto& p : ptr->points)
        data_ = set(data_, p);
    }

    // wires
    data_ = set(data_, (uint64_t) (world_->getWires().size()));

    for (auto &sw: world_->getWires())
      data_ = set(data_, sw->getP1(), sw->getP2());

    // External forces
    size_t numExtForce = 0;
    size_t numExtTorque = 0;

    for (auto *ob: world_->getObjList()) {
      numExtForce += ob->getExternalForce().size();
      numExtTorque += ob->getExternalTorque().size();
    }

    data_ = set(data_, (uint64_t) numExtForce);
    for (auto *ob: world_->getObjList()) {
      for (size_t extNum = 0; extNum < ob->getExternalForce().size(); extNum++) {
        data_ = set(data_, ob->getExternalForcePosition()[extNum]);
        data_ = set(data_, ob->getExternalForce()[extNum]);
      }
    }

    data_ = set(data_, (uint64_t) numExtTorque);
    for (auto *ob: world_->getObjList()) {
      for (size_t extNum = 0; extNum < ob->getExternalTorque().size(); extNum++) {
        data_ = set(data_, ob->getExternalTorquePosition()[extNum]);
        data_ = set(data_, ob->getExternalTorque()[extNum]);
      }
    }

    // object information
    if(objectId_ > -1) {
      RSFATAL_IF(objectId_ >= world_->getObjList().size(), "The client is requesting non-existent object")
      auto *obSelected = world_->getObjList()[objectId_];
      if(obSelected->getObjectType() == ObjectType::ARTICULATED_SYSTEM) {
        data_ = set(data_, int32_t(1));
        auto* as = reinterpret_cast<ArticulatedSystem*>(obSelected);
        data_ = set(data_, int32_t(as->getGeneralizedCoordinateDim()));
        data_ = set(data_, int32_t(as->getDOF()));
        data_ = set(data_, int32_t(as->getMovableJointNames().size()));
        data_ = set(data_, int32_t(as->getFrames().size()));

        for(int i=0; i<as->getGeneralizedCoordinateDim(); i++)
          data_ = set(data_, float(as->getGeneralizedCoordinate()[i]));

        for(int i=0; i<as->getDOF(); i++)
          data_ = set(data_, float(as->getGeneralizedVelocity()[i]));

        for(int i=0; i<as->getMovableJointNames().size(); i++) {
          data_ = set(data_, as->getMovableJointNames()[i]);
          int j = i;
          if(as->getJointType(0) == Joint::Type::FIXED)
            j = i + 1;

          Vec<3> vec;
          data_ = set(data_, int(as->getJointType(j)));
          as->getPosition(j, {0,0,0}, vec);
          data_ = setInFloat(data_, vec);
          vec = as->getJointAxis(j);
          data_ = setInFloat(data_, vec);
        }

        Vec<3> pos;
        Vec<4> quat;
        Mat<3,3> rot;
        for(int i=0; i<as->getFrames().size(); i++) {
          data_ = set(data_, as->getFrames()[i].name);
          as->getFramePosition(i, pos);
          as->getFrameOrientation(i, rot);
          rotMatToQuat(rot, quat);
          data_ = setInFloat(data_, pos, quat);
        }

        // COM position
        if(as->getJointType(0) == Joint::FLOATING) {
          data_ = set(data_, int32_t(0));
          data_ = setInFloat(data_, as->getCOM());
        } else
          data_ = set(data_, int32_t(1));

      } else {
        data_ = set(data_, int32_t(0));
        auto* sb = reinterpret_cast<SingleBodyObject*>(obSelected);
        auto pos = sb->getPosition();
        auto quat = sb->getQuaternion();
        data_ = setInFloat(data_, pos, quat, sb->getLinearVelocity(), sb->getAngularVelocity());
      }
    } else {
      data_ = set(data_, int32_t(-1));
    }

    // charts
    data_ = set(data_, (uint64_t) (_charts.size()));
    for (auto c: _charts) {
      data_ = set(data_, int32_t(c.second->getType()));
      data_ = c.second->serialize(data_);
    }
  }

  inline void serializeObjects() {
    using namespace server;
    auto &objList = world_->getObjList();

    data_ = set(data_, ServerMessageType::INITIALIZATION);
    data_ = set(data_, (uint64_t) world_->getConfigurationNumber() + visualConfiguration_);
    data_ = set(data_, (uint64_t) (objList.size()));

    for (auto *ob : objList) {
      // set name length
      data_ = set(data_, (uint64_t) ob->getIndexInWorld(), ob->getObjectType(), ob->getName());

      switch (ob->getObjectType()) {
        case SPHERE:
          data_ = set(data_, dynamic_cast<SingleBodyObject *>(ob)->getAppearance());
          data_ = set(data_, float(dynamic_cast<Sphere *>(ob)->getRadius()));
          break;

        case BOX:
          data_ = set(data_, dynamic_cast<SingleBodyObject *>(ob)->getAppearance());
          for (int i = 0; i < 3; i++)
            data_ = set(data_, float(dynamic_cast<Box *>(ob)->getDim()[i]));
          break;

        case CYLINDER:
          data_ = set(data_, dynamic_cast<SingleBodyObject *>(ob)->getAppearance());
          data_ = set(data_, float(dynamic_cast<Cylinder *>(ob)->getRadius()));
          data_ = set(data_, float(dynamic_cast<Cylinder *>(ob)->getHeight()));
          break;

        case CONE:
          break;

        case CAPSULE:
          data_ = set(data_, dynamic_cast<SingleBodyObject *>(ob)->getAppearance());
          data_ = set(data_, float(dynamic_cast<Capsule *>(ob)->getRadius()));
          data_ = set(data_, float(dynamic_cast<Capsule *>(ob)->getHeight()));
          break;

        case HALFSPACE:
          data_ = set(data_, dynamic_cast<SingleBodyObject *>(ob)->getAppearance());
          data_ = set(data_, float(dynamic_cast<Ground *>(ob)->getHeight()));
          break;

        case COMPOUND:
          data_ = set(data_, dynamic_cast<SingleBodyObject *>(ob)->getAppearance());
          data_ = set(data_, (uint64_t) (dynamic_cast<Compound *>(ob)->getObjList().size()));

          for (auto &vob : dynamic_cast<Compound *>(ob)->getObjList()) {
            data_ = set(data_, vob.objectType);
            data_ = set(data_, vob.appearance);

            switch (vob.objectType) {
              case BOX:
                data_ = set(data_, vob.objectParam[0], vob.objectParam[1], vob.objectParam[2]);
                break;
              case CAPSULE:
              case CYLINDER:
                data_ = set(data_, vob.objectParam[0], vob.objectParam[1]);
                break;
              case SPHERE:
                data_ = set(data_, vob.objectParam[0]);
                break;

              default:
                break;
            }
          }

          break;

        case MESH:
          data_ = set(data_, dynamic_cast<Mesh *>(ob)->getAppearance());
          data_ = set(data_, dynamic_cast<Mesh *>(ob)->getMeshFileName());
          data_ = set(data_, float(dynamic_cast<Mesh *>(ob)->getScale()));
          break;

        case HEIGHTMAP:
        {
          auto hm = dynamic_cast<HeightMap *>(ob);
          data_ = set(data_, hm->getAppearance());
          data_ = setInFloat(data_, hm->getCenterX(), hm->getCenterY(), hm->getXSize(), hm->getYSize());
          data_ = set(data_, (uint64_t) hm->getXSamples(), (uint64_t) hm->getYSamples());
          data_ = setInFloat(data_, hm->getHeightVector());
        }
          break;

        case ARTICULATED_SYSTEM:
        {
          std::string resDir =
              dynamic_cast<ArticulatedSystem *>(ob)->getResourceDir();
          data_ = set(data_, resDir);

          for (uint64_t i = 0; i < 2; i++) {
            std::vector<VisObject> *visOb;
            if (i == 0)
              visOb = &(dynamic_cast<ArticulatedSystem *>(ob)->getVisOb());
            else
              visOb = &(dynamic_cast<ArticulatedSystem *>(ob)->getVisColOb());

            data_ = set(data_, (uint64_t) (visOb->size()));

            for (auto &vob : *visOb) {
              data_ = set(data_, vob.shape, vob.material, vob.color, i);
              if (vob.shape == Shape::Mesh) {
                data_ = set(data_, vob.fileName, vob.scale);
              } else {
                data_ = set(data_, (uint64_t) (vob.visShapeParam.size()));
                for (auto vparam : vob.visShapeParam)
                  data_ = set(data_, vparam);
              }
            }
          }
        }
          break;

        case UNRECOGNIZED:
          break;
      }
    }

    // constraints
    data_ = set(data_, (uint64_t) (world_->getWires().size()));

    // charts
    data_ = set(data_, (uint64_t) (_charts.size()));
    for (auto c: _charts) {
      data_ = set(data_, int32_t(c.second->getType()));
      data_ = c.second->initialize(data_);
    }
  }

  inline void serializeContacts() {
    using namespace server;
    auto *contactList = world_->getContactProblem();

    // set message type
    data_ = set(data_, ServerMessageType::CONTACT_INFO_UPDATE);

    // Data begins
    size_t contactSize = contactList->size();
    for (const auto &con : *contactList)
      if (con.rank == 5) contactSize--;

    size_t contactIncrement = 0;
    auto contactSizeLocation = data_;

    data_ = set(data_, (uint64_t) (contactSize));

    for (auto *obj : world_->getObjList()) {
      for (auto &contact : obj->getContacts()) {
        if (!contact.isObjectA() &&
            contact.getPairObjectBodyType() != raisim::BodyType::STATIC)
          continue;
        contactIncrement++;
        // contact points
        auto contactPos = contact.getPosition();
        data_ = set(data_, contactPos);

        // contact forces
        auto impulseB = contact.getImpulse();
        auto contactFrame = contact.getContactFrame();

        Vec<3> impulseW;
        raisim::matTransposevecmul(contactFrame, impulseB, impulseW);
        impulseW /= world_->getTimeStep();
        data_ = set(data_, impulseW);
      }
    }
    set(contactSizeLocation, contactIncrement);
  }

  inline void serializeVisuals() {
    using namespace server;
    data_ = set(data_, (uint64_t) (_visualObjects.size() + _visualArticulatedSystem.size()));
    data_ = set(data_, (uint64_t) (_visualObjects.size()));
    data_ = set(data_, (uint64_t) (_visualArticulatedSystem.size()));

    for (auto &kAndVo : _visualObjects) {
      auto &vo = kAndVo.second;
      data_ = set(data_, vo->type, vo->name, vo->color, vo->material, vo->glow, vo->shadow);

      switch (vo->type) {
        case Visuals::VisualSphere:
          data_ = set(data_, (float) vo->size[0]);
          break;

        case Visuals::VisualBox:
          data_ = setInFloat(data_, vo->size);
          break;

        case Visuals::VisualMesh:
          data_ = setInFloat(data_, vo->size);
          data_ = set(data_, vo->meshFileName);
          break;

        case Visuals::VisualCylinder:
        case Visuals::VisualCapsule:
        case Visuals::VisualArrow:
          data_ = set(data_, (float) vo->size[0], (float) vo->size[1]);
          break;

        default:
          break;
      }
    }

    for (auto &vas : _visualArticulatedSystem) {
      auto *ob = &vas.second->obj;
      data_ = set(data_, vas.first);
      std::string resDir = static_cast<ArticulatedSystem *>(ob)->getResourceDir();
      data_ = set(data_, resDir);

      for (uint64_t i = 0; i < 2; i++) {
        std::vector<VisObject> *visOb = i==0 ? &(ob->getVisOb()) : &(ob->getVisColOb());
        data_ = set(data_, (uint64_t) (visOb->size()));

        for (auto &vob : *visOb) {
          data_ = set(data_, vob.shape, vob.material, vas.second->color, i);
          if (vob.shape == Shape::Mesh) {
            data_ = set(data_, vob.fileName, vob.scale);
          } else {
            data_ = set(data_, (uint64_t) (vob.visShapeParam.size()));
            for (auto vparam : vob.visShapeParam)
              data_ = set(data_, vparam);
          }
        }
      }
    }
  }

  inline void changeRealTimeFactor() {
    using namespace server;
    get(&receive_buffer[0], &realTimeFactor);
    data_ = set(data_, ServerMessageType::NO_MESSAGE);
  }

  char *data_;
  World *world_;
  std::vector<char> receive_buffer, send_buffer;
  bool connected_ = false;
  char tempBuffer[MAXIMUM_PACKET_SIZE];
  int state_ = STATUS_RENDERING;
  std::vector<ServerRequestType> serverRequest_;
  std::string focusedObjectName_;
  std::string videoName_;
  int objectId_;
  double realTimeFactor = 1.0;
  std::atomic<bool> terminateRequested_ = {false};

  int client_;
  int server_fd;
  sockaddr_in address;
  int addrlen;
  std::thread serverThread_;
  std::future<bool> threadResult_;

  std::mutex serverMutex_;

  uint64_t visualConfiguration_ = 0;
  void updateVisualConfig() { visualConfiguration_++; }

  int raisimPort_ = 8080;
  Eigen::Vector3d position_, lookAt_;

  std::vector<char> screenShot_;
  int screenShotWidth_, screenShotHeight_;
  bool screenShotReady_ = false;

  // version
  constexpr static int version_ = 10006;

  class Chart {
   public:
    virtual char* initialize(char* data) = 0;
    virtual char* serialize(char* data) = 0;

   protected:
    std::string title_;
    enum class Type : int32_t {
      TIME_SERIES = 0,
      BAR_CHART
    } type_;

   public:
    Type getType() { return type_; }
  };

  class TimeSeriesGraph : public Chart {
   public:
    TimeSeriesGraph(std::string title, std::vector<std::string> names, std::string xAxis, std::string yAxis) :
        size_(names.size()), xAxis_(std::move(xAxis)), yAxis_(std::move(yAxis)), names_(std::move(names)) {
      title_ = std::move(title);
      type_ = Type::TIME_SERIES;
    }

    void addDataPoints(double time, const raisim::VecDyn& d) {
      RSFATAL_IF(size_ != d.n, "Dimension mismatch. The chart has " << size_ << " categories and the inserted data is " << d.n << "dimension");
      {
        std::lock_guard<std::mutex> guard(mtx_);
        timeStamp_.push(time);
        data_.push(d);

        /// keep the data buffer below than maximum allowed
        if (timeStamp_.size() > BUFFER_SIZE) {
          timeStamp_.pop();
          data_.pop();
        }
      }
    }

    // not for users //
    void clearData() {
      timeStamp_ = std::queue<double>();
      data_ = std::queue<raisim::VecDyn>();
    }

    char* initialize(char* data) final {
      using namespace server;
      return set(data, title_, names_, xAxis_, yAxis_);
    }

    char* serialize(char* data) final {
      using namespace server;
      std::lock_guard<std::mutex> guard(mtx_);
      data = set(data, (uint64_t)timeStamp_.size( ));
      while (!timeStamp_.empty()) {
        data = set(data, timeStamp_.front());
        timeStamp_.pop();
      }
      data = set(data, (uint64_t)data_.size( ));
      while (!data_.empty()) {
        data = set(data, data_.front());
        data_.pop();
      }
      clearData();
      return data;
    }

    [[nodiscard]] uint64_t size() const { return size_; }

   private:
    uint64_t size_;
    std::mutex mtx_;
    std::string xAxis_, yAxis_;
    std::vector<std::string> names_;
    std::queue<raisim::VecDyn> data_;
    std::queue<double> timeStamp_;
    constexpr static int BUFFER_SIZE = 500;
  };

  class BarChart : public Chart {
   public:
    BarChart(std::string title, std::vector<std::string> names) :
        size_(names.size()), names_(std::move(names)) {
      title_ = std::move(title);
      type_ = Type::BAR_CHART;
    }

    void setData(const std::vector<float>& data) {
      RSFATAL_IF(size_ != data.size(), "Dimension mismatch. The chart has " << size_ << " categories and the inserted data is " << data.size() << "dimension");
      {
        std::lock_guard<std::mutex> guard(mtx_);
        data_ = data;
      }
    }

    char* initialize(char* data) final {
      using namespace server;
      return set(data, title_, names_);
    }

    char* serialize(char* data) final {
      using namespace server;
      std::lock_guard<std::mutex> guard(mtx_);
      return set(data, data_);
    }

   private:
    uint64_t size_;
    std::mutex mtx_;
    std::vector<std::string> names_;
    std::vector<float> data_;

  };

  std::unordered_map<std::string, Visuals *> _visualObjects;
  std::unordered_map<std::string, PolyLine *> _polyLines;
  std::unordered_map<std::string, ArticulatedSystemVisual *> _visualArticulatedSystem;
  std::map<std::string, Chart *> _charts;

 public:
  inline TimeSeriesGraph* addTimeSeriesGraph(std::string title, std::vector<std::string> names, std::string xAxis, std::string yAxis) {
    RSFATAL_IF(_charts.find(title) != _charts.end(), "A chart named "<<title<< "already exists")
    auto chart = new TimeSeriesGraph(std::ref(title), std::ref(names), std::ref(xAxis), std::ref(yAxis));
    _charts[title] = chart;
    return chart;
  }

  inline BarChart* addBarChart(std::string title, std::vector<std::string> names) {
    RSFATAL_IF(_charts.find(title) != _charts.end(), "A chart named "<<title<< "already exists")
    auto chart = new BarChart(std::ref(title), std::ref(names));
    _charts[title] = chart;
    return chart;
  }

};

}  // namespace raisim

#endif  // RAISIM_RAISIMSERVER_HPP
