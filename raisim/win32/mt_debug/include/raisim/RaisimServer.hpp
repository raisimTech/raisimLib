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
#include "raisim/Sensors.hpp"

namespace raisim {

class RaisimServer final {
 public:
  static constexpr int SEND_BUFFER_SIZE = 33554432;
  static constexpr int MAXIMUM_PACKET_SIZE = 32384;
  static constexpr int FOOTER_SIZE = sizeof(char);
  static constexpr int RECEIVE_BUFFER_SIZE = 33554432;

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
    REQUEST_SERVER_STATUS,
    REQUEST_SENSOR_UPDATE
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

  ~RaisimServer() = default;

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
        client_ = int(accept(server_fd, NULL, NULL));
        connected_ = client_ != INVALID_SOCKET;
      }

      while (connected_) {
        if (terminateRequested_) {
          state_ = STATUS_TERMINATING;
          connected_ = false;
        }

        connected_ = processRequests();

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
    updateVisualConfig();
    return visualAs_[name];
  }

  /**
   * @param[in] as ArticulatedSystemVisual to be removed
   * remove a visualized articulated system */
  inline void removeVisualArticulatedSystem(ArticulatedSystemVisual* as) {
    auto it = visualAs_.begin();

    // Search for an element with value 2
    while(it != visualAs_.end()) {
      if(it->second == as)
        break;
      it++;
    }

    // Erase the element pointed by iterator it
    if (it != visualAs_.end())
      visualAs_.erase(it);

    delete as;
  }

  inline InstancedVisuals *addInstancedVisuals(const std::string &name,
                                               InstancedVisuals::VisualType type,
                                               const Vec<3>& size,
                                               const Vec<4>& color1,
                                               const Vec<4>& color2) {
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
    visuals_[name]->type = Visuals::VisualType::VisualSphere;
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
    visuals_[name]->type = Visuals::VisualType::VisualBox;
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
    visuals_[name]->type = Visuals::VisualType::VisualCylinder;
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
    visuals_[name]->type = Visuals::VisualType::VisualCapsule;
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
    visuals_[name] = new Visuals();
    visuals_[name]->type = Visuals::VisualType::VisualMesh;
    visuals_[name]->name = name;
    visuals_[name]->meshFileName = file;
    visuals_[name]->size = scale;
    visuals_[name]->color = {colorR, colorG, colorB, colorA};
    visuals_[name]->glow = glow;
    visuals_[name]->shadow = shadow;
    return visuals_[name];
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
    visuals_[name]->type = Visuals::VisualType::VisualArrow;
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


#if defined __linux__ || __APPLE__
  inline bool hasPendingData(int seconds) {
    struct pollfd fds[2];
    int ret;

    // 표준 입력에 대한 이벤트를 감시하기 위한 준비를 합니다.
    fds[0].fd = STDIN_FILENO;
    fds[0].events = POLLOUT;
    ret = poll(fds, 2, 10000);
    if ( ret == 0) {
      RSWARN("The client failed to respond in 20 seconds. Looking for a new client");
      return false;
    } else if( ret == -1 ) {
      RSWARN("The client error. Failed to communicate.");
      return false;
    }
    return true;
  }
#elif WIN32
  inline bool hasPendingData(int seconds) {
    fd_set fds ;
    int n ;
    struct timeval tv ;
    FD_ZERO(&fds) ;
    FD_SET(client_, &fds) ;
    tv.tv_sec = seconds ;
    tv.tv_usec = 10000 ;
    n = select ( server_fd+1, &fds, NULL, NULL, &tv ) ;
    if ( n == 0) {
      RSWARN("The client failed to respond in "<< seconds <<" seconds. Looking for a new client");
      return false;
    } else if( n == -1 ) {
      RSWARN("The client error. Failed to communicate.");
      return false;
    }
    return true;
  }
#endif

  inline bool receiveMessage() {
    int receivedData = 0;
    data_ = &send_buffer[0];
    int BytesRead = 0;
    char footer = 'c';
    int valread;

    while (footer == 'c') {
      receive_buffer[receivedData + MAXIMUM_PACKET_SIZE - FOOTER_SIZE] = 'c';

      valread = 0;
      while (valread < MAXIMUM_PACKET_SIZE) {
        if (!hasPendingData(20)) {
          RSWARN("Failed to recv from the server. Resetting")
          return false;
        }
        BytesRead = recv(client_, &receive_buffer[0] + receivedData, MAXIMUM_PACKET_SIZE, 0);
        if (BytesRead == 0) RSWARN("Client did not send any data")
        valread += BytesRead;
        receivedData += BytesRead;
      }

      footer = receive_buffer[receivedData - FOOTER_SIZE];
      receivedData -= FOOTER_SIZE;
    }

    if (footer != 'e') return false;
    return receivedData > 0;
  }

  inline bool processRequests() {
    using namespace server;
    ClientMessageType type;
    int recv_size;

    if (hasPendingData(10))
      recv_size = recv(client_, &receive_buffer[0], MAXIMUM_PACKET_SIZE, 0);
    else
      return false;

    if (recv_size <= 0)
      return false;

    int clientVersion;
    rData_ = get(&receive_buffer[0], &clientVersion);
    data_ = set(&send_buffer[0], version_);

    if (clientVersion == version_) {
      rData_ = get(rData_, &type, &objectId_);
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
            data_ = set(data_, position_, lookAt_);
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
          case REQUEST_INITIALIZE_VISUALS:
          case REQUEST_VISUAL_POSITION:
          case REQUEST_SERVER_STATUS:
            return false;

          default:
            break;
        }
      }
      unlockVisualizationServerMutex();
    } else {
      RSWARN("Version mismatch. Make sure you have the correct visualizer version")
      return false;
    }

    bool eom = false;
    char *startPtr = &send_buffer[0];
    while (!eom) {
      int sentBytes = 0;
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
      if (sentBytes <= 0) return false;
    }

    if (needsSensorUpdate_) {
      if (!receiveData())
        return false;

      updateSensorMeasurements();
      needsSensorUpdate_ = false;
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
        auto as = dynamic_cast<ArticulatedSystem *>(ob);
        data_ = set(data_, (uint64_t) (as->getVisOb().size() +
            as->getVisColOb().size()));
        data_ = set(data_, (uint64_t) as->getSensors().size());

        auto& visOb = as->getVisOb();
        for (uint64_t k = 0; k < visOb.size(); k++) {
          auto &vob = visOb[k];
          std::string name = std::to_string(ob->getIndexInWorld()) +
              "/" + std::to_string(0) + "/" +
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

        auto& colOb = as->getCollisionBodies();
        for (uint64_t k = 0; k < colOb.size(); k++) {
          auto &vob = colOb[k];
          std::string name = std::to_string(ob->getIndexInWorld()) +
              "/" + std::to_string(1) + "/" +
              std::to_string(k);
          data_ = set(data_, name);

          Vec<3> pos, offsetInWorld;
          Vec<4> quat;
          Mat<3, 3> bodyRotation, rot;
          ob->getPosition(vob.localIdx, pos);
          ob->getOrientation(vob.localIdx, bodyRotation);
          matvecmul(bodyRotation, vob.posOffset, offsetInWorld);
          matmul(bodyRotation, vob.rotOffset, rot);
          raisim::rotMatToQuat(rot, quat);
          pos = pos + offsetInWorld;
          data_ = set(data_, pos, quat);
        }

        // add sensors to be updated
        for (auto& sensor: as->getSensors()) {
          if (sensor.second->getUpdateTimeStamp() + 1. / sensor.second->getUpdateRate() < world_->getWorldTime()) {
            sensor.second->setUpdateTimeStamp(world_->getWorldTime());
            sensor.second->updatePose(*world_);
            Vec<4> quat;
            auto& pos = sensor.second->getPos();
            auto& rot = sensor.second->getRot();
            rotMatToQuat(rot, quat);
            data_ = set(data_, int(1));
            data_ = setInFloat(data_, pos, quat);
            needsSensorUpdate_ = true;
          } else {
            data_ = set(data_, int(0));
          }
        }
      } else if (ob->getObjectType() == ObjectType::COMPOUND) {
        auto *com = dynamic_cast<Compound *>(ob);
        data_ = set(data_, (uint64_t) (com->getObjList().size()));
        data_ = set(data_, (uint64_t) 0); // sensor size

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
        data_ = set(data_, (uint64_t) 0); // sensor size
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
    data_ = set(data_, (uint64_t) (visuals_.size()));
    data_ = set(data_, (uint64_t) (instancedvisuals_.size()));
    data_ = set(data_, (uint64_t) (visualAs_.size()));

    // single visuals
    for (auto &kAndVo : visuals_) {
      auto *vo = kAndVo.second;
      Vec<3> pos = vo->getPosition();
      Vec<4> quat = vo->getOrientation();
      data_ = set(data_, vo->name, pos, quat, vo->type, vo->color, vo->size);
    }

    // instanced visuals
    for (auto &iv: instancedvisuals_) {
      auto* v = iv.second;
      data_ = set(data_, (uint64_t) v->count(), v->name);
      for (size_t i=0; i < v->count(); i++) {
        data_ = setInFloat(data_, v->data[i].pos, v->data[i].quat, v->data[i].scale);
        data_ = set(data_, v->data[i].colorWeight);
      }
    }

    // ArticulatedSystemVisuals
    for (auto &vis : visualAs_) {
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
          Mat<3,3> bodyRotation, rot;

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
    data_ = set(data_, (uint64_t) (polyLines_.size()));
    for (auto &pl : polyLines_) {
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
    numExtForce = 0;
    for (auto *ob: world_->getObjList()) {
      for (size_t extNum = 0; extNum < ob->getExternalForce().size(); extNum++) {
        numExtForce++;
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
    data_ = set(data_, (uint64_t) (charts_.size()));
    for (auto c: charts_) {
      data_ = set(data_, int32_t(c.second->getType()));
      data_ = c.second->serialize(data_);
    }
  }

  inline bool receiveData() {
    int counter = 0;
    int receivedData = 0;

    while (counter++ < 1 && receivedData == 0) {
      int BytesRead = 0;
      char footer = 'c';
      int valread;

      while (footer == 'c') {
        receive_buffer[receivedData + MAXIMUM_PACKET_SIZE - FOOTER_SIZE] = 'c';
        valread = 0;
        while (valread < MAXIMUM_PACKET_SIZE) {
          if (hasPendingData(5))
            BytesRead = recv(client_, &receive_buffer[0] + receivedData, MAXIMUM_PACKET_SIZE, 0);
          else return false;

          if (BytesRead <= 0) return false;

          valread += BytesRead;
          receivedData += BytesRead;
        }
        footer = receive_buffer[receivedData - FOOTER_SIZE];
        receivedData -= FOOTER_SIZE;
      }
    }

    data_ = &receive_buffer[0];
    return true;
  }

  inline bool updateSensorMeasurements() {
    using namespace server;
    std::lock_guard<std::mutex> guard(serverMutex_);
    ClientMessageType cMsgType;
    int nASs;
    data_ = get(data_, &cMsgType, &nASs);

    if (cMsgType != ClientMessageType::REQUEST_SENSOR_UPDATE) return false;

    auto& obList = world_->getObjList();

    for (int i=0; i < nASs; i++) {
      int obIndex, nSensors;
      data_ = get(data_, &obIndex, &nSensors);

      auto* as = dynamic_cast<ArticulatedSystem*>(obList[obIndex]);

      RSFATAL_IF(as->getSensors().size() != nSensors, "updateSensorMeasurements: sensor size mismatch. This must be a bug. Please report")
      for (int j=0; j < nSensors; j++) {
        int needsUpdate;
        data_ = get(data_, &needsUpdate);
        if (needsUpdate == 0) continue;

        Sensor::Type type;
        std::string name;
        data_ = get(data_, &type, &name);
        auto sensor = as->getSensors()[name];

        if (type == Sensor::Type::RGB) {
          int width, height;
          data_ = get(data_, &width, &height);
          auto& img = std::static_pointer_cast<RGBCamera>(sensor)->getImageBuffer();
          RSFATAL_IF(width*height*4 != img.size(), "Image size mismatch. Sensor module not working properly")
          data_ = getN(data_, img.data(), width*height*4);
        } else if (type == Sensor::Type::DEPTH) {
          int width, height;
          data_ = get(data_, &width, &height);
          auto depthArray = std::static_pointer_cast<DepthCamera>(sensor)->getDepthArray();
          RSFATAL_IF(width * height != depthArray.size(), "Image size mismatch. Sensor module not working properly")
          data_ = getN(data_, depthArray.data(), width * height);
        }
      }
    }

    return true;
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
          auto as = dynamic_cast<ArticulatedSystem *>(ob);
          data_ = set(data_, as->getResourceDir());

          for (uint64_t i = 0; i < 2; i++) {
            std::vector<VisObject> *visOb;
            if (i == 0)
              visOb = &as->getVisOb();
            else
              visOb = &as->getVisColOb();

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

          // add sensors
          data_ = set(data_, (uint64_t) as->getSensors().size());
          for (auto& sensor: as->getSensors())
            data_ = sensor.second->serializeProp(data_);
        }
          break;

        case UNRECOGNIZED:
          break;
      }
    }

    // constraints
    data_ = set(data_, (uint64_t) (world_->getWires().size()));

    // charts
    data_ = set(data_, (uint64_t) (charts_.size()));
    for (auto c: charts_) {
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
    data_ = set(data_, (uint64_t) (visuals_.size()));
    data_ = set(data_, (uint64_t) (instancedvisuals_.size()));
    data_ = set(data_, (uint64_t) (visualAs_.size()));

    for (auto &kAndVo : visuals_) {
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

    for (auto &iv : instancedvisuals_) {
      auto v = iv.second;
      data_ = set(data_, v->name, v->type);
      data_ = setInFloat(data_, v->size, v->color1, v->color2);
    }

    for (auto &vas : visualAs_) {
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

  char *data_, *rData_;
  bool needsSensorUpdate_ = false;
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
  int screenShotWidth_, screenShotHeight_;

  // version
  constexpr static int version_ = 10007;

  std::unordered_map<std::string, Visuals *> visuals_;
  std::unordered_map<std::string, InstancedVisuals *> instancedvisuals_;
  std::unordered_map<std::string, PolyLine *> polyLines_;
  std::unordered_map<std::string, ArticulatedSystemVisual *> visualAs_;
  std::map<std::string, Chart *> charts_;

 public:
  inline TimeSeriesGraph* addTimeSeriesGraph(std::string title, std::vector<std::string> names, std::string xAxis, std::string yAxis) {
    RSFATAL_IF(charts_.find(title) != charts_.end(), "A chart named " << title << "already exists")
    auto chart = new TimeSeriesGraph(std::ref(title), std::ref(names), std::ref(xAxis), std::ref(yAxis));
    charts_[title] = chart;
    return chart;
  }

  inline BarChart* addBarChart(std::string title, std::vector<std::string> names) {
    RSFATAL_IF(charts_.find(title) != charts_.end(), "A chart named " << title << "already exists")
    auto chart = new BarChart(std::ref(title), std::ref(names));
    charts_[title] = chart;
    return chart;
  }
};

}  // namespace raisim

#endif  // RAISIM_RAISIMSERVER_HPP
