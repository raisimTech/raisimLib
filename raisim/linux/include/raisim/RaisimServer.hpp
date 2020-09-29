//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_RAISIMSERVER_HPP
#define RAISIM_RAISIMSERVER_HPP
#include <chrono>

#ifdef __linux__
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#define RAISIM_SERVER_SOCKET_OPTION SO_REUSEADDR | SO_REUSEPORT
#elif WIN32
#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#define RAISIM_SERVER_SOCKET_OPTION SO_REUSEADDR | SO_BROADCAST
#endif

#include <tinyxml_rai/tinyxml_rai.h>

#include <atomic>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>

#include "raisim/World.hpp"
#include "raisim/helper.hpp"
#include "raisim/object/ArticulatedSystem/JointAndBodies.hpp"

namespace raisim {

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

  void setPosition(double x, double y, double z) { position = {x, y, z}; }
  void setOrientation(double w, double x, double y, double z) { quaternion = {w, x, y, z}; }
  Vec<3> &getPosition() { return position; }
  Vec<4> &getOrientation() { return quaternion; }

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
    REQUEST_SERVER_STATUS,
  };

  enum ServerMessageType : int {
    INITIALIZATION = 0,
    OBJECT_POSITION_UPDATE = 1,
    STATUS = 2,
    NO_MESSAGE = 3,
    CONTACT_INFO_UPDATE = 4,
    CONFIG_XML = 5,
    VISUAL_INITILIZATION = 6,
    VISUAL_POSITION_UPDATE = 7,
  };

  enum class ServerRequestType : int {
    NO_REQUEST = 0,
    START_RECORD_VIDEO = 1,
    STOP_RECORD_VIDEO = 2,
    FOCUS_ON_SPECIFIC_OBJECT = 3,
    SET_CAMERA_TO = 4
  };

  enum Status : int {
    STATUS_RENDERING = 0,
    STATUS_HIBERNATING = 1,
    STATUS_TERMINATING = 2
  };

  explicit RaisimServer(World *world) : world_(world) {
    receive_buffer.resize(RECEIVE_BUFFER_SIZE);
    send_buffer.resize(SEND_BUFFER_SIZE);
  }

#if __linux__
  inline void loop() {
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    RSFATAL_IF((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0, "socket failed")
    RSFATAL_IF(setsockopt(server_fd, SOL_SOCKET, RAISIM_SERVER_SOCKET_OPTION,
                          (char *)&opt, sizeof(opt)), "setsockopt")

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(raisimPort_);

    // Forcefully attaching socket to the port 8080
    RSFATAL_IF(bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0, "bind failed")
    RSFATAL_IF(listen(server_fd, 3) < 0, "listen failed")

    while (!terminateRequested_) {
      if (waitForReadEvent(2.0)) {
        RSFATAL_IF((client_ = accept(server_fd, (struct sockaddr *)&address,
                                     (socklen_t *)&addrlen)) < 0, "accept failed")
        connected_ = true;
      }

      std::chrono::steady_clock::time_point lastChecked, current;
      lastChecked = std::chrono::steady_clock::now();

      while (connected_) {
        if (terminateRequested_) {
          state_ = STATUS_TERMINATING;
          connected_ = false;
        }

        if (processRequests()) {
          lastChecked = std::chrono::steady_clock::now();
        } else {
          current = std::chrono::steady_clock::now();
          std::cout<<"disconnected for "<<std::chrono::duration_cast<std::chrono::seconds>(current - lastChecked).count()<<std::endl;

          if (std::chrono::duration_cast<std::chrono::seconds>(current - lastChecked).count() > 1.0) {
            connected_ = false;
          }
        }

        if(state_==STATUS_HIBERNATING)
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
        socket(result->ai_family, result->ai_socktype, result->ai_protocol);
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
        RSFATAL_IF((client_ = accept(server_fd, NULL, NULL)) < 0,
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

        if (processRequests()) {
          lastChecked = std::chrono::steady_clock::now();
        } else {
          current = std::chrono::steady_clock::now();

          if (std::chrono::duration_cast<std::chrono::seconds>(current -
                                                               lastChecked)
                  .count() > 1.0) {
            connected_ = false;
          }
        }

        if (state_ == STATUS_HIBERNATING)
          std::this_thread::sleep_for(std::chrono::microseconds(100000));
      }
    }
    closesocket(server_fd);
    WSACleanup();
    state_ = STATUS_RENDERING;
  }
#endif

  template <typename T>
  static inline char *set(char *data, T *val) {
    memcpy(data, val, sizeof(T));
    data += sizeof(T);
    return data;
  }

  template <typename T>
  static inline char *setN(char *data, T *val, int64_t N) {
    memcpy(data, val, sizeof(T) * N);
    data += sizeof(T) * N;
    return data;
  }

  template <typename T>
  static inline char *set(char *data, T val) {
    T temp = val;
    memcpy(data, &temp, sizeof(T));
    data += sizeof(T);
    return data;
  }

  template <typename T>
  static inline char *get(char *data, T *val) {
    memcpy(val, data, sizeof(T));
    data += sizeof(T);
    return data;
  }

  template <typename T>
  static inline char *getN(char *data, T *val, uint64_t N) {
    memcpy(val, data, sizeof(T) * N);
    data += sizeof(T) * N;
    return data;
  }

  static inline char *getString(char *data, std::string &str) {
    uint64_t size;
    data = get(data, &size);
    str.resize(size);
    data = getN(data, const_cast<char *>(str.c_str()), size);
    return data;
  }

  static inline char *setString(char *data, const std::string &val) {
    data = set(data, (uint64_t)(val.size()));
    memcpy(data, val.data(), sizeof(char) * val.size());
    data += sizeof(char) * val.size();
    return data;
  }

  template <typename T>
  static inline char *setStdVector(char *data, const std::vector<T> &str) {
    data = set(data, int64_t(str.size()));
    data = setN(data, str.data(), uint64_t(str.size()));
    return data;
  }

  template <typename T>
  static inline char *getStdVector(char *data, std::vector<T> &str) {
    uint64_t size;
    data = get(data, &size);
    str.resize(size);
    data = getN(data, str.data(), (uint64_t)(str.size()));
    return data;
  }

  inline void launchServer(int port = 8080) {
    raisimPort_ = port;
    serverThread_ = std::thread(&raisim::RaisimServer::loop, this);
  }

  inline void integrateWorldThreadSafe() {
    // std::lock_guard<std::mutex> guard(serverMutex_);
    world_->integrate();
    updateReady_ = true;
  }

  inline void hibernate() { std::lock_guard<std::mutex> guard(serverMutex_); state_ = STATUS_HIBERNATING; }

  inline void wakeup() { std::lock_guard<std::mutex> guard(serverMutex_); state_ = STATUS_RENDERING; }

  inline void killServer() {
    terminateRequested_ = true;
    if(serverThread_.joinable())
      serverThread_.join();
    terminateRequested_ = false;
  }

  inline void informClientForUpdate() { updateReady_ = true; }

  inline void lockVisualizationServerMutex() { serverMutex_.lock(); }

  inline void unlockVisualizationServerMutex() { serverMutex_.unlock(); }

  inline bool isPauseRequested() { return pauseRequested_; }

  inline bool isResumeRuested() { return resumeRequested_; }

  inline bool isTerminateRequested() { return terminateRequested_; }

  inline Visuals *addVisualSphere(const std::string name, double radius,
                                  double colorR = 1, double colorG = 1,
                                  double colorB = 1, double colorA = 1,
                                  const std::string &material = "",
                                  bool glow = true, bool shadow = false) {
    if (_visualObjects.find(name) != _visualObjects.end())
    RSFATAL("Duplicated visual object name: " + name)
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

  inline Visuals *addVisualBox(const std::string name, double xLength,
                               double yLength, double zLength,
                               double colorR = 1, double colorG = 1,
                               double colorB = 1, double colorA = 1,
                               const std::string &material = "",
                               bool glow = true, bool shadow = false) {
    if (_visualObjects.find(name) != _visualObjects.end())
    RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();;
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

  inline Visuals *addVisualCylinder(const std::string name, double radius,
                                    double length, double colorR = 1,
                                    double colorG = 1, double colorB = 1,
                                    double colorA = 1,
                                    const std::string &material = "",
                                    bool glow = true, bool shadow = false) {
    if (_visualObjects.find(name) != _visualObjects.end())
    RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();;
    _visualObjects[name]->type = Visuals::VisualType::VisualCylinder;
    _visualObjects[name]->name = name;
    _visualObjects[name]->size[0] = radius;
    _visualObjects[name]->size[1] = length;
    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
    _visualObjects[name]->glow = glow;
    _visualObjects[name]->shadow = shadow;
    return _visualObjects[name];
  }

  inline Visuals *addVisualCapsule(const std::string name, double radius,
                                   double length, double colorR = 1,
                                   double colorG = 1, double colorB = 1,
                                   double colorA = 1,
                                   const std::string &material = "",
                                   bool glow = true, bool shadow = false) {
    if (_visualObjects.find(name) != _visualObjects.end())
    RSFATAL("Duplicated visual object name: " + name)
    updateVisualConfig();
    _visualObjects[name] = new Visuals();;
    _visualObjects[name]->type = Visuals::VisualType::VisualCapsule;
    _visualObjects[name]->name = name;
    _visualObjects[name]->size[0] = radius;
    _visualObjects[name]->size[1] = length;
    _visualObjects[name]->color = {colorR, colorG, colorB, colorA};
    _visualObjects[name]->glow = glow;
    _visualObjects[name]->shadow = shadow;
    return _visualObjects[name];
  }

  inline Visuals *getVisualObject(std::string name) {
    if (_visualObjects.find(name) == _visualObjects.end())
    RSFATAL("Visual object with name \"" + name + "\" doesn't exist.")
    return _visualObjects[name];
  }

  //  void addVisualMesh() {
  //    _visualObjects.emplace(name, new VisualObject);
  //  }

  inline void removeVisualObject(std::string name) {
    if (_visualObjects.find(name) == _visualObjects.end())
    RSFATAL("Visual object with name \"" + name + "\" doesn't exist.")
    updateVisualConfig();
    _visualObjects.erase(name);
  }


  inline void startRecordingVideo(const std::string& videoName) {
    serverRequest_.push_back(ServerRequestType::START_RECORD_VIDEO);
    videoName_ = videoName;
  }

  inline void stopRecordingVideo() {
    serverRequest_.push_back(ServerRequestType::STOP_RECORD_VIDEO);
  }

  inline bool waitForReadEvent(int timeout) {
    fd_set sdset;
    struct timeval tv;

    tv.tv_sec = timeout;
    tv.tv_usec = 0;
    FD_ZERO(&sdset);
    FD_SET(server_fd, &sdset);
    return select(server_fd + 1, &sdset, NULL, NULL, &tv) > 0;
  }

  void setCameraPositionAndLookAt (const Eigen::Vector3d& pos, const Eigen::Vector3d& lookAt) {
    serverRequest_.push_back(ServerRequestType::STOP_RECORD_VIDEO);
    position_ = pos; lookAt_ = lookAt;
  }

  void focusOn (raisim::Object* obj) {
    RSFATAL_IF(obj==nullptr, "object does not exist.")
    serverRequest_.push_back(ServerRequestType::FOCUS_ON_SPECIFIC_OBJECT);
    focusedObjectName_ = obj->getName();
  }

  bool isConnected () const { return connected_; }

 private:
  inline bool processRequests() {
    data_ = &send_buffer[0];
    ClientMessageType type;

    if (recv(client_, &receive_buffer[0], MAXIMUM_PACKET_SIZE, 0) == 0) {
      connected_ = false;
      return false;
    }
    get(&receive_buffer[0], &type);

    data_ = set(data_, state_);

    // set server request
    data_ = set(data_, (uint64_t)serverRequest_.size());
    for(const auto& sr : serverRequest_) {
      data_ = set(data_, (int)sr);

      switch (sr) {
        case ServerRequestType::NO_REQUEST:
        case ServerRequestType::STOP_RECORD_VIDEO:
          break;

        case ServerRequestType::START_RECORD_VIDEO:
          data_ = setString(data_, videoName_);
          break;

        case ServerRequestType::SET_CAMERA_TO:
          data_ = setN(data_, position_.data(), 3);
          data_ = setN(data_, lookAt_.data(), 3);
          break;

        case ServerRequestType::FOCUS_ON_SPECIFIC_OBJECT:
          data_ = setString(data_, focusedObjectName_);
          break;
      }
    }

    serverRequest_.clear();

    lockVisualizationServerMutex();

    if(state_ != Status::STATUS_HIBERNATING) {
      switch (type) {
        case REQUEST_OBJECT_POSITION:
          serializeWorld();
          break;

        case REQUEST_INITIALIZATION:
          serializeObjects();
          break;

        case REQUEST_CHANGE_REALTIME_FACTOR:
          changeRealTimeFactor();
          break;

        case REQUEST_CONTACT_INFOS:
          serializeContacts();
          break;

        case REQUEST_CONFIG_XML:
          serializeToXML();
          break;

        case REQUEST_INITIALIZE_VISUALS:
          serializeVisuals();
          break;

        case REQUEST_VISUAL_POSITION:
          break;

        case REQUEST_SERVER_STATUS:
          // Do nothing
          return false;
          break;
      }
    }
    unlockVisualizationServerMutex();

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
    if (!updateReady_) {
      data_ = set(data_, ServerMessageType::NO_MESSAGE);
      return;
    }

    // std::lock_guard<std::mutex> guard(serverMutex_);
    auto &objList = world_->getObjList();

    // set message type
    data_ = set(data_, ServerMessageType::OBJECT_POSITION_UPDATE);

    data_ = set(data_, (uint64_t)world_->getConfigurationNumber() + visualConfiguration_);

    // Data begins
    data_ = set(data_, (uint64_t)(objList.size()));

    for (auto *ob : objList) {
      // set gc
      if (ob->getObjectType() == ObjectType::ARTICULATED_SYSTEM) {
        data_ = set(data_, (uint64_t)(static_cast<ArticulatedSystem *>(ob)->getVisOb().size() +
                    static_cast<ArticulatedSystem *>(ob)->getVisColOb().size()));

        for (uint64_t i = 0; i < 2; i++) {
          std::vector<VisObject> *visOb;
          if (i == 0)
            visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisOb());
          else
            visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisColOb());

          for (uint64_t k = 0; k < (*visOb).size(); k++) {
            auto &vob = (*visOb)[k];
            std::string name = std::to_string(ob->getIndexInWorld()) +
                "/" + std::to_string(i) + "/" +
                std::to_string(k);
            data_ = setString(data_, name);

            Vec<3> pos, offsetInWorld;
            Vec<4> quat;
            Mat<3, 3> bodyRotation, rot;

            ob->getPosition(vob.localIdx, pos);
            ob->getOrientation(vob.localIdx, bodyRotation);
            matvecmul(bodyRotation, vob.offset, offsetInWorld);
            matmul(bodyRotation, vob.rot, rot);
            data_ = set(data_, pos[0] + offsetInWorld[0]);
            data_ = set(data_, pos[1] + offsetInWorld[1]);
            data_ = set(data_, pos[2] + offsetInWorld[2]);

            raisim::rotMatToQuat(rot, quat);

            data_ = set(data_, quat[0]);
            data_ = set(data_, quat[1]);
            data_ = set(data_, quat[2]);
            data_ = set(data_, quat[3]);
          }
        }
      } else if (ob->getObjectType() == ObjectType::COMPOUND) {
        auto* com = static_cast<Compound *>(ob);
        data_ = set(data_, (uint64_t)(com->getObjList().size() * 2));
        for (uint64_t i = 0; i < 2; i++) {
          for (uint64_t k = 0; k < com->getObjList().size(); k++) {
            std::string name = std::to_string(ob->getIndexInWorld()) + "/" + std::to_string(i) + "/" + std::to_string(k);
            data_ = setString(data_, name);
            Vec<3> pos, center; com->getPosition(center);

            matvecmul(com->getRotationMatrix(), com->getObjList()[k].trans.pos, pos);
            data_ = set(data_, pos[0] + center[0]);
            data_ = set(data_, pos[1] + center[1]);
            data_ = set(data_, pos[2] + center[2]);

            Mat<3,3> rot;
            matmul(com->getRotationMatrix(), com->getObjList()[k].trans.rot, rot);
            Vec<4> quat;
            raisim::rotMatToQuat(rot, quat);
            data_ = set(data_, quat[0]);
            data_ = set(data_, quat[1]);
            data_ = set(data_, quat[2]);
            data_ = set(data_, quat[3]);
          }
        }

      } else {
        data_ = set(data_, (uint64_t)(1));
        Vec<3> pos;
        Vec<4> quat;
        std::string name = std::to_string(ob->getIndexInWorld());
        data_ = setString(data_, name);
        dynamic_cast<SingleBodyObject *>(ob)->getPosition(pos);
        dynamic_cast<SingleBodyObject *>(ob)->getQuaternion(quat);
        data_ = setN(data_, pos.ptr(), 3);
        data_ = setN(data_, quat.ptr(), 4);
      }
    }

    // visuals
    data_ = set(data_, (uint64_t)(_visualObjects.size()));

    for (auto &kAndVo : _visualObjects) {
      auto &vo = kAndVo.second;

      Vec<3> pos = vo->getPosition();
      Vec<4> quat = vo->getOrientation();

      data_ = setString(data_, vo->name);
      data_ = setN(data_, pos.ptr(), 3);
      data_ = setN(data_, quat.ptr(), 4);
    }

    updateReady_ = true;
  }

  inline void serializeObjects() {
    // std::lock_guard<std::mutex> guard(serverMutex_);

    auto &objList = world_->getObjList();

    // set message type
    data_ = set(data_, ServerMessageType::INITIALIZATION);

    // set configuration number.
    data_ = set(data_, (uint64_t)world_->getConfigurationNumber() + visualConfiguration_);

    // Data begins
    data_ = set(data_, (uint64_t)(objList.size()));

    for (auto *ob : objList) {
      // set name length
      data_ = set(data_, (uint64_t)ob->getIndexInWorld());

      // object type
      data_ = set(data_, ob->getObjectType());

      // object name
      data_ = setString(data_, ob->getName());

      switch (ob->getObjectType()) {
        case SPHERE:
          data_ = set(data_, float(static_cast<Sphere *>(ob)->getRadius()));
          break;

        case BOX:
          for (int i = 0; i < 3; i++)
            data_ = set(data_, float(static_cast<Box *>(ob)->getDim()[i]));
          break;

        case CYLINDER:
          data_ = set(data_, float(static_cast<Cylinder *>(ob)->getRadius()));
          data_ = set(data_, float(static_cast<Cylinder *>(ob)->getHeight()));
          break;

        case CONE:
          break;

        case CAPSULE:
          data_ = set(data_, float(static_cast<Capsule *>(ob)->getRadius()));
          data_ = set(data_, float(static_cast<Capsule *>(ob)->getHeight()));
          break;

        case HALFSPACE:
          data_ = set(data_, float(static_cast<Ground *>(ob)->getHeight()));
          break;

        case COMPOUND:
          for (uint64_t i = 0; i < 2; i++) {
            data_ = set(data_, (uint64_t)(static_cast<Compound*>(ob)->getObjList().size()));

            for (auto &vob : static_cast<Compound*>(ob)->getObjList()) {
              data_ = set(data_, vob.objectType);

              switch (vob.objectType) {
                case BOX:
                  data_ = set(data_, vob.objectParam[0]);
                  data_ = set(data_, vob.objectParam[1]);
                  data_ = set(data_, vob.objectParam[2]);
                  break;
                case CAPSULE:
                case CYLINDER:
                  data_ = set(data_, vob.objectParam[0]);
                  data_ = set(data_, vob.objectParam[1]);
                  break;
                case SPHERE:
                  data_ = set(data_, vob.objectParam[0]);
                  break;
              }
            }
          }
          break;

        case MESH:
          data_ = setString(data_, static_cast<Mesh *>(ob)->getMeshFileName());
          data_ = set(data_, float(static_cast<Mesh *>(ob)->getScale()));
          break;

        case HEIGHTMAP:
          // misc data
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getCenterX()));
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getCenterY()));
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getXSize()));
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getYSize()));
          data_ = set(data_, (uint64_t)(static_cast<HeightMap *>(ob)->getXSamples()));
          data_ = set(data_, (uint64_t)(static_cast<HeightMap *>(ob)->getYSamples()));

          // size of height map
          data_ = set(data_, (uint64_t)(static_cast<HeightMap *>(ob)->getHeightVector().size()));

          // height values in float
          for (auto h : static_cast<HeightMap *>(ob)->getHeightVector())
            data_ = set(data_, float(h));

          break;

        case ARTICULATED_SYSTEM:
          std::string resDir =
              static_cast<ArticulatedSystem *>(ob)->getResourceDir();
          data_ = setString(data_, resDir);

          for (uint64_t i = 0; i < 2; i++) {
            std::vector<VisObject> *visOb;
            if (i == 0)
              visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisOb());
            else
              visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisColOb());

            data_ = set(data_, (uint64_t)(visOb->size()));

            for (auto &vob : *visOb) {
              data_ = set(data_, vob.shape);
              data_ = set(data_, i);
              if (vob.shape == Shape::Mesh) {
                data_ = setString(data_, vob.fileName);
                data_ = set(data_, vob.scale[0]);
                data_ = set(data_, vob.scale[1]);
                data_ = set(data_, vob.scale[2]);
              } else {
                data_ = set(data_, (uint64_t)(vob.visShapeParam.size()));
                for (auto vparam : vob.visShapeParam)
                  data_ = set(data_, vparam);
              }
            }
          }
          break;
      }
    }
  }

  inline void serializeContacts() {
    // std::lock_guard<std::mutex> guard(serverMutex_);

    auto *contactList = world_->getContactProblem();

    // set message type
    data_ = set(data_, ServerMessageType::CONTACT_INFO_UPDATE);

    // Data begins
    size_t contactSize = contactList->size();
    for(const auto& con : *contactList)
      if (con.rank == 5) contactSize--;

    size_t contactIncrement = 0;
    auto contactSizeLocation = data_;

    data_ = set(data_, (uint64_t)(contactSize));

    for (auto *obj : world_->getObjList()) {
      for (auto &contact : obj->getContacts()) {
        if (!contact.isObjectA() &&
            contact.getPairObjectBodyType() != raisim::BodyType::STATIC)
          continue;
        contactIncrement++;
        // contact points
        auto contactPos = contact.getPosition();
        data_ = set(data_, contactPos[0]);
        data_ = set(data_, contactPos[1]);
        data_ = set(data_, contactPos[2]);

        // contact forces
        auto *impulseB = contact.getImpulse();
        auto contactFrame = contact.getContactFrame();

        Vec<3> impulseW;
        raisim::matvecmul(contactFrame, *impulseB, impulseW);
        data_ = set(data_, impulseW[0]);
        data_ = set(data_, impulseW[1]);
        data_ = set(data_, impulseW[2]);
      }
    }

    set(contactSizeLocation, contactIncrement);
  }

  inline void serializeToXML() {
    // std::lock_guard<std::mutex> guard(serverMutex_);

    // check if world is initialized by config xml
    const std::string configFile = world_->getConfigFile();

    if (configFile.empty()) {
      data_ = set(data_, ServerMessageType::NO_MESSAGE);
      return;
    }

    data_ = set(data_, ServerMessageType::CONFIG_XML);
    TiXmlDocument doc(configFile.c_str());
    RSFATAL_IF(!doc.LoadFile(), "cannot read file: " << configFile)

    TiXmlNode *raisim = doc.FirstChildElement("raisim");
    if (!raisim) RSFATAL("Cannot find raisim element")

    // create visuals if config has visuals
    TiXmlNode *visual = raisim->FirstChildElement("visuals");
    if (visual) {
      for (TiXmlElement *vis = visual->FirstChildElement(); vis != NULL;
           vis = vis->NextSiblingElement()) {
        try {
          // name
          auto *name = vis->Attribute("name");
          if (!name) throw std::runtime_error("visual objects require name.");

          // color
          auto *colorAttrb = vis->Attribute("color");
          double r = 1.0;
          double g = 1.0;
          double b = 1.0;
          double a = 1.0;
          if (colorAttrb) {
            std::istringstream iss(colorAttrb);
            std::vector<std::string> results(
                (std::istream_iterator<std::string>(iss)),
                std::istream_iterator<std::string>());
            if (results.size() == 4) {
              r = std::stod(results[0]);
              g = std::stod(results[1]);
              b = std::stod(results[2]);
              a = std::stod(results[3]);
            } else if (results.size() == 3) {
              r = std::stod(results[0]);
              g = std::stod(results[1]);
              b = std::stod(results[2]);
            } else {
              throw std::runtime_error(
                  "visual object color should be either \"r g b a\" or \"r g "
                  "b\"");
            }
          }

          // material
          auto *materialAttrb = vis->Attribute("material");
          std::string material = "";
          if (materialAttrb) {
            material = materialAttrb;
          }
          // glow
          auto *glowAttrb = vis->Attribute("glow");
          bool glow = true;
          if (glowAttrb) {
            if (strcmp(glowAttrb, "true") == 0)
              glow = true;
            else if (strcmp(glowAttrb, "false") == 0)
              glow = false;
          }

          // sphere
          if (strcmp(vis->ValueTStr().c_str(), "sphere") == 0) {
            auto *radius = vis->Attribute("radius");
            if (!radius) throw std::runtime_error("ERROR while reading sphere");
            addVisualSphere(name, std::stod(radius), r, g, b, a, material,
                            glow);
          }
            // box
          else if (strcmp(vis->ValueTStr().c_str(), "box") == 0) {
            auto *xLength = vis->Attribute("xLength");
            auto *yLength = vis->Attribute("yLength");
            auto *zLength = vis->Attribute("zLength");
            if (!xLength || !yLength || !zLength)
              throw std::runtime_error("ERROR while reading box");
            addVisualBox(name, std::stod(xLength), std::stod(yLength),
                         std::stod(zLength), r, g, b, a, material, glow);
          }
            // cylinder
          else if (strcmp(vis->ValueTStr().c_str(), "cylinder") == 0) {
            auto *radius = vis->Attribute("radius");
            auto *height = vis->Attribute("height");
            if (!radius || !height)
              throw std::runtime_error("ERROR while reading cylinder");
            addVisualCylinder(name, std::stod(radius), std::stod(height), r, g,
                              b, a, material, glow);
          }
            // capsule
          else if (strcmp(vis->ValueTStr().c_str(), "capsule") == 0) {
            auto *radius = vis->Attribute("radius");
            auto *height = vis->Attribute("height");
            if (!radius || !height)
              throw std::runtime_error("ERROR while reading capsule");
            addVisualCapsule(name, std::stod(radius), std::stod(height), r, g,
                             b, a, material, glow);
          }

          // pos
          auto *pos = vis->FirstChildElement("pos");
          if (pos) {
            auto *x = pos->Attribute("x");
            auto *y = pos->Attribute("y");
            auto *z = pos->Attribute("z");

            if (!x || !y || !z)
              throw std::runtime_error("ERROR while reading pos");
            _visualObjects[name]->setPosition(std::stod(x), std::stod(y),
                                             std::stod(z));
          }

          // quat
          auto *quat = vis->FirstChildElement("quat");
          if (quat) {
            auto *w = quat->Attribute("w");
            auto *x = quat->Attribute("x");
            auto *y = quat->Attribute("y");
            auto *z = quat->Attribute("z");

            if (!w || !x || !y || !z)
              throw std::runtime_error("ERROR while reading quat");
            _visualObjects[name]->setOrientation(std::stod(w), std::stod(x),
                                                std::stod(y), std::stod(z));
          }
        } catch (std::exception &e) {
          // terminate with exception message
          std::string msg = std::string(e.what()) +
              " (line: " + std::to_string(vis->Row()) +
              ", column: " + std::to_string(vis->Column()) + ")";
          RSFATAL(msg)
        }
      }
    }

    TiXmlPrinter printer;
    doc.Accept(&printer);
    const char *xmlString = printer.CStr();

    // send xml into string
    data_ = setString(data_, xmlString);
  }

  inline void serializeVisuals() {
    // std::lock_guard<std::mutex> guard(serverMutex_);

    // set message type
    data_ = set(data_, ServerMessageType::VISUAL_INITILIZATION);

    // set configuration number
    data_ = set(data_, visualConfiguration_);

    // Data begins
    data_ = set(data_, (uint64_t)(_visualObjects.size()));

    for (auto &kAndVo : _visualObjects) {
      auto &vo = kAndVo.second;

      // object type
      data_ = set(data_, vo->type);

      // object name
      data_ = setString(data_, vo->name);

      // object color
      data_ = set(data_, (float)vo->color[0]);  // r
      data_ = set(data_, (float)vo->color[1]);  // g
      data_ = set(data_, (float)vo->color[2]);  // b
      data_ = set(data_, (float)vo->color[3]);  // a

      // object material
      data_ = setString(data_, vo->material);

      // object glow
      data_ = set(data_, vo->glow);

      // object shadow
      data_ = set(data_, vo->shadow);

      switch (vo->type) {
        case Visuals::VisualSphere:
          data_ = set(data_, (float)vo->size[0]);
          break;

        case Visuals::VisualBox:
          for (int i = 0; i < 3; i++) data_ = set(data_, (float)vo->size[i]);
          break;

        case Visuals::VisualCylinder:
          data_ = set(data_, (float)vo->size[0]);
          data_ = set(data_, (float)vo->size[1]);
          break;

        case Visuals::VisualCapsule:
          data_ = set(data_, (float)vo->size[0]);
          data_ = set(data_, (float)vo->size[1]);
          break;

          //        case VisualObject::VisualMesh:
          //          data_ = setString(data_, static_cast<Mesh
          //          *>(ob)->getMeshFileName()); data_ = set(data_,
          //          float(static_cast<Mesh *>(ob)->getScale())); break;
      }
    }
  }

  inline void changeRealTimeFactor() {
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

  bool pauseRequested_ = false;
  bool resumeRequested_ = false;
  double realTimeFactor = 1.0;
  std::atomic<bool> terminateRequested_ = {false};

  int client_;
  int server_fd, valread;
  sockaddr_in address;
  int addrlen;
  std::thread serverThread_;
  bool updateReady_ = true;

  std::mutex serverMutex_;

  std::unordered_map<std::string, Visuals*> _visualObjects;
  uint64_t visualConfiguration_ = 0;
  void updateVisualConfig() { visualConfiguration_++; }

  int raisimPort_ = 8080;
  double lastWaitTime = 0.;

  Eigen::Vector3d position_, lookAt_;

};

}  // namespace raisim

#endif  // RAISIM_RAISIMSERVER_HPP
