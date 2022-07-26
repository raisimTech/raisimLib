//----------------------------//
// This file is part of RaiSim//
// Copyright 2021, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_HPP_

#include <string>
#include "Eigen/Core"
#include "raisim/math.hpp"
#include "raisim/helper.hpp"
#include "raisim/server/SerializationHelper.hpp"


namespace raisim {

class Child;

class Sensor {
  friend class raisim::Child;

 public:
  enum class Type : int {
    UNKNOWN = 0,
    RGB,
    DEPTH
  };

  enum class MeasurementSource : int {
    RAISIM = 0, // raisim automatically updates the measurements according to the simulation time
    VISUALIZER, // visualizer automatically updates the measurements according to the simulation time
    MANUAL // user manually update the measurements whenever needed.
  };

  Sensor (std::string name, Type type, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot) :
      name_(std::move(name)), type_(type), as_(as), posB_(pos), rotB_(rot), posFrame_(pos), rotFrame_(rot) { }
  virtual ~Sensor() = default;
  void setPose(const Vec<3>& pos, const Mat<3,3>& rot) {
    pos_ = pos;
    rot_ = rot;
  }

  [[nodiscard]] const Vec<3>& getPos() { return pos_; }
  [[nodiscard]] const Mat<3,3>& getRot() { return rot_; }
  [[nodiscard]] const Vec<3>& getFramePosition() { return posFrame_; }
  [[nodiscard]] const Mat<3,3>& getFrameRotation() { return rotFrame_; }


  [[nodiscard]] const Vec<3>& getPosInSensorFrame() { return posB_; }
  [[nodiscard]] const Mat<3,3>& getRotInSensorFrame() { return rotB_; }

  const std::string& getName() { return name_; }
  [[nodiscard]] Type getType() { return type_; }
  void setFrameId(size_t id) { frameId_ = id; }
  [[nodiscard]] double getUpdateRate() const { return updateRate_; }
  [[nodiscard]] double getUpdateTimeStamp() const { return updateTimeStamp_; }
  void setUpdateRate(double rate) { updateRate_ = rate; }
  void setUpdateTimeStamp(double time) { updateTimeStamp_ = time; }
  virtual char* serializeProp (char* data) const = 0;
  virtual void updatePose(class World &world) = 0;
  [[nodiscard]] MeasurementSource getMeasurementSource() { return source_; }
  void setMeasurementSource(MeasurementSource source) { source_ = source; }
  virtual void update (class World& world) = 0;

 protected:
  void setFramePosition(const Vec<3>& pos) { posFrame_ = pos; }
  void setFrameRotation(const Mat<3,3>& rot) { rotFrame_ = rot; }

  Type type_;
  Vec<3> pos_, posB_, posFrame_;
  Mat<3,3> rot_, rotB_, rotFrame_;
  size_t frameId_;
  class ArticulatedSystem* as_;
  MeasurementSource source_ = MeasurementSource::MANUAL;

 private:
  std::string name_;
  double updateRate_ = 1., updateTimeStamp_ = -1.;
};

static inline std::string toString(Sensor::Type type) {
  switch (type) {
    case Sensor::Type::DEPTH:
      return "depth";
    case Sensor::Type::RGB:
      return "rgb";
    default:
      return "unknown";
  }
  return "unknown";
}



}
#endif //RAISIM_INCLUDE_RAISIM_SENSORS_HPP_
