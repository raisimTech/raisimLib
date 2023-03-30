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
    DEPTH,
    IMU
  };

  enum class MeasurementSource : int {
    RAISIM = 0, // [NOT IMPLEMENTED YET] raisim automatically updates the measurements according to the simulation time
    VISUALIZER, // visualizer automatically updates the measurements according to the simulation time
    MANUAL // user manually update the measurements whenever needed.
  };

  Sensor (std::string name, Type type, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot) :
      name_(std::move(name)), type_(type), as_(as), posB_(pos), rotB_(rot), posFrame_(pos), rotFrame_(rot) { }
  virtual ~Sensor() = default;

  /**
   * The pose is updated using updatePose() method. updatePose() can be called by the user. If the visualizer updates the measurement, the server will call this method before the measurement update.
   * @return The position of the sensor frame
   */
  [[nodiscard]] const Vec<3>& getPosition() { return pos_; }

  /**
   * The pose is updated using updatePose() method. updatePose() can be called by the user. If the visualizer updates the measurement, the server will call this method before the measurement update.
   * @return The orientation of the sensor frame
   */
  [[nodiscard]] const Mat<3,3>& getOrientation() { return rot_; }

  /**
   * @return The position of the frame w.r.t. the nearest moving parent. It is used to compute the frame position in the world frame
   */
  [[nodiscard]] const Vec<3>& getFramePosition() { return posFrame_; }

  /**
   * @return The orientation of the frame w.r.t. the nearest moving parent. It is used to compute the frame position in the world frame
   */
  [[nodiscard]] const Mat<3,3>& getFrameOrientation() { return rotFrame_; }


  /**
   * @return The position of the frame w.r.t. the sensor frame, which is the frame of the parent joint
   */
  [[nodiscard]] const Vec<3>& getPosInSensorFrame() { return posB_; }

  /**
   * @return The orientation of the frame w.r.t. the sensor frame, which is the frame of the parent joint
   */
  [[nodiscard]] const Mat<3,3>& getOriInSensorFrame() { return rotB_; }

  /**
   * @return The name of the sensor
   */
  [[nodiscard]] const std::string& getName() { return name_; }

  /**
   * @return The type of the sensor
   */
  [[nodiscard]] Type getType() { return type_; }

  /**
   * @return The update frequency in Hz
   */
  [[nodiscard]] double getUpdateRate() const { return updateRate_; }

  /**
   * @return The time when the last sensor measurement was recorded
   */
  [[nodiscard]] double getUpdateTimeStamp() const { return updateTimeStamp_; }

  /**
   * change the update rate of the sensor. The rate is given in Hz
   * @param[in] rate the update rate in Hz
   */
  void setUpdateRate(double rate) { updateRate_ = rate; }

  /**
   * Set the time stamp for the last sensor measurement
   * @param[in] time the time stamp
   */
  void setUpdateTimeStamp(double time) { updateTimeStamp_ = time; }

  /**
   * Used by the server. Do not use it if you don't know what it does. It serialized the camera property
   * @param[in] data the pointer where the property is written
   * @return the pointer where the next data should be written
   */
  [[nodiscard]] virtual char* serializeProp (char* data) const = 0;

  /**
   * Used by the server. Do not use it if you don't know what it does. It serialized the camera property
   * @param[in] data the pointer where the property is written
   * @return the pointer where the next data should be written
   */
  [[nodiscard]] virtual char* serializeMeasurements (char* data) const { return data; };

  /**
   * update the pose of the sensor from the articulated system
   */
  void updatePose();

  /**
   * @return The measurement source.
   */
  [[nodiscard]] MeasurementSource getMeasurementSource() { return source_; }

  /**
   * change the measurement source
   * @param[in] source The measurement source.
   */
  void setMeasurementSource(MeasurementSource source) {
    source_ = source;
  }

  /**
   * Update the sensor measurement using raisim if possible
   * @param[in] world the world object
   */
  virtual void update (class World& world) = 0;

  /**
   * Get the id of the frame on which the sensor is attached
   * @return frame id
   */
  size_t getFrameId() {
    return frameId_;
  }

 protected:
  void setFramePosition(const Vec<3>& pos) { posFrame_ = pos; }
  void setFrameRotation(const Mat<3,3>& rot) { rotFrame_ = rot; }
  void setFrameId(size_t id) { frameId_ = id; }
  virtual void validateMeasurementSource() {};

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
