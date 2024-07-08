//----------------------------//
// This file is part of RaiSim//
// Copyright 2021, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_HPP_

#include <string>
#include <mutex>
#include "Eigen/Core"
#include "raisim/math.hpp"
#include "raisim/helper.hpp"
#include "raisim/server/SerializationHelper.hpp"


namespace raisim {

class Child;
class ArticulatedSystem;

class Sensor {
  friend class raisim::Child;
  friend class raisim::ArticulatedSystem;

 public:
  enum class Type : int {
    UNKNOWN = 0,
    RGB,
    DEPTH,
    IMU,
    SPINNING_LIDAR
  };

  enum class MeasurementSource : int {
    RAISIM = 0, // [NOT IMPLEMENTED YET] raisim automatically updates the measurements according to the simulation time
    VISUALIZER, // visualizer automatically updates the measurements according to the simulation time
    MANUAL // user manually update the measurements whenever needed.
  };

  Sensor (std::string name, std::string fullName, Type type, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot, MeasurementSource source) :
      name_(std::move(name)), fullName_(std::move(fullName)), type_(type), as_(as), posB_(pos), rotB_(rot), posFrame_(pos), rotFrame_(rot), source_(source) { }
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
   * @return The name of the sensor. This name is given in the sensor xml file
   */
  [[nodiscard]] const std::string& getName() { return name_; }

  /**
   * @return The full name of the sensor. This name includes the sensor set's name
   */
    [[nodiscard]] const std::string& getFullName() { return fullName_; }

  /**
   * @return The name of the sensor
   */
  [[nodiscard]] const std::string& getSensorSetModel() { return model_; }

  /**
   * @return The serial number of the sensor
   */
   [[nodiscard]] const std::string& getSerialNumber() { return serialNumber_; };

   /**
    * Set the serial number of the sensor
    * @param serialNumber
    * @return
    */
   void setSerialNumber(const std::string& serialNumber) { serialNumber_ = serialNumber; };

  /**
   * @return The type of the sensor
   */
  [[nodiscard]] Type getType() { return type_; }

  /**
   * @return The update frequency in Hz
   */
  [[nodiscard]] double getUpdateRate() const { return updateRate_; }

  /**
   * This method will return negative value if it has never been updated
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
  void setMeasurementSource(MeasurementSource source) { source_ = source; }

  /**
   * Get the id of the frame on which the sensor is attached
   * @return frame id
   */
  [[nodiscard]] size_t getFrameId() const { return frameId_; }

  /**
   * locks sensor mutex. This can be used if you use raisim in a multi-threaded environment.
   */
  void lockMutex() { mutex_.lock(); }

  /**
   * unlock sensor mutex. This can be used if you use raisim in a multi-threaded environment.
   */
  void unlockMutex() { mutex_.unlock(); }

 protected:

  virtual void update (class World& world) = 0;
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
  std::string serialNumber_;
  std::string model_;

 private:
  std::string name_, fullName_;
  std::mutex mutex_;
  double updateRate_ = 1., updateTimeStamp_ = -1.;
};

static inline std::string toString(Sensor::Type type) {
  switch (type) {
    case Sensor::Type::DEPTH:
      return "depth";
    case Sensor::Type::RGB:
      return "rgb";
    case Sensor::Type::SPINNING_LIDAR:
      return "spinning_lidar";
    default:
      return "unknown";
  }
  return "unknown";
}



}
#endif //RAISIM_INCLUDE_RAISIM_SENSORS_HPP_
