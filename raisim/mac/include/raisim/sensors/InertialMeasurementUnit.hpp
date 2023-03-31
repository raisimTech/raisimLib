//----------------------------//
// This file is part of RaiSim//
// Copyright 2023, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_INERTIALMEASUREMENTUNIT_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_INERTIALMEASUREMENTUNIT_HPP_


#include "raisim/sensors/Sensors.hpp"

namespace raisim {

class InertialMeasurementUnit final : public Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct ImuProperties {
    std::string name;
    double maxAcc = std::numeric_limits<double>::max();
    double maxAngVel = std::numeric_limits<double>::max();

    /// noise type
    enum class NoiseType : int {
      GAUSSIAN = 0,
      UNIFORM,
      NO_NOISE
    } noiseType;

    static NoiseType stringToNoiseType(const std::string& type) {
      if(type == "gaussian" || type == "Gaussian")
        return NoiseType::GAUSSIAN;
      else if (type == "uniform" || type == "Uniform")
        return NoiseType::UNIFORM;
      else
        return NoiseType::NO_NOISE;
    }
    double mean = 0., std;
  };

  explicit InertialMeasurementUnit(const ImuProperties& prop, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot) :
      Sensor(prop.name, Sensor::Type::IMU, as, pos, rot), prop_(prop) {
    linearAcc_.setZero();
    angularVel_.setZero();
    source_ = MeasurementSource::RAISIM;
  }
  ~InertialMeasurementUnit() final = default;

  char* serializeProp (char* data) const final {
    return server::set(data, type_, prop_.name, prop_.maxAcc, prop_.maxAngVel);
  }

  [[nodiscard]] char* serializeMeasurements (char* data) const {
    Vec<3> linA, angV;
    linA.e() = getLinearAcceleration();
    angV.e() = getAngularVelocity();
    return server::setInFloat(data, linA, angV);
  };

  /**
   * Get the linear acceleration measured by the sensor.
   * In simulation, this is updated every update loop (set by the updateRate).
   * On the real robot, this value should be set externally by setLinearAcceleration()
   * @return linear acceleration
   */
  [[nodiscard]] const Eigen::Vector3d & getLinearAcceleration () const { return linearAcc_; }

  /**
   * Get the angular velocity measured by the sensor.
   * In simulation, this is updated every update loop (set by the updateRate).
   * On the real robot, this value should be set externally by setAngularVelocity()
   * @return angular velocity
   */
  [[nodiscard]] const Eigen::Vector3d & getAngularVelocity () const { return angularVel_; }


  /**
   * This set method make sense only on the real robot.
   * @param acc acceleration measured from sensor
   */
  void setLinearAcceleration (const Eigen::Vector3d & acc) { linearAcc_ = acc; }

  /**
   * This set method make sense only on the real robot.
   * @param vel angular velocity measured from sensor
   */
  void setAngularVelocity (const Eigen::Vector3d & vel) { angularVel_ = vel; }

  [[nodiscard]] const ImuProperties& getProperties () const { return prop_; }
  [[nodiscard]] static Type getType() { return Type::IMU; }

  /*
   * Manually update the sensor pose then measurements using Raisim ray cast function.
   */
  void update (class World& world) final;

 protected:
  void validateMeasurementSource() final {
    RSFATAL_IF(source_ == MeasurementSource::VISUALIZER, "IMU cannot be updated by the visualizer")
  };

 private:
  ImuProperties prop_;
  Eigen::Vector3d linearAcc_;
  Eigen::Vector3d angularVel_;
};

}

#endif //RAISIM_INCLUDE_RAISIM_SENSORS_INERTIALMEASUREMENTUNIT_HPP_
