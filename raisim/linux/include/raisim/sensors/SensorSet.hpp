//
// Created by jemin on 24. 6. 12.
//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_SENSORSET_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_SENSORSET_HPP_

#include <memory>
#include <vector>
#include "Sensors.hpp"

namespace raisim {

namespace urdf {
  class LoadFromURDF2;
}

class SensorSet {
 public:
  friend class raisim::Sensor;
  friend class raisim::Child;
  friend class raisim::ArticulatedSystem;
  friend class raisim::urdf::LoadFromURDF2;
  using SensorSetDataType = std::unordered_map<std::string, std::shared_ptr<Sensor>, std::hash<std::string>, std::equal_to<>, AlignedAllocator<std::pair<const std::string, std::shared_ptr<Sensor>>,32>>;

  SensorSet(std::string nameI, std::string modelI, std::string serialNumberI) :
   name(std::move(nameI)), model(std::move(modelI)), serialNumber(std::move(serialNumberI)) {}


  [[nodiscard]] SensorSetDataType& getSensors() {
    return sensors;
  }

  [[nodiscard]] const SensorSetDataType& getSensors() const {
    return sensors;
  }

  /**
 * @return sensor of the specified type
 */
  template<class T>
  T* getSensor(const std::string& nameIn) {
    RSFATAL_IF(sensors.find(nameIn) == sensors.end(), "Cannot find \""<<nameIn<<"\"")
    auto sensor = sensors.find(nameIn)->second;
    RSFATAL_IF(sensor->getType() != T::getType(), "Type mismatch. "
        << nameIn << " has a type of " << toString(sensor->getType()) << " and the requested type is "
        << toString(T::getType()))
    return reinterpret_cast<T*>(sensor.get());
  }

  const std::string& getModel() { return model; }
  const std::string& getSerialNumber() { return serialNumber; };

 protected:

  void addSensor (const std::string& nameIn, const std::shared_ptr<Sensor>& sensor) {
    sensors[nameIn] = sensor;
  }

  std::string name;
  std::string serialNumber;
  std::string model;
  SensorSetDataType sensors;
};

}

#endif //RAISIM_INCLUDE_RAISIM_SENSORS_SENSORSET_HPP_
