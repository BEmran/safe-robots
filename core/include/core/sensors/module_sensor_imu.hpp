#ifndef CORE_IMU_SENSORS_MODULE_SENSOR_HPP
#define CORE_IMU_SENSORS_MODULE_SENSOR_HPP

#include "core/sensors/module_sensor.hpp"
#include "core/utils/cash.hpp"
#include "core/utils/data.hpp"
#include <string>

namespace core::sensors
{
/**
 * @brief a class module sensor used to interface the Imu sensor 
 *
 */
class ImuSensorModule : public SensorModuleAbs
{
 public:
  /**
   * @brief Construct a new Imu Sensor Module object
   *
   * @param name sensor module's name
   */
  explicit ImuSensorModule(const std::string& name);

  /**
   * @brief Destroy the Imu Sensor Module object
   *
   */
  ~ImuSensorModule() = default;
  
  virtual bool initialize() {return true;}
  virtual bool probe() {return true;}
  virtual void update() {};

  utils::ImuData GetData() const;

  void ClearData();

  protected:
  void SetData(const utils::ImuData& data);
  void SetData();

 private:
  utils::ImuData data_;
  utils::Cash<utils::ImuData> cashed_data_;
};

}  // namespace core::sensors
#endif  // CORE_IMU_SENSORS_MODULE_SENSOR_HPP