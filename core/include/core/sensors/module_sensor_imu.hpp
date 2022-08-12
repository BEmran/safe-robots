#ifndef CORE_SENSOR_MODULE_SENSOR_IMU_HPP
#define CORE_SENSOR_MODULE_SENSOR_IMU_HPP

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
   * @param debug enable/disable debug, default is false
   */
  explicit ImuSensorModule(const std::string& name, const bool debug = false);

  /**
   * @brief Destroy the Imu Sensor Module object
   *
   */
  ~ImuSensorModule() = default;
  
  virtual void Initialize() {}
  virtual bool Probe() {return false;}
  virtual void Update() {};

  utils::ImuData GetData() const;

  void ClearData();

  protected:
  void SetData(const utils::ImuData& data);
  void SetData();
  
  utils::ImuData data_;

 private:
  utils::Cash<utils::ImuData> cashed_data_;
};

}  // namespace core::sensors
#endif  // CORE_SENSOR_MODULE_SENSOR_IMU_HPP