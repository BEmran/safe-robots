#ifndef CORE_SENSORS_MODULE_SENSOR_HPP
#define CORE_SENSORS_MODULE_SENSOR_HPP

#include "core/utils/module.hpp"
#include <string>

namespace core::sensors
{
/**
 * @brief defines sensor module type
 *
 */
enum class SensorModuleType
{
  BAR,
  ADC,
  GPS,
  IMU,
  UNDEFINED
};

/**
 * @brief Converts sensor module type to string
 *
 * @param type sensor module type to be converted
 * @return std::string name of module type
 */
std::string SensorModuleTypeToString(const SensorModuleType type);

/**
 * @brief Abustract class used to define main object of beaglebone hardware
 *
 */
class SensorModuleAbs : public utils::ModuleAbs
{
 public:
  /**
   * @brief Construct a new Sensor Module object
   *
   * @param sensor_type sensor module's type
   * @param name sensor module's name
   * @param debug enable/disable debug
   */
  SensorModuleAbs(const SensorModuleType sensor_type, const std::string& name,
                  const bool debug);

  /**
   * @brief Destroy the Sensor Module Abs object
   *
   */
  virtual ~SensorModuleAbs() = default;

  /**
   * @brief returns the type of the Sensor Module Abs object
   *
   * @return SensorModuleType sensor module's type
   */
  SensorModuleType SensorType() const;

 private:
  SensorModuleType sensor_type_{SensorModuleType::UNDEFINED};
};

}  // namespace core::sensors
#endif  // CORE_SENSORS_MODULE_SENSOR_HPP