#include "core/sensors/module_sensor.hpp"

namespace core::sensors
{
using namespace std::literals::string_literals;  // to use s literals

std::string SensorModuleTypeToString(const SensorModuleType type)
{
  switch (type)
  {
    case SensorModuleType::ADC:
      return "ADC"s;
    case SensorModuleType::BAR:
      return "BAROMETER"s;
    case SensorModuleType::GPS:
      return "GPS"s;
    case SensorModuleType::IMU:
      return "IMU"s;
    case SensorModuleType::ACCELOMETER:
      return "ACCELOMETER"s;
    case SensorModuleType::GYROSCOPE:
      return "GYROSCOPE"s;
    case SensorModuleType::MAGNETOMETER:
      return "MAGNETOMETER"s;
    case SensorModuleType::TEMPERATURE:
      return "TEMPERATURE"s;
    default:
      return "UNDEFINED"s;
  }
}

}  // namespace core::sensors