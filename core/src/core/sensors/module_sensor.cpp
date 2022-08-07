#include "core/sensors/module_sensor.hpp"

namespace core::sensors
{
using namespace std::literals::string_literals;       // to use s literals

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
    default:
      return "UNDEFINED"s;
  }
}

SensorModuleAbs::SensorModuleAbs(const SensorModuleType sensor_type)
  : SensorModuleAbs(sensor_type, "Unnamed-"s + SensorModuleTypeToString(sensor_type))
{
}

SensorModuleAbs::SensorModuleAbs(const SensorModuleType sensor_type,
                                 const std::string& name)
  : ModuleAbs(utils::ModuleType::SENSOR, name), sensor_type_{sensor_type}
{
}

SensorModuleType SensorModuleAbs::SensorType() const
{
  return sensor_type_;
}
}  // namespace core::sensors