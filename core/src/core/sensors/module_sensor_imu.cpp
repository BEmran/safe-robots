#include "core/sensors/module_sensor_imu.hpp"

namespace core::sensors
{
ImuSensorModule::ImuSensorModule(const std::string& name, const bool debug)
  : SensorModuleAbs(SensorModuleType::IMU, name, debug)
{
}

utils::ImuData ImuSensorModule::GetData() const
{
  return cashed_data_.Get();
}

void ImuSensorModule::SetData(const utils::ImuData& data)
{
  return cashed_data_.Set(data);
}

void ImuSensorModule::SetData()
{
  return cashed_data_.Set(data_);
}

void ImuSensorModule::ClearData()
{
  return cashed_data_.Clear();
}
}  // namespace core::sensors