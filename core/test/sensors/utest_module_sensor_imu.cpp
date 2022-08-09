#include <gtest/gtest.h>

#include "core/sensors/module_sensor_imu.hpp"

using namespace core::sensors;
constexpr auto Name = "sens1";

TEST(SensorModuleImu, Type)
{
  const ImuSensorModule imu(Name);
  EXPECT_EQ(core::utils::ModuleType::SENSOR, imu.Type());
}

TEST(SensorModuleImu, SensorType)
{
  const ImuSensorModule imu(Name);
  EXPECT_EQ(SensorModuleType::IMU, imu.SensorType());
}

TEST(SensorModuleType, Name)
{
  const ImuSensorModule imu(Name);
  EXPECT_EQ(Name, imu.Name());
}
