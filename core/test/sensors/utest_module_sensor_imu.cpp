#include <gtest/gtest.h>

#include "core/sensors/module_sensor_imu.hpp"

using namespace core::sensors;
constexpr auto Name = "sens1";
constexpr auto Debug = false;

TEST(SensorModuleImu, Type)
{
  const ImuSensorModule imu(Name, Debug);
  EXPECT_EQ(core::utils::ModuleType::SENSOR, imu.Type());
}

TEST(SensorModuleImu, SensorType)
{
  const ImuSensorModule imu(Name, Debug);
  EXPECT_EQ(SensorModuleType::IMU, imu.SensorType());
}

TEST(SensorModuleType, Name)
{
  const ImuSensorModule imu(Name, Debug);
  EXPECT_EQ(Name, imu.Name());
}
