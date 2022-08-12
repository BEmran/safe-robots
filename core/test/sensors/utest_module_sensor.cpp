#include <gtest/gtest.h>

#include "core/sensors/module_sensor.hpp"

using namespace core::sensors;

constexpr auto ModuleTypeSensor = core::utils::ModuleType::SENSOR;
constexpr auto UndefinedSensorType = SensorModuleType::UNDEFINED;
constexpr auto Name = "name";
constexpr auto Debug = false;

TEST(SensorModuleType, ToString)
{
  EXPECT_EQ("ADC", SensorModuleTypeToString(SensorModuleType::ADC));
  EXPECT_EQ("BAROMETER", SensorModuleTypeToString(SensorModuleType::BAR));
  EXPECT_EQ("GPS", SensorModuleTypeToString(SensorModuleType::GPS));
  EXPECT_EQ("IMU", SensorModuleTypeToString(SensorModuleType::IMU));
  EXPECT_EQ("UNDEFINED", SensorModuleTypeToString(SensorModuleType::UNDEFINED));
}

TEST(SensorModuleType, Type)
{
  const SensorModuleAbs sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(ModuleTypeSensor, sensor.Type());
}

TEST(SensorModuleType, SensorType)
{
  const SensorModuleAbs sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(UndefinedSensorType, sensor.SensorType());
}
TEST(SensorModuleType, Name)
{
  const SensorModuleAbs sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(Name, sensor.Name());
}
