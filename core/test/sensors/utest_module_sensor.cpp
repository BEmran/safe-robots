#include <gtest/gtest.h>

#include "core/sensors/module_sensor.hpp"

using namespace core::sensors;

constexpr auto ModuleTypeSensor = core::utils::ModuleType::SENSOR;
constexpr auto UndefinedSensorType = SensorModuleType::UNDEFINED;

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
  const SensorModuleAbs sensor(UndefinedSensorType);
  EXPECT_EQ(ModuleTypeSensor, sensor.Type());
}

TEST(SensorModuleType, SensorType)
{
  const SensorModuleAbs sensor(UndefinedSensorType);
  EXPECT_EQ(UndefinedSensorType, sensor.SensorType());
}

TEST(SensorModuleType, DefaultName)
{
  const SensorModuleAbs sensor(UndefinedSensorType);
  EXPECT_EQ("Unnamed-UNDEFINED", sensor.Name());
}

TEST(SensorModuleType, Name)
{
  const std::string name{"name"};
  const SensorModuleAbs sensor(UndefinedSensorType, name);
  EXPECT_EQ(name, sensor.Name());
}
