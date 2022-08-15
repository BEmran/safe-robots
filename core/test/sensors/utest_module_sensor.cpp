#include <gtest/gtest.h>

#include "core/sensors/module_sensor.hpp"
#include "core/utils/data.hpp"

using namespace core::sensors;
using core::utils::DoubleData;

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
  const SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(ModuleTypeSensor, sensor.Type());
}

TEST(SensorModuleType, SensorType)
{
  const SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(UndefinedSensorType, sensor.SensorType());
}

TEST(SensorModuleType, Name)
{
  const SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(Name, sensor.Name());
}

TEST(SensorModuleType, Prope)
{
  SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(false, sensor.Probe());
}

TEST(SensorModuleType, Init)
{
  SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  sensor.Initialize();
  EXPECT_DOUBLE_EQ(0.0, sensor.GetData().value);
}

TEST(SensorModuleType, Test)
{
  SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  EXPECT_EQ(false, sensor.Test());
}

TEST(SensorModuleType, GetData)
{
  const SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  EXPECT_DOUBLE_EQ(0.0, sensor.GetData().value);
}

TEST(SensorModuleType, Update)
{
  SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  sensor.Update();
}

TEST(SensorModuleType, ClearData)
{
  SensorModuleAbs<DoubleData> sensor(UndefinedSensorType, Name, Debug);
  sensor.ClearData();
}