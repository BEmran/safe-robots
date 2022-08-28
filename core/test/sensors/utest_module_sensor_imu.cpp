// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/sensors/module_sensor_imu.hpp"

using core::sensors::ImuSensorModule;
using core::sensors::SensorModuleType;
using core::utils::ModuleType;

constexpr auto Name = "sens1";
constexpr auto Debug = false;

TEST(SensorModuleImu, Type) {
  const ImuSensorModule imu(Name, Debug);
  EXPECT_EQ(ModuleType::SENSOR, imu.Type());
}

TEST(SensorModuleImu, SensorType) {
  const ImuSensorModule imu(Name, Debug);
  EXPECT_EQ(SensorModuleType::IMU, imu.SensorType());
}

TEST(SensorModuleType, Name) {
  const ImuSensorModule imu(Name, Debug);
  EXPECT_EQ(Name, imu.Name());
}
