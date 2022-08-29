// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/sensors/module_sensor_imu.hpp"

namespace core::sensors {
ImuSensorModule::ImuSensorModule(const std::string& name, bool debug)
  : SensorModuleAbs(SensorModuleType::IMU, name, debug) {
}

void ImuSensorModule::Initialize() {
}

bool ImuSensorModule::Probe() {
  return false;
}

bool ImuSensorModule::Test() {
  return false;
}

void ImuSensorModule::Update() {
}
}  // namespace core::sensors
