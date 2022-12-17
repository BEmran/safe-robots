// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_SENSORS_MPU_MPU9250_HPP_
#define HARDWARE_COMMON_SENSORS_MPU_MPU9250_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "common/comm/communication_abs.hpp"
#include "common/sensors/mpu/def.hpp"
#include "common/sensors/mpu/mpu9250_register_map.hpp"
#include "common/sensors/utils.hpp"
#include "core/sensors/module_sensor.hpp"
#include "core/utils/data.hpp"
#include "core/utils/node.hpp"

namespace hardware::common::sensors::mpu {
using ImuData = core::utils::ImuData;

class Mpu9250 : public ImuSensorModule {
  using AccelData = core::utils::AccelData;
  using GyroData = core::utils::GyroData;
  using MagData = core::utils::MagData;
  using TemperatureData = core::utils::TemperatureData;
  typedef std::array<int16_t, 3> SensorFullBits;
  static constexpr const char* SensorName = "mpu9250";

 public:
  Mpu9250(const Config& config, std::unique_ptr<comm::CommunicationAbs> comm,
          std::unique_ptr<core::utils::Node> node);

  /**
   * @implement Initialize function
   *
   */
  void Initialize() override;

  /**
   * @brief Check if able to communicate with MPU and AK8963 sensors
   *
   * @return true if both sensors can be probe
   * @return false otherwise
   */
  bool Probe() override;

  bool Test() override;

  void Update() override;

  void Calibrate() override;

  SensorRawData ReadRawData() const;

  /**
   * @brief reset sensor registers
   *
   */
  bool Reset() const;

 protected:
  void ReadCalibrationFile();

  void ConfigureI2C() const;
  /**
   * @brief Check MPU WHO AM I register, expected value is 0x71 (decimal 113)
   *
   * @return true if able to communicate with MPU sensor
   * @return false otherwise
   */
  bool ProbeMpu() const;

  /**
   * @brief Check Ak8963 WHO AM I register, expected value is 0x48 (decimal 72)
   *
   * @return true if able to communicate with AK8963 sensor
   * @return false otherwise
   */
  bool ProbeAk8963() const;

  bool ResetMpu() const;
  bool RestAk8963() const;

  /**
   * @brief Initialize accelerometer full-scale range and sample rate
   * configuration
   *
   */
  bool InitializeAccel() const;

  /**
   * @brief Initialize Gyro and Thermometer bandwidths and gyroscope full scale
   * configuration range
   *
   */
  bool InitializeGyro() const;

  /**
   * @brief Initialize the magnetometer for continuous mode data acquisition and
   * sample rates configuration
   *
   */
  bool InitializeMag() const;

  /**
   * @brief Apply sensor specs on the passed raw data
   *
   * @param raw raw data
   * @return ImuData holds the proceeded data
   */
  ImuData ApplySensorSpecs(const SensorRawData& raw) const;

  static std::vector<int16_t> ExtractFullBits(const std::vector<uint8_t>& data);
  static SensorRawData FullBitsToRawData(const std::vector<int16_t>& full_bits);

  bool EnableAutoRequest() const;

  AccelData ExtractAccelerometer(const SensorFullBits& full_bits) const;
  GyroData ExtractGyroscope(const SensorFullBits& full_bits) const;
  MagData ExtractMagnetometer(const SensorFullBits& full_bits,
                              bool over_flow) const;
  static Vec3 EstimateRPY(const ImuData& imu);

  bool SetAccelBW() const;
  bool SetAccelScale() const;
  bool SetGyroBW() const;
  bool SetGyroScale() const;
  bool SetMagMode(const std::optional<MagMode> mag_mode) const;
  bool SetMagScale() const;
  uint8_t SetSampleRateDevisor() const;

  std::optional<AccelBW> ReadAccelBW() const;
  std::optional<AccelScale> ReadAccelScale() const;
  std::optional<GyroBW> ReadGyroBW() const;
  std::optional<GyroScale> ReadGyroScale() const;
  std::optional<MagMode> ReadMagMode() const;
  std::optional<MagScale> ReadMagScale() const;
  uint8_t ReadSampleRateDevisor() const;
  std::pair<bool, Config> ValidateConfiguration() const;

  /**
   * @brief Extract magnetometer manufacture sensitivity adjustment values
   * @details calculate xyz-axis sensitivity using manufacture formula
   */
  void ExtractMagnetometerSensitivityAdjustmentValues();

  uint8_t ReadRegister(uint8_t reg) const;
  std::vector<uint8_t> ReadRegisters(uint8_t reg, uint8_t count) const;
  uint8_t ReadAK8963Register(uint8_t reg) const;
  bool RequestReadAK8963Registers(uint8_t reg, uint8_t count) const;
  std::vector<uint8_t> ReadAK8963Registers(uint8_t reg, uint8_t count) const;
  bool SetRegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const;
  bool SetAK8963RegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const;
  bool WriteRegister(uint8_t reg, uint8_t data) const;
  bool WriteAK8963Register(uint8_t reg, uint8_t data) const;

 private:
  Config config_;
  std::unique_ptr<comm::CommunicationAbs> comm_;
  std::unique_ptr<core::utils::Node> node_;
  SensorSpecs<3> mag_spec_;
  SensorSpecs<3> gyro_spec_;
  SensorSpecs<3> accel_spec_;
  SensorSpecs<1> temp_spec_;

  Vec3 mag_sensitivity_calibration_{1.0F, 1.0F, 1.0F};  // factory calibration
};
}  // namespace hardware::common::sensors::mpu
#endif  // SENSORS_MPU_MPU9250_HPP_
