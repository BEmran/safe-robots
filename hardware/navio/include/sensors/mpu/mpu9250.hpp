// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef SENSORS_MPU_MPU9250_HPP_
#define SENSORS_MPU_MPU9250_HPP_

#include <unistd.h>

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "core/sensors/module_sensor.hpp"
#include "core/utils/data.hpp"
#include "core/utils/node.hpp"
#include "navio/spi.hpp"
#include "sensors/common/utils.hpp"
#include "sensors/mpu/mpu9250_register_map.hpp"

namespace sensors::mpu {
namespace cu = common::utils;

struct SensorRawData {
  cu::Vec3 accel = cu::Vec3::Zero();
  cu::Vec3 gyro = cu::Vec3::Zero();
  cu::Vec3 mag = cu::Vec3::Zero();
  cu::MATH_TYPE temp = 0;
  bool mag_over_flow = false;
};

struct Config {
  AccelScale accel_scale = AccelScale::FS_16G;
  AccelBandWidthHz accel_bw = AccelBandWidthHz::BW_44HZ;
  GyroScale gyro_scale = GyroScale::FS_2000DPS;
  GyroBandWidthHz gyro_bw = GyroBandWidthHz::BW_184HZ;
  MagScale mag_scale = MagScale::FS_16BITS;
  MagMode mag_mode = MagMode::CONTINUES_100HZ;
  uint8_t sample_rate_divisor = 4;
};

class Mpu9250 : public cu::ImuSensorModule {
  using AccelData = core::utils::AccelData;
  using GyroData = core::utils::GyroData;
  using MagData = core::utils::MagData;
  using TemperatureData = core::utils::TemperatureData;
  typedef std::array<int16_t, 3> SensorFullBits;
  static constexpr const char* SensorName = "mpu9250";

 public:
  Mpu9250(const Config& config, std::unique_ptr<navio::SPI> comm,
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

  /**
   * @brief reset sensor registers
   *
   */
  void Reset() const;

  /**
   * @brief Initialize accelerometer full-scale range and sample rate
   * configuration
   *
   */
  void InitializeAccel() const;

  /**
   * @brief Initialize Gyro and Thermometer bandwidths and gyroscope full scale
   * configuration range
   *
   */
  void InitializeGyro() const;

  /**
   * @brief Initialize the magnetometer for continuous mode data acquisition and
   * sample rates configuration
   *
   */
  void InitializeMag() const;

  /**
   * @brief Apply sensor specs on the passed raw data
   *
   * @param raw raw data
   * @return ImuData holds the proceeded data
   */
  cu::ImuData ApplySensorSpecs(const SensorRawData& raw) const;

  static std::vector<int16_t> ExtractFullBits(const std::vector<uint8_t>& data);
  static SensorRawData FullBitsToRawData(const std::vector<int16_t>& full_bits);

  cu::ImuData ReadAccelGyroTemp() const;
  MagData ReadMagnetometer() const;

  AccelData ExtractAccelerometer(const SensorFullBits& full_bits) const;
  GyroData ExtractGyroscope(const SensorFullBits& full_bits) const;
  MagData ExtractMagnetometer(const SensorFullBits& full_bits,
                              bool over_flow) const;
  static cu::Vec3 EstimateRPY(const cu::ImuData& imu);

  AccelBandWidthHz ReadAccelBandWidth() const;
  AccelScale ReadAccelScale() const;
  GyroBandWidthHz ReadGyroBandWidth() const;
  GyroScale ReadGyroScale() const;
  MagMode ReadMagMode() const;
  MagScale ReadMagScale() const;
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
  void RequestReadAK8963Registers(uint8_t reg, uint8_t count) const;
  std::vector<uint8_t> ReadAK8963Registers(uint8_t reg, uint8_t count) const;
  void SetRegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const;
  void WriteRegister(uint8_t reg, uint8_t data) const;
  void WriteAK8963Register(uint8_t reg, uint8_t data) const;

 private:
  Config config_;
  std::unique_ptr<navio::SPI> comm_;
  std::unique_ptr<core::utils::Node> node_;
  mutable std::map<core::sensors::SensorModuleType, cu::SensorSpecs>
    sensor_specs_map_;

  cu::Vec3 mag_sensitivity_calibration_{1.0F, 1.0F,
                                        1.0F};  // factory calibration
};
}  // namespace sensors::mpu
#endif  // SENSORS_MPU_MPU9250_HPP_
