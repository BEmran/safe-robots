// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef SENSORS_LSM_LSM9DS1_HPP_
#define SENSORS_LSM_LSM9DS1_HPP_

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
#include "sensors/lsm/lsm9ds1_register_map.hpp"

namespace sensors::lsm {
namespace cu = common::utils;

struct SensorRawData {
  cu::Vec3 accel = cu::Vec3::Zero();
  cu::Vec3 gyro = cu::Vec3::Zero();
  cu::Vec3 mag = cu::Vec3::Zero();
  cu::MATH_TYPE temp = 0;
};

struct Config {
  bool accel_bw_selection = false;
  bool accel_high_res_enabled = false;
  AccelODR accel_odr = AccelODR::ODR_952HZ;
  AccelScale accel_scale = AccelScale::SCALE_16G;
  AccelAntiAliasingBW accel_anti_alias = AccelAntiAliasingBW::AA_BW_408;
  AccelHighResolutionBW accel_high_res_bw = AccelHighResolutionBW::HR_BW_DIV_50;

  GyroODR gyro_odr = GyroODR::ODR_952HZ;
  GyroScale gyro_scale = GyroScale::SCALE_245DPS;

  MagODR mag_odr = MagODR::ODR_80;
  MagScale mag_scale = MagScale::SCALE_16GS;
  MagXYMode mag_xy_mode = MagXYMode::ULTRA_HIGH;
  MagZMode mag_z_mode = MagZMode::ULTRA_HIGH;
  MagOperatingMode mag_operating_mode = MagOperatingMode::CONTINUOUS;
  MagTempCompensation mag_temp_compensation = MagTempCompensation::DISABLED;
};

class Lsm94s1 : public cu::ImuSensorModule {
  using AccelData = core::utils::AccelData;
  using GyroData = core::utils::GyroData;
  using MagData = core::utils::MagData;
  using TemperatureData = core::utils::TemperatureData;
  typedef std::array<int16_t, 3> SensorFullBits;
  static constexpr const char* SensorName = "lsm9ds1";

 public:
  Lsm94s1(const Config& config, std::unique_ptr<navio::SPI> comm_ag,
          std::unique_ptr<navio::SPI> comm_mag,
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

  /**
   * @brief Check MPU WHO AM I register, expected value is 0x71 (decimal 113)
   *
   * @return true if able to communicate with MPU sensor
   * @return false otherwise
   */
  bool ProbeAG() const;

  /**
   * @brief Check Ak8963 WHO AM I register, expected value is 0x48 (decimal 72)
   *
   * @return true if able to communicate with AK8963 sensor
   * @return false otherwise
   */
  bool ProbeMag() const;

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

  cu::Vec3 ReadRawAccelData() const;
  cu::Vec3 ReadRawGyroData() const;
  cu::Vec3 ReadRawMagData() const;
  cu::MATH_TYPE ReadRawTemperatureData() const;

  static std::vector<int16_t> ExtractFullBits(const std::vector<uint8_t>& data);

  /**
   * @brief Apply sensor specs on the passed raw data
   *
   * @param raw raw data
   * @return ImuData holds the proceeded data
   */
  cu::ImuData ApplySensorSpecs(const SensorRawData& raw) const;

  static cu::Vec3 EstimateRPY(const cu::ImuData& imu);

  AccelScale ReadAccelScale() const;
  AccelODR ReadAccelODR() const;
  AccelAntiAliasingBW ReadAccelAccelAntiAlias() const;
  AccelHighResolutionBW ReadAccelHighResBW() const;
  GyroODR ReadGyroODR() const;
  GyroScale ReadGyroScale() const;
  MagODR ReadMagODR() const;
  MagXYMode ReadMagXYMode() const;
  MagZMode ReadMagZMode() const;
  MagOperatingMode ReadMagOperationMode() const;
  MagScale ReadMagScale() const;

  std::pair<bool, Config> ValidateConfiguration() const;

  // /**
  //  * @brief Extract magnetometer manufacture sensitivity adjustment values
  //  * @details calculate xyz-axis sensitivity using manufacture formula
  //  */
  // void ExtractMagnetometerSensitivityAdjustmentValues();

  uint8_t ReadAGRegister(uint8_t reg) const;
  std::vector<uint8_t> ReadAGRegisters(uint8_t reg, uint8_t count) const;
  uint8_t ReadMagRegister(uint8_t reg) const;
  std::vector<uint8_t> ReadMagRegisters(uint8_t reg, uint8_t count) const;
  void WriteAGRegister(uint8_t reg, uint8_t data) const;
  void WriteMagRegister(uint8_t reg, uint8_t data) const;
  void SetAGRegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const;
  void SetMagRegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const;
  // void RequestReadMagRegisters(uint8_t reg, uint8_t count) const;

 private:
  Config config_;
  std::unique_ptr<navio::SPI> comm_ag_;
  std::unique_ptr<navio::SPI> comm_mag_;
  std::unique_ptr<core::utils::Node> node_;
  cu::SensorSpecs<3> mag_spec_;
  cu::SensorSpecs<3> gyro_spec_;
  cu::SensorSpecs<3> accel_spec_;
  cu::SensorSpecs<1> temp_spec_;

  cu::Vec3 mag_sensitivity_calibration_{1.0F, 1.0F,
                                        1.0F};  // factory calibration
};
}  // namespace sensors::lsm
#endif  // SENSORS_LSM_LSM9DS1_HPP_
