#ifndef MPU_MPU9250_HPP
#define MPU_MPU9250_HPP

#include "mpu/my_utils.hpp"

#include <core/sensors/module_sensor.hpp>
#include <core/utils/data.hpp>
#include <core/utils/node.hpp>

#include <stdint.h>
#include <unistd.h>
#include <array>
#include <map>
#include <memory>

namespace mpu
{

enum class GyroScale : uint8_t
{
  FS_250DPS,
  FS_500DPS,
  FS_1000DPS,
  FS_2000DPS
};

enum class GyroBandWidthHz : uint8_t
{
  BW_250HZ,
  BW_184HZ,
  BW_92HZ,
  BW_41HZ,
  BW_20HZ,
  BW_10HZ,
  BW_5HZ,
  BW_3600HZ
};

enum class AccelScale : uint8_t
{
  FS_2G,
  FS_4G,
  FS_8G,
  FS_16G
};

enum class AccelBandWidthHz : uint8_t
{
  BW_218HZ,
  BW_99HZ,
  BW_44HZ,
  BW_21HZ,
  BW_10HZ,
  BW_5HZ
};

enum class MagScale : uint8_t
{
  FS_14BITS,
  FS_16BITS
};

enum class MagMode : uint8_t
{
  POWER_DOWN,
  SINGLE_MEASUREMENT,
  CONTINUES_8HZ,
  EXTERNAL_TRIGGER,
  CONTINUES_100HZ,
  SELF_TEST,
  FUSE_ROM_ACCESS
};

struct SensorRawData
{
  Vec3 accel = Vec3::Zero();
  Vec3 gyro = Vec3::Zero();
  Vec3 mag = Vec3::Zero();
  MATH_TYPE temp = 0;
  bool mag_over_flow = false;
};

struct Config
{
  AccelScale accel_scale = AccelScale::FS_16G;
  AccelBandWidthHz accel_bw = AccelBandWidthHz::BW_44HZ;
  GyroScale gyro_scale = GyroScale::FS_2000DPS;
  GyroBandWidthHz gyro_bw = GyroBandWidthHz::BW_184HZ;
  MagScale mag_scale = MagScale::FS_16BITS;
  MagMode mag_mode = MagMode::CONTINUES_100HZ;
  uint8_t sample_rate_divisor = 4;
};

class Mpu9250 : public ImuSensorModule
{
  using AccelData = core::utils::AccelData;
  using GyroData = core::utils::GyroData;
  using MagData = core::utils::MagData;
  using TemperatureData = core::utils::TemperatureData;
  typedef std::array<int16_t, 3> SensorFullBits;
  static constexpr const char* SensorName = "mpu9250";

 public:
  Mpu9250(const Config& config, std::unique_ptr<SPI> comm,
          std::unique_ptr<core::utils::Node> node);

  void fake();

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

  ImuData ApplySensorSpecs(const SensorRawData& raw) const;

  static std::vector<int16_t> ExtractFullBits(const std::vector<uint8_t>& data);
  static SensorRawData FullBitsToRawData(const std::vector<int16_t>& full_bits);

  ImuData ReadAccelGyroTemp() const;
  MagData ReadMagnetometer() const;

  AccelData ExtractAccelerometer(const SensorFullBits& full_bits) const;
  GyroData ExtractGyroscope(const SensorFullBits& full_bits) const;
  MagData ExtractMagnetometer(const SensorFullBits& full_bits,
                              const bool over_flow) const;
  static Vec3 EstimateRPY(const ImuData& imu);

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

  uint8_t ReadRegister(const uint8_t reg) const;
  std::vector<uint8_t> ReadRegisters(const uint8_t reg,
                                     const uint8_t count) const;
  uint8_t ReadAK8963Register(const uint8_t reg) const;
  void RequestReadAK8963Registers(const uint8_t reg, const uint8_t count) const;
  std::vector<uint8_t> ReadAK8963Registers(const uint8_t reg,
                                           const uint8_t count) const;
  void SetRegisterByte(const uint8_t reg, const uint8_t byte,
                       const uint8_t mask) const;
  void WriteRegister(const uint8_t reg, const uint8_t data) const;
  void WriteAK8963Register(const uint8_t reg, const uint8_t data) const;

 private:
  Config config_;
  std::unique_ptr<SPI> comm_;
  std::unique_ptr<core::utils::Node> node_;
  mutable std::map<core::sensors::SensorModuleType, SensorSpecs>
      sensor_specs_map;

  std::array<float, 3> mag_sensitivity_calibration_ = {
      1.0F, 1.0F, 1.0F};  // factory calibration

  // std::array<float, 3> bias_correction_ = {0.0F, 0.0F,
  //                                          0.0F};  // hard iron correction
  // std::array<float, 3> scale_correction_ = {1.0F, 1.0F,
  //                                           1.0F};  // soft iron correction
};
}  // namespace mpu
#endif  // MPU_MPU9250_HPP