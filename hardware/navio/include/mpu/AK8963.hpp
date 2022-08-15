#ifndef MPU_AK8963_HPP
#define MPU_AK8963_HPP

#include "mpu/my_utils.hpp"

#include <core/sensors/module_sensor.hpp>
#include <core/utils/data.hpp>

#include <stdint.h>
#include <unistd.h>
#include <array>

namespace mpu
{

void delay(uint32_t msec);

using MagData = core::utils::MagData;
using SensorModuleMagnetometer = core::sensors::SensorModuleAbs<MagData>;
constexpr auto MagnetometerType = core::sensors::SensorModuleType::MAGNETOMETER;
constexpr const char* MagnetometerSensorName = "AK8963";

enum class MagScale : uint8_t
{
  MFS_14BITS = 0x00,  // 0.6 mG per LSB
  MFS_16BITS = 0x10   // 0.15 mG per LSB
};

enum class MagMode : uint8_t
{
  CONTINUES_8HZ_MODE = 0x02,
  CONTINUES_100HZ_MODE = 0x06
};

struct AK8963Config
{
  MagScale scale;
  MagMode mode;
};

class AK8963 : public SensorModuleMagnetometer
{
 public:
  AK8963(const AK8963Config& config, const bool debug);

  void Initialize() override;

  bool Probe() override;

  bool Test() override;

  void Update() override;

  void Calibrate() override;

 protected:
  void Reset();

  std::array<int16_t, 3> ReadMagFullBits() const;
  MagData ReadMagnetometer() const;
  void MagRes();

 private:
  typedef enum
  {
    POWER_DOWN_MODE = 0x00,
    SINGLE_MEASUREMENT_MODE = 0x01,
    EXTERNAL_TRIGGER_MODE = 0x04,
    SELF_TEST_MODE = 0x08,
    FUSE_ROM_ACCESS_MODE = 0x0F
  } mode_t;

  /**
   * @brief Extract the factory calibration for each magnetometer axis
   *
   */
  void ExtractSensitivityAdjustmentValues();
  void ConfigureScaleAndMode() const;
  uint8_t Mode() const;
  uint8_t Scale() const;

  uint8_t ReadRegister(uint8_t reg) const;
  void WriteRegister(const uint8_t reg, const uint8_t data) const;
  void ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t* dest) const;

  AK8963Config config_;
  float resolution_{1.0};  // bit resolution
  std::array<float, 3> sensitivity_calibration_ = {
      1.0F, 1.0F, 1.0F};  // factory calibration
  std::array<float, 3> bias_correction_ = {0.0F, 0.0F,
                                           0.0F};  // hard iron correction
  std::array<float, 3> scale_correction_ = {1.0F, 1.0F,
                                            1.0F};  // soft iron correction
};
};      // namespace mpu
#endif  // MPU_AK8963_HPP