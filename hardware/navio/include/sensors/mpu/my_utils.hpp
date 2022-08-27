#ifndef MPU_MY_UTILS_HPP
#define MPU_MY_UTILS_HPP

#include <stdint.h>
#include <unistd.h>
#include <array>

#include <core/sensors/module_sensor.hpp>
#include <core/utils/data.hpp>
#include <core/utils/math.hpp>

namespace mpu
{

constexpr int max_bit_val = 32767;  // Measurement range is from -32760 ~ +32760
                                    // decimal in 16-bit output.
constexpr int max_utesla = 10 * 4912;  // Magnetic flux density in milliGauss

using core::utils::GRAVITY;
using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::PI;
using core::utils::Vec3;

using core::utils::ImuData;
using ImuSensorModule = core::sensors::SensorModuleAbs<ImuData>;

/**
 * @brief User litterer to define a number of type uint8_t
 *
 */
inline constexpr uint8_t operator"" _uc(unsigned long long arg) noexcept
{
  return static_cast<uint8_t>(arg);
}

/**
 * @brief Hold sensor measurement specifications used to convert sensor raw
 * reading to usable data (scalded and in iso unit)
 *
 */
struct SensorSpecs
{
  MATH_TYPE sensitivity;  // the smallest absolute amount of change that can be
                          // detected by a measurement = max_bit_count / scale
  MATH_TYPE unit_conversion;  // convert a raw value into iso unit

  Vec3 bias = Vec3::Zero();
  Vec3 offset = Vec3::Zero();
  Mat3 misalignment = Mat3::Identity();
  Mat3 A = Mat3::Identity();
  Vec3 b = Vec3::Zero();
  /**
   * @brief Construct a new Sensor Specs object
   *
   */
  SensorSpecs() : SensorSpecs(1.0, 1.0)
  {
  }

  /**
   * @brief Construct a new Sensor Specs object
   *
   * @param sen sensitivity value
   * @param unit unit conversion row -> iso unit
   */
  SensorSpecs(const MATH_TYPE sen, const MATH_TYPE unit)
    : sensitivity(sen), unit_conversion(unit)
  {
  }

  /**
   * @brief Construct a new Sensor Specs object
   *
   * @param sen sensitivity value
   * @param unit unit conversion row -> iso unit
   */
  SensorSpecs(const MATH_TYPE sen, const MATH_TYPE unit, const Vec3& bias_,
              const Vec3& offset_)
    : sensitivity(sen), unit_conversion(unit), bias(bias_), offset(offset_)
  {
    UpdateEquation();
  }

  /**
   * @brief apply specs on a raw measurement
   *
   * @param raw raw data
   * @return MATH_TYPE the post proceeded data
   */
  MATH_TYPE Apply(const MATH_TYPE raw) const
  {
    // return (raw - bias[0]) * (unit_conversion / sensitivity) + offset[0];
    return A(0) * raw + b[0];
  }

  void SetCalibration(const Mat3& m, const Vec3& bias_, const Vec3& offset_)
  {
    misalignment = m;
    bias = bias_;
    offset = offset_;
    UpdateEquation();
  }

  void UpdateEquation()
  {
    A = misalignment * unit_conversion / sensitivity;
    b = misalignment * (offset - bias / sensitivity) * unit_conversion;
  }

  /**
   * @brief apply specs on a vector raw measurement
   *
   * @param raw raw data vector
   * @return Vec3 the post proceeded data
   */
  Vec3 Apply(const Vec3& raw) const
  {
    // return ((misalignment * (raw - bias) / sensitivity) + offset) *
    // unit_conversion;
    return A * raw + b;
  }
};

/**
 * @brief Construct a new Sensor Specs object using scale and uint info
 *
 * @param scale measurement scale
 * @param unit unit conversion row -> iso unit
 */
SensorSpecs CreateSensorSpecs(const MATH_TYPE scale, const MATH_TYPE unit);

/**
 * @brief Sleep for some milliseconds, calls linux usleep function
 *
 * @param msec milli seconds to put thread in sleep
 */
void Delay(uint32_t msec);

// void ResetSensors();

/**
 * @brief Turn the MSB and LSB into a signed 16-bit value
 *
 * @param msb most significant bits (High 8-bits)
 * @param lsb least significant bits (Low 8-bits)
 * @return int16_t full number
 */
int16_t To16Bit(const uint8_t msb, const uint8_t lsb);

/**
 * @brief Convert a std::array data to Vec3
 *
 * @param array std array of size 3
 * @return Vec3 vector of dim 3
 */
Vec3 ArrayToVec3(const std::array<MATH_TYPE, 3>& array);

Vec3 Vec3From16BitsVector(const std::vector<int16_t>::const_iterator begin);

/**
 * @brief Apply sensor specs on the passed raw data
 *
 * @param raw raw data
 * @param post post process data
 * @return std::array<MATH_TYPE, 3> std array of size 3 holds the proceeded data
 */
std::array<MATH_TYPE, 3> ApplySensorSpecs(const std::array<int16_t, 3>& raw,
                                          const SensorSpecs& spec);
                                          
Vec3 ApplySensorSpecs(const Vec3& raw, const SensorSpecs& spec);

/**
 * @brief Estimate roll and pitch angles from accelerometer
 * 
 * @param accel accelerometer sensor data
 * @return Vec3 angle vector [roll, pitch, yaw]
 */
Vec3 EstimateRPY(const Vec3& accel);

}  // namespace mpu

#endif  // MPU_MY_UTILS_HPP