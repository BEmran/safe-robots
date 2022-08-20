#ifndef MPU_MY_UTILS_HPP
#define MPU_MY_UTILS_HPP

#include <stdint.h>
#include <unistd.h>
#include <array>

#include "mpu/spi.hpp"
#include "mpu/mpu9250_register_map.h"
#include <core/utils/math.hpp>

namespace mpu
{

constexpr int max_bit_val = 32767;  // Measurement range is from -32760 ~ +32760
                                    // decimal in 16-bit output.
constexpr int max_utesla = 4912;  // Magnetic flux density in micro Tesla

using core::utils::GRAVITY;
using core::utils::MATH_TYPE;
using core::utils::PI;

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
  /**
   * @brief Construct a new Sensor Specs object
   *
   */
  SensorSpecs() : sensitivity(1.0), unit_conversion(1.0)
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
   * @brief apply specs on a raw measurement
   *
   * @param raw raw data
   * @return MATH_TYPE the post proceeded data
   */
  MATH_TYPE Apply(const MATH_TYPE raw) const
  {
    return raw / sensitivity * unit_conversion;
  }
};

/**
 * @brief Construct a new Sensor Specs object using scale and uint info
 *
 * @param scale measurement scale
 * @param unit unit conversion row -> iso unit
 */
SensorSpecs CreateSensorSpecs(const MATH_TYPE scale, const MATH_TYPE unit);

// /**
//  * @brief Get the Spi object
//  *
//  * @return spi::SPI* pointer to local static variable
//  */
// spi::SPI* GetSpi();

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
 * @return core::utils::Vec3 vector of dim 3
 */
core::utils::Vec3 ArrayToVec3(const std::array<MATH_TYPE, 3>& array);

/**
 * @brief Apply sensor specs on the passed raw data
 *
 * @param raw raw data
 * @param post post process data
 * @return std::array<MATH_TYPE, 3> std array of size 3 holds the proceeded data
 */
std::array<MATH_TYPE, 3> ApplySensorSpecs(const std::array<int16_t, 3>& raw,
                                          const SensorSpecs& spec);

void PrintVec(const std::vector<uint8_t>& vec);

}  // namespace mpu

#endif  // MPU_MY_UTILS_HPP