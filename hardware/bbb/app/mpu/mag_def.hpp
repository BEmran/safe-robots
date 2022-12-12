#ifndef MAG_DEF_HPP
#define MAG_DEF_HPP

#include <array>
#include <cstdint>

enum class MagSelect { NONE, DIRECT, SLAVE };

/**
 * @brief configuration of the mpu sensor
 *
 */
struct MagConfig {
  /// @brief compass sampling rate (Hz). Sampling rate must be between 1Hz and
  /// 100Hz.
  uint16_t sample_rate{100};
  // select tye of magnetometer
  MagSelect select{MagSelect::NONE};
};

/**
 * @brief data struct populated with new sensor data
 *
 */
struct MagData {
  /// @brief magnetometer (XYZ) in units of uT
  std::array<double, 3> calib{0., 0., 0.};
  /// @brief raw (XYZ) from 16-bit ADC
  std::array<int16_t, 3> raw{0, 0, 0};
};

#endif  // MAG_DEF_HPP
