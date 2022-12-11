#ifndef RC_MAG_HPP
#define RC_MAG_HPP

#include <array>
#include <cstdint>

#include "i2c.hpp"
#include "mpu_defs.h"

/**
 * @brief configuration of the mpu sensor
 *
 */
struct MagConfig {
  /// @brief compass sampling rate (Hz). Sampling rate must be between 1Hz and
  /// 100Hz.
  uint16_t sample_rate{100};
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

class Mag {
 public:
  /**
   * @brief Sets up the MPU in normal one-shot sampling mode.
   *
   * First create an instance of the MpuData struct and pass its pointer to
   * rc_mpu_initialize which will then write to. Also pass an MpuConfig
   * struct with your configuration settings.
   *
   * This function will populate the accel_to_ms2 and gyro_to_deg fields of the
   * MpuData struct appropriately based on the user-configured full scale
   * ranges.
   *
   * After this, you may read sensor data at any time with the functions
   * rc_mpu_read_accel(), rc_mpu_read_gyro(), and rc_mpu_read_temp(). The
   * magnetometer can also be read with rc_mpu_read_mag() if using an MPU9150 or
   * MPU9250 and the enable_magnetometer field in the MpuConfig struct has
   * been set to 1.
   *
   * Be sure to power off the MPU at the end of your program with
   * rc_mpu_power_off().
   *
   * @param data pointer to user's data struct
   * @param[in] conf user configuration data
   *
   * @return 0 on success or -1 on failure.
   */
  bool Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf);

  bool Prop();
  bool ExtractFactoryCalibration();
  bool SetMode(const MagMode mode, const MagBitScale scale);

  /**
   * @brief Set compass sampling rate.
   * @details The compass on the auxiliary I2C bus is read by the MPU hardware
   * at a maximum of 100Hz. The actual rate can be set to a fraction of the gyro
   * sampling rate.
   *
   * @param[in] rate Desired compass sampling rate (Hz).
   * @return true if successful
   * @return false otherwise
   */
  bool SetSampleRate(const uint16_t rate);

  /**
   * @brief Powers off the MPU
   *
   * Only call this after powering on the MPU with rc_mpu_initialize or
   * rc_mpu_initialize_dmp. This should generally be called at the end of your
   * main function to make sure the MPU is put to sleep.
   *
   * @return 0 on success or -1 on failure.
   */
  bool PowerOff();

  /**
   * int SetBypass(unsigned char bypass_on)
   *
   * configures the USER_CTRL and INT_PIN_CFG registers to turn on and off the
   * i2c bypass mode for talking to the magnetometer. In random read mode this
   * is used to turn on the bypass and left as is. In DMP mode bypass is turned
   * off after configuration and the MPU fetches magnetometer data
   *automatically. USER_CTRL - based on global variable dsp_en INT_PIN_CFG based
   *on requested bypass state
   **/
  bool SetBypass(const bool enable);

  /**
   * @brief Reads magnetometer data from the MPU
   *
   * Note this requires use of an MPU9150 or MPU9250, the MPU6050 and MPU6500 do
   * not have magnetometers. Additionally, the enable_magnetometer flag must has
   * been set in the user's MpuConfig when it was passed to
   * rc_mpu_initialize()
   *
   * @param data Pointer to user's data struct where new data will be
   * written
   *
   * @return 0 on success or -1 on failure.
   */
  std::optional<std::array<int16_t, 3>> ReadRaw();
  std::array<double, 3> ReadCalibrated(const std::array<int16_t, 3>& raw);
  MagData ReadData();

  bool WriteByte(const uint8_t reg, const uint8_t data);
  std::vector<uint8_t> ReadBytes(const uint8_t reg, const uint8_t count);
  std::optional<uint8_t> ReadByte(const uint8_t reg);

 private:
  MagConfig config_;
  std::shared_ptr<I2C> i2c_;
  bool bypass_enabled_{false};
  std::array<double, 3> factory_adjust_{0., 0., 0.};
  std::array<double, 3> offsets_{0., 0., 0.};
  std::array<double, 3> scales_{1., 1., 1.};
};

#endif  // RC_MAG_HPP
