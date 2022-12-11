#ifndef RC_MPU_H
#define RC_MPU_H

#include <array>
#include <cstdint>
#include <memory>

#include "i2c.hpp"
#include "i2c_mag_slave.hpp"
#include "mag.hpp"
#include "mpu_defs.h"

/**
 * @brief accelerometer full scale range options
 *
 * @details The full scale ranges of the accelerometer have units of gravity
 * (G). The default values is ACCEL_FSR_2G.
 */
enum class MpuAccelFSR {
  ACCEL_FSR_2G,  //
  ACCEL_FSR_4G,  //
  ACCEL_FSR_8G,  //
  ACCEL_FSR_16G  //
};

/**
 * @brief gyroscope full scale range options
 *
 * @details The full scale ranges of the gyroscope have units of degrees per
 * second (DPS). The default values is GYRO_FSR_2000DPS.
 */
enum class MpuGyroFSR {
  GYRO_FSR_250DPS,
  GYRO_FSR_500DPS,
  GYRO_FSR_1000DPS,
  GYRO_FSR_2000DPS
};

/**
 * @brief accelerometer digital low-pass filter options
 *
 * @details The filter runs at 1kz and helps to reduce sensor noise when
 * sampling more slowly. The default values is ACCEL_DLPF_184. Lower cut-off
 * frequencies incur phase-loss in measurements. Number is cutoff frequency in
 * hz.
 */
enum class MpuAccelDLPF {
  ACCEL_DLPF_OFF,
  ACCEL_DLPF_460,
  ACCEL_DLPF_184,
  ACCEL_DLPF_92,
  ACCEL_DLPF_41,
  ACCEL_DLPF_20,
  ACCEL_DLPF_10,
  ACCEL_DLPF_5
};

/**
 * @brief gyroscope digital low-pass filter options
 *
 * @details The filter runs at 1kz and helps to reduce
 * sensor noise when sampling more slowly. The default values is GYRO_DLPF_184.
 * Lower cut-off frequencies incur phase-loss in measurements. Number is cutoff
 * frequency in hz.
 */
enum class MpuGyroDLPF {
  GYRO_DLPF_OFF,
  GYRO_DLPF_250,
  GYRO_DLPF_184,
  GYRO_DLPF_92,
  GYRO_DLPF_41,
  GYRO_DLPF_20,
  GYRO_DLPF_10,
  GYRO_DLPF_5
};

/**
 * @brief configuration of the mpu sensor
 */
struct MpuConfig {
  /// @brief which ship address to use default is 0x68 or 0x69
  uint8_t i2c_addr{MPU_DEFAULT_I2C_ADDR};
  /// @brief sampling rate (Hz). Sampling rate must be between 4Hz and 1kHz.
  uint16_t sample_rate{200};
  /// @brief accelerometer full scale range default ACCEL_FSR_8G
  MpuAccelFSR accel_fsr{MpuAccelFSR::ACCEL_FSR_8G};
  /// @brief gyroscope full scale range default GYRO_FSR_2000DPS
  MpuGyroFSR gyro_fsr{MpuGyroFSR::GYRO_FSR_2000DPS};
  /// @brief accel internal low pass filter cutoff default ACCEL_DLPF_184
  MpuAccelDLPF accel_dlpf{MpuAccelDLPF::ACCEL_DLPF_184};
  /// @brief gyro internal low pass filter cutoff default GYRO_DLPF_184
  MpuGyroDLPF gyro_dlpf{MpuGyroDLPF::GYRO_DLPF_184};
  /// @brief magnetometer use is optional, true to enable and false to disable,
  /// default true
  bool enable_magnetometer{true};
};

/**
 * @brief data struct populated with new sensor data
 *
 */
struct MpuData {
  /// @brief accelerometer (XYZ) in units of m/s^2
  std::array<double, 3> accel{0., 0., 0.};
  /// @brief gyroscope (XYZ) in units of degrees/s
  std::array<double, 3> gyro{0., 0., 0.};
  /// @brief magnetometer (XYZ) in units of uT
  std::array<double, 3> mag{0., 0., 0.};
  /// @brief thermometer, in units of degrees Celsius
  double temp{0.};
  /// @brief raw gyroscope (XYZ)from 16-bit ADC
  std::array<int16_t, 3> raw_gyro{0, 0, 0};
  /// @brief raw accelerometer (XYZ) from 16-bit ADC
  std::array<int16_t, 3> raw_accel{0, 0, 0};
  /// @brief raw magnetometer (XYZ) from 16-bit ADC
  std::array<int16_t, 3> raw_mag{0, 0, 0};
};

class MPU {
 public:
  MPU(const int i2c_bus)
    : i2c_{std::make_shared<I2C>()}
    , mag_{std::make_shared<Mag>()}
    , i2c_bus_{i2c_bus} {
  }

  ~MPU() {
    i2c_->Close();
  }

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
  bool Initialize(const MpuConfig& conf);

  /**
   * @brief Prop MPU sensor by checking the who_am_i register to make sure the
   * chip is alive.
   *
   * @return true if successful
   * @return false otherwise
   */
  bool Prop();

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

  bool ResetMpu();

  /**
   * @brief Reads accelerometer data from the MPU
   *
   * @param data Pointer to user's data struct where new data will be
   * written
   *
   * @return 0 on success or -1 on failure.
   */
  std::optional<std::array<int16_t, 3>> ReadAccelRaw();
  std::array<double, 3> ReadAccelCalibrated(const std::array<int16_t, 3>& raw);
  /**
   * @brief Reads gyroscope data from the MPU
   *
   * @param data Pointer to user's data struct where new data will be
   * written
   *
   * @return 0 on success or -1 on failure.
   */
  std::optional<std::array<int16_t, 3>> ReadGyroRaw();
  std::array<double, 3> ReadGyroCalibrated(const std::array<int16_t, 3>& raw);

  /**
   * @brief Reads thermometer data from the MPU
   *
   * Note this is the internal termperature of the chip, not abient temperature.
   *
   * @param data Pointer to user's data struct where new data will be
   * written
   *
   * @return 0 on success or -1 on failure.
   */
  std::optional<double> ReadTemp();

  MpuData ReadData();

  /**
   * @brief Set sampling rate. Sampling rate must be between 4Hz and 1kHz.
   *
   * @param[in] rate Desired sampling rate (Hz)
   * @return true if successful
   * @return false otherwise
   */
  bool SetSampleRate(const uint16_t rate);

  bool SetAccelFSR(const MpuAccelFSR fsr);
  bool SetGyroFSR(const MpuGyroFSR fsr);
  bool SetAccelDLPF(const MpuAccelDLPF dlpf);
  bool SetGyroDLPF(const MpuGyroDLPF dlpf);

 private:
  MpuConfig config_;
  MpuData data_;

  std::shared_ptr<I2C> i2c_;
  std::shared_ptr<Mag> mag_;

  /// @brief which bus to use, default 2 on BeagleBone Blue
  int i2c_bus_{0};
  double accel_lengths_[3]{1., 1., 1.};
  ///< conversion rate from raw accelerometer to m/s^2
  double accel_to_ms2_{1.};
  ///< conversion rate from raw gyroscope to degrees/s
  double gyro_to_degs_{1.};
};

#endif  // RC_MPU_H
