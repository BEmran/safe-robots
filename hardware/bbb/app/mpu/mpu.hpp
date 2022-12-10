#ifndef RC_MPU_H
#define RC_MPU_H

#include <pthread.h>

#include <array>
#include <cstdint>

#include "i2c.hpp"

/*
// defines for index location within TaitBryan and quaternion vectors
/// @brief Index of the dmp_TaitBryan[] array corresponding to the Pitch (X)
/// axis.
constexpr uint8_t TB_PITCH_X = 0;
/// @brief Index of the dmp_TaitBryan[] array corresponding to the Roll (Y)
/// axis.
constexpr uint8_t TB_ROLL_Y = 1;
/// @brief Index of the dmp_TaitBryan[] array corresponding to the Yaw (Z) axis.
constexpr uint8_t TB_YAW_Z = 2;

/// @brief First index of the dmp_quat[] quaternion vector
constexpr uint8_t QUAT_W = 0;
/// @brief Second index of the dmp_quat[] quaternion vector
constexpr uint8_t QUAT_X = 1;
/// @brief Third index of the dmp_quat[] quaternion vector
constexpr uint8_t QUAT_Y = 2;
/// @brief Fourth index of the dmp_quat[] quaternion vector
constexpr uint8_t QUAT_Z = 3;
*/

/// @brief multiply to convert degrees to radians
constexpr double DEG_TO_RAD = 0.0174532925199;
/// @brief multiply to convert radians to degrees
constexpr double RAD_TO_DEG = 57.295779513;
/// @brief multiply to convert radians to degrees
constexpr double MS2_TO_G = 0.10197162129;
/// @brief multiply to convert G to m/s^2, standard gravity definition
constexpr double G_TO_MS2 = 9.80665;

/**
 * @brief sleep the process for passed microseconds
 *
 * @param micro microseconds to sleep for
 */
void MicroSleep(const size_t micro);

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
 * @brief Orientation of the sensor.
 *
 * @details This is only applicable when operating in DMP mode. This is the
 * orientation that the DMP considers neutral, aka where roll/pitch/yaw are
 * zero.
 */
enum class MpuOrientation {
  ORIENTATION_Z_UP = 136,
  ORIENTATION_Z_DOWN = 396,
  ORIENTATION_X_UP = 14,
  ORIENTATION_X_DOWN = 266,
  ORIENTATION_Y_UP = 112,
  ORIENTATION_Y_DOWN = 336,
  ORIENTATION_X_FORWARD = 133,
  ORIENTATION_X_BACK = 161
};

/**
 * @brief configuration of the mpu sensor
 *
 * @details Configuration struct passed to MPU::Initialize() and
 * MPU::InitializeDmp().
 */
struct MpuConfig {
  /// @brief which bus to use, default 2 on BeagleBone Blue
  int i2c_bus{0};
  /// @brief which ship address to use default is 0x68 or 0x69
  uint8_t i2c_addr{0};
  /// @brief set to 1 to print i2c_bus warnings for debug
  int show_warnings{false};
  /// @brief sampling rate (Hz). Sampling rate must be between 4Hz and 1kHz.
  uint16_t sample_rate{1000};
  /// @brief compass sampling rate (Hz). Sampling rate must be between 1Hz and
  /// 100Hz.
  uint16_t compass_sample_rate{100};
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
  int enable_magnetometer{true};

  /** @name DMP settings, only used with DMP mode */
  /*
  /// @brief gpio pin, default 3 on Robotics Cape and BB Blue
  int gpio_interrupt_pin_chip{3};
  /// @brief gpio pin, default 21 on Robotics Cape and BB Blue
  int gpio_interrupt_pin{21};
  /// @brief sample rate in hertz, 200, 100, 50, 40, 25, 20, 10, 8, 5, 4
  int dmp_sample_rate;
  /// @brief set to 1 to optionally raw accel / gyro when reading DMP
  /// quaternion, default: 0 (off)
  int dmp_fetch_accel_gyro;
  /// @brief set to 1 to let DMP auto calibrate the gyro while in use, default:
  /// 0 (off)
  int dmp_auto_calibrate_gyro;
  /// @brief DMP orientation matrix, see MpuOrientation
  MpuOrientation orient;
  /// @brief time constant (seconds) for filtering compass with gyroscope yaw
  /// value, default 25
  double compass_time_constant;
  /// @brief Scheduler policy for DMP interrupt handler and user callback,
  /// default SCHED_OTHER
  int dmp_interrupt_sched_policy;
  /// @brief scheduler priority for DMP interrupt handler and user callback,
  /// default 0
  int dmp_interrupt_priority;
  /// @brief reads magnetometer after DMP callback function to improve latency,
  /// default 1 (true)
  int read_mag_after_callback;
  /// @brief magnetometer_sample_rate = dmp_sample_rate/mag_sample_rate_div,
  /// default: 4
  int mag_sample_rate_div;
  /// @brief threshold impulse for triggering a tap in units of mg/ms
  int tap_threshold;
  */
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

  // /// @name DMP data
  // /// @brief normalized quaternion from DMP based on ONLY Accel/Gyro
  // std::array<double, 4> dmp_quat{1., 0., 0., 0.};
  // /// @brief Tait-Bryan angles (roll pitch yaw) in radians from DMP based on
  // /// ONLY Accel/Gyro
  // std::array<double, 3> dmp_TaitBryan{0., 0., 0.};
  // /// @brief set to 1 if there was a tap detect on the last dmp sample, reset
  // to
  // /// 0 on next sample
  // bool tap_detected{false};
  // /// @brief direction of last tap, 1-6 corresponding to X+ X- Y+ Y- Z+ Z-
  // int last_tap_direction;
  // /// @brief current counter of rapid consecutive taps
  // int last_tap_count;

  // /// @name fused DMP data filtered with magnetometer
  // /// @brief fused and normalized quaternion
  // std::array<double, 4> fused_quat{1., 0., 0., 0.};
  // /// @brief fused Tait-Bryan angles (roll pitch yaw) in radians
  // std::array<double, 3> fused_TaitBryan{0., 0., 0.};
  // /// @brief fused heading filtered with gyro and accel data, same as
  // Tait-Bryan
  // /// yaw
  // double compass_heading{0};
  // /// @brief unfiltered heading from magnetometer
  // double compass_heading_raw{0};
};

/**
 * @brief Returns an MpuConfig struct with default settings.
 *
 * @return MpuConfig with default values.
 */
MpuConfig MpuDefaultConfig();

class MPU {
 public:
  MPU() {
  }

  ~MPU() {
    i2c.Close();
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

  bool CheckWhoAmI(const uint8_t reg, const uint8_t expected_result);
  bool CheckWhoAmIMPU();
  bool CheckWhoAmIMagnetometer();
  bool InitMagnetometer(const int cal_mode);
  bool SetBypass(const bool bypass_on);

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

  bool PowerOffMagnetometer();
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
  std::optional<std::array<int16_t, 3>> ReadMagRaw();
  std::array<double, 3> ReadMagCalibrated(const std::array<int16_t, 3>& raw);

  MpuData ReadData();

  /**
   * @brief Set sampling rate. Sampling rate must be between 4Hz and 1kHz.
   *
   * @param[in] rate Desired sampling rate (Hz)
   * @return true if successful
   * @return false otherwise
   */
  bool SetSampleRate(const uint16_t rate);

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
  bool SetCompassSampleRate(const uint16_t rate);

  bool SetAccelFSR(const MpuAccelFSR fsr);
  bool SetGyroFSR(const MpuGyroFSR fsr);
  bool SetAccelDLPF(const MpuAccelDLPF dlpf);
  bool SetGyroDLPF(const MpuGyroDLPF dlpf);

 private:
  MpuConfig config_;
  MpuData data_;

  I2C i2c;
  bool bypass_enabled_{false};
  bool dmp_en_{false};
  double mag_factory_adjust[3]{0., 0., 0.};
  double mag_offsets[3]{0., 0., 0.};
  double mag_scales[3]{1., 1., 1.};
  double accel_lengths[3]{1., 1., 1.};
  ///< conversion rate from raw accelerometer to m/s^2
  double accel_to_ms2{1.};
  ///< conversion rate from raw gyroscope to degrees/s
  double gyro_to_degs{1.};
};

/** @name common functions */
///@{

///@} end common functions

// /** @name interrupt-driven DMP mode functions */
// ///@{

// /**
// * @brief Initializes the MPU in DMP mode, see rc_test_dmp example
// *
// * After calling this the user does not need to call the normal read
// functions
// * rc_mpu_read_accel(), rc_mpu_read_gyro(), or rc_mpu_read mag(). Instead the
// * data will automatically be read into the user's data struct at the
// * dmp_sample_rate set in the config struct.
// *
// * @param data Pointer to user's data struct where new data will be
// * written
// * @param[in] conf User's configuration struct
// *
// * @return 0 on success or -1 on failure.
// */
// int rc_mpu_initialize_dmp(MpuData* data, MpuConfig conf);

// /**
// * @brief Sets the callback function that will be triggered when new DMP
// * data is ready.
// *
// * @param[in] func user's callback function
// *
// * @return 0 on success or -1 on failure.
// */
// int rc_mpu_set_dmp_callback(void (*func)(void));

// /**
// * @brief blocking function that returns once new DMP data is available
// *
// * @return Returns 0 once new data is available, 1 if the MPU is shutting
// * down due to rc_mpu_power_off, or -1 on error.
// */
// int rc_mpu_block_until_dmp_data(void);

// /**
// * @brief calculates number of nanoseconds since the last DMP interrupt
// *
// * @return nanoseconds since last interrupt, or -1 if no interrupt
// received
// * yet.
// */
// int64_t rc_mpu_nanos_since_last_dmp_interrupt(void);

// /**
// * @brief sets the callback function triggered when a tap is detected
// *
// * @param[in] func user's callback function
// *
// * @return 0 on success or -1 on failure.
// */
// int rc_mpu_set_tap_callback(void (*func)(int direction, int counter));

// /**
// * @brief blocking function that returns when a tap is detected
// *
// * @return Returns 0 once a tap is detected, 1 if the MPU is shutting
// down
// * due to rc_mpu_power_off(), or -1 on error.
// */
// int rc_mpu_block_until_tap(void);

// /**
// * @brief calculates nanoseconds since last tap was detected
// *
// * @return nanoseconds since last tap, or -1 if no tap has been detected
// * yet.
// */
// int64_t rc_mpu_nanos_since_last_tap(void);
// ///@} end interrupt-driven DMP mode functions

// /** @name calibration functions */
// ///@{

// /**
// * @brief Runs gyroscope calibration routine
// *
// * This should generally not be used by the user unless they absolutely want
// to
// * calibrate the gyroscope inside their own program. Instead call the
// * rc_calibrate_gyro example program.
// *
// * @param[in] conf Config struct, only used to configure i2c bus and
// address.
// *
// * @return 0 on success, -1 on failure
// */
// int rc_mpu_calibrate_gyro_routine(MpuConfig conf);

// /**
// * @brief Runs magnetometer calibration routine
// *
// * This should generally not be used by the user unless they absolutely want
// to
// * calibrate the magnetometer inside their own program. Instead call the
// * rc_calibrate_mag example program.
// *
// * @param[in] conf Config struct, only used to configure i2c bus and
// address.
// *
// * @return 0 on success, -1 on failure
// */
// int rc_mpu_calibrate_mag_routine(MpuConfig conf);

// /**
// * @brief Runs accelerometer calibration routine
// *
// * This should generally not be used by the user unless they absolutely want
// to
// * calibrate the accelerometer inside their own program. Instead call the
// * rc_calibrate_accel example program.
// *
// * @param[in] conf Config struct, only used to configure i2c bus and
// address.
// *
// * @return 0 on success, -1 on failure
// */
// int rc_mpu_calibrate_accel_routine(MpuConfig conf);

// /**
// * @brief Checks if a gyro calibration file is saved to disk
// *
// * generally used to warn the user that they are running a program without
// * calibration. Can also be used to decide if calibration should be done at
// the
// * beginning of user's program.
// *
// * @return 1 if calibrated, 0 if not
// */
// int rc_mpu_is_gyro_calibrated(void);

// /**
// * @brief Checks if a magnetometer calibration file is saved to disk
// *
// * generally used to warn the user that they are running a program without
// * calibration. Can also be used to decide if calibration should be done at
// the
// * beginning of user's program.
// *
// * @return 1 if calibrated, 0 if not
// */
// int rc_mpu_is_mag_calibrated(void);

// /**
// * @brief Checks if an accelerometer calibration file is saved to disk
// *
// * generally used to warn the user that they are running a program without
// * calibration. Can also be used to decide if calibration should be done at
// the
// * beginning of user's program.
// *
// * @return 1 if calibrated, 0 if not
// */
// int rc_mpu_is_accel_calibrated(void);

///@} end calibration functions

#endif  // RC_MPU_H

/** @} end group IMU_MPU*/
