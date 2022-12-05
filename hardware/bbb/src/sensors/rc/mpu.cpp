#include "sensors/rc/mpu.hpp"

// #include <poll.h>
// #include <rc/gpio.h>
// #include <rc/math/algebra.h>
// #include <rc/math/filter.h>
// #include <rc/math/matrix.h>
// #include <rc/math/quaternion.h>
// #include <rc/math/vector.h>
// #include <rc/pthread.h>
// #include <rc/time.h>
#include <sys/stat.h>
#include <sys/stat.h>   // for mkdir and chmod
#include <sys/types.h>  // for mkdir and chmod
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <core/utils/logger_macros.hpp>
#include <core/utils/system.hpp>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>

#include "common.hpp"
// #include "dmpKey.h"
// #include "dmp_firmware.h"
// #include "dmpmap.h"

#include "mpu_defs.hpp"

// Calibration File Locations
#define ACCEL_CAL_FILE "accel.cal"
#define GYRO_CAL_FILE "gyro.cal"
#define MAG_CAL_FILE "mag.cal"

/// @brief I2C bus and address definitions for Robotics Cape & bealgebone blue
constexpr uint8_t RC_IMU_BUS = 2;

// #define RC_IMU_INTERRUPT_PIN_CHIP 3
// #define RC_IMU_INTERRUPT_PIN_PIN 21  // gpio3.21 P9.25

#define PI M_PI
#define TWO_PI (2.0 * M_PI)

// there should be 28 or 35 bytes in the FIFO if the magnetometer is disabled
// or enabled.
// #define FIFO_LEN_QUAT_TAP 20             // 16 for quat, 4 for tap
// #define FIFO_LEN_QUAT_ACCEL_GYRO_TAP 32  // 16 quat, 6 accel, 6 gyro, 4 tap
// #define MAX_FIFO_BUFFER (FIFO_LEN_QUAT_ACCEL_GYRO_TAP * 5)

// // error threshold checks
// #define QUAT_ERROR_THRESH (1L << 16)  // very precise threshold
// #define QUAT_MAG_SQ_NORMALIZED (1L << 28)
// #define QUAT_MAG_SQ_MIN (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
// #define QUAT_MAG_SQ_MAX (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
// #define GYRO_CAL_THRESH 50    // std dev below which to consider still
// #define ACCEL_CAL_THRESH 100  // std dev below which to consider still
// #define GYRO_OFFSET_THRESH 500

// Thread control
// static pthread_mutex_t read_mutex = PTHREAD_MUTEX_INITIALIZER;
// static pthread_cond_t read_condition = PTHREAD_COND_INITIALIZER;
// static pthread_mutex_t tap_mutex = PTHREAD_MUTEX_INITIALIZER;
// static pthread_cond_t tap_condition = PTHREAD_COND_INITIALIZER;

/**
 *	Local variables
 **/
// static int dmp_en = 0;
// static int packet_len;
// static pthread_t imu_interrupt_thread;
// static int thread_running_flag;
// static void (*dmp_callback_func)() = NULL;
// static void (*tap_callback_func)(int dir, int cnt) = NULL;
// static int last_read_successful;
// static uint64_t last_interrupt_timestamp_nanos;
// static uint64_t last_tap_timestamp_nanos;
// static MpuData data;
// static int imu_shutdown_flag = 0;
// static rc_filter_t low_pass, high_pass;  // for magnetometer Yaw filtering
// static int was_last_steady = 0;
// static double startMagYaw = 0.0;

/**
 * functions for internal use only
 **/
// static int __mpu_set_bypass(unsigned char bypass_on);
// static int __mpu_write_mem(unsigned short mem_addr, unsigned short length,
//                            unsigned char* data);
// static int __mpu_read_mem(unsigned short mem_addr, unsigned short length,
//                           unsigned char* data);
// static int __dmp_load_motion_driver_firmware(void);
// static int __dmp_set_orientation(unsigned short orient);
// static int __dmp_enable_gyro_cal(unsigned char enable);
// static int __dmp_enable_lp_quat(unsigned char enable);
// static int __dmp_enable_6x_lp_quat(unsigned char enable);
// static int __dmp_set_tap_thresh(unsigned char axis, unsigned short thresh);
// static int __dmp_set_tap_axes(unsigned char axis);
// static int __dmp_set_tap_count(unsigned char min_taps);
// static int __dmp_set_tap_time(unsigned short time);
// static int __dmp_set_tap_time_multi(unsigned short time);
// static int __dmp_set_shake_reject_thresh(long sf, unsigned short thresh);
// static int __dmp_set_shake_reject_time(unsigned short time);
// static int __dmp_set_shake_reject_timeout(unsigned short time);
// static int __collect_accel_samples(int* avg_raw);
// static int __mpu_reset_fifo(void);
// static int __mpu_set_sample_rate(int rate);
// static int __dmp_set_fifo_rate(unsigned short rate);
// static int __dmp_enable_feature(unsigned short mask);
// static int __mpu_set_dmp_state(unsigned char enable);
// static int __set_int_enable(unsigned char enable);
// static int __dmp_set_interrupt_mode(unsigned char mode);
// static int __load_gyro_calibration(void);
// static int __load_mag_calibration(void);
// static int __load_accel_calibration(void);
// static int __write_gyro_cal_to_disk(std::array<int16_t, 3> offsets);
// static int __write_mag_cal_to_disk(std::array<double, 3> offsets,
//                                    std::array<double, 3> scale);
// static int __write_accel_cal_to_disk(double* center, double* lengths);
// static void* __dmp_interrupt_handler(void* ptr);
// static int __read_dmp_fifo(rc_mpu_data_t* data);
// static int __data_fusion(rc_mpu_data_t* data);
// static int __mag_correct_orientation(std::array<double, 3> mag_vec);

namespace {
void MicroSleep(uint32_t micro) {
  usleep(micro);
}

template <typename T, size_t SIZE>
std::optional<std::array<T, SIZE>> ReadNumbers(std::ifstream& file) {
  if (not file.is_open()) {
    SYS_LOG_WARN("ReadNumbers: file is not open");
    return {};
  }
  std::vector<T> numbers;
  while (not file.eof()) {
    T x;
    file >> x;
    numbers.push_back(x);
  }

  if (numbers.size() == SIZE) {
    SYS_LOG_WARN("ReadNumbers: file doesn't have the correct numbers to read");
    return {};
  }
  std::array<T, SIZE> array;
  std::copy(numbers.begin(), numbers.end(), array.begin());
  return array;
}

template <typename T, size_t SIZE>
std::string ArrayToString(std::string_view header,
                          const std::array<T, SIZE>& arr) {
  std::string msg = header.data();
  for (size_t i = 0; i < arr.size(); i++) {
    msg += std::to_string(arr[i]) + " ";
  }
  return msg;
}

/**
 * @brief Writes an array values to disk for a specific directory and filename
 *
 * @tparam T array type
 * @tparam SIZE size of array
 * @param directory directory path
 * @param filename file name to write at
 * @param array values to write
 * @return true if success
 * @return false otherwise
 */
template <typename T, size_t SIZE>
bool WriteCalToDisk(std::string_view directory, std::string_view filename,
                    std::array<T, SIZE> array) {
  try {
    core::utils::CreateDirectories(directory);
  } catch (...) {
    SYS_LOG_WARN("ERROR in WriteCalToDisk making calibration file directory");
    return false;
  }

  // remove old file
  std::string full_filename{directory};
  full_filename += filename.data();
  remove(full_filename.c_str());
  std::ofstream file;
  file.open(full_filename, std::ios_base::out);
  if (not file.is_open()) {
    SYS_LOG_WARN("ERROR in WriteCalToDisk cannot open file" +
                 full_filename.c_str());
    return false;
  }

  // write to the file, close, and exit
  for (size_t i = 0; i < array.size(); i++) {
    file << array[i] << " ";
  }
  file.close();

  // now give proper permissions
  if (chmod(full_filename.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1) {
    SYS_LOG_WARN(
      "ERROR in WriteCalToDisk opening calibration file for writing");
    SYS_LOG_WARN("most likely you ran this as root in the past and are now");
    SYS_LOG_WARN("running it as a normal user. try deleting the file");
    SYS_LOG_WARN("sudo rm /var/lib/robotcontrol/*.cal");
    return false;
  }

  return true;
}
}  // namespace

MpuConfig DefaultConfig() {
  MpuConfig conf;
  // connectivity
  // conf.gpio_interrupt_pin_chip = RC_IMU_INTERRUPT_PIN_CHIP;
  // conf.gpio_interrupt_pin = RC_IMU_INTERRUPT_PIN_PIN;
  conf.i2c_bus = RC_IMU_BUS;
  conf.i2c_addr = RC_MPU_DEFAULT_I2C_ADDR;
  conf.show_warnings = true;

  // general stuff
  conf.accel_fsr = AccelFSR::ACCEL_FSR_8G;
  conf.gyro_fsr = GyroFSR::GYRO_FSR_2000DPS;
  conf.accel_dlpf = AccelDLPF::ACCEL_DLPF_184;
  conf.gyro_dlpf = GyroDLPF::GYRO_DLPF_184;
  conf.enable_magnetometer = true;

  // DMP stuff
  // conf.dmp_sample_rate = 100;
  // conf.dmp_fetch_accel_gyro = 0;
  // conf.dmp_auto_calibrate_gyro = 0;
  // conf.orient = ORIENTATION_Z_UP;
  // conf.compass_time_constant = 20.0;
  // conf.dmp_interrupt_sched_policy = SCHED_OTHER;
  // conf.dmp_interrupt_priority = 0;
  // conf.read_mag_after_callback = 1;
  // conf.mag_sample_rate_div = 4;
  // conf.tap_threshold = 210;

  return conf;
}

bool MPU::Initialize(const MpuConfig conf) {
  config_ = conf;

  // // make sure the bus is not currently in use by another thread
  // // do not proceed to prevent interfering with that process
  // if (rc_i2c_get_lock(config_.i2c_bus)) {
  //   printf("i2c bus claimed by another process");
  //   printf("Continuing with rc_mpu_initialize() anyway.");
  // }

  // start the i2c bus
  if (not i2c_.Initialize(config_.i2c_addr)) {
    SYS_LOG_WARN("failed to initialize i2c bus");
    return false;
  }

  // restart the device so we start with clean registers
  if (not ResetMpu()) {
    SYS_LOG_WARN("failed to reset_mpu9250");
    return false;
  }

  if (not CheckWhoAmI()) {
    SYS_LOG_WARN("failed to check mpu register");
    return false;
  }

  // // load in accel calibration offsets from disk
  // if (not LoadAcellCalibration()) {
  //   SYS_LOG_WARN("failed to load accel calibration offsets");
  //   return false;
  // }

  // // load in gyro calibration offsets from disk
  // if (not LoadGyroCalibration()) {
  //   SYS_LOG_WARN("failed to load gyro calibration offsets");
  //   return false;
  // }

  // Set sample rate = 1000/(1 + SMPLRT_DIV)
  // here we use a divider of 0 for 1khz sample
  if (not i2c_.WriteByte(SMPLRT_DIV, 0x00)) {
    SYS_LOG_WARN("I2C bus write error");
    return false;
  }

  // set full scale ranges and filter constants
  if (not SetGyroFSR(config_.gyro_fsr)) {
    SYS_LOG_WARN("failed to set gyro fsr");
    return false;
  }

  if (not SetAccelFSR(config_.accel_fsr)) {
    SYS_LOG_WARN("failed to set accel fsr");
    return false;
  }

  if (not SetGyroDLPF(config_.gyro_dlpf)) {
    SYS_LOG_WARN("failed to set gyro dlpf");
    return false;
  }

  if (not SetAccelDLPF(config_.accel_dlpf)) {
    SYS_LOG_WARN("failed to set accel_dlpf");
    return false;
  }

  // initialize the magnetometer too if requested in config
  if (config_.enable_magnetometer) {
    // start magnetometer NOT in cal mode (0)
    if (InitMagnetometer(0)) {
      SYS_LOG_WARN("failed to initialize magnetometer");
      return false;
    }
  } else {
    PowerOffMagnetometer();
  }

  return true;
}

std::vector<double> MPU::ReadAccel() {
  // set the device address
  if (not i2c_.SetDeviceAddress(config_.i2c_addr)) {
    SYS_LOG_WARN("Resetting MPU, failed to set i2c device address");
    return {};
  }

  // Read the six raw data registers into data array
  const auto raw = i2c_.ReadWords(ACCEL_XOUT_H, 3, ByteOrder::BIG_ENDIAN_ORDER);
  if (raw.empty()) {
    SYS_LOG_WARN("failed to read IMU accelerometer registers");
    return {};
  }

  // Fill in real unit values
  std::vector<double> accel(3);
  for (size_t i = 0; i < raw.size(); i++) {
    accel[i] = raw[i] * accel_to_ms2_ / accel_lengths_[i];
  }
  return accel;
}

std::vector<double> MPU::ReadGyro() {
  // set the device address
  if (not i2c_.SetDeviceAddress(config_.i2c_addr)) {
    SYS_LOG_WARN("Resetting MPU, failed to set i2c device address");
    return {};
  }

  // Read the six raw data registers into data array
  const auto raw = i2c_.ReadWords(GYRO_XOUT_H, 3, ByteOrder::BIG_ENDIAN_ORDER);
  if (raw.empty()) {
    SYS_LOG_WARN("failed to read IMU gyroscope registers");
    return {};
  }

  // Fill in real unit values
  std::vector<double> gyro(3);
  for (size_t i = 0; i < raw.size(); i++) {
    gyro[i] = raw[i] * gyro_to_degs_;
  }
  return gyro;
}

std::vector<double> CorrectMagRotation(std::vector<double> in) {
  if (in.size() != 3) {
    SYS_LOG_WARN("CorrectMagRotation: expecte vector size of 3");
    return {0., 0., 0.};
  }
  return {in[1], in[0], -in[2]};
}

std::vector<double> MPU::ReadMag() {
  // uint8_t raw[7];
  // int16_t adc[3];
  // double factory_cal_data[3];
  if (!config_.enable_magnetometer) {
    SYS_LOG_WARN(
      "Can't read magnetometer unless it is enabled in "
      "MPUConfig struct and initialized");
    return {};
  }
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  // MPU9250 was put into passthrough mode
  if (not i2c_.SetDeviceAddress(AK8963_ADDR)) {
    SYS_LOG_WARN("In rc_mpu_read_mag, failed to set i2c address");
    return {};
  }
  // don't worry about checking data ready bit, not worth the time
  // read the data ready bit to see if there is new data
  const auto st1 = i2c_.ReadByte(AK8963_ST1);
  if (not st1.has_value()) {
    SYS_LOG_WARN("ERROR reading Magnetometer, i2c_bypass is probably not set");
    return {};
  }

  SYS_LOG_DEBUG("st1: " + std::to_string(st1.value()));

  const bool is_data_ready = st1.value() & MAG_DATA_READY;
  if (not is_data_ready) {
    SYS_LOG_WARN("no new magnetometer data ready, skipping read");
    return {};
  }

  // Read the seven raw data registers into data array
  const auto raw = i2c_.ReadBytes(AK8963_XOUT_L, 7);
  if (raw.empty()) {
    SYS_LOG_WARN("rc_mpu_read_mag failed to read data register");
    return {};
  }

  // check if the readings saturated such as because
  // of a local field source, discard data if so
  const bool is_saturated = raw[6] & MAGNETOMETER_SATURATION;
  if (is_saturated) {
    SYS_LOG_WARN("WARNING: magnetometer saturated, discarding data");
    return {};
  }

  std::vector<double> mag(3);
  for (size_t i = 0; i < raw.size() - 1; i++) {
    const size_t idx = i * 2;
    // Turn the MSB and LSB into a signed 16-bit, register stored as little
    // Endian
    const int16_t raw_word = RegisterBytesToWord(
      raw[idx], raw[idx + 1], ByteOrder::LITTLE_ENDIAN_ORDER);
    // multiply by the sensitivity adjustment and convert to units of uT micro
    // Teslas.
    const double factory_cal_data =
      raw_word * mag_factory_adjust_[i] * MAG_RAW_TO_uT;
    // now apply out own calibration,
    mag[i] = (factory_cal_data - mag_offsets_[i]) * mag_scales_[i];
  }
  // Also correct the coordinate system as someone in invensense
  // thought it would be bright idea to have the magnetometer coordinate
  // system aligned differently than the accelerometer and gyro.... -__-

  return CorrectMagRotation(mag);
}

std::optional<double> MPU::ReadTemp() {
  // set the device address
  if (not i2c_.SetDeviceAddress(config_.i2c_addr)) {
    SYS_LOG_WARN("Resetting MPU, failed to set i2c device address");
    return {};
  }
  // Read the two raw data registers
  const auto adc = i2c_.ReadWord(TEMP_OUT_H, ByteOrder::BIG_ENDIAN_ORDER);
  if (not adc.has_value()) {
    SYS_LOG_WARN("failed to read IMU temperature registers");
    return {};
  }
  // convert to real units
  return 21.0 + adc.value() / TEMP_SENSITIVITY;
}

bool MPU::ResetMpu() {
  // // disable the interrupt to prevent it from doing things while we reset
  // imu_shutdown_flag = 1;
  // set the device address
  if (not i2c_.SetDeviceAddress(config_.i2c_addr)) {
    SYS_LOG_WARN("Resetting MPU, failed to set i2c device address");
    return false;
  }

  // write the reset bit
  if (not i2c_.WriteByte(PWR_MGMT_1, H_RESET)) {
    // wait and try again
    MicroSleep(10000);
    if (not i2c_.WriteByte(PWR_MGMT_1, H_RESET)) {
      SYS_LOG_WARN("Resetting MPU, I2C write to reset bit failed");
      return false;
    }
  }
  MicroSleep(10000);
  return true;
}

bool MPU::CheckWhoAmI() {
  // check the who am i register to make sure the chip is alive
  const auto chip = i2c_.ReadByte(WHO_AM_I_MPU9250);
  if (chip.has_value()) {
    SYS_LOG_WARN("failed reading who_am_i register");
    return false;
  }
  // check which chip we are looking at
  // 0x71 for mpu9250, ox73 or 0x75 for mpu9255, or 0x68 for mpu9150
  // 0x70 for mpu6500,  0x68 or 0x69 for mpu6050
  if (chip.value() != 0x71) {
    SYS_LOG_WARN("invalid who_am_i register: 0x" +
                 std::to_string(chip.value()));
    SYS_LOG_WARN("expected 0x71 for mpu9250");
    return false;
  }
  return true;
}

bool MPU::SetAccelFSR(const AccelFSR fsr) {
  uint8_t conf{ACCEL_FSR_CFG_2G};
  switch (fsr) {
    case AccelFSR::ACCEL_FSR_2G:
      conf = ACCEL_FSR_CFG_2G;
      accel_to_ms2_ = 9.80665 * 2.0 / 32768.0;
      break;
    case AccelFSR::ACCEL_FSR_4G:
      conf = ACCEL_FSR_CFG_4G;
      accel_to_ms2_ = 9.80665 * 4.0 / 32768.0;
      break;
    case AccelFSR::ACCEL_FSR_8G:
      conf = ACCEL_FSR_CFG_8G;
      accel_to_ms2_ = 9.80665 * 8.0 / 32768.0;
      break;
    case AccelFSR::ACCEL_FSR_16G:
      conf = ACCEL_FSR_CFG_16G;
      accel_to_ms2_ = 9.80665 * 16.0 / 32768.0;
      break;
    default:
      SYS_LOG_WARN("invalid accel fsr");
      return false;
  }
  return i2c_.WriteByte(ACCEL_CONFIG, conf);
}

bool MPU::SetGyroFSR(const GyroFSR fsr) {
  uint8_t conf{GYRO_FSR_CFG_250};
  switch (fsr) {
    case GyroFSR::GYRO_FSR_250DPS:
      conf = GYRO_FSR_CFG_250 | FCHOICE_B_DLPF_EN;
      gyro_to_degs_ = 250.0 / 32768.0;
      break;
    case GyroFSR::GYRO_FSR_500DPS:
      conf = GYRO_FSR_CFG_500 | FCHOICE_B_DLPF_EN;
      gyro_to_degs_ = 500.0 / 32768.0;
      break;
    case GyroFSR::GYRO_FSR_1000DPS:
      conf = GYRO_FSR_CFG_1000 | FCHOICE_B_DLPF_EN;
      gyro_to_degs_ = 1000.0 / 32768.0;
      break;
    case GyroFSR::GYRO_FSR_2000DPS:
      conf = GYRO_FSR_CFG_2000 | FCHOICE_B_DLPF_EN;
      gyro_to_degs_ = 2000.0 / 32768.0;
      break;
    default:
      SYS_LOG_WARN("invalid gyro fsr");
      return false;
  }
  return i2c_.WriteByte(GYRO_CONFIG, conf);
}

bool MPU::SetAccelDLPF(const AccelDLPF dlpf) {
  uint8_t conf{ACCEL_FCHOICE_1KHZ | BIT_FIFO_SIZE_1024};
  switch (dlpf) {
    case AccelDLPF::ACCEL_DLPF_OFF:
      conf = ACCEL_FCHOICE_4KHZ | BIT_FIFO_SIZE_1024;
      break;
    case AccelDLPF::ACCEL_DLPF_460:
      conf |= 0;
      break;
    case AccelDLPF::ACCEL_DLPF_184:
      conf |= 1;
      break;
    case AccelDLPF::ACCEL_DLPF_92:
      conf |= 2;
      break;
    case AccelDLPF::ACCEL_DLPF_41:
      conf |= 3;
      break;
    case AccelDLPF::ACCEL_DLPF_20:
      conf |= 4;
      break;
    case AccelDLPF::ACCEL_DLPF_10:
      conf |= 5;
      break;
    case AccelDLPF::ACCEL_DLPF_5:
      conf |= 6;
      break;
    default:
      SYS_LOG_WARN("invalid config.accel_dlpf");
      return false;
  }
  return i2c_.WriteByte(ACCEL_CONFIG_2, conf);
}

bool MPU::SetGyroDLPF(const GyroDLPF dlpf) {
  uint8_t conf{FIFO_MODE_REPLACE_OLD};
  switch (dlpf) {
    case GyroDLPF::GYRO_DLPF_OFF:
      conf |= 7;  // not really off, but 3600Hz bandwidth
      break;
    case GyroDLPF::GYRO_DLPF_250:
      conf |= 0;
      break;
    case GyroDLPF::GYRO_DLPF_184:
      conf |= 1;
      break;
    case GyroDLPF::GYRO_DLPF_92:
      conf |= 2;
      break;
    case GyroDLPF::GYRO_DLPF_41:
      conf |= 3;
      break;
    case GyroDLPF::GYRO_DLPF_20:
      conf |= 4;
      break;
    case GyroDLPF::GYRO_DLPF_10:
      conf |= 5;
      break;
    case GyroDLPF::GYRO_DLPF_5:
      conf |= 6;
      break;
    default:
      SYS_LOG_WARN("invalid gyro_dlpf");
      return false;
  }
  return i2c_.WriteByte(CONFIG, conf);
}

bool MPU::InitMagnetometer(const int cal_mode) {
  // Enable i2c bypass to allow talking to magnetometer
  if (SetBypass(true)) {
    SYS_LOG_WARN("failed to set mpu9250 into bypass i2c mode");
    return false;
  }
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  if (not i2c_.SetDeviceAddress(AK8963_ADDR)) {
    SYS_LOG_WARN("InitMagnetometer, failed to set i2c device address");
    return false;
  }

  // Power down magnetometer
  if (not i2c_.WriteByte(AK8963_CNTL, MAG_POWER_DN)) {
    SYS_LOG_WARN(
      "InitMagnetometer, failed to write to AK8963_CNTL register to power "
      "down");
    return false;
  }
  MicroSleep(1000);

  // Enter Fuse ROM access mode
  if (not i2c_.WriteByte(AK8963_CNTL, MAG_FUSE_ROM)) {
    SYS_LOG_WARN("InitMagnetometer, failed to write to AK8963_CNTL register");
    return false;
  }
  MicroSleep(1000);

  // Read the xyz sensitivity adjustment values
  std::vector<uint8_t> raw_sensitivity = i2c_.ReadBytes(AK8963_ASAX, 3);
  if (raw_sensitivity.empty()) {
    SYS_LOG_WARN(
      "InitMagnetometer, failed to read magnetometer adjustment register");
    i2c_.SetDeviceAddress(config_.i2c_addr);
    //__mpu_set_bypass(0);
    return false;
  }

  // Return sensitivity adjustment values
  std::transform(raw_sensitivity.begin(), raw_sensitivity.end(),
                 mag_factory_adjust_.begin(),
                 [](const uint8_t raw) { return (raw - 128) / 256.0 + 1.0; });

  // Power down magnetometer again
  if (not i2c_.WriteByte(AK8963_CNTL, MAG_POWER_DN)) {
    SYS_LOG_WARN(
      "InitMagnetometer, failed to write to AK8963_CNTL register to power "
      "on");
    return false;
  }
  MicroSleep(100);

  // Configure the magnetometer for 16 bit resolution
  // and continuous sampling mode 2 (100hz)
  const uint8_t c = MSCALE_16 | MAG_CONT_MES_2;
  if (not i2c_.WriteByte(AK8963_CNTL, c)) {
    SYS_LOG_WARN(
      "InitMagnetometer, failed to write to AK8963_CNTL register to set "
      "sampling mode");
    return false;
  }
  MicroSleep(100);

  // go back to configuring the IMU, leave bypass on
  i2c_.SetDeviceAddress(config_.i2c_addr);

  // // load in magnetometer calibration
  // if (!cal_mode) {
  //   __load_mag_calibration();
  // }

  return true;
}

bool MPU::PowerOffMagnetometer() {
  i2c_.SetDeviceAddress(config_.i2c_addr);
  // Enable i2c bypass to allow talking to magnetometer
  if (SetBypass(true)) {
    SYS_LOG_WARN("failed to set mpu9250 into bypass i2c mode");
    return false;
  }
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  i2c_.SetDeviceAddress(AK8963_ADDR);
  // Power down magnetometer
  if (not i2c_.WriteByte(AK8963_CNTL, MAG_POWER_DN)) {
    SYS_LOG_WARN("failed to write to magnetometer");
    return false;
  }
  i2c_.SetDeviceAddress(config_.i2c_addr);
  return true;
}

bool MPU::PowerOff() {
  // imu_shutdown_flag = 1;
  // wait for the interrupt thread to exit if it hasn't already
  // // allow up to 1 second for thread cleanup
  // if (thread_running_flag) {
  //   if (rc_pthread_timed_join(imu_interrupt_thread, NULL, 1.0) == 1) {
  //     SYS_LOG_WARN("WARNING: mpu interrupt thread exit timeout");
  //   }
  //   // cleanup mutexes
  //   pthread_cond_destroy(&read_condition);
  //   pthread_mutex_destroy(&read_mutex);
  //   pthread_cond_destroy(&tap_condition);
  //   pthread_mutex_destroy(&tap_mutex);
  // }

  // shutdown magnetometer first if on since that requires
  // the imu to the on for bypass to work
  if (config_.enable_magnetometer) {
    PowerOffMagnetometer();
  }

  // set the device address to write the shutdown register
  i2c_.SetDeviceAddress(config_.i2c_addr);
  // write the reset bit
  if (not i2c_.WriteByte(PWR_MGMT_1, H_RESET)) {
    // wait and try again
    MicroSleep(1000);
    if (not i2c_.WriteByte(PWR_MGMT_1, H_RESET)) {
      SYS_LOG_WARN("failed to shutdown MPU9250");
      return false;
    }
  }

  // write the sleep bit
  if (not i2c_.WriteByte(PWR_MGMT_1, MPU_SLEEP)) {
    // wait and try again
    MicroSleep(1000);
    if (not i2c_.WriteByte(PWR_MGMT_1, MPU_SLEEP)) {
      SYS_LOG_WARN("failed to sleep MPU9250");
      return false;
    }
  }

  // // if in dmp mode, also unexport the interrupt pin
  // if (dmp_en) {
  //   rc_gpio_cleanup(config.gpio_interrupt_pin_chip,
  //   config.gpio_interrupt_pin);
  // }

  return true;
}

// int rc_mpu_initialize_dmp(rc_mpu_data_t* data, rc_mpu_config_t conf) {
//   int i;
//   uint8_t tmp;
//   // range check
//   if (conf.dmp_sample_rate > DMP_MAX_RATE ||
//       conf.dmp_sample_rate < DMP_MIN_RATE) {
//     SYS_LOG_WARN("ERROR:dmp_sample_rate must be between %d & %d\n",
//             DMP_MIN_RATE, DMP_MAX_RATE);
//     return false;
//   }
//   // make sure the sample rate is a divisor so we can find a neat rate
//   divider if (DMP_MAX_RATE % conf.dmp_sample_rate != 0) {
//     SYS_LOG_WARN("DMP sample rate must be a divisor of 200");
//     SYS_LOG_WARN("acceptable values: 200,100,50,40,25,20,10,8,5,4
//     (HZ)"); return false;
//   }
//   // make sure the compass filter time constant is valid
//   if (conf.enable_magnetometer && conf.compass_time_constant <= 0.1) {
//     SYS_LOG_WARN("ERROR: compass time constant must be greater than
//     0.1"); return false;
//   }

//   // update local copy of config and data struct with new values
//   config = conf;
//   data_ptr = data;

//   // check dlpf
//   if (conf.gyro_dlpf == GYRO_DLPF_OFF || conf.gyro_dlpf == GYRO_DLPF_250) {
//     fprintf(stderr,
//             "WARNING, gyro dlpf bandwidth must be <= 184hz in DMP mode");
//     SYS_LOG_WARN("setting to 184hz automatically");
//     conf.gyro_dlpf = GYRO_DLPF_184;
//   }
//   if (conf.accel_dlpf == ACCEL_DLPF_OFF || conf.accel_dlpf == ACCEL_DLPF_460)
//   {
//     fprintf(stderr,
//             "WARNING, accel dlpf bandwidth must be <= 184hz in DMP mode");
//     SYS_LOG_WARN("setting to 184hz automatically");
//     conf.accel_dlpf = ACCEL_DLPF_184;
//   }
//   // check FSR
//   if (conf.gyro_fsr != GYRO_FSR_2000DPS) {
//     SYS_LOG_WARN("WARNING, gyro FSR must be GYRO_FSR_2000DPS in DMP
//     mode"); SYS_LOG_WARN("setting to 2000DPS automatically");
//     config.gyro_fsr = GYRO_FSR_2000DPS;
//   }
//   if (conf.accel_fsr != ACCEL_FSR_8G) {
//     SYS_LOG_WARN("WARNING, accel FSR must be ACCEL_FSR_8G in DMP mode");
//     SYS_LOG_WARN("setting to ACCEL_FSR_8G automatically");
//     config.accel_fsr = ACCEL_FSR_8G;
//   }

//   // start the i2c bus
//   if (rc_i2c_init(config_.i2c_bus, config_.i2c_addr)) {
//     SYS_LOG_WARN("rc_mpu_initialize_dmp failed at rc_i2c_init");
//     return false;
//   }
//   // configure the gpio interrupt pin
//   if (rc_gpio_init_event(config.gpio_interrupt_pin_chip,
//                          config.gpio_interrupt_pin, 0,
//                          GPIOEVENT_REQUEST_FALLING_EDGE) == -1) {
//     fprintf(stderr,
//             "ERROR: in rc_mpu_initialize_dmp, failed to initialize GPIO");
//     SYS_LOG_WARN("probably insufficient privileges");
//     return false;
//   }

//   // claiming the bus does no guarantee other code will not interfere
//   // with this process, but best to claim it so other code can check
//   rc_i2c_lock_bus(config_.i2c_bus);
//   // restart the device so we start with clean registers
//   if (__reset_mpu() < 0) {
//     SYS_LOG_WARN("failed to __reset_mpu()");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }
//   if (__check_who_am_i()) {
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }
//   // MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
//   // first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
//   // this is also set in set_accel_dlpf but we set here early on
//   tmp = BIT_FIFO_SIZE_1024 | 0x8;
//   if (rc_i2c_write_byte(config_.i2c_bus, ACCEL_CONFIG_2, tmp)) {
//     fprintf(stderr,
//             "ERROR: in rc_mpu_initialize_dmp, failed to write to "
//             "ACCEL_CONFIG_2 register");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }
//   // load in calibration offsets from disk
//   if (__load_gyro_calibration() < 0) {
//     SYS_LOG_WARN("ERROR: failed to load gyro calibration offsets");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }
//   if (__load_accel_calibration() < 0) {
//     SYS_LOG_WARN("ERROR: failed to load accel calibration offsets");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // set full scale ranges. It seems the DMP only scales the gyro properly
//   // at 2000DPS. I'll assume the same is true for accel and use 2G like their
//   // example
//   if (__set_gyro_fsr(config.gyro_fsr, data_ptr) == -1) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_initialize_dmp, failed to set gyro_fsr "
//             "register");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }
//   if (__set_accel_fsr(config.accel_fsr, data_ptr) == -1) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_initialize_dmp, failed to set accel_fsr "
//             "register");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // set dlpf, these values already checked for bounds above
//   if (__set_gyro_dlpf(conf.gyro_dlpf)) {
//     SYS_LOG_WARN("failed to set gyro dlpf");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }
//   if (__set_accel_dlpf(conf.accel_dlpf)) {
//     SYS_LOG_WARN("failed to set accel_dlpf");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // This actually sets the rate of accel/gyro sampling which should always
//   be
//   // 200 as the dmp filters at that rate
//   if (__mpu_set_sample_rate(200) < 0) {
//     // if(__mpu_set_sample_rate(config.dmp_sample_rate)<0){
//     SYS_LOG_WARN("ERROR: setting IMU sample rate");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // enable bypass, more importantly this also configures the interrupt pin
//   // behavior
//   if (__mpu_set_bypass(1)) {
//     SYS_LOG_WARN("failed to run __mpu_set_bypass");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // initialize the magnetometer too if requested in config
//   if (conf.enable_magnetometer) {
//     if (__init_magnetometer(0)) {
//       SYS_LOG_WARN("ERROR: failed to initialize_magnetometer");
//       rc_i2c_unlock_bus(config_.i2c_bus);
//       return false;
//     }
//     if (rc_mpu_read_mag(data) == -1) {
//       SYS_LOG_WARN("ERROR: failed to initialize_magnetometer");
//       rc_i2c_unlock_bus(config_.i2c_bus);
//       return false;
//     }
//     // collect some mag data to get a starting heading
//     double x_sum = 0.0;
//     double y_sum = 0.0;
//     double mag_vec[3];
//     for (i = 0; i < 20; i++) {
//       rc_mpu_read_mag(data);
//       // correct for orientation and put data into mag_vec
//       if (__mag_correct_orientation(mag_vec))
//         return false;
//       x_sum += mag_vec[0];
//       y_sum += mag_vec[1];
//       MicroSleep(10000);
//     }
//     startMagYaw = -atan2(y_sum, x_sum);
//   } else
//     __power_off_magnetometer();

//   // set up the DMP, order is important, from motiondrive_tutorial.pdf:
//   // 1) load firmware
//   // 2) set orientation matrix
//   // 3) enable callbacks (we don't do this here)
//   // 4) enable features
//   // 5) set fifo rate
//   // 6) set any feature-specific control functions
//   // 7) turn dmp on
//   dmp_en = 1;  // log locally that the dmp will be running
//   if (__dmp_load_motion_driver_firmware() < 0) {
//     SYS_LOG_WARN("failed to load DMP motion driver");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // set the orientation of dmp quaternion
//   if (__dmp_set_orientation((unsigned short)conf.orient) < 0) {
//     SYS_LOG_WARN("ERROR: failed to set dmp orientation");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   /// enbale quaternion feature and accel/gyro if requested
//   // due to a known bug in the DMP, the tap feature must be enabled to
//   // get interrupts slower than 200hz
//   unsigned short feature_mask = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP;

//   // enable gyro calibration is requested
//   if (config.dmp_auto_calibrate_gyro) {
//     feature_mask |= DMP_FEATURE_GYRO_CAL;
//   }
//   // enable reading accel/gyro is requested
//   if (config.dmp_fetch_accel_gyro) {
//     feature_mask |= DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_ANY_GYRO;
//   }
//   if (__dmp_enable_feature(feature_mask) < 0) {
//     SYS_LOG_WARN("ERROR: failed to enable DMP features");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // this changes the rate new dmp data is put in the fifo
//   // fixing at 200 causes gyro scaling issues at lower mpu sample rates
//   if (__dmp_set_fifo_rate(config.dmp_sample_rate) < 0) {
//     SYS_LOG_WARN("ERROR: failed to set DMP fifo rate");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // turn the dmp on
//   if (__mpu_set_dmp_state(1) < 0) {
//     SYS_LOG_WARN("ERROR: __mpu_set_dmp_state(1) failed");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // set interrupt mode to continuous as opposed to GESTURE
//   if (__dmp_set_interrupt_mode(DMP_INT_CONTINUOUS) < 0) {
//     SYS_LOG_WARN("ERROR: failed to set DMP interrupt mode to
//     continuous"); rc_i2c_unlock_bus(config_.i2c_bus); return false;
//   }

//   // done writing to bus for now
//   rc_i2c_unlock_bus(config_.i2c_bus);

//   // get ready to start the interrupt handler thread
//   data_ptr->tap_detected = 0;
//   imu_shutdown_flag = 0;
//   dmp_callback_func = NULL;
//   tap_callback_func = NULL;

//   // start the thread
//   if (rc_pthread_create(&imu_interrupt_thread, __dmp_interrupt_handler, NULL,
//                         config.dmp_interrupt_sched_policy,
//                         config.dmp_interrupt_priority) < 0) {
//     SYS_LOG_WARN("ERROR failed to start dmp handler thread");
//     return false;
//   }
//   thread_running_flag = 1;

//   // sleep for a ms so the thread can start predictably
//   MicroSleep(1000);
//   return 0;
// }

// /**
//  *  @brief      Write to the DMP memory.
//  *  This function prevents I2C writes past the bank boundaries. The DMP
//  memory
//  *  is only accessible when the chip is awake.
//  *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
//  *  @param[in]  length      Number of bytes to write.
//  *  @param[in]  data        Bytes to write to memory.
//  *  @return     0 if successful.
//  **/
// int __mpu_write_mem(unsigned short mem_addr, unsigned short length,
//                     unsigned char* data) {
//   unsigned char tmp[2];
//   if (!data) {
//     SYS_LOG_WARN("ERROR: in mpu_write_mem, NULL pointer");
//     return false;
//   }
//   tmp[0] = (unsigned char)(mem_addr >> 8);
//   tmp[1] = (unsigned char)(mem_addr & 0xFF);
//   // Check bank boundaries.
//   if (tmp[1] + length > MPU6500_BANK_SIZE) {
//     SYS_LOG_WARN("mpu_write_mem exceeds bank size");
//     return false;
//   }
//   if (rc_i2c_write_bytes(config_.i2c_bus, MPU6500_BANK_SEL, 2, tmp))
//     return false;
//   if (rc_i2c_write_bytes(config_.i2c_bus, MPU6500_MEM_R_W, length, data))
//     return false;
//   return 0;
// }

// /**
//  *  @brief      Read from the DMP memory.
//  *  This function prevents I2C reads past the bank boundaries. The DMP memory
//  *  is only accessible when the chip is awake.
//  *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
//  *  @param[in]  length      Number of bytes to read.
//  *  @param[out] data        Bytes read from memory.
//  *  @return     0 if successful.
//  **/
// int __mpu_read_mem(unsigned short mem_addr, unsigned short length,
//                    unsigned char* data) {
//   unsigned char tmp[2];
//   if (!data) {
//     SYS_LOG_WARN("ERROR: in mpu_write_mem, NULL pointer");
//     return false;
//   }
//   tmp[0] = (unsigned char)(mem_addr >> 8);
//   tmp[1] = (unsigned char)(mem_addr & 0xFF);
//   // Check bank boundaries.
//   if (tmp[1] + length > MPU6500_BANK_SIZE) {
//     printf("mpu_read_mem exceeds bank size");
//     return false;
//   }
//   if (rc_i2c_write_bytes(config_.i2c_bus, MPU6500_BANK_SEL, 2, tmp))
//     return false;
//   if (rc_i2c_read_bytes(config_.i2c_bus, MPU6500_MEM_R_W, length, data) !=
//       length)
//     return false;
//   return 0;
// }

// /**
//  * int __dmp_load_motion_driver_firmware()
//  *
//  * loads pre-compiled firmware binary from invensense onto dmp
//  **/
// int __dmp_load_motion_driver_firmware(void) {
//   unsigned short ii;
//   unsigned short this_write;
//   // Must divide evenly into st.hw->bank_size to avoid bank crossings.
//   unsigned char cur[DMP_LOAD_CHUNK], tmp[2];
//   // make sure the address is set correctly
//   rc_i2c_set_device_address(config_.i2c_bus, config_.i2c_addr);
//   // loop through 16 bytes at a time and check each write for corruption
//   for (ii = 0; ii < DMP_CODE_SIZE; ii += this_write) {
//     this_write = min(DMP_LOAD_CHUNK, DMP_CODE_SIZE - ii);
//     if (__mpu_write_mem(ii, this_write, (uint8_t*)&dmp_firmware[ii])) {
//       SYS_LOG_WARN("dmp firmware write failed");
//       return false;
//     }
//     if (__mpu_read_mem(ii, this_write, cur)) {
//       SYS_LOG_WARN("dmp firmware read failed");
//       return false;
//     }
//     if (memcmp(dmp_firmware + ii, cur, this_write)) {
//       SYS_LOG_WARN("dmp firmware write corrupted");
//       return -2;
//     }
//   }
//   // Set program start address.
//   tmp[0] = dmp_start_addr >> 8;
//   tmp[1] = dmp_start_addr & 0xFF;
//   if (rc_i2c_write_bytes(config_.i2c_bus, MPU6500_PRGM_START_H, 2, tmp)) {
//     SYS_LOG_WARN("ERROR writing to MPU6500_PRGM_START register");
//     return false;
//   }
//   return 0;
// }

// /**
//  *  @brief      Push gyro and accel orientation to the DMP.
//  *  The orientation is represented here as the output of
//  *  @e __inv_orientation_matrix_to_scalar.
//  *  @param[in]  orient  Gyro and accel orientation in body frame.
//  *  @return     0 if successful.
//  **/
// int __dmp_set_orientation(unsigned short orient) {
//   unsigned char gyro_regs[3], accel_regs[3];
//   const unsigned char gyro_axes[3] = {DINA4C, DINACD, DINA6C};
//   const unsigned char accel_axes[3] = {DINA0C, DINAC9, DINA2C};
//   const unsigned char gyro_sign[3] = {DINA36, DINA56, DINA76};
//   const unsigned char accel_sign[3] = {DINA26, DINA46, DINA66};
//   // populate fata to be written
//   gyro_regs[0] = gyro_axes[orient & 3];
//   gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
//   gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
//   accel_regs[0] = accel_axes[orient & 3];
//   accel_regs[1] = accel_axes[(orient >> 3) & 3];
//   accel_regs[2] = accel_axes[(orient >> 6) & 3];
//   // Chip-to-body, axes only.
//   if (__mpu_write_mem(FCFG_1, 3, gyro_regs)) {
//     SYS_LOG_WARN("ERROR: in dmp_set_orientation, failed to write dmp
//     mem"); return false;
//   }
//   if (__mpu_write_mem(FCFG_2, 3, accel_regs)) {
//     SYS_LOG_WARN("ERROR: in dmp_set_orientation, failed to write dmp
//     mem"); return false;
//   }
//   memcpy(gyro_regs, gyro_sign, 3);
//   memcpy(accel_regs, accel_sign, 3);
//   if (orient & 4) {
//     gyro_regs[0] |= 1;
//     accel_regs[0] |= 1;
//   }
//   if (orient & 0x20) {
//     gyro_regs[1] |= 1;
//     accel_regs[1] |= 1;
//   }
//   if (orient & 0x100) {
//     gyro_regs[2] |= 1;
//     accel_regs[2] |= 1;
//   }
//   // Chip-to-body, sign only.
//   if (__mpu_write_mem(FCFG_3, 3, gyro_regs)) {
//     SYS_LOG_WARN("ERROR: in dmp_set_orientation, failed to write dmp
//     mem"); return false;
//   }
//   if (__mpu_write_mem(FCFG_7, 3, accel_regs)) {
//     SYS_LOG_WARN("ERROR: in dmp_set_orientation, failed to write dmp
//     mem"); return false;
//   }
//   return 0;
// }

// /**
//  *  @brief      Set DMP output rate.
//  *  Only used when DMP is on.
//  *  @param[in]  rate    Desired fifo rate (Hz).
//  *  @return     0 if successful.
//  **/
// int __dmp_set_fifo_rate(unsigned short rate) {
//   const unsigned char regs_end[12] = {DINAFE, DINAF2, DINAAB, 0xc4,
//                                       DINAAA, DINAF1, DINADF, DINADF,
//                                       0xBB,   0xAF,   DINADF, DINADF};
//   unsigned short div;
//   unsigned char tmp[8];
//   if (rate > DMP_MAX_RATE) {
//     return false;
//   }
//   // set the DMP scaling factors
//   div = DMP_MAX_RATE / rate - 1;
//   tmp[0] = (unsigned char)((div >> 8) & 0xFF);
//   tmp[1] = (unsigned char)(div & 0xFF);
//   if (__mpu_write_mem(D_0_22, 2, tmp)) {
//     SYS_LOG_WARN("ERROR: writing dmp sample rate reg");
//     return false;
//   }
//   if (__mpu_write_mem(CFG_6, 12, (unsigned char*)regs_end)) {
//     SYS_LOG_WARN("ERROR: writing dmp regs_end");
//     return false;
//   }
//   return 0;
// }

/**
 * int __mpu_set_bypass(unsigned char bypass_on)
 *
 * configures the USER_CTRL and INT_PIN_CFG registers to turn on and off the
 * i2c bypass mode for talking to the magnetometer. In random read mode this
 * is used to turn on the bypass and left as is. In DMP mode bypass is turned
 * off after configuration and the MPU fetches magnetometer data
 automatically.
 * USER_CTRL - based on global variable dsp_en
 * INT_PIN_CFG based on requested bypass state
 **/
bool MPU::SetBypass(const bool bypass_on) {
  uint8_t tmp = 0;
  i2c_.SetDeviceAddress(config_.i2c_addr);
  // set up USER_CTRL first
  // DONT USE FIFO_EN_BIT in DMP mode, or the MPU will generate lots of
  // unwanted interrupts
  // if (dmp_en) {
  //   tmp |= FIFO_EN_BIT;  // enable fifo for dsp mode
  // }
  if (!bypass_on) {
    tmp |= I2C_MST_EN;  // i2c master mode when not in bypass
  }
  if (not i2c_.WriteByte(USER_CTRL, tmp)) {
    SYS_LOG_WARN("mpu_set_bypass, failed to write USER_CTRL register");
    return false;
  }
  MicroSleep(3000);
  // INT_PIN_CFG settings
  // tmp = LATCH_INT_EN | INT_ANYRD_CLEAR | ACTL_ACTIVE_LOW;  // latching
  // tmp =  ACTL_ACTIVE_LOW;	// non-latching
  if (bypass_on) {
    tmp |= BYPASS_EN;
  }
  if (not i2c_.WriteByte(INT_PIN_CFG, tmp)) {
    SYS_LOG_WARN("mpu_set_bypass, failed to write INT_PIN_CFG register");
    return false;
  }
  is_bypass_ = bypass_on;
  return true;
}

// /**
//  * int __dmp_enable_gyro_cal(unsigned char enable)
//  *
//  * Taken straight from the Invensense DMP code. This enabled the automatic
//  gyro
//  * calibration feature in the DMP. This this feature is fine for cell phones
//  * but annoying in control systems we do not use it here and instead ask
//  users
//  * to run our own gyro_calibration routine.
//  **/
// int __dmp_enable_gyro_cal(unsigned char enable) {
//   if (enable) {
//     unsigned char regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4,
//                              0x98, 0x0d, 0x35, 0x5d};
//     return __mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
//   } else {
//     unsigned char regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0,
//                              0x88, 0xc3, 0xc5, 0xc7};
//     return __mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
//   }
// }

// /**
//  * int __dmp_enable_6x_lp_quat(unsigned char enable)
//  *
//  * Taken straight from the Invensense DMP code. This enabled quaternion
//  *filtering with accelerometer and gyro filtering.
//  **/
// int __dmp_enable_6x_lp_quat(unsigned char enable) {
//   unsigned char regs[4];
//   if (enable) {
//     regs[0] = DINA20;
//     regs[1] = DINA28;
//     regs[2] = DINA30;
//     regs[3] = DINA38;
//   } else {
//     memset(regs, 0xA3, 4);
//   }
//   __mpu_write_mem(CFG_8, 4, regs);
//   return 0;
// }

// /**
//  * int __dmp_enable_lp_quat(unsigned char enable)
//  *
//  * sets the DMP to do gyro-only quaternion filtering. This is not actually
//  used
//  * here but remains as a vestige of the Invensense DMP code.
//  **/
// int __dmp_enable_lp_quat(unsigned char enable) {
//   unsigned char regs[4];
//   if (enable) {
//     regs[0] = DINBC0;
//     regs[1] = DINBC2;
//     regs[2] = DINBC4;
//     regs[3] = DINBC6;
//   } else {
//     memset(regs, 0x8B, 4);
//   }
//   __mpu_write_mem(CFG_LP_QUAT, 4, regs);
//   return 0;
// }

// /**
//  * int __mpu_reset_fifo()
//  *
//  * This is mostly from the Invensense open source codebase but modified to
//  also
//  * allow magnetometer data to come in through the FIFO. This just turns off
//  the
//  * interrupt, resets fifo and DMP, then starts them again. Used once while
//  * initializing (probably no necessary) then again if the fifo gets too full.
//  **/
// int __mpu_reset_fifo(void) {
//   uint8_t data;
//   // make sure the i2c address is set correctly.
//   // this shouldn't take any time at all if already set
//   rc_i2c_set_device_address(config_.i2c_bus, config_.i2c_addr);
//   // turn off interrupts, fifo, and usr_ctrl which is where the dmp fifo is
//   // enabled
//   data = 0;
//   if (rc_i2c_write_byte(config_.i2c_bus, INT_ENABLE, data))
//     return false;
//   if (rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, data))
//     return false;
//   if (rc_i2c_write_byte(config_.i2c_bus, USER_CTRL, data))
//     return false;

//   // reset fifo and wait
//   data = BIT_FIFO_RST | BIT_DMP_RST;
//   if (rc_i2c_write_byte(config_.i2c_bus, USER_CTRL, data))
//     return false;
//   // MicroSleep(1000); // how I had it
//   MicroSleep(50000);  // invensense standard

//   // enable the fifo and DMP fifo flags again
//   // enabling DMP but NOT BIT_FIFO_EN gives quat out of bounds
//   // but also no empty interrupts
//   data = BIT_DMP_EN | BIT_FIFO_EN;
//   if (rc_i2c_write_byte(config_.i2c_bus, USER_CTRL, data)) {
//     return false;
//   }

//   // turn on dmp interrupt enable bit again
//   data = BIT_DMP_INT_EN;
//   if (rc_i2c_write_byte(config_.i2c_bus, INT_ENABLE, data))
//     return false;
//   data = 0;
//   if (rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, data))
//     return false;

//   return 0;
// }

// /**
//  * int __dmp_set_interrupt_mode(unsigned char mode)
//  *
//  * This is from the Invensense open source DMP code. It configures the DMP to
//  * trigger an interrupt either every sample or only on gestures. Here we only
//  * ever configure for continuous sampling.
//  *
//  * @param[in]  mode  The mode
//  *
//  * @return     { description_of_the_return_value }
//  */
// int __dmp_set_interrupt_mode(unsigned char mode) {
//   const unsigned char regs_continuous[11] = {0xd8, 0xb1, 0xb9, 0xf3, 0x8b,
//   0xa3,
//                                              0x91, 0xb6, 0x09, 0xb4, 0xd9};
//   const unsigned char regs_gesture[11] = {0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3,
//                                           0x91, 0xb6, 0xda, 0xb4, 0xda};
//   switch (mode) {
//     case DMP_INT_CONTINUOUS:
//       return __mpu_write_mem(CFG_FIFO_ON_EVENT, 11,
//                              (unsigned char*)regs_continuous);
//     case DMP_INT_GESTURE:
//       return __mpu_write_mem(CFG_FIFO_ON_EVENT, 11,
//                              (unsigned char*)regs_gesture);
//     default:
//       return false;
//   }
// }

// /**
//  *  @brief      Set tap threshold for a specific axis.
//  *  @param[in]  axis    1, 2, and 4 for XYZ accel, respectively.
//  *  @param[in]  thresh  Tap threshold, in mg/ms.
//  *  @return     0 if successful.
//  */
// int __dmp_set_tap_thresh(unsigned char axis, unsigned short thresh) {
//   unsigned char tmp[4];
//   double scaled_thresh;
//   unsigned short dmp_thresh, dmp_thresh_2;
//   if (!(axis & TAP_XYZ) || thresh > 1600)
//     return false;

//   scaled_thresh = (double)thresh / DMP_SAMPLE_RATE;

//   switch (config.accel_fsr) {
//     case ACCEL_FSR_2G:
//       dmp_thresh = (unsigned short)(scaled_thresh * 16384);
//       /* dmp_thresh * 0.75 */
//       dmp_thresh_2 = (unsigned short)(scaled_thresh * 12288);
//       break;
//     case ACCEL_FSR_4G:
//       dmp_thresh = (unsigned short)(scaled_thresh * 8192);
//       /* dmp_thresh * 0.75 */
//       dmp_thresh_2 = (unsigned short)(scaled_thresh * 6144);
//       break;
//     case ACCEL_FSR_8G:
//       dmp_thresh = (unsigned short)(scaled_thresh * 4096);
//       /* dmp_thresh * 0.75 */
//       dmp_thresh_2 = (unsigned short)(scaled_thresh * 3072);
//       break;
//     case ACCEL_FSR_16G:
//       dmp_thresh = (unsigned short)(scaled_thresh * 2048);
//       /* dmp_thresh * 0.75 */
//       dmp_thresh_2 = (unsigned short)(scaled_thresh * 1536);
//       break;
//     default:
//       return false;
//   }
//   tmp[0] = (unsigned char)(dmp_thresh >> 8);
//   tmp[1] = (unsigned char)(dmp_thresh & 0xFF);
//   tmp[2] = (unsigned char)(dmp_thresh_2 >> 8);
//   tmp[3] = (unsigned char)(dmp_thresh_2 & 0xFF);

//   if (axis & TAP_X) {
//     if (__mpu_write_mem(DMP_TAP_THX, 2, tmp))
//       return false;
//     if (__mpu_write_mem(D_1_36, 2, tmp + 2))
//       return false;
//   }
//   if (axis & TAP_Y) {
//     if (__mpu_write_mem(DMP_TAP_THY, 2, tmp))
//       return false;
//     if (__mpu_write_mem(D_1_40, 2, tmp + 2))
//       return false;
//   }
//   if (axis & TAP_Z) {
//     if (__mpu_write_mem(DMP_TAP_THZ, 2, tmp))
//       return false;
//     if (__mpu_write_mem(D_1_44, 2, tmp + 2))
//       return false;
//   }
//   return 0;
// }

// /**
//  *  @brief      Set which axes will register a tap.
//  *  @param[in]  axis    1, 2, and 4 for XYZ, respectively.
//  *  @return     0 if successful.
//  */
// int __dmp_set_tap_axes(unsigned char axis) {
//   unsigned char tmp = 0;

//   if (axis & TAP_X)
//     tmp |= 0x30;
//   if (axis & TAP_Y)
//     tmp |= 0x0C;
//   if (axis & TAP_Z)
//     tmp |= 0x03;
//   return __mpu_write_mem(D_1_72, 1, &tmp);
// }

// /**
//  *  @brief      Set minimum number of taps needed for an interrupt.
//  *  @param[in]  min_taps    Minimum consecutive taps (1-4).
//  *  @return     0 if successful.
//  */
// int __dmp_set_tap_count(unsigned char min_taps) {
//   unsigned char tmp;

//   if (min_taps < 1)
//     min_taps = 1;
//   else if (min_taps > 4)
//     min_taps = 4;

//   tmp = min_taps - 1;
//   return __mpu_write_mem(D_1_79, 1, &tmp);
// }

// /**
//  *  @brief      Set length between valid taps.
//  *  @param[in]  time    Milliseconds between taps.
//  *  @return     0 if successful.
//  */
// int __dmp_set_tap_time(unsigned short time) {
//   unsigned short dmp_time;
//   unsigned char tmp[2];

//   dmp_time = time / (1000 / DMP_SAMPLE_RATE);
//   tmp[0] = (unsigned char)(dmp_time >> 8);
//   tmp[1] = (unsigned char)(dmp_time & 0xFF);
//   return __mpu_write_mem(DMP_TAPW_MIN, 2, tmp);
// }

// /**
//  *  @brief      Set max time between taps to register as a multi-tap.
//  *  @param[in]  time    Max milliseconds between taps.
//  *  @return     0 if successful.
//  */
// int __dmp_set_tap_time_multi(unsigned short time) {
//   unsigned short dmp_time;
//   unsigned char tmp[2];

//   dmp_time = time / (1000 / DMP_SAMPLE_RATE);
//   tmp[0] = (unsigned char)(dmp_time >> 8);
//   tmp[1] = (unsigned char)(dmp_time & 0xFF);
//   return __mpu_write_mem(D_1_218, 2, tmp);
// }

// /**
//  *  @brief      Set shake rejection threshold.
//  *  If the DMP detects a gyro sample larger than @e thresh, taps are
//  rejected.
//  *  @param[in]  sf      Gyro scale factor.
//  *  @param[in]  thresh  Gyro threshold in dps.
//  *  @return     0 if successful.
//  */
// int __dmp_set_shake_reject_thresh(long sf, unsigned short thresh) {
//   unsigned char tmp[4];
//   long thresh_scaled = sf / 1000 * thresh;
//   tmp[0] = (unsigned char)(((long)thresh_scaled >> 24) & 0xFF);
//   tmp[1] = (unsigned char)(((long)thresh_scaled >> 16) & 0xFF);
//   tmp[2] = (unsigned char)(((long)thresh_scaled >> 8) & 0xFF);
//   tmp[3] = (unsigned char)((long)thresh_scaled & 0xFF);
//   return __mpu_write_mem(D_1_92, 4, tmp);
// }

// /**
//  *  @brief      Set shake rejection time.
//  *  Sets the length of time that the gyro must be outside of the threshold
//  set
//  *  by @e gyro_set_shake_reject_thresh before taps are rejected. A mandatory
//  *  60 ms is added to this parameter.
//  *  @param[in]  time    Time in milliseconds.
//  *  @return     0 if successful.
//  */
// int __dmp_set_shake_reject_time(unsigned short time) {
//   unsigned char tmp[2];

//   time /= (1000 / DMP_SAMPLE_RATE);
//   tmp[0] = time >> 8;
//   tmp[1] = time & 0xFF;
//   return __mpu_write_mem(D_1_90, 2, tmp);
// }

// /**
//  *  @brief      Set shake rejection timeout.
//  *  Sets the length of time after a shake rejection that the gyro must stay
//  *  inside of the threshold before taps can be detected again. A mandatory
//  *  60 ms is added to this parameter.
//  *  @param[in]  time    Time in milliseconds.
//  *  @return     0 if successful.
//  */
// int __dmp_set_shake_reject_timeout(unsigned short time) {
//   unsigned char tmp[2];

//   time /= (1000 / DMP_SAMPLE_RATE);
//   tmp[0] = time >> 8;
//   tmp[1] = time & 0xFF;
//   return __mpu_write_mem(D_1_88, 2, tmp);
// }

// /**
//  * int __dmp_enable_feature(unsigned short mask)
//  *
//  * This is mostly taken from the Invensense DMP code and serves to turn on
//  and
//  * off DMP features based on the feature mask. We modified to remove some
//  * irrelevant features and set our own fifo-length variable. This probably
//  isn't
//  * necessary to remain in its current form as rc_mpu_initialize_dmp uses a
//  fixed
//  * set of features but we keep it as is since it works fine.
//  *
//  * @param[in]  mask  The mask
//  *
//  * @return     0 on success, -1 on failure
//  */
// int __dmp_enable_feature(unsigned short mask) {
//   unsigned char tmp[10];
//   // Set integration scale factor.
//   tmp[0] = (unsigned char)((GYRO_SF >> 24) & 0xFF);
//   tmp[1] = (unsigned char)((GYRO_SF >> 16) & 0xFF);
//   tmp[2] = (unsigned char)((GYRO_SF >> 8) & 0xFF);
//   tmp[3] = (unsigned char)(GYRO_SF & 0xFF);
//   if (__mpu_write_mem(D_0_104, 4, tmp) < 0) {
//     SYS_LOG_WARN("ERROR: in dmp_enable_feature, failed to write mpu
//     mem"); return false;
//   }
//   // Send sensor data to the FIFO.
//   tmp[0] = 0xA3;
//   if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
//     tmp[1] = 0xC0;
//     tmp[2] = 0xC8;
//     tmp[3] = 0xC2;
//   } else {
//     tmp[1] = 0xA3;
//     tmp[2] = 0xA3;
//     tmp[3] = 0xA3;
//   }
//   if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
//     tmp[4] = 0xC4;
//     tmp[5] = 0xCC;
//     tmp[6] = 0xC6;
//   } else {
//     tmp[4] = 0xA3;
//     tmp[5] = 0xA3;
//     tmp[6] = 0xA3;
//   }
//   tmp[7] = 0xA3;
//   tmp[8] = 0xA3;
//   tmp[9] = 0xA3;
//   if (__mpu_write_mem(CFG_15, 10, tmp) < 0) {
//     SYS_LOG_WARN("ERROR: in dmp_enable_feature, failed to write mpu
//     mem"); return false;
//   }
//   // Send gesture data to the FIFO.
//   if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
//     tmp[0] = DINA20;
//   } else {
//     tmp[0] = 0xD8;
//   }
//   if (__mpu_write_mem(CFG_27, 1, tmp)) {
//     SYS_LOG_WARN("ERROR: in dmp_enable_feature, failed to write mpu
//     mem"); return false;
//   }

//   if (mask & DMP_FEATURE_GYRO_CAL)
//     __dmp_enable_gyro_cal(1);
//   else
//     __dmp_enable_gyro_cal(0);

//   if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
//     if (mask & DMP_FEATURE_SEND_CAL_GYRO) {
//       tmp[0] = 0xB2;
//       tmp[1] = 0x8B;
//       tmp[2] = 0xB6;
//       tmp[3] = 0x9B;
//     } else {
//       tmp[0] = DINAC0;
//       tmp[1] = DINA80;
//       tmp[2] = DINAC2;
//       tmp[3] = DINA90;
//     }
//     __mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
//   }

//   // configure tap feature
//   if (mask & DMP_FEATURE_TAP) {
//     /* Enable tap. */
//     tmp[0] = 0xF8;
//     __mpu_write_mem(CFG_20, 1, tmp);
//     __dmp_set_tap_thresh(TAP_XYZ, config.tap_threshold);
//     __dmp_set_tap_axes(TAP_XYZ);
//     __dmp_set_tap_count(1);   // minimum number of taps needed for an
//     interrupt
//                               // (1-4)
//     __dmp_set_tap_time(100);  // ms between taps (factory default 100)
//     __dmp_set_tap_time_multi(600);  // max time between taps for multitap
//                                     // detection (factory default 500)

//     // shake rejection ignores taps when system is moving, set threshold
//     // high so this doesn't happen too often
//     __dmp_set_shake_reject_thresh(GYRO_SF, 300);  // default was 200
//     __dmp_set_shake_reject_time(80);
//     __dmp_set_shake_reject_timeout(100);
//   } else {
//     tmp[0] = 0xD8;
//     __mpu_write_mem(CFG_20, 1, tmp);
//   }

//   if (mask & DMP_FEATURE_ANDROID_ORIENT) {
//     tmp[0] = 0xD9;
//   } else
//     tmp[0] = 0xD8;
//   __mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);

//   if (mask & DMP_FEATURE_LP_QUAT) {
//     __dmp_enable_lp_quat(1);
//   } else {
//     __dmp_enable_lp_quat(0);
//   }
//   if (mask & DMP_FEATURE_6X_LP_QUAT) {
//     __dmp_enable_6x_lp_quat(1);
//   } else {
//     __dmp_enable_6x_lp_quat(0);
//   }
//   __mpu_reset_fifo();
//   packet_len = 0;
//   if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
//     packet_len += 6;
//   }
//   if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
//     packet_len += 6;
//   }
//   if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
//     packet_len += 16;
//   }
//   if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
//     packet_len += 4;
//   }
//   return 0;
// }
// /**
//  * @brief      This is a vestige of the invensense mpu open source code and
//  is
//  *             probably not necessary but remains here anyway.
//  *
//  * @param[in]  enable  The enable
//  *
//  * @return     0 on success, -1 on failure
//  */
// int __set_int_enable(unsigned char enable) {
//   unsigned char tmp;
//   if (enable) {
//     tmp = BIT_DMP_INT_EN;
//   } else {
//     tmp = 0x00;
//   }
//   if (rc_i2c_write_byte(config_.i2c_bus, INT_ENABLE, tmp)) {
//     fprintf(stderr,
//             "ERROR: in set_int_enable, failed to write INT_ENABLE
//             register");
//     return false;
//   }
//   // disable all other FIFO features leaving just DMP
//   if (rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, 0)) {
//     fprintf(stderr,
//             "ERROR: in set_int_enable, failed to write FIFO_EN register");
//     return false;
//   }
//   return 0;
// }

// /**
//  * @brief      Sets the clock rate divider for sensor sampling
//  *
//  * @param[in]  rate  The rate
//  *
//  * @return     0 on success, -1 on failure
//  */
// int __mpu_set_sample_rate(int rate) {
//   if (rate > 1000 || rate < 4) {
//     SYS_LOG_WARN("ERROR: sample rate must be between 4 & 1000");
//     return false;
//   }
//   /* Keep constant sample rate, FIFO rate controlled by DMP. */
//   uint8_t div = (1000 / rate) - 1;
// #ifdef DEBUG
//   printf("setting divider to %d\n", div);
// #endif
//   if (rc_i2c_write_byte(config_.i2c_bus, SMPLRT_DIV, div)) {
//     fprintf(stderr,
//             "ERROR: in mpu_set_sample_rate, failed to write SMPLRT_DIV "
//             "register");
//     return false;
//   }
//   return 0;
// }

// /**
//  * This turns on and off the DMP interrupt and resets the FIFO. This probably
//  * isn't necessary as rc_mpu_initialize_dmp sets these registers but it
//  remains
//  * here as a vestige of the invensense open source dmp code.
//  *
//  * @param[in]  enable  The enable
//  *
//  * @return     0 on success, -1 on failure
//  */
// int __mpu_set_dmp_state(unsigned char enable) {
//   if (enable) {
//     // Disable data ready interrupt.
//     __set_int_enable(0);
//     // make sure bypass mode is enabled
//     __mpu_set_bypass(1);
//     // Remove FIFO elements.
//     rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, 0);
//     // Enable DMP interrupt.
//     __set_int_enable(1);
//     __mpu_reset_fifo();
//   } else {
//     // Disable DMP interrupt.
//     __set_int_enable(0);
//     // Restore FIFO settings.
//     rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, 0);
//     __mpu_reset_fifo();
//   }
//   return 0;
// }

// /**
//  * Here is where the magic happens. This function runs as its own thread and
//  * monitors the gpio pin config.gpio_interrupt_pin with the blocking function
//  * call poll(). If a valid interrupt is received from the IMU then mark the
//  * timestamp, read in the IMU data, and call the user-defined interrupt
//  function
//  * if set.
//  *
//  * @return     0 on success, -1 on failure
//  */
// void* __dmp_interrupt_handler(__attribute__((unused)) void* ptr) {
//   // struct pollfd fdset[1];
//   int ret;
//   // start magnetometer read divider at the end of the counter
//   // so it reads on the first run
//   int mag_div_step = config.mag_sample_rate_div;
//   // char buf[64];
//   int first_run = 1;
//   __mpu_reset_fifo();

//   while (!imu_shutdown_flag) {
//     // system hangs here until IMU FIFO interrupt
//     ret =
//       rc_gpio_poll(config.gpio_interrupt_pin_chip, config.gpio_interrupt_pin,
//                    IMU_POLL_TIMEOUT, &last_interrupt_timestamp_nanos);
//     // check for bad things that may have happened
//     if (imu_shutdown_flag)
//       break;
//     if (ret == RC_GPIOEVENT_ERROR) {
//       SYS_LOG_WARN("ERROR in IMU interrupt handler calling poll");
//       continue;
//     }
//     if (ret == RC_GPIOEVENT_TIMEOUT) {
//       if (config.show_warnings) {
//         SYS_LOG_WARN("WARNING, gpio poll timeout");
//       }
//       continue;
//     }

//     // try to load fifo no matter the claim bus state
//     if (rc_i2c_get_lock(config_.i2c_bus)) {
//       SYS_LOG_WARN("WARNING: Something has claimed the I2C bus when
//       an"); SYS_LOG_WARN("IMU interrupt was received. Reading IMU
//       anyway.");
//     }
//     // aquires bus
//     rc_i2c_lock_bus(config_.i2c_bus);
//     // aquires mutex
//     pthread_mutex_lock(&read_mutex);
//     pthread_mutex_lock(&tap_mutex);
//     // read data
//     ret = __read_dmp_fifo(data_ptr);
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     // record if it was successful or not
//     if (ret == 0) {
//       last_read_successful = 1;
//       if (data_ptr->tap_detected) {
//         last_tap_timestamp_nanos = last_interrupt_timestamp_nanos;
//       }
//     } else {
//       last_read_successful = 0;
//     }
//     // if reading mag before callback, check divider and do it now
//     if (config.enable_magnetometer && !config.read_mag_after_callback) {
//       if (mag_div_step >= config.mag_sample_rate_div) {
// #ifdef DEBUG
//         printf("reading mag before callback");
// #endif
//         rc_mpu_read_mag(data_ptr);
//         // reset address back for next read
//         rc_i2c_set_device_address(config_.i2c_bus, config_.i2c_addr);
//         mag_div_step = 1;
//       } else
//         mag_div_step++;
//     }
//     // releases bus
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     // call the user function if not the first run
//     if (first_run == 1) {
//       first_run = 0;
//     } else if (last_read_successful) {
//       if (dmp_callback_func != NULL)
//         dmp_callback_func();
//       // signals that a measurement is available to blocking function
//       pthread_cond_broadcast(&read_condition);
//       // additionally call tap callback if one was received
//       if (data_ptr->tap_detected) {
//         if (tap_callback_func != NULL)
//           tap_callback_func(data_ptr->last_tap_direction,
//                             data_ptr->last_tap_count);
//         pthread_cond_broadcast(&tap_condition);
//       }
//     }

//     // releases mutex
//     pthread_mutex_unlock(&read_mutex);
//     pthread_mutex_unlock(&tap_mutex);

//     // if reading mag after interrupt, check divider and do it now
//     if (config.enable_magnetometer && config.read_mag_after_callback) {
//       if (mag_div_step >= config.mag_sample_rate_div) {
// #ifdef DEBUG
//         printf("reading mag after ISR");
// #endif
//         rc_i2c_lock_bus(config_.i2c_bus);
//         rc_mpu_read_mag(data_ptr);
//         rc_i2c_unlock_bus(config_.i2c_bus);
//         // reset address back for next read
//         rc_i2c_set_device_address(config_.i2c_bus, config_.i2c_addr);
//         mag_div_step = 1;
//       } else
//         mag_div_step++;
//     }
//   }

//   // shutting down now, do some cleanup
//   // aquires mutex
//   pthread_mutex_lock(&read_mutex);
//   // /releases other threads
//   pthread_cond_broadcast(&read_condition);
//   // releases mutex
//   pthread_mutex_unlock(&read_mutex);
//   thread_running_flag = 0;
//   return 0;
// }

// /**
//  * sets a user function to be called when new data is read
//  *
//  * @param[in]  func  The function
//  *
//  * @return     0 on success, -1 on failure
//  */
// int rc_mpu_set_dmp_callback(void (*func)(void)) {
//   if (func == NULL) {
//     fprintf(stderr,
//             "ERROR: trying to assign NULL pointer to dmp_callback_func");
//     return false;
//   }
//   dmp_callback_func = func;
//   return 0;
// }

// int rc_mpu_set_tap_callback(void (*func)(int dir, int cnt)) {
//   if (func == NULL) {
//     fprintf(stderr,
//             "ERROR: trying to assign NULL pointer to tap_callback_func");
//     return false;
//   }
//   tap_callback_func = func;
//   return 0;
// }

// /**
//  * Reads the FIFO buffer and populates the data struct. Here is where we see
//  * bad/empty/double packets due to i2c bus errors and the IMU failing to have
//  * data ready in time. enabling warnings in the config struct will let this
//  * function print out warnings when these conditions are detected. If write
//  * errors are detected then this function tries some i2c transfers a second
//  * time.
//  *
//  * @param      data  The data pointer
//  *
//  * @return     0 on success, -1 on failure
//  */
// int __read_dmp_fifo(rc_mpu_data_t* data) {
//   unsigned char raw[MAX_FIFO_BUFFER];
//   int32_t quat_q14[4], quat[4], quat_mag_sq;
//   uint16_t fifo_count;
//   int ret;
//   int i = 0;                 // position of beginning of quaternion
//   int j = 0;                 // position of beginning of accel/gyro data
//   static int first_run = 1;  // set to 0 after first call
//   double q_tmp[4];
//   double sum, qlen;

//   if (!dmp_en) {
//     printf("only use mpu_read_fifo in dmp mode");
//     return false;
//   }

//   // if the fifo packet_len variable not set up yet, this function must
//   // have been called prematurely
//   if (packet_len != FIFO_LEN_QUAT_ACCEL_GYRO_TAP &&
//       packet_len != FIFO_LEN_QUAT_TAP) {
//     SYS_LOG_WARN("ERROR: packet_len is set incorrectly for
//     read_dmp_fifo"); return false;
//   }

//   // make sure the i2c address is set correctly.
//   // this shouldn't take any time at all if already set
//   rc_i2c_set_device_address(config_.i2c_bus, config_.i2c_addr);
//   int is_new_dmp_data = 0;

//   // check fifo count register to make sure new data is there
//   if (rc_i2c_read_word(config_.i2c_bus, FIFO_COUNTH, &fifo_count) < 0) {
//     if (config.show_warnings) {
//       printf("fifo_count i2c error: %s\n", strerror(errno));
//     }
//     return false;
//   }
// #ifdef DEBUG
//   printf("fifo_count: %d\n", fifo_count);
// #endif

//   /***********************************************************************
//    * Check how many packets are in the fifo buffer
//    ***********************************************************************/

//   // if empty FIFO, just return, nothing else to do
//   if (fifo_count == 0) {
//     if (config.show_warnings && first_run != 1) {
//       printf("WARNING: empty fifo");
//     }
//     return false;
//   }
//   // one packet, perfect!
//   else if (fifo_count == packet_len) {
//     i = 0;  // set quaternion offset to 0
//   }
//   // if exactly 2 or 3 packets are there we just missed some (whoops)
//   // read both in and set the offset i to one packet length
//   // the last packet data will be read normally
//   else if (fifo_count == 2 * packet_len) {
//     if (config.show_warnings && first_run != 1) {
//       printf("warning: imu fifo contains two packets");
//     }
//     i = packet_len;
//   } else if (fifo_count == 3 * packet_len) {
//     if (config.show_warnings && first_run != 1) {
//       printf("warning: imu fifo contains three packets");
//     }
//     i = 2 * packet_len;
//   } else if (fifo_count == 4 * packet_len) {
//     if (config.show_warnings && first_run != 1) {
//       printf("warning: imu fifo contains four packets");
//     }
//     i = 2 * packet_len;
//   } else if (fifo_count == 5 * packet_len) {
//     if (config.show_warnings && first_run != 1) {
//       printf("warning: imu fifo contains five packets");
//     }
//     i = 2 * packet_len;
//   }
//   // finally, if we got a weird packet length, reset the fifo
//   else {
//     if (config.show_warnings && first_run != 1) {
//       printf("warning: %d bytes in FIFO, expected %d\n", fifo_count,
//              packet_len);
//     }
//     __mpu_reset_fifo();
//     return false;
//   }

//   /***********************************************************************
//    * read in the fifo
//    ******************\\\**************************************************/
//   memset(raw, 0, MAX_FIFO_BUFFER);
//   // read it in!
//   ret = rc_i2c_read_bytes(config_.i2c_bus, FIFO_R_W, fifo_count, &raw[0]);
//   if (ret < 0) {
//     // if i2c_read returned -1 there was an error, try again
//     ret = rc_i2c_read_bytes(config_.i2c_bus, FIFO_R_W, fifo_count, &raw[0]);
//   }
//   if (ret != fifo_count) {
//     if (config.show_warnings) {
//       SYS_LOG_WARN("ERROR: failed to read fifo buffer register");
//       printf("read %d bytes, expected %d\n", ret, packet_len);
//     }
//     return false;
//   }

//   // now we can read the quaternion which is always first
//   // parse the quaternion data from the buffer
//   quat[0] = ((int32_t)raw[i + 0] << 24) | ((int32_t)raw[i + 1] << 16) |
//             ((int32_t)raw[i + 2] << 8) | raw[i + 3];
//   quat[1] = ((int32_t)raw[i + 4] << 24) | ((int32_t)raw[i + 5] << 16) |
//             ((int32_t)raw[i + 6] << 8) | raw[i + 7];
//   quat[2] = ((int32_t)raw[i + 8] << 24) | ((int32_t)raw[i + 9] << 16) |
//             ((int32_t)raw[i + 10] << 8) | raw[i + 11];
//   quat[3] = ((int32_t)raw[i + 12] << 24) | ((int32_t)raw[i + 13] << 16) |
//             ((int32_t)raw[i + 14] << 8) | raw[i + 15];

//   // increment poisition in buffer after 16 bits of quaternion
//   i += 16;

//   // check the quaternion size, make sure it's correct
//   quat_q14[0] = quat[0] >> 16;
//   quat_q14[1] = quat[1] >> 16;
//   quat_q14[2] = quat[2] >> 16;
//   quat_q14[3] = quat[3] >> 16;
//   quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] +
//                 quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
//   if ((quat_mag_sq < QUAT_MAG_SQ_MIN) || (quat_mag_sq > QUAT_MAG_SQ_MAX)) {
//     if (config.show_warnings) {
//       printf("warning: Quaternion out of bounds, fifo_count: %d\n",
//       fifo_count);
//     }
//     __mpu_reset_fifo();
//     return false;
//   }

//   // do double-precision quaternion normalization since the numbers
//   // in raw format are huge
//   for (j = 0; j < 4; j++)
//     q_tmp[j] = (double)quat[j];
//   sum = 0.0;
//   for (j = 0; j < 4; j++)
//     sum += q_tmp[j] * q_tmp[j];
//   qlen = sqrt(sum);
//   for (j = 0; j < 4; j++)
//     q_tmp[j] /= qlen;
//   // make floating point and put in output
//   for (j = 0; j < 4; j++)
//     data->dmp_quat[j] = (double)q_tmp[j];

//   // fill in tait-bryan angles to the data struct
//   rc_quaternion_to_tb_array(data->dmp_quat, data->dmp_TaitBryan);
//   is_new_dmp_data = 1;

//   if (packet_len == FIFO_LEN_QUAT_ACCEL_GYRO_TAP) {
//     // Read Accel values and load into imu_data struct
//     // Turn the MSB and LSB into a signed 16-bit value
//     data->raw_accel[0] = (int16_t)(((uint16_t)raw[i + 0] << 8) | raw[i + 1]);
//     data->raw_accel[1] = (int16_t)(((uint16_t)raw[i + 2] << 8) | raw[i + 3]);
//     data->raw_accel[2] = (int16_t)(((uint16_t)raw[i + 4] << 8) | raw[i + 5]);
//     i += 6;
//     // Fill in real unit values
//     data->accel[0] = data->raw_accel[0] * data->accel_to_ms2 /
//     accel_lengths[0]; data->accel[1] = data->raw_accel[1] *
//     data->accel_to_ms2 / accel_lengths[1]; data->accel[2] =
//     data->raw_accel[2] * data->accel_to_ms2 / accel_lengths[2];

//     // Read gyro values and load into imu_data struct
//     // Turn the MSB and LSB into a signed 16-bit value
//     data->raw_gyro[0] = (int16_t)(((int16_t)raw[0 + i] << 8) | raw[1 + i]);
//     data->raw_gyro[1] = (int16_t)(((int16_t)raw[2 + i] << 8) | raw[3 + i]);
//     data->raw_gyro[2] = (int16_t)(((int16_t)raw[4 + i] << 8) | raw[5 + i]);
//     i += 6;

//     // Fill in real unit values
//     data->gyro[0] = data->raw_gyro[0] * data->gyro_to_degs;
//     data->gyro[1] = data->raw_gyro[1] * data->gyro_to_degs;
//     data->gyro[2] = data->raw_gyro[2] * data->gyro_to_degs;
//   }

//   // TODO read in tap data
//   unsigned char tap;
//   // android_orient = gesture[3] & 0xC0;
//   tap = 0x3F & raw[i + 3];

//   if (raw[i + 1] & INT_SRC_TAP) {
//     unsigned char direction, count;
//     direction = tap >> 3;
//     count = (tap % 8) + 1;
//     data_ptr->last_tap_direction = direction;
//     data_ptr->last_tap_count = count;
//     data_ptr->tap_detected = 1;
//   } else
//     data_ptr->tap_detected = 0;

//   // run data_fusion to filter yaw with compass
//   if (is_new_dmp_data && config.enable_magnetometer) {
// #ifdef DEBUG
//     printf("running data_fusion");
// #endif
//     __data_fusion(data);
//   }

//   // if we finally got dmp data, turn off the first run flag
//   if (is_new_dmp_data)
//     first_run = 0;

//   // finally, our return value is based on the presence of DMP data only
//   // even if new magnetometer data was read, the expected timing must come
//   // from the DMP samples only
//   if (is_new_dmp_data)
//     return 0;
//   else
//     return false;
// }

// /**
//  * This fuses the magnetometer data with the quaternion straight from the DMP
//  to
//  * correct the yaw heading to a compass heading. Much thanks to Pansenti for
//  * open sourcing this routine. In addition to the Pansenti implementation I
//  also
//  * correct the magnetometer data for DMP orientation, initialize yaw with the
//  * magnetometer to prevent initial rise time, and correct the
//  yaw_mixing_factor
//  * with the sample rate so the filter rise time remains constant with
//  different
//  * sample rates.
//  *
//  * @param      data  The data pointer
//  *
//  * @return     0 on success, -1 on failure
//  */
// int __data_fusion(rc_mpu_data_t* data) {
//   double tilt_tb[3], tilt_q[4], mag_vec[3];
//   static double newMagYaw = 0;
//   static double newDMPYaw = 0;
//   double lastDMPYaw, lastMagYaw, newYaw;
//   static int dmp_spin_counter = 0;
//   static int mag_spin_counter = 0;
//   static int first_run = 1;  // set to 0 after first call to this function

//   // start by filling in the roll/pitch components of the fused euler
//   // angles from the DMP generated angles. Ignore yaw for now, we have to
//   // filter that later.
//   tilt_tb[0] = data->dmp_TaitBryan[TB_PITCH_X];
//   tilt_tb[1] = data->dmp_TaitBryan[TB_ROLL_Y];
//   tilt_tb[2] = 0.0;

//   // generate a quaternion rotation of just roll/pitch
//   rc_quaternion_from_tb_array(tilt_tb, tilt_q);

//   // correct for orientation and put data into
//   if (__mag_correct_orientation(mag_vec))
//     return false;

//   // tilt that vector by the roll/pitch of the IMU to align magnetic field
//   // vector such that Z points vertically
//   rc_quaternion_rotate_vector_array(mag_vec, tilt_q);
//   // from the aligned magnetic field vector, find a yaw heading
//   // check for validity and make sure the heading is positive
//   lastMagYaw = newMagYaw;  // save from last loop
//   newMagYaw = -atan2(mag_vec[1], mag_vec[0]);

//   if (isnan(newMagYaw)) {
// #ifdef WARNINGS
//     printf("newMagYaw NAN");
// #endif
//     return false;
//   }
//   data->compass_heading_raw = newMagYaw;
//   // save DMP last from time and record newDMPYaw for this time
//   lastDMPYaw = newDMPYaw;
//   newDMPYaw = data->dmp_TaitBryan[TB_YAW_Z];

//   // the outputs from atan2 and dmp are between -PI and PI.
//   // for our filters to run smoothly, we can't have them jump between -PI
//   // to PI when doing a complete spin. Therefore we check for a skip and
//   // increment or decrement the spin counter
//   if (newMagYaw - lastMagYaw < -PI)
//     mag_spin_counter++;
//   else if (newMagYaw - lastMagYaw > PI)
//     mag_spin_counter--;
//   if (newDMPYaw - lastDMPYaw < -PI)
//     dmp_spin_counter++;
//   else if (newDMPYaw - lastDMPYaw > PI)
//     dmp_spin_counter--;

//   // if this is the first run, set up filters
//   if (first_run) {
//     lastMagYaw = newMagYaw;
//     lastDMPYaw = newDMPYaw;
//     mag_spin_counter = 0;
//     dmp_spin_counter = 0;
//     // generate complementary filters
//     double dt = 1.0 / config.dmp_sample_rate;
//     rc_filter_first_order_lowpass(&low_pass, dt,
//     config.compass_time_constant); rc_filter_first_order_highpass(&high_pass,
//     dt,
//                                    config.compass_time_constant);
//     rc_filter_prefill_inputs(&low_pass, startMagYaw);
//     rc_filter_prefill_outputs(&low_pass, startMagYaw);
//     rc_filter_prefill_inputs(&high_pass, newDMPYaw);
//     rc_filter_prefill_outputs(&high_pass, 0);
//     first_run = 0;
//   }

//   // new Yaw is the sum of low and high pass complementary filters.
//   double lp =
//     rc_filter_march(&low_pass, newMagYaw + (TWO_PI * mag_spin_counter));
//   double hp =
//     rc_filter_march(&high_pass, newDMPYaw + (TWO_PI * dmp_spin_counter));
//   newYaw = lp + hp;

//   newYaw = fmod(newYaw, TWO_PI);  // remove the effect of the spins
//   if (newYaw > PI)
//     newYaw -= TWO_PI;  // bound between +- PI
//   else if (newYaw < -PI)
//     newYaw += TWO_PI;  // bound between +- PI

//   // TB angles expect a yaw between -pi to pi so slide it again and
//   // store in the user-accessible fused tb angle
//   data->compass_heading = newYaw;
//   data->fused_TaitBryan[2] = newYaw;
//   data->fused_TaitBryan[0] = data->dmp_TaitBryan[0];
//   data->fused_TaitBryan[1] = data->dmp_TaitBryan[1];

//   // Also generate a new quaternion from the filtered tb angles
//   rc_quaternion_from_tb_array(data->fused_TaitBryan, data->fused_quat);
//   return 0;
// }

bool MPU::LoadAccelCalibration() {
  std::array<int, 3> offset{0, 0, 0};
  constexpr size_t size{offset.size()};

  const std::string filename = CALIBRATION_DIR ACCEL_CAL_FILE;
  std::ifstream file;
  file.open(filename, std::ios_base::in);
  if (file.is_open()) {
    const auto offset_numbers = ReadNumbers<int, size>(file);
    const auto scale_numbers = ReadNumbers<int, size>(file);
    if (not offset_numbers.has_value() || not scale_numbers.has_value()) {
      // copy red number to offset data if size is correct
      offset = offset_numbers.value();
      accel_lengths_ = scale_numbers.value();
    } else {
      SYS_LOG_WARN(
        "Loading accel offsets, calibration file empty or malformed");
      SYS_LOG_WARN("Please run rc_calibrate_accel to recalibrate");
      SYS_LOG_WARN("using default offsets for now");
    }
  } else {
    SYS_LOG_WARN("no accelerometer calibration data found");
    SYS_LOG_WARN("Please run rc_calibrate_gyro");
  }
  file.close();

  // print debug msg
  SYS_LOG_DEBUG(ArrayToString("accel offsets", offset));
  SYS_LOG_DEBUG(ArrayToString("accel scale", accel_lengths_));

  // write adjusted factory bias
  const std::vector<uint8_t> registers{XA_OFFSET_H, YA_OFFSET_H, ZA_OFFSET_H};
  for (size_t i = 0; i < registers.size(); i++) {
    if (not AdjustAccelFactoryBias(registers[i], offset[i])) {
      SYS_LOG_WARN("failed to read factory bias");
    }
  }
  return true;
}

bool MPU::AdjustAccelFactoryBias(const uint8_t reg, const int offset) {
  // read factory bias
  const auto offset_raw = i2c_.ReadWord(reg, ByteOrder::BIG_ENDIAN_ORDER);
  if (not offset_raw) {
    SYS_LOG_WARN("failed to read factory bias");
    return false;
  }

  // convert offset in g to bias register which is 15-bits, 16G FSR
  constexpr double resolution = 0.0009765615;
  const int16_t bias =
    offset_raw.value() - static_cast<int16_t>(std::round(offset / resolution));

  // Push accel biases to hardware registers
  if (i2c_.WriteWord(reg, bias, ByteOrder::BIG_ENDIAN_ORDER)) {
    SYS_LOG_WARN("failed to write accel offsets into IMU register");
    return false;
  }
  return true;
}

bool MPU::LoadGyroCalibration() {
  std::array<int, 3> offset{0, 0, 0};
  constexpr size_t size{offset.size()};

  const std::string filename = CALIBRATION_DIR GYRO_CAL_FILE;
  std::ifstream file;
  file.open(filename, std::ios_base::in);
  if (file.is_open()) {
    const auto numbers = ReadNumbers<int, size>(file);
    if (not numbers.has_value()) {
      // copy red number to offset data if size is correct
      offset = numbers.value();
    } else {
      SYS_LOG_WARN("Loading gyro offsets, calibration file empty or malformed");
      SYS_LOG_WARN("Please run rc_calibrate_gyro to recalibrate");
      SYS_LOG_WARN("using default offsets for now");
    }
    file.close();
  } else {
    // calibration file doesn't exist yet
    SYS_LOG_WARN("no gyro calibration data found");
    SYS_LOG_WARN("Please run rc_calibrate_gyro");
  }

  // print debug msg
  SYS_LOG_DEBUG(ArrayToString("gyro offsets", offset));

  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format. also make negative since we wish to subtract out the steady
  // state offset
  std::vector<int16_t> data(3);
  for (size_t i = 0; i < 3; i++) {
    data[i] = -static_cast<int16_t>(offset[i] / 4);
  }

  // Push gyro biases to hardware registers
  if (not i2c_.WriteWords(XG_OFFSET_H, data, ByteOrder::BIG_ENDIAN_ORDER)) {
    SYS_LOG_ERROR("failed to load gyro offsets into IMU register");
    return false;
  }
  return true;
}

bool MPU::LoadMagCalibration() {
  constexpr size_t size{3};
  const std::string filename = CALIBRATION_DIR MAG_CAL_FILE;
  std::ifstream file;
  file.open(filename, std::ios_base::in);
  if (file.is_open()) {
    const auto offset_numbers = ReadNumbers<double, size>(file);
    const auto scale_numbers = ReadNumbers<double, size>(file);
    if (not offset_numbers.has_value() && not scale_numbers.has_value()) {
      // copy red number to offset data if size is correct
      mag_offsets_ = offset_numbers.value();
      mag_scales_ = scale_numbers.value();
    } else {
      SYS_LOG_WARN(
        "Loading magnetometer offsets, calibration file empty or malformed");
      SYS_LOG_WARN("Please run rc_calibrate_mag to recalibrate");
      SYS_LOG_WARN("using default offsets for now");
    }
    file.close();
  } else {
    // calibration file doesn't exist yet
    SYS_LOG_WARN("no magnetometer calibration data found");
    SYS_LOG_WARN("Please run rc_calibrate_mag");
  }

  // print debug msg
  SYS_LOG_DEBUG(ArrayToString("mag offsets", mag_offsets_));
  SYS_LOG_DEBUG(ArrayToString("mag scale", mag_scales_));
  return true;
}

/**
 * @brief Writes accelerometer scale and offsets to disk. This is basically the
 * origin and dimensions of the ellipse made during calibration.
 *
 * @param center The center of ellipsoid in g
 * @param lengths The lengths of ellipsoid in g
 * @return true if success
 * @return false otherwise
 */
bool WriteAccelCalToDisk(const std::array<int, 3>& center,
                         const std::array<int, 3>& lengths) {
  std::array<int, 6> array;
  std::copy(center.begin(), center.end(), array.begin());
  std::copy(lengths.begin(), lengths.end(), array.begin() + 3);
  return WriteCalToDisk(CALIBRATION_DIR, MAG_CAL_FILE, array);
}

/**
 * @brief Writes gyro offsets to disk.
 *
 * @param offset offset values
 * @return true if success
 * @return false otherwise
 */
bool WriteGyroCalToDisk(std::array<int, 3> offset) {
  return WriteCalToDisk(CALIBRATION_DIR, GYRO_CAL_FILE, offset);
}

/**
 * @brief Writes magnetometer scale and offsets to disk. This is basically the
 * origin and dimensions of the ellipse made during calibration.
 *
 * @param offset The offsets values
 * @param scale The scale values
 * @return true if success
 * @return false otherwise
 */
bool WriteMagCalToDisk(const std::array<double, 3>& offset,
                       const std::array<double, 3>& scale) {
  std::array<double, 6> array;
  std::copy(offset.begin(), offset.end(), array.begin());
  std::copy(scale.begin(), scale.end(), array.begin() + 3);
  return WriteCalToDisk(CALIBRATION_DIR, MAG_CAL_FILE, array);
}

// int rc_mpu_calibrate_gyro_routine(rc_mpu_config_t conf) {
//   uint8_t c, data[6];
//   int32_t gyro_sum[3] = {0, 0, 0};
//   int16_t offsets[3];
//   was_last_steady = 1;

//   // save bus and address globally for other functions to use
//   config_.i2c_bus = conf.i2c_bus;
//   config_.i2c_addr = conf.i2c_addr;

//   // make sure the bus is not currently in use by another thread
//   // do not proceed to prevent interfering with that process
//   if (rc_i2c_get_lock(conf.i2c_bus)) {
//     SYS_LOG_WARN("i2c bus claimed by another process");
//     SYS_LOG_WARN("aborting gyro calibration()");
//     return false;
//   }

//   // if it is not claimed, start the i2c bus
//   if (rc_i2c_init(conf.i2c_bus, conf.i2c_addr) == -1) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_calibrate_gyro_routine, failed to init i2c
//             bus");
//     return false;
//   }

//   // claiming the bus does no guarantee other code will not interfere
//   // with this process, but best to claim it so other code can check
//   // like we did above
//   rc_i2c_lock_bus(conf.i2c_bus);

//   // reset device, reset all registers
//   if (__reset_mpu() == -1) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_calibrate_gyro_routine, failed to reset "
//             "MPU9250");
//     return false;
//   }

//   // set up the IMU specifically for calibration.
//   rc_i2c_write_byte(conf.i2c_bus, PWR_MGMT_1, 0x01);
//   rc_i2c_write_byte(conf.i2c_bus, PWR_MGMT_2, 0x00);
//   MicroSleep(200000);

//   // // set bias registers to 0
//   // // Push gyro biases to hardware registers
//   // uint8_t zeros[] = {0,0,0,0,0,0};
//   // if(rc_i2c_write_bytes(conf.i2c_bus, XG_OFFSET_H, 6, zeros)){
//   // fprintf(stderr,"ERROR: failed to load gyro offsets into IMU
//   register");
//   // return false;
//   // }

//   rc_i2c_write_byte(conf.i2c_bus, INT_ENABLE, 0x00);  // Disable all
//   interrupts rc_i2c_write_byte(conf.i2c_bus, FIFO_EN, 0x00);     // Disable
//   FIFO rc_i2c_write_byte(conf.i2c_bus, PWR_MGMT_1,
//                     0x00);  // Turn on internal clock source
//   rc_i2c_write_byte(conf.i2c_bus, I2C_MST_CTRL, 0x00);  // Disable I2C master
//   rc_i2c_write_byte(conf.i2c_bus, USER_CTRL,
//                     0x00);  // Disable FIFO and I2C master
//   rc_i2c_write_byte(conf.i2c_bus, USER_CTRL, 0x0C);  // Reset FIFO and DMP
//   MicroSleep(15000);

//   // Configure MPU9250 gyro and accelerometer for bias calculation
//   rc_i2c_write_byte(conf.i2c_bus, CONFIG,
//                     0x01);  // Set low-pass filter to 188 Hz
//   rc_i2c_write_byte(conf.i2c_bus, SMPLRT_DIV,
//                     0x04);  // Set sample rate to 200hz
//   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
//   rc_i2c_write_byte(conf.i2c_bus, GYRO_CONFIG, 0x00);
//   // Set accelerometer full-scale to 2 g, maximum sensitivity
//   rc_i2c_write_byte(conf.i2c_bus, ACCEL_CONFIG, 0x00);

// COLLECT_DATA:

//   // Configure FIFO to capture gyro data for bias calculation
//   rc_i2c_write_byte(conf.i2c_bus, USER_CTRL, 0x40);  // Enable FIFO
//   // Enable gyro sensors for FIFO (max size 512 bytes in MPU-9250)
//   c = FIFO_GYRO_X_EN | FIFO_GYRO_Y_EN | FIFO_GYRO_Z_EN;
//   rc_i2c_write_byte(conf.i2c_bus, FIFO_EN, c);
//   // 6 bytes per sample. 200hz. wait 0.4 seconds
//   MicroSleep(400000);

//   // At end of sample accumulation, turn off FIFO sensor read
//   rc_i2c_write_byte(conf.i2c_bus, FIFO_EN, 0x00);
//   // read FIFO sample count and log number of samples
//   rc_i2c_read_bytes(conf.i2c_bus, FIFO_COUNTH, 2, &data[0]);
//   int16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
//   int samples = fifo_count / 6;

// #ifdef DEBUG
//   printf("calibration samples: %d\n", samples);
// #endif

//   int i;
//   int16_t x, y, z;
//   rc_vector_t vx = rc_vector_empty();
//   rc_vector_t vy = rc_vector_empty();
//   rc_vector_t vz = rc_vector_empty();
//   rc_vector_alloc(&vx, samples);
//   rc_vector_alloc(&vy, samples);
//   rc_vector_alloc(&vz, samples);
//   double dev_x, dev_y, dev_z;
//   gyro_sum[0] = 0;
//   gyro_sum[1] = 0;
//   gyro_sum[2] = 0;
//   for (i = 0; i < samples; i++) {
//     // read data for averaging
//     if (rc_i2c_read_bytes(conf.i2c_bus, FIFO_R_W, 6, data) < 0) {
//       SYS_LOG_WARN("ERROR: failed to read FIFO");
//       return false;
//     }
//     x = (int16_t)(((int16_t)data[0] << 8) | data[1]);
//     y = (int16_t)(((int16_t)data[2] << 8) | data[3]);
//     z = (int16_t)(((int16_t)data[4] << 8) | data[5]);
//     gyro_sum[0] += (int32_t)x;
//     gyro_sum[1] += (int32_t)y;
//     gyro_sum[2] += (int32_t)z;
//     vx.d[i] = (double)x;
//     vy.d[i] = (double)y;
//     vz.d[i] = (double)z;
//   }
//   dev_x = rc_vector_std_dev(vx);
//   dev_y = rc_vector_std_dev(vy);
//   dev_z = rc_vector_std_dev(vz);
//   rc_vector_free(&vx);
//   rc_vector_free(&vy);
//   rc_vector_free(&vz);

// #ifdef DEBUG
//   printf("gyro sums: %d %d %d\n", gyro_sum[0], gyro_sum[1], gyro_sum[2]);
//   printf("std_deviation: %6.2f %6.2f %6.2f\n", dev_x, dev_y, dev_z);
// #endif

//   // try again is standard deviation is too high
//   if (dev_x > GYRO_CAL_THRESH || dev_y > GYRO_CAL_THRESH ||
//       dev_z > GYRO_CAL_THRESH) {
//     printf("Gyro data too noisy, put me down on a solid surface!");
//     printf("trying again");
//     was_last_steady = 0;
//     goto COLLECT_DATA;
//   }
//   // this skips the first steady reading after a noisy reading
//   // to make sure IMU has settled after being picked up.
//   if (was_last_steady == 0) {
//     was_last_steady = 1;
//     goto COLLECT_DATA;
//   }
//   // average out the samples
//   offsets[0] = (int16_t)(gyro_sum[0] / (int32_t)samples);
//   offsets[1] = (int16_t)(gyro_sum[1] / (int32_t)samples);
//   offsets[2] = (int16_t)(gyro_sum[2] / (int32_t)samples);

//   // also check for values that are way out of bounds
//   if (abs(offsets[0]) > GYRO_OFFSET_THRESH ||
//       abs(offsets[1]) > GYRO_OFFSET_THRESH ||
//       abs(offsets[2]) > GYRO_OFFSET_THRESH) {
//     printf("Gyro data out of bounds, put me down on a solid surface!");
//     printf("trying again");
//     goto COLLECT_DATA;
//   }
//   // done with I2C for now
//   rc_i2c_unlock_bus(conf.i2c_bus);
// #ifdef DEBUG
//   printf("offsets: %d %d %d\n", offsets[0], offsets[1], offsets[2]);
// #endif
//   // write to disk
//   if (__write_gyro_cal_to_disk(offsets) < 0) {
//     fprintf(stderr,
//             "ERROR in rc_calibrate_gyro_routine, failed to write to disk");
//     return false;
//   }
//   return 0;
// }

// int rc_mpu_calibrate_mag_routine(rc_mpu_config_t conf) {
//   int i;
//   double new_scale[3];
//   const int samples = 200;
//   const int sample_time_us = 12000000;  // 12 seconds ()
//   const int loop_wait_us = sample_time_us / samples;
//   const int sample_rate_hz = 1000000 / loop_wait_us;

//   rc_matrix_t A = rc_matrix_empty();
//   rc_vector_t center = rc_vector_empty();
//   rc_vector_t lengths = rc_vector_empty();
//   rc_mpu_data_t imu_data;  // to collect magnetometer data
//   // wipe it with defaults to avoid problems
//   config = rc_mpu_default_config();
//   // configure with user's i2c bus info
//   config.enable_magnetometer = 1;
//   config_.i2c_bus = conf.i2c_bus;
//   config_.i2c_addr = conf.i2c_addr;

//   // make sure the bus is not currently in use by another thread
//   // do not proceed to prevent interfering with that process
//   if (rc_i2c_get_lock(config_.i2c_bus)) {
//     SYS_LOG_WARN("i2c bus claimed by another process");
//     SYS_LOG_WARN("aborting magnetometer calibration()");
//     return false;
//   }

//   // if it is not claimed, start the i2c bus
//   if (rc_i2c_init(config_.i2c_bus, config_.i2c_addr)) {
//     SYS_LOG_WARN("ERROR rc_calibrate_mag_routine failed at
//     rc_i2c_init"); return false;
//   }

//   // claiming the bus does no guarantee other code will not interfere
//   // with this process, but best to claim it so other code can check
//   // like we did above
//   rc_i2c_lock_bus(config_.i2c_bus);

//   // reset device, reset all registers
//   if (__reset_mpu() < 0) {
//     SYS_LOG_WARN("ERROR: failed to reset MPU9250");
//     return false;
//   }
//   // check the who am i register to make sure the chip is alive
//   if (__check_who_am_i()) {
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }
//   if (__init_magnetometer(1)) {
//     SYS_LOG_WARN("ERROR: failed to initialize_magnetometer");
//     rc_i2c_unlock_bus(config_.i2c_bus);
//     return false;
//   }

//   // set local calibration to initial values and prepare variables
//   mag_offsets[0] = 0.0;
//   mag_offsets[1] = 0.0;
//   mag_offsets[2] = 0.0;
//   mag_scales[0] = 1.0;
//   mag_scales[1] = 1.0;
//   mag_scales[2] = 1.0;
//   if (rc_matrix_alloc(&A, samples, 3)) {
//     fprintf(stderr,
//             "ERROR: in rc_calibrate_mag_routine, failed to alloc data "
//             "matrix");
//     return false;
//   }

//   // sample data
//   i = 0;
//   while (i < samples) {
//     if (rc_mpu_read_mag(&imu_data) < 0) {
//       SYS_LOG_WARN("ERROR: failed to read magnetometer");
//       break;
//     }
//     // make sure the data is non-zero
//     if (fabs(imu_data.mag[0]) < zero_tolerance &&
//         fabs(imu_data.mag[1]) < zero_tolerance &&
//         fabs(imu_data.mag[2]) < zero_tolerance) {
//       SYS_LOG_WARN("ERROR: retreived all zeros from magnetometer");
//       break;
//     }
//     // save data to matrix for ellipse fitting
//     A.d[i][0] = imu_data.mag[0];
//     A.d[i][1] = imu_data.mag[1];
//     A.d[i][2] = imu_data.mag[2];
//     i++;

//     // print "keep going" every 4 seconds
//     if (i % (sample_rate_hz * 4) == sample_rate_hz * 2) {
//       printf("keep spinning");
//     }
//     // print "you're doing great" every 4 seconds
//     if (i % (sample_rate_hz * 4) == 0) {
//       printf("you're doing great");
//     }

//     MicroSleep(loop_wait_us);
//   }
//   // done with I2C for now
//   rc_mpu_power_off();
//   rc_i2c_unlock_bus(config_.i2c_bus);

//   printf("\n\nOkay Stop!");
//   printf("Calculating calibration constants.....");
//   fflush(stdout);

//   // if data collection loop exited without getting enough data, warn the
//   // user and return false, otherwise keep going normally
//   if (i < samples) {
//     printf("exiting rc_calibrate_mag_routine without saving new data");
//     return false;
//   }
//   // make empty vectors for ellipsoid fitting to populate
//   if (rc_algebra_fit_ellipsoid(A, &center, &lengths) < 0) {
//     SYS_LOG_WARN("failed to fit ellipsoid to magnetometer data");
//     rc_matrix_free(&A);
//     return false;
//   }
//   // empty memory, we are done with A
//   rc_matrix_free(&A);
//   // do some sanity checks to make sure data is reasonable
//   if (fabs(center.d[0]) > 200 || fabs(center.d[1]) > 200 ||
//       fabs(center.d[2]) > 200) {
//     SYS_LOG_WARN("ERROR: center of fitted ellipsoid out of bounds");
//     rc_vector_free(&center);
//     rc_vector_free(&lengths);
//     return false;
//   }
//   if (lengths.d[0] > 200 || lengths.d[0] < 5 || lengths.d[1] > 200 ||
//       lengths.d[1] < 5 || lengths.d[2] > 200 || lengths.d[2] < 5) {
//     SYS_LOG_WARN("WARNING: length of fitted ellipsoid out of bounds");
//     fprintf(stderr,
//             "Saving suspicious calibration data anyway in case this is "
//             "intentional");
//   }
//   // all seems well, calculate scaling factors to map ellipse lengths to
//   // a sphere of radius 70uT, this scale will later be multiplied by the
//   // factory corrected data
//   new_scale[0] = 70.0 / lengths.d[0];
//   new_scale[1] = 70.0 / lengths.d[1];
//   new_scale[2] = 70.0 / lengths.d[2];
//   // print results
//   printf("");
//   printf("Offsets X: %7.3f Y: %7.3f Z: %7.3f\n", center.d[0], center.d[1],
//          center.d[2]);
//   printf("Scales  X: %7.3f Y: %7.3f Z: %7.3f\n", new_scale[0], new_scale[1],
//          new_scale[2]);
//   // write to disk
//   if (__write_mag_cal_to_disk(center.d, new_scale) < 0) {
//     rc_vector_free(&center);
//     rc_vector_free(&lengths);
//     return false;
//   }
//   rc_vector_free(&center);
//   rc_vector_free(&lengths);
//   return 0;
// }

// /**
//  * @brief      collects a second of accelerometer data
//  *
//  * @param      avg   pointer to array to write averages to
//  *
//  * @return     0 on success, -1 on error, 1 if data was just too noisy
//  */
// int __collect_accel_samples(int* avg_raw) {
//   uint8_t data[6];
//   int32_t sum[3];
//   int i, samples, fifo_count;
//   int16_t x, y, z;
//   double dev_x, dev_y, dev_z;
//   rc_vector_t vx = rc_vector_empty();
//   rc_vector_t vy = rc_vector_empty();
//   rc_vector_t vz = rc_vector_empty();

//   // Configure FIFO to capture gyro data for bias calculation
//   rc_i2c_write_byte(config_.i2c_bus, USER_CTRL, 0x40);  // Enable FIFO
//   // Enable accel sensors for FIFO (max size 512 bytes in MPU-9250)
//   rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, FIFO_ACCEL_EN);
//   // 6 bytes per sample. 200hz. wait 0.4 seconds
//   MicroSleep(400000);

//   // At end of sample accumulation, turn off FIFO sensor read
//   rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, 0x00);
//   // read FIFO sample count and log number of samples
//   rc_i2c_read_bytes(config_.i2c_bus, FIFO_COUNTH, 2, &data[0]);
//   fifo_count = ((uint16_t)data[0] << 8) | data[1];
//   samples = fifo_count / 6;

// #ifdef DEBUG
//   printf("calibration samples: %d\n", samples);
// #endif

//   rc_vector_alloc(&vx, samples);
//   rc_vector_alloc(&vy, samples);
//   rc_vector_alloc(&vz, samples);

//   sum[0] = 0;
//   sum[1] = 0;
//   sum[2] = 0;
//   for (i = 0; i < samples; i++) {
//     // read data for averaging
//     if (rc_i2c_read_bytes(config_.i2c_bus, FIFO_R_W, 6, data) < 0) {
//       fprintf(stderr,
//               "ERROR in rc_mpu_calibrate_accel_routine, failed to read
//               FIFO");
//       return false;
//     }
//     x = (int16_t)(((int16_t)data[0] << 8) | data[1]);
//     y = (int16_t)(((int16_t)data[2] << 8) | data[3]);
//     z = (int16_t)(((int16_t)data[4] << 8) | data[5]);
//     sum[0] += (int32_t)x;
//     sum[1] += (int32_t)y;
//     sum[2] += (int32_t)z;
//     vx.d[i] = (double)x;
//     vy.d[i] = (double)y;
//     vz.d[i] = (double)z;
//   }
//   dev_x = rc_vector_std_dev(vx);
//   dev_y = rc_vector_std_dev(vy);
//   dev_z = rc_vector_std_dev(vz);
//   rc_vector_free(&vx);
//   rc_vector_free(&vy);
//   rc_vector_free(&vz);

// #ifdef DEBUG
//   printf("sums: %d %d %d\n", sum[0], sum[1], sum[2]);
//   printf("std_deviation: %6.2f %6.2f %6.2f\n", dev_x, dev_y, dev_z);
// #endif

//   // try again if standard deviation is too high
//   if (dev_x > ACCEL_CAL_THRESH || dev_y > ACCEL_CAL_THRESH ||
//       dev_z > ACCEL_CAL_THRESH) {
//     was_last_steady = 0;
//     printf("data too noisy, please hold me still");
//     return 1;
//   }
//   // this skips the first steady reading after a noisy reading
//   // to make sure IMU has settled after being picked up.
//   if (was_last_steady == 0) {
//     was_last_steady = 1;
//     return 1;
//   }
//   // average out the samples
//   avg_raw[0] = (sum[0] / (int32_t)samples);
//   avg_raw[1] = (sum[1] / (int32_t)samples);
//   avg_raw[2] = (sum[2] / (int32_t)samples);

// #ifdef DEBUG
//   printf("avg: %d %d %d\n", avg_raw[0], avg_raw[1], avg_raw[2]);
// #endif
//   return 0;
// }

// int rc_mpu_calibrate_accel_routine(rc_mpu_config_t conf) {
//   int ret, i, j;
//   int avg_raw[6][3];

//   // save bus and address globally for other functions to use
//   config_.i2c_bus = conf.i2c_bus;
//   config_.i2c_addr = conf.i2c_addr;

//   // make sure the bus is not currently in use by another thread
//   // do not proceed to prevent interfering with that process
//   if (rc_i2c_get_lock(config_.i2c_bus)) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_calibrate_accel_routine, i2c bus claimed by "
//             "another process");
//     return false;
//   }

//   // if it is not claimed, start the i2c bus
//   if (rc_i2c_init(config_.i2c_bus, config_.i2c_addr)) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_calibrate_accel_routine, failed at
//             rc_i2c_init");
//     return false;
//   }

//   // claiming the bus does no guarantee other code will not interfere
//   // with this process, but best to claim it so other code can check
//   // like we did above
//   rc_i2c_lock_bus(config_.i2c_bus);

//   // reset device, reset all registers
//   if (__reset_mpu() < 0) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_calibrate_accel_routine failed to reset "
//             "MPU9250");
//     return false;
//   }

//   // set up the IMU specifically for calibration.
//   rc_i2c_write_byte(config_.i2c_bus, PWR_MGMT_1, 0x01);
//   rc_i2c_write_byte(config_.i2c_bus, PWR_MGMT_2, 0x00);
//   MicroSleep(200000);

//   rc_i2c_write_byte(config_.i2c_bus, INT_ENABLE,
//                     0x00);                           // Disable all
//                     interrupts
//   rc_i2c_write_byte(config_.i2c_bus, FIFO_EN, 0x00);  // Disable FIFO
//   rc_i2c_write_byte(config_.i2c_bus, PWR_MGMT_1,
//                     0x00);  // Turn on internal clock source
//   rc_i2c_write_byte(config_.i2c_bus, I2C_MST_CTRL, 0x00);  // Disable I2C
//   master rc_i2c_write_byte(config_.i2c_bus, USER_CTRL,
//                     0x00);  // Disable FIFO and I2C master
//   rc_i2c_write_byte(config_.i2c_bus, USER_CTRL, 0x0C);  // Reset FIFO and DMP
//   MicroSleep(15000);

//   // Configure MPU9250 gyro and accelerometer for bias calculation
//   rc_i2c_write_byte(config_.i2c_bus, CONFIG,
//                     0x01);  // Set low-pass filter to 188 Hz
//   rc_i2c_write_byte(config_.i2c_bus, SMPLRT_DIV,
//                     0x04);  // Set sample rate to 200hz
//   rc_i2c_write_byte(config_.i2c_bus, GYRO_CONFIG, 0x00);   // set G FSR to
//   250dps rc_i2c_write_byte(config_.i2c_bus, ACCEL_CONFIG, 0x00);  // set A
//   FSR to 2G

//   // collect an orientation
//   printf("\nOrient Z pointing up and hold as still as possible");
//   printf("When ready, press any key to sample accelerometer");
//   getchar();
//   ret = 1;
//   was_last_steady = 0;
//   while (ret) {
//     ret = __collect_accel_samples(avg_raw[0]);
//     if (ret == -1)
//       return false;
//   }
//   printf("success");
//   // collect an orientation
//   printf("\nOrient Z pointing down and hold as still as possible");
//   printf("When ready, press any key to sample accelerometer");
//   getchar();
//   ret = 1;
//   was_last_steady = 0;
//   while (ret) {
//     ret = __collect_accel_samples(avg_raw[1]);
//     if (ret == -1)
//       return false;
//   }
//   printf("success");
//   // collect an orientation
//   printf("\nOrient X pointing up and hold as still as possible");
//   printf("When ready, press any key to sample accelerometer");
//   getchar();
//   ret = 1;
//   was_last_steady = 0;
//   while (ret) {
//     ret = __collect_accel_samples(avg_raw[2]);
//     if (ret == -1)
//       return false;
//   }
//   printf("success");
//   // collect an orientation
//   printf("\nOrient X pointing down and hold as still as possible");
//   printf("When ready, press any key to sample accelerometer");
//   getchar();
//   ret = 1;
//   was_last_steady = 0;
//   while (ret) {
//     ret = __collect_accel_samples(avg_raw[3]);
//     if (ret == -1)
//       return false;
//   }
//   printf("success");
//   // collect an orientation
//   printf("\nOrient Y pointing up and hold as still as possible");
//   printf("When ready, press any key to sample accelerometer");
//   getchar();
//   ret = 1;
//   was_last_steady = 0;
//   while (ret) {
//     ret = __collect_accel_samples(avg_raw[4]);
//     if (ret == -1)
//       return false;
//   }
//   printf("success");
//   // collect an orientation
//   printf("\nOrient Y pointing down and hold as still as possible");
//   printf("When ready, press any key to sample accelerometer");
//   getchar();
//   ret = 1;
//   was_last_steady = 0;
//   while (ret) {
//     ret = __collect_accel_samples(avg_raw[5]);
//     if (ret == -1)
//       return false;
//   }
//   printf("success");

//   // done with I2C for now
//   rc_mpu_power_off();
//   rc_i2c_unlock_bus(config_.i2c_bus);

//   // fit the ellipse
//   rc_matrix_t A = rc_matrix_empty();
//   rc_vector_t center = rc_vector_empty();
//   rc_vector_t lengths = rc_vector_empty();

//   if (rc_matrix_alloc(&A, 6, 3)) {
//     fprintf(stderr,
//             "ERROR: in rc_mpu_calibrate_accel_routine, failed to alloc data "
//             "matrix");
//     return false;
//   }

//   // convert to G and put in matrix
//   for (i = 0; i < 6; i++) {
//     for (j = 0; j < 3; j++) {
//       A.d[i][j] = (avg_raw[i][j] / 16384.0);
//     }
//   }

//   // make empty vectors for ellipsoid fitting to populate
//   if (rc_algebra_fit_ellipsoid(A, &center, &lengths) < 0) {
//     SYS_LOG_WARN("failed to fit ellipsoid to magnetometer data");
//     rc_matrix_free(&A);
//     return false;
//   }
//   // empty memory, we are done with A
//   rc_matrix_free(&A);
//   // do some sanity checks to make sure data is reasonable
//   for (i = 0; i < 3; i++) {
//     if (fabs(center.d[i]) > 0.3) {
//       fprintf(stderr,
//               "ERROR in rc_mpu_calibrate_accel_routine, center of fitted "
//               "ellipsoid out of bounds");
//       fprintf(stderr,
//               "most likely the unit was held in incorrect orientation during
//               " "data collection");
//       rc_vector_free(&center);
//       rc_vector_free(&lengths);
//       return false;
//     }
//     if (isnan(center.d[i]) || isnan(lengths.d[i])) {
//       fprintf(stderr,
//               "ERROR in rc_mpu_calibrate_accel_routine, data fitting produced
//               " "NaN");
//       fprintf(stderr,
//               "most likely the unit was held in incorrect orientation during
//               " "data collection");
//       rc_vector_free(&center);
//       rc_vector_free(&lengths);
//       return false;
//     }
//     if (lengths.d[i] > 1.3 || lengths.d[i] < 0.7) {
//       fprintf(stderr,
//               "ERROR in rc_mpu_calibrate_accel_routine, scale out of
//               bounds");
//       fprintf(stderr,
//               "most likely the unit was held in incorrect orientation during
//               " "data collection");
//       rc_vector_free(&center);
//       rc_vector_free(&lengths);
//       return false;
//     }
//   }

//   // print results
//   printf("");
//   printf("Offsets X: %7.3f Y: %7.3f Z: %7.3f\n", center.d[0], center.d[1],
//          center.d[2]);
//   printf("Scales  X: %7.3f Y: %7.3f Z: %7.3f\n", lengths.d[0], lengths.d[1],
//          lengths.d[2]);

//   // write to disk
//   if (__write_accel_cal_to_disk(center.d, lengths.d) == -1) {
//     fprintf(stderr,
//             "ERROR in rc_mpu_calibrate_accel_routine, failed to write to "
//             "disk");
//     return false;
//   }
//   rc_vector_free(&center);
//   rc_vector_free(&lengths);
//   return 0;
// }

int rc_mpu_is_gyro_calibrated(void) {
  // if (!access(CALIBRATION_DIR GYRO_CAL_FILE, F_OK))
  //   return 1;
  // else
  return 0;
}

int rc_mpu_is_mag_calibrated(void) {
  // if (!access(CALIBRATION_DIR MAG_CAL_FILE, F_OK))
  //   return 1;
  // else
  return 0;
}

int rc_mpu_is_accel_calibrated(void) {
  // if (!access(CALIBRATION_DIR ACCEL_CAL_FILE, F_OK))
  //   return 1;
  // else
  return 0;
}

// int64_t rc_mpu_nanos_since_last_dmp_interrupt(void) {
//   if (last_interrupt_timestamp_nanos == 0)
//     return false;
//   return rc_nanos_since_epoch() - last_interrupt_timestamp_nanos;
// }

// int64_t rc_mpu_nanos_since_last_tap(void) {
//   if (last_tap_timestamp_nanos == 0)
//     return false;
//   return rc_nanos_since_epoch() - last_tap_timestamp_nanos;
// }

// int rc_mpu_block_until_dmp_data(void) {
//   if (imu_shutdown_flag != 0) {
//     fprintf(stderr,
//             "ERROR: call to rc_mpu_block_until_dmp_data after shutting down "
//             "mpu");
//     return false;
//   }
//   if (!thread_running_flag) {
//     fprintf(stderr,
//             "ERROR: call to rc_mpu_block_until_dmp_data when DMP handler not
//             " "running");
//     return false;
//   }
//   // wait for condition signal which unlocks mutex
//   pthread_mutex_lock(&read_mutex);
//   pthread_cond_wait(&read_condition, &read_mutex);
//   pthread_mutex_unlock(&read_mutex);
//   // check if condition was broadcast due to shutdown
//   if (imu_shutdown_flag)
//     return 1;
//   // otherwise return 0 on actual button press
//   return 0;
// }

// int rc_mpu_block_until_tap(void) {
//   if (imu_shutdown_flag != 0) {
//     fprintf(stderr,
//             "ERROR: call to rc_mpu_block_until_tap after shutting down mpu");
//     return false;
//   }
//   if (!thread_running_flag) {
//     fprintf(stderr,
//             "ERROR: call to rc_mpu_block_until_tap when DMP handler not "
//             "running");
//     return false;
//   }
//   // wait for condition signal which unlocks mutex
//   pthread_mutex_lock(&tap_mutex);
//   pthread_cond_wait(&tap_condition, &tap_mutex);
//   pthread_mutex_unlock(&tap_mutex);
//   // check if condition was broadcast due to shutdown
//   if (imu_shutdown_flag)
//     return 1;
//   // otherwise return 0 on actual button press
//   return 0;
// }

// static int __mag_correct_orientation(double mag_vec[3]) {
//   // create a quaternion vector from the current magnetic field vector
//   // in IMU body coordinate frame. Since the DMP quaternion is aligned with
//   // a particular orientation, we must be careful to orient the magnetometer
//   // data to match.
//   switch (config.orient) {
//     case ORIENTATION_Z_UP:
//       mag_vec[0] = data_ptr->mag[TB_PITCH_X];
//       mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
//       mag_vec[2] = data_ptr->mag[TB_YAW_Z];
//       break;
//     case ORIENTATION_Z_DOWN:
//       mag_vec[0] = -data_ptr->mag[TB_PITCH_X];
//       mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
//       mag_vec[2] = -data_ptr->mag[TB_YAW_Z];
//       break;
//     case ORIENTATION_X_UP:
//       mag_vec[0] = -data_ptr->mag[TB_YAW_Z];
//       mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
//       mag_vec[2] = data_ptr->mag[TB_PITCH_X];
//       break;
//     case ORIENTATION_X_DOWN:
//       mag_vec[0] = data_ptr->mag[TB_YAW_Z];
//       mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
//       mag_vec[2] = -data_ptr->mag[TB_PITCH_X];
//       break;
//     case ORIENTATION_Y_UP:
//       mag_vec[0] = data_ptr->mag[TB_PITCH_X];
//       mag_vec[1] = -data_ptr->mag[TB_YAW_Z];
//       mag_vec[2] = data_ptr->mag[TB_ROLL_Y];
//       break;
//     case ORIENTATION_Y_DOWN:
//       mag_vec[0] = data_ptr->mag[TB_PITCH_X];
//       mag_vec[1] = data_ptr->mag[TB_YAW_Z];
//       mag_vec[2] = -data_ptr->mag[TB_ROLL_Y];
//       break;
//     case ORIENTATION_X_FORWARD:
//       mag_vec[0] = -data_ptr->mag[TB_ROLL_Y];
//       mag_vec[1] = data_ptr->mag[TB_PITCH_X];
//       mag_vec[2] = data_ptr->mag[TB_YAW_Z];
//       break;
//     case ORIENTATION_X_BACK:
//       mag_vec[0] = data_ptr->mag[TB_ROLL_Y];
//       mag_vec[1] = -data_ptr->mag[TB_PITCH_X];
//       mag_vec[2] = data_ptr->mag[TB_YAW_Z];
//       break;
//     default:
//       SYS_LOG_WARN("ERROR: reading magnetometer, invalid orientation");
//       return false;
//   }
//   return 0;
// }
// Phew, that was a lot of code....
