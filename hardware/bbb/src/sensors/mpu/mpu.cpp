#include "sensors/mpu/mpu.hpp"

#include <errno.h>

#include "sensors/common/utils.hpp"
// #include <math.h>
// #include <poll.h>
// #include <rc/gpio.h>
// #include <rc/math/algebra.h>
// #include <rc/math/filter.h>
// #include <rc/math/matrix.h>
// #include <rc/math/quaternion.h>
// #include <rc/math/vector.h>
// #include <rc/pthread.h>
#include <bbb/hardware_utils.hpp>
#include <bbb/time.hpp>
#include <core/utils/logger_macros.hpp>
#include <fstream>
#include <optional>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <sys/stat.h>   // for mkdir and chmod
// #include <sys/types.h>  // for mkdir and chmod
// #include <unistd.h>

#include "bbb/i2c.hpp"
#include "dmpKey.h"
#include "dmp_firmware.h"
#include "dmpmap.h"
#include "mpu_defs.h"

// Calibration File Locations
#define ACCEL_CAL_FILE "accel.cal"
#define GYRO_CAL_FILE "gyro.cal"
#define MAG_CAL_FILE "mag.cal"

// I2C bus and address definitions for Robotics Cape & BeagleBone blue
#define RC_IMU_BUS 2
#define RC_IMU_INTERRUPT_PIN_CHIP 3
#define RC_IMU_INTERRUPT_PIN_PIN 21  // gpio3.21 P9.25

// macros
// #define ARRAY_SIZE(array) sizeof(array) / sizeof(array[0])
#define min(a, b) ((a < b) ? a : b)
#define unlikely(x) __builtin_expect(!!(x), 0)
#define __unused __attribute__((unused))

// #define DEG_TO_RAD 0.0174532925199
// #define RAD_TO_DEG 57.295779513
#define PI M_PI
#define TWO_PI (2.0 * M_PI)

// there should be 28 or 35 bytes in the FIFO if the magnetometer is disabled
// or enabled.
#define FIFO_LEN_QUAT_TAP 20             // 16 for quat, 4 for tap
#define FIFO_LEN_QUAT_ACCEL_GYRO_TAP 32  // 16 quat, 6 accel, 6 gyro, 4 tap
#define MAX_FIFO_BUFFER (FIFO_LEN_QUAT_ACCEL_GYRO_TAP * 5)

// error threshold checks
#define QUAT_ERROR_THRESH (1L << 16)  // very precise threshold
#define QUAT_MAG_SQ_NORMALIZED (1L << 28)
#define QUAT_MAG_SQ_MIN (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#define GYRO_CAL_THRESH 50    // std dev below which to consider still
#define ACCEL_CAL_THRESH 100  // std dev below which to consider still
#define GYRO_OFFSET_THRESH 500

// Thread control
// static pthread_mutex_t read_mutex = PTHREAD_MUTEX_INITIALIZER;
// static pthread_cond_t read_condition = PTHREAD_COND_INITIALIZER;
// static pthread_mutex_t tap_mutex = PTHREAD_MUTEX_INITIALIZER;
// static pthread_cond_t tap_condition = PTHREAD_COND_INITIALIZER;

/**
 *	Local variables
 **/
static rc_mpu_config_t config;
static int bypass_en;
static int dmp_en = 0;
// static int packet_len;
// static pthread_t imu_interrupt_thread;
// static int thread_running_flag;
// static void (*dmp_callback_func)() = NULL;
// static void (*tap_callback_func)(int dir, int cnt) = NULL;
static double mag_factory_adjust[3];
static int mag_offsets[3] = {0, 0, 0};
static int mag_scales[3] = {1, 1, 1};
static double accel_lengths[3] = {1.0, 1.0, 1.0};
// static int last_read_successful;
// static uint64_t last_interrupt_timestamp_nanos;
// static uint64_t last_tap_timestamp_nanos;
// static rc_mpu_data_t* data_ptr;
static int imu_shutdown_flag = 0;
// static rc_filter_t low_pass, high_pass;  // for magnetometer Yaw filtering
// static int was_last_steady = 0;
// static double startMagYaw = 0.0;
static I2CManager i2c_manager;
static std::shared_ptr<I2C> i2c{nullptr};

/**
 * functions for internal use only
 **/
static int __reset_mpu(void);
static int __check_who_am_i(void);
static bool __set_gyro_fsr(rc_mpu_gyro_fsr_t fsr, rc_mpu_data_t* data);
static bool __set_accel_fsr(rc_mpu_accel_fsr_t, rc_mpu_data_t* data);
static bool __set_gyro_dlpf(rc_mpu_gyro_dlpf_t dlpf);
static bool __set_accel_dlpf(rc_mpu_accel_dlpf_t dlpf);
static int __init_magnetometer(int cal_mode);
static int __power_off_magnetometer(void);
static int __mpu_set_bypass(unsigned char bypass_on);
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
static bool __load_gyro_calibration(void);
static bool __load_mag_calibration(void);
static bool __load_accel_calibration(void);
// static int __write_gyro_cal_to_disk(int16_t offsets[3]);
// static int __write_mag_cal_to_disk(double offsets[3], double scale[3]);
// static int __write_accel_cal_to_disk(double* center, double* lengths);
// static void* __dmp_interrupt_handler(void* ptr);
// static int __read_dmp_fifo(rc_mpu_data_t* data);
// static int __data_fusion(rc_mpu_data_t* data);
// static int __mag_correct_orientation(double mag_vec[3]);

rc_mpu_config_t rc_mpu_default_config(void) {
  rc_mpu_config_t conf;

  // connectivity
  conf.gpio_interrupt_pin_chip = RC_IMU_INTERRUPT_PIN_CHIP;
  conf.gpio_interrupt_pin = RC_IMU_INTERRUPT_PIN_PIN;
  conf.show_warnings = 0;

  // general stuff
  conf.accel_fsr = ACCEL_FSR_8G;
  conf.gyro_fsr = GYRO_FSR_2000DPS;
  conf.accel_dlpf = ACCEL_DLPF_184;
  conf.gyro_dlpf = GYRO_DLPF_184;
  conf.enable_magnetometer = 0;

  // DMP stuff
  conf.dmp_sample_rate = 100;
  conf.dmp_fetch_accel_gyro = 0;
  conf.dmp_auto_calibrate_gyro = 0;
  conf.orient = ORIENTATION_Z_UP;
  conf.compass_time_constant = 20.0;
  conf.dmp_interrupt_sched_policy = SCHED_OTHER;
  conf.dmp_interrupt_priority = 0;
  conf.read_mag_after_callback = 1;
  conf.mag_sample_rate_div = 4;
  conf.tap_threshold = 210;

  return conf;
}

int rc_mpu_set_config_to_default(rc_mpu_config_t* conf) {
  *conf = rc_mpu_default_config();
  return 0;
}

int rc_mpu_initialize(rc_mpu_data_t* data, rc_mpu_config_t conf) {
  // update local copy of config struct with new values
  config = conf;

  // // make sure the bus is not currently in use by another thread
  // // do not proceed to prevent interfering with that process
  // if (rc_i2c_get_lock(config.i2c_bus)) {
  //   printf("i2c bus claimed by another process");
  //   printf("Continuing with rc_mpu_initialize() anyway.");
  // }

  i2c = i2c_manager.CreateI2C(RC_IMU_BUS, RC_MPU_DEFAULT_I2C_ADDR);
  // if it is not claimed, start the i2c bus
  if (not i2c) {
    SYS_LOG_ERROR("failed to initialize i2c bus");
    return -1;
  }

  // claiming the bus does no guarantee other code will not interfere
  // with this process, but best to claim it so other code can check
  // like we did above
  // rc_i2c_lock_bus(config.i2c_bus);

  // restart the device so we start with clean registers
  if (__reset_mpu() < 0) {
    SYS_LOG_ERROR("ERROR: failed to reset_mpu9250");
    return -1;
  }

  if (__check_who_am_i()) {
    return -1;
  }

  // load in gyro calibration offsets from disk
  if (not __load_gyro_calibration()) {
    SYS_LOG_ERROR("failed to load gyro calibration offsets");
    return -1;
  }
  if (not __load_accel_calibration()) {
    SYS_LOG_ERROR("failed to load accel calibration offsets");
    return -1;
  }

  // Set sample rate = 1000/(1 + SMPLRT_DIV)
  // here we use a divider of 0 for 1khz sample
  if (not i2c->WriteByte(SMPLRT_DIV, 0x00)) {
    SYS_LOG_ERROR("I2C bus write error");
    return -1;
  }

  // set full scale ranges and filter constants
  if (not __set_gyro_fsr(conf.gyro_fsr, data)) {
    SYS_LOG_ERROR("failed to set gyro fsr");
    return -1;
  }
  if (not __set_accel_fsr(conf.accel_fsr, data)) {
    SYS_LOG_ERROR("failed to set accel fsr");
    return -1;
  }
  if (not __set_gyro_dlpf(conf.gyro_dlpf)) {
    SYS_LOG_ERROR("failed to set gyro dlpf");
    return -1;
  }
  if (not __set_accel_dlpf(conf.accel_dlpf)) {
    SYS_LOG_ERROR("failed to set accel_dlpf");
    return -1;
  }

  // initialize the magnetometer too if requested in config
  if (conf.enable_magnetometer) {
    // start magnetometer NOT in cal mode (0)
    if (__init_magnetometer(1)) {
      SYS_LOG_ERROR("failed to initialize magnetometer");
      return -1;
    }
  } else
    __power_off_magnetometer();

  return 0;
}

int rc_mpu_read_accel(rc_mpu_data_t* data) {
  // set the device address
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);

  // Read the six raw data registers into data array
  size_t count{6};
  std::vector<uint8_t> raw = i2c->ReadBytes(ACCEL_XOUT_H, count);
  if (raw.size() < count) {
    return -1;
  }

  for (size_t i = 0; i < 3; i++) {
    // Turn the MSB and LSB into a signed 16-bit value
    const size_t idx = i * 2;
    data->raw_accel[i] =
      sensors::common::utils::ToWord({raw[idx], raw[idx + 1]});
    // Fill in real unit values
    data->accel[i] = data->raw_accel[i] * data->accel_to_ms2 / accel_lengths[i];
  }
  return 0;
}

int rc_mpu_read_gyro(rc_mpu_data_t* data) {
  // set the device address
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);

  // Read the six raw data registers into data array
  size_t count{6};
  std::vector<uint8_t> raw = i2c->ReadBytes(GYRO_XOUT_H, count);
  if (raw.size() < count) {
    return -1;
  }

  for (size_t i = 0; i < 3; i++) {
    // Turn the MSB and LSB into a signed 16-bit value
    const size_t idx = i * 2;
    data->raw_gyro[i] =
      sensors::common::utils::ToWord({raw[idx], raw[idx + 1]});
    // Fill in real unit values
    data->gyro[i] = data->raw_gyro[i] * data->gyro_to_degs;
  }
  return 0;
}

int rc_mpu_read_mag(rc_mpu_data_t* data) {
  if (!config.enable_magnetometer) {
    fprintf(stderr,
            "ERROR: can't read magnetometer unless it is enabled in "
            "");
    fprintf(stderr,
            "rc_mpu_config_t struct before calling "
            "rc_mpu_initialize");
    return -1;
  }
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  // MPU9250 was put into passthrough mode
  i2c->SetDeviceAddress(AK8963_ADDR);
  // don't worry about checking data ready bit, not worth thet time
  // read the data ready bit to see if there is new data
  uint8_t st1 = i2c->ReadByte(AK8963_ST1);

#ifdef DEBUG
  printf("st1: %d", st1);
#endif
  if (!(st1 & MAG_DATA_READY)) {
    if (config.show_warnings) {
      printf("no new magnetometer data ready, skipping read");
    }
    return 0;
  }
  const size_t count = 7;
  std::vector<uint8_t> raw = i2c->ReadBytes(AK8963_XOUT_L, count);

  // Read the six raw data regs into data array
  if (raw.size() != 7) {
    SYS_LOG_ERROR("ERROR: rc_mpu_read_mag failed to read data register");
    return -1;
  }
  // check if the readings saturated such as because
  // of a local field source, discard data if so
  if (raw[6] & MAGNETOMETER_SATURATION) {
    if (config.show_warnings) {
      SYS_LOG_WARN("WARNING: magnetometer saturated, discarding data");
    }
    return -1;
  }
  // Turn the MSB and LSB into a signed 16-bit value
  // Data stored as little Endian
  int16_t adc[3];
  for (size_t i = 0; i < 3; i++) {
    const size_t idx = i * 2;
    adc[i] = sensors::common::utils::ToWord({raw[idx + 1], raw[idx]});
  }
#ifdef DEBUG
  printf("raw mag:%d %d %d\n", adc[0], adc[1], adc[2]);
#endif

  // multiply by the sensitivity adjustment and convert to units of uT micro
  // Teslas. Also correct the coordinate system as someone in invensense
  // thought it would be bright idea to have the magnetometer coordinate
  // system aligned differently than the accelerometer and gyro.... -__-
  double factory_cal_data[3];
  factory_cal_data[0] = adc[1] * mag_factory_adjust[1] * MAG_RAW_TO_uT;
  factory_cal_data[1] = adc[0] * mag_factory_adjust[0] * MAG_RAW_TO_uT;
  factory_cal_data[2] = -adc[2] * mag_factory_adjust[2] * MAG_RAW_TO_uT;

  // now apply out own calibration,
  data->mag[0] = (factory_cal_data[0] - mag_offsets[0]) * mag_scales[0];
  data->mag[1] = (factory_cal_data[1] - mag_offsets[1]) * mag_scales[1];
  data->mag[2] = (factory_cal_data[2] - mag_offsets[2]) * mag_scales[2];

  return 0;
}

int rc_mpu_read_temp(rc_mpu_data_t* data) {
  // set the device address
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);

  // Read the six raw data registers into data array
  // size_t count{6};
  int16_t adc = i2c->ReadWord(TEMP_OUT_H);

  // convert to real units
  data->temp = 21.0 + adc / TEMP_SENSITIVITY;
  return 0;
}

int __reset_mpu(void) {
  // disable the interrupt to prevent it from doing things while we reset
  imu_shutdown_flag = 1;

  // set the device address
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);

  // write the reset bit
  if (not i2c->WriteByte(PWR_MGMT_1, H_RESET)) {
    // wait and try again
    rc_usleep(10000);
    if (not i2c->WriteByte(PWR_MGMT_1, H_RESET)) {
      SYS_LOG_ERROR("ERROR resetting MPU, I2C write to reset bit failed");
      return -1;
    }
  }
  rc_usleep(10000);
  return 0;
}

int __check_who_am_i(void) {
  // check the who am i register to make sure the chip is alive
  const uint8_t c = i2c->ReadByte(WHO_AM_I_MPU9250);

  // check which chip we are looking at
  // 0x71 for mpu9250, ox73 or 0x75 for mpu9255, or 0x68 for mpu9150
  // 0x70 for mpu6500,  0x68 or 0x69 for mpu6050
  if (c != 0x68 && c != 0x69 && c != 0x70 && c != 0x71 && c != 0x73 &&
      c != 75) {
    SYS_LOG_ERROR(
      "invalid who_am_i register expected 0x68 or 0x69 for mpu6050/9150, 0x70 "
      "for mpu6500, 0x71 for mpu9250, 0x75 for mpu9255,");
    return -1;
  }
  return 0;
}

bool __set_accel_fsr(rc_mpu_accel_fsr_t fsr, rc_mpu_data_t* data) {
  uint8_t c;
  switch (fsr) {
    case ACCEL_FSR_2G:
      c = ACCEL_FSR_CFG_2G;
      data->accel_to_ms2 = 9.80665 * 2.0 / 32768.0;
      break;
    case ACCEL_FSR_4G:
      c = ACCEL_FSR_CFG_4G;
      data->accel_to_ms2 = 9.80665 * 4.0 / 32768.0;
      break;
    case ACCEL_FSR_8G:
      c = ACCEL_FSR_CFG_8G;
      data->accel_to_ms2 = 9.80665 * 8.0 / 32768.0;
      break;
    case ACCEL_FSR_16G:
      c = ACCEL_FSR_CFG_16G;
      data->accel_to_ms2 = 9.80665 * 16.0 / 32768.0;
      break;
    default:
      SYS_LOG_ERROR("invalid accel fsr");
      return -1;
  }
  return i2c->WriteByte(ACCEL_CONFIG, c);
}

bool __set_gyro_fsr(rc_mpu_gyro_fsr_t fsr, rc_mpu_data_t* data) {
  uint8_t c;
  switch (fsr) {
    case GYRO_FSR_250DPS:
      c = GYRO_FSR_CFG_250 | FCHOICE_B_DLPF_EN;
      data->gyro_to_degs = 250.0 / 32768.0;
      break;
    case GYRO_FSR_500DPS:
      c = GYRO_FSR_CFG_500 | FCHOICE_B_DLPF_EN;
      data->gyro_to_degs = 500.0 / 32768.0;
      break;
    case GYRO_FSR_1000DPS:
      c = GYRO_FSR_CFG_1000 | FCHOICE_B_DLPF_EN;
      data->gyro_to_degs = 1000.0 / 32768.0;
      break;
    case GYRO_FSR_2000DPS:
      c = GYRO_FSR_CFG_2000 | FCHOICE_B_DLPF_EN;
      data->gyro_to_degs = 2000.0 / 32768.0;
      break;
    default:
      SYS_LOG_ERROR("invalid gyro fsr");
      return -1;
  }
  return i2c->WriteByte(GYRO_CONFIG, c);
}

bool __set_accel_dlpf(rc_mpu_accel_dlpf_t dlpf) {
  uint8_t c = ACCEL_FCHOICE_1KHZ | BIT_FIFO_SIZE_1024;
  switch (dlpf) {
    case ACCEL_DLPF_OFF:
      c = ACCEL_FCHOICE_4KHZ | BIT_FIFO_SIZE_1024;
      break;
    case ACCEL_DLPF_460:
      c |= 0;
      break;
    case ACCEL_DLPF_184:
      c |= 1;
      break;
    case ACCEL_DLPF_92:
      c |= 2;
      break;
    case ACCEL_DLPF_41:
      c |= 3;
      break;
    case ACCEL_DLPF_20:
      c |= 4;
      break;
    case ACCEL_DLPF_10:
      c |= 5;
      break;
    case ACCEL_DLPF_5:
      c |= 6;
      break;
    default:
      SYS_LOG_ERROR("invalid config.accel_dlpf");
      return -1;
  }
  return i2c->WriteByte(ACCEL_CONFIG_2, c);
}

bool __set_gyro_dlpf(rc_mpu_gyro_dlpf_t dlpf) {
  uint8_t c = FIFO_MODE_REPLACE_OLD;
  switch (dlpf) {
    case GYRO_DLPF_OFF:
      c |= 7;  // not really off, but 3600Hz bandwidth
      break;
    case GYRO_DLPF_250:
      c |= 0;
      break;
    case GYRO_DLPF_184:
      c |= 1;
      break;
    case GYRO_DLPF_92:
      c |= 2;
      break;
    case GYRO_DLPF_41:
      c |= 3;
      break;
    case GYRO_DLPF_20:
      c |= 4;
      break;
    case GYRO_DLPF_10:
      c |= 5;
      break;
    case GYRO_DLPF_5:
      c |= 6;
      break;
    default:
      SYS_LOG_ERROR("invalid gyro_dlpf");
      return -1;
  }
  return i2c->WriteByte(CONFIG, c);
}

int __init_magnetometer(int cal_mode) {
  // Enable i2c bypass to allow talking to magnetometer
  if (__mpu_set_bypass(1)) {
    SYS_LOG_ERROR("failed to set mpu9250 into bypass i2c mode");
    return -1;
  }
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  i2c->SetDeviceAddress(AK8963_ADDR);

  // Power down magnetometer
  if (not i2c->WriteByte(AK8963_CNTL, MAG_POWER_DN)) {
    SYS_LOG_ERROR("failed to write to AK8963_CNTL register to power down");
    return -1;
  }
  rc_usleep(1000);
  // Enter Fuse ROM access mode
  if (not i2c->WriteByte(AK8963_CNTL, MAG_FUSE_ROM)) {
    SYS_LOG_ERROR("failed to write to AK8963_CNTL register");
    return -1;
  }
  rc_usleep(1000);
  // Read the xyz sensitivity adjustment values
  const size_t count = 3;
  const std::vector<uint8_t> calibration_data =
    i2c->ReadBytes(AK8963_ASAX, count);
  if (calibration_data.size() != 3) {
    SYS_LOG_ERROR("failed to read magnetometer adjustment register");
    i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);
    //__mpu_set_bypass(0);
    return -1;
  }
  // Return sensitivity adjustment values
  for (size_t i = 0; i < count; i++) {
    mag_factory_adjust[i] = (calibration_data[i] - 128) / 256.0 + 1.0;
  }
  // Power down magnetometer again
  if (not i2c->WriteByte(AK8963_CNTL, MAG_POWER_DN)) {
    SYS_LOG_ERROR(" failed to write to AK8963_CNTL register to power on");
    return -1;
  }
  rc_usleep(100);
  // Configure the magnetometer for 16 bit resolution
  // and continuous sampling mode 2 (100hz)
  const uint8_t c = MSCALE_16 | MAG_CONT_MES_2;
  if (not i2c->WriteByte(AK8963_CNTL, c)) {
    SYS_LOG_ERROR(
      "failed to write to AK8963_CNTL register to set sampling mode");
    return -1;
  }
  rc_usleep(100);
  // go back to configuring the IMU, leave bypass on
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);
  // load in magnetometer calibration
  if (!cal_mode) {
    __load_mag_calibration();
  }
  return 0;
}

int __power_off_magnetometer(void) {
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);
  // Enable i2c bypass to allow talking to magnetometer
  if (__mpu_set_bypass(1)) {
    SYS_LOG_ERROR("failed to set mpu9250 into bypass i2c mode");
    return -1;
  }
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  i2c->SetDeviceAddress(AK8963_ADDR);
  // Power down magnetometer
  if (not i2c->WriteByte(AK8963_CNTL, MAG_POWER_DN)) {
    SYS_LOG_ERROR("failed to write to magnetometer");
    return -1;
  }
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);
  return 0;
}

/**
 * int __mpu_set_bypass(unsigned char bypass_on)
 *
 * configures the USER_CTRL and INT_PIN_CFG registers to turn on and off the
 * i2c bypass mode for talking to the magnetometer. In random read mode this
 * is used to turn on the bypass and left as is. In DMP mode bypass is turned
 * off after configuration and the MPU fetches magnetometer data automatically.
 * USER_CTRL - based on global variable dsp_en
 * INT_PIN_CFG based on requested bypass state
 **/
int __mpu_set_bypass(uint8_t bypass_on) {
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);
  // set up USER_CTRL first
  // DONT USE FIFO_EN_BIT in DMP mode, or the MPU will generate lots of
  // unwanted interrupts
  uint8_t tmp = 0;
  if (dmp_en) {
    tmp |= FIFO_EN_BIT;  // enable fifo for dsp mode
  }
  if (!bypass_on) {
    tmp |= I2C_MST_EN;  // i2c master mode when not in bypass
  }
  if (not i2c->WriteByte(USER_CTRL, tmp)) {
    SYS_LOG_ERROR("failed to write USER_CTRL register");
    return -1;
  }
  rc_usleep(3000);
  // INT_PIN_CFG settings
  tmp = LATCH_INT_EN | INT_ANYRD_CLEAR | ACTL_ACTIVE_LOW;  // latching
  // tmp =  ACTL_ACTIVE_LOW;	// non-latching
  if (bypass_on)
    tmp |= BYPASS_EN;
  if (not i2c->WriteByte(INT_PIN_CFG, tmp)) {
    SYS_LOG_ERROR("failed to write INT_PIN_CFG register");
    return -1;
  }
  if (bypass_on) {
    bypass_en = 1;
  } else {
    bypass_en = 0;
  }
  return 0;
}

int rc_mpu_power_off(void) {
  imu_shutdown_flag = 1;
  // // wait for the interrupt thread to exit if it hasn't already
  // // allow up to 1 second for thread cleanup
  // if (thread_running_flag) {
  //   if (rc_pthread_timed_join(imu_interrupt_thread, NULL, 1.0) == 1) {
  //     SYS_LOG_ERROR("WARNING: mpu interrupt thread exit timeout");
  //   }
  //   // cleanup mutexes
  //   pthread_cond_destroy(&read_condition);
  //   pthread_mutex_destroy(&read_mutex);
  //   pthread_cond_destroy(&tap_condition);
  //   pthread_mutex_destroy(&tap_mutex);
  // }
  // shutdown magnetometer first if on since that requires
  // the imu to the on for bypass to work
  if (config.enable_magnetometer)
    __power_off_magnetometer();
  // set the device address to write the shutdown register
  i2c->SetDeviceAddress(RC_MPU_DEFAULT_I2C_ADDR);
  // write the reset bit
  if (not i2c->WriteByte(PWR_MGMT_1, H_RESET)) {
    // wait and try again
    rc_usleep(1000);
    if (not i2c->WriteByte(PWR_MGMT_1, H_RESET)) {
      SYS_LOG_ERROR("I2C write to MPU9250 Failed");
      return -1;
    }
  }
  // write the sleep bit
  if (not i2c->WriteByte(PWR_MGMT_1, MPU_SLEEP)) {
    // wait and try again
    rc_usleep(1000);
    if (not i2c->WriteByte(PWR_MGMT_1, MPU_SLEEP)) {
      SYS_LOG_ERROR("I2C write to MPU9250 Failed");
      return -1;
    }
  }

  // // if in dmp mode, also unexport the interrupt pin
  // if (dmp_en) {
  //   rc_gpio_cleanup(config.gpio_interrupt_pin_chip,
  //   config.gpio_interrupt_pin);
  // }

  return 0;
}

template <typename T>
std::optional<std::vector<T>> ReadNumbers(std::ifstream& file, size_t n) {
  if (not file.is_open()) {
    return {};
  }
  std::vector<T> numbers;
  while (not file.eof()) {
    if (numbers.size() == n) {
      return numbers;
    }
    T x;
    file >> x;
    numbers.push_back(x);
  }
  return {};
}

/**
 * Loads steady state gyro offsets from the disk and puts them in the IMU's
 * gyro offset register. If no calibration file exists then make a new one.
 *
 * @return true on success, false on failure
 */
bool __load_gyro_calibration(void) {
  int16_t offset[3] = {0, 0, 0};
  std::ifstream file;
  std::string filename = CALIBRATION_DIR GYRO_CAL_FILE;
  file.open(filename, std::ios_base::in);
  if (file.is_open()) {
    auto numbers = ReadNumbers<int16_t>(file, 3);
    if (numbers) {
      // copy red number to offset data if size is correct
      std::copy(numbers.value().begin(), numbers.value().end(), offset);
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
  std::string debug_msg = "gyro offsets: ";
  for (size_t i = 0; i < 3; i++) {
    debug_msg += std::to_string(offset[i]) + " ";
  }
  SYS_LOG_DEBUG(debug_msg);

  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format. also make negative since we wish to subtract out the steady
  // state offset
  std::vector<int16_t> data(3);
  for (size_t i = 0; i < 3; i++) {
    data[i] = -offset[i] / 4;
  }

  // Push gyro biases to hardware registers
  if (not i2c->WriteWords(XG_OFFSET_H, data)) {
    SYS_LOG_ERROR("failed to load gyro offsets into IMU register");
    return false;
  }
  return true;
}

/**
 * Loads steady state magnetometer offsets and scale from the disk into global
 * variables for correction later by read_magnetometer and FIFO read functions
 *
 * @return     true on success, false on failure
 */
bool __load_mag_calibration(void) {
  std::ifstream file;
  std::string filename = CALIBRATION_DIR MAG_CAL_FILE;
  file.open(filename, std::ios_base::in);
  if (file.is_open()) {
    auto offset_numbers = ReadNumbers<int>(file, 3);
    auto scale_numbers = ReadNumbers<int>(file, 3);
    if (offset_numbers && scale_numbers) {
      // copy red number to offset data if size is correct
      std::copy(offset_numbers.value().begin(), offset_numbers.value().end(),
                mag_offsets);
      std::copy(scale_numbers.value().begin(), scale_numbers.value().end(),
                mag_scales);
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
  std::string offset_msg = "mag offsets: ";
  std::string scale_msg = "mag scale: ";
  for (size_t i = 0; i < 3; i++) {
    offset_msg += std::to_string(mag_offsets[i]) + " ";
    scale_msg += std::to_string(mag_scales[i]) + " ";
  }
  SYS_LOG_DEBUG(offset_msg);
  SYS_LOG_DEBUG(scale_msg);

  return true;
}

/**
 * Loads steady state accel offsets from the disk and puts them in the IMU's
 * accel offset register. If no calibration file exists then make a new one.
 *
 * @return     0 on success, -1 on failure
 */
bool __load_accel_calibration(void) {
  int offset[3] = {0, 0, 0};
  int scale[3] = {1, 1, 1};
  std::ifstream file;
  std::string filename = CALIBRATION_DIR ACCEL_CAL_FILE;
  file.open(filename, std::ios_base::in);
  if (file.is_open()) {
    auto offset_numbers = ReadNumbers<int>(file, 3);
    auto scale_numbers = ReadNumbers<int>(file, 3);
    if (offset_numbers && scale_numbers) {
      // copy red number to offset data if size is correct
      std::copy(offset_numbers.value().begin(), offset_numbers.value().end(),
                offset);
      std::copy(scale_numbers.value().begin(), scale_numbers.value().end(),
                scale);
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
  std::string offset_msg = "accel offsets: ";
  std::string scale_msg = "accel scale: ";
  for (size_t i = 0; i < 3; i++) {
    offset_msg += std::to_string(offset[i]) + " ";
    scale_msg += std::to_string(scale[i]) + " ";
  }
  SYS_LOG_DEBUG(offset_msg);
  SYS_LOG_DEBUG(scale_msg);

  // save scales globally
  for (size_t i = 0; i < 3; i++) {
    accel_lengths[i] = scale[i];
  }

  // read factory bias
  std::vector<uint8_t> raw = i2c->ReadBytes(XA_OFFSET_H, 6);
  if (raw.size() != 6) {
    SYS_LOG_WARN("failed to read factory bias");
    return false;
  }

  int16_t factory[3];
  // Turn the MSB and LSB into a signed 16-bit value
  for (size_t i = 0; i < 3; i++) {
    const size_t idx = i * 2;
    const int16_t msb = static_cast<int16_t>(raw[idx]);
    const int16_t lsb = static_cast<int16_t>(raw[idx + 1]);
    factory[i] = static_cast<int16_t>((msb << 7) | (lsb >> 1));
  }

  constexpr double resolution = 0.0009765615;
  for (size_t i = 0; i < 3; i++) {
    // convert offset in g to bias register which is 15-bits, 16G FSR
    const int16_t bias =
      factory[i] - static_cast<int16_t>(std::round(offset[i] / resolution));
    // convert 16-bit bias to characters to write
    const size_t idx = i * 2;
    raw[idx] = static_cast<uint8_t>((bias >> 7) & 0xFF);
    raw[idx + 1] = static_cast<uint8_t>((bias << 1) & 0xFF);
  }

  // Push accel biases to hardware registers
  if (not i2c->WriteBytes(XA_OFFSET_H, raw)) {
    SYS_LOG_WARN("failed to write accel offsets into IMU register");
    return false;
  }

  return true;
}
