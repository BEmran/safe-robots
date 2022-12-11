#include "mpu.hpp"

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>  // min max
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "common.hpp"
#include "logger.hpp"
#include "mpu_defs.h"

constexpr uint BIT_RESOLUTION = 1 << 15;

bool SetBypass(std::shared_ptr<I2C> i2c_, const bool bypass_on) {
  // read current USER_CTRL configuration
  const auto current_config = i2c_->ReadByte(USER_CTRL);
  if (not current_config.has_value()) {
    fprintf(stderr, "ERROR in SetBypass, failed to read USER_CTRL register\n");
    return false;
  }

  // set up USER_CTRL first
  uint8_t user_config = current_config.value();
  if (bypass_on) {
    user_config &= ~I2C_MST_EN;  // disable i2c master mode when not in bypass
  } else {
    user_config |= I2C_MST_EN;  // enable i2c master mode when in bypass
  }

  if (not i2c_->WriteByte(USER_CTRL, user_config)) {
    fprintf(stderr, "ERROR in SetBypass, failed to write USER_CTRL register\n");
    return false;
  }
  MilliSleep(3);

  uint8_t int_config{0};
  if (bypass_on) {
    int_config |= BIT_BYPASS_EN;
  }
  // INT_PIN_CFG settings: active high and cleared if any read operation is
  // performed
  int_config |= ACTL_ACTIVE_HIGH | LATCH_INT_EN | INT_ANYRD_CLEAR;
  if (not i2c_->WriteByte(INT_PIN_CFG, int_config)) {
    fprintf(stderr,
            "ERROR in SetBypass, failed to write INT_PIN_CFG register\n");
    return false;
  }

  // bypass_enabled_ = bypass_on;
  return true;
}

bool MPU::Initialize(const MpuConfig& conf) {
  // update local copy of config struct with new values
  config_ = conf;

  // make sure the bus is not currently in use by another thread
  // do not proceed to prevent interfering with that process
  if (i2c_->GetLock()) {
    printf("i2c bus claimed by another process\n");
    printf("Continuing with rc_mpu_initialize() anyway.\n");
  }

  if (not i2c_->Initialize(MPU_BUS, config_.i2c_addr)) {
    fprintf(stderr, "ERROR in Initialize: failed to initialize i2c bus\n");
    return false;
  }

  // claiming the bus does no guarantee other code will not interfere
  // with this process, but best to claim it so other code can check
  // like we did above
  i2c_->Lock(true);

  // restart the device so we start with clean registers
  if (not ResetMpu()) {
    fprintf(stderr, "ERROR in Initialize: failed to reset mpu\n");
    i2c_->Lock(false);
    return false;
  }

  if (not Prop()) {
    i2c_->Lock(false);
    return false;
  }

  if (not SetSampleRate(config_.sample_rate)) {
    fprintf(stderr, "ERROR in Initialize: failed to set sample rate\n");
    i2c_->Lock(false);
    return false;
  }

  if (not SetGyroFSR(config_.gyro_fsr)) {
    fprintf(stderr, "ERROR in Initialize: failed to set gyro fsr\n");
    i2c_->Lock(false);
    return false;
  }

  if (not SetAccelFSR(config_.accel_fsr)) {
    fprintf(stderr, "ERROR in Initialize: failed to set accel fsr\n");
    i2c_->Lock(false);
    return false;
  }

  if (not SetGyroDLPF(config_.gyro_dlpf)) {
    fprintf(stderr, "ERROR in Initialize: failed to set gyro dlpf\n");
    i2c_->Lock(false);
    return false;
  }

  if (not SetAccelDLPF(config_.accel_dlpf)) {
    fprintf(stderr, "ERROR in Initialize: failed to set accel_dlpf\n");
    i2c_->Lock(false);
    return false;
  }

  if (config_.enable_magnetometer) {
    if (not SetBypass(i2c_, true)) {
      fprintf(stderr, "ERROR in Initialize: failed to SetBypass\n");
      return false;
    }
    if (not mag_->Initialize(i2c_, MagConfig())) {
      fprintf(stderr,
              "ERROR in Initialize: failed to Initialize magnetometer\n");
      return false;
    }
  }
  i2c_->Lock(false);
  return true;
}

bool MPU::Prop() {
  const std::optional<uint8_t> result = i2c_->ReadByte(WHO_AM_I_MPU9250);
  if (not result.has_value()) {
    return false;
  }
  // check result
  if (result.value() != WHO_AM_I_MPU9250_RESULT) {
    fprintf(stderr, "invalid who_am_i register: expected 0x%x and got 0x%x\n",
            WHO_AM_I_MPU9250_RESULT, result.value());
    return false;
  }
  printf("Successfully prop MPU\n");
  return true;
}

bool MPU::PowerOff() {
  // write the reset bit
  if (not i2c_->WriteByte(PWR_MGMT_1, MPU_RESET)) {
    // wait and try again
    MicroSleep(1000);
    if (not i2c_->WriteByte(PWR_MGMT_1, MPU_RESET)) {
      fprintf(stderr, "I2C write to MPU9250 Failed\n");
      return false;
    }
  }

  // write the sleep bit
  if (not i2c_->WriteByte(PWR_MGMT_1, MPU_SLEEP)) {
    // wait and try again
    MicroSleep(1000);
    if (not i2c_->WriteByte(PWR_MGMT_1, MPU_SLEEP)) {
      fprintf(stderr, "I2C write to MPU9250 Failed\n");
      return false;
    }
  }

  return true;
}

bool MPU::ResetMpu() {
  // write the reset bit
  if (not i2c_->WriteByte(PWR_MGMT_1, MPU_RESET)) {
    // wait and try again
    fprintf(stderr, "ERROR ResetMpu, failed to reset. Will try again...\n");
    MilliSleep(10);
    if (not i2c_->WriteByte(PWR_MGMT_1, MPU_RESET)) {
      fprintf(stderr, "ERROR ResetMpu, failed to reset\n");
      return false;
    }
  }
  MilliSleep(10);

  // wake up chip, enable all sensors and auto select clock source
  constexpr uint8_t WAKE_UP_INT_CLK = AUTO_CLK & !MPU_SLEEP;
  if (not i2c_->WriteByte(PWR_MGMT_1, WAKE_UP_INT_CLK)) {
    fprintf(stderr, "ERROR ResetMpu, failed to wake up and set clock\n");
    return false;
  }
  MilliSleep(100);

  return true;
}

bool MPU::SetSampleRate(const uint16_t rate) {
  constexpr uint16_t MAX_RATE = 1000;
  constexpr uint16_t MIN_RATE = 4;
  // the INTERNAL_SAMPLE_RATE should actually depend on MpuGyroDLPF value.
  // most common is 1k but could be higher
  constexpr uint16_t INTERNAL_SAMPLE_RATE = 1000;

  uint16_t adjusted_rate = rate;
  if (adjusted_rate > MAX_RATE) {
    adjusted_rate = MAX_RATE;
    printf("WARNING SetSampleRate: adjust sample rate to 1000\n");
  } else if (adjusted_rate < MIN_RATE) {
    adjusted_rate = MIN_RATE;
    printf("WARNING SetSampleRate: adjust sample rate to 4\n");
  }
  printf("SetSampleRate: sample rate will be set to %d\n", adjusted_rate);

  const uint8_t sample_div =
    static_cast<uint8_t>(INTERNAL_SAMPLE_RATE / adjusted_rate - 1);
  if (not i2c_->WriteByte(SMPLRT_DIV, sample_div)) {
    fprintf(stderr, "ERROR SetSampleRate: failed to write sample rate\n");
    return false;
  }

  return true;
}

bool MPU::SetAccelFSR(const MpuAccelFSR fsr) {
  constexpr double scale = G_TO_MS2 / BIT_RESOLUTION;
  uint8_t config;
  switch (fsr) {
    case MpuAccelFSR::ACCEL_FSR_2G:
      config = ACCEL_FSR_CFG_2G;
      accel_to_ms2_ = 2 * scale;
      break;
    case MpuAccelFSR::ACCEL_FSR_4G:
      config = ACCEL_FSR_CFG_4G;
      accel_to_ms2_ = 4 * scale;
      break;
    case MpuAccelFSR::ACCEL_FSR_8G:
      config = ACCEL_FSR_CFG_8G;
      accel_to_ms2_ = 8 * scale;
      break;
    case MpuAccelFSR::ACCEL_FSR_16G:
      config = ACCEL_FSR_CFG_16G;
      accel_to_ms2_ = 16 * scale;
      break;
    default:
      fprintf(stderr, "ERROR SetAccelFSR: invalid accel fsr option\n");
      return false;
  }
  return i2c_->WriteByte(ACCEL_CONFIG, config);
}

bool MPU::SetGyroFSR(const MpuGyroFSR fsr) {
  uint8_t config = FCHOICE_B_DLPF_EN;
  switch (fsr) {
    case MpuGyroFSR::GYRO_FSR_250DPS:
      config |= GYRO_FSR_CFG_250;
      gyro_to_degs_ = 250.0 / BIT_RESOLUTION;
      break;
    case MpuGyroFSR::GYRO_FSR_500DPS:
      config |= GYRO_FSR_CFG_500;
      gyro_to_degs_ = 500.0 / BIT_RESOLUTION;
      break;
    case MpuGyroFSR::GYRO_FSR_1000DPS:
      config |= GYRO_FSR_CFG_1000;
      gyro_to_degs_ = 1000.0 / BIT_RESOLUTION;
      break;
    case MpuGyroFSR::GYRO_FSR_2000DPS:
      config |= GYRO_FSR_CFG_2000;
      gyro_to_degs_ = 2000.0 / BIT_RESOLUTION;
      break;
    default:
      fprintf(stderr, "ERROR SetGyroFSR: invalid gyro fsr option\n");
      return -1;
  }
  return i2c_->WriteByte(GYRO_CONFIG, config);
}

bool MPU::SetAccelDLPF(const MpuAccelDLPF dlpf) {
  uint8_t config = ACCEL_FCHOICE_EN;
  switch (dlpf) {
    case MpuAccelDLPF::ACCEL_DLPF_OFF:
      config = ACCEL_FCHOICE_DISABLE;
      break;
    case MpuAccelDLPF::ACCEL_DLPF_460:
      config |= 0;
      break;
    case MpuAccelDLPF::ACCEL_DLPF_184:
      config |= 1;
      break;
    case MpuAccelDLPF::ACCEL_DLPF_92:
      config |= 2;
      break;
    case MpuAccelDLPF::ACCEL_DLPF_41:
      config |= 3;
      break;
    case MpuAccelDLPF::ACCEL_DLPF_20:
      config |= 4;
      break;
    case MpuAccelDLPF::ACCEL_DLPF_10:
      config |= 5;
      break;
    case MpuAccelDLPF::ACCEL_DLPF_5:
      config |= 6;
      break;
    default:
      fprintf(stderr, "ERROR SetAccelDLPF: invalid accel dlpf option\n");
      return -1;
  }
  return i2c_->WriteByte(ACCEL_CONFIG_2, config);
}

bool MPU::SetGyroDLPF(const MpuGyroDLPF dlpf) {
  uint8_t config = FIFO_MODE_REPLACE_OLD;
  switch (dlpf) {
    case MpuGyroDLPF::GYRO_DLPF_OFF:
      config |= 7;  // not really off, but 3600Hz bandwidth
      break;
    case MpuGyroDLPF::GYRO_DLPF_250:
      config |= 0;
      break;
    case MpuGyroDLPF::GYRO_DLPF_184:
      config |= 1;
      break;
    case MpuGyroDLPF::GYRO_DLPF_92:
      config |= 2;
      break;
    case MpuGyroDLPF::GYRO_DLPF_41:
      config |= 3;
      break;
    case MpuGyroDLPF::GYRO_DLPF_20:
      config |= 4;
      break;
    case MpuGyroDLPF::GYRO_DLPF_10:
      config |= 5;
      break;
    case MpuGyroDLPF::GYRO_DLPF_5:
      config |= 6;
      break;
    default:
      fprintf(stderr, "ERROR SetGyroDLPF: invalid gyro dlpf option\n");
      return -1;
  }
  return i2c_->WriteByte(CONFIG, config);
}

std::optional<std::array<int16_t, 3>> MPU::ReadAccelRaw() {
  // set the device address
  if (not i2c_->SetSlaveAddress(config_.i2c_addr)) {
    return {};
  }
  // Read the six raw data registers into data array
  const std::vector<int16_t> raw =
    i2c_->ReadWords(ACCEL_XOUT_H, 3, EndianByteOrder::BIG);
  if (raw.empty()) {
    return {};
  }
  return CopyVectorToArray<int16_t, 3>(raw);
}

std::array<double, 3>
MPU::ReadAccelCalibrated(const std::array<int16_t, 3>& raw) {
  // Fill in real unit values and apply calibration
  constexpr size_t size = 3;
  std::array<double, size> accel;
  for (size_t i = 0; i < accel.size(); i++) {
    accel[i] = raw[i] * accel_to_ms2_ / accel_lengths_[i];
  }
  return accel;
}

std::optional<std::array<int16_t, 3>> MPU::ReadGyroRaw() {
  // set the device address
  if (not i2c_->SetSlaveAddress(config_.i2c_addr)) {
    return {};
  }
  // Read the six raw data registers into data array
  const std::vector<int16_t> raw =
    i2c_->ReadWords(GYRO_XOUT_H, 3, EndianByteOrder::BIG);
  if (raw.empty()) {
    return {};
  }
  return CopyVectorToArray<int16_t, 3>(raw);
}

std::array<double, 3>
MPU::ReadGyroCalibrated(const std::array<int16_t, 3>& raw) {
  // Fill in real unit values
  constexpr size_t size = 3;
  std::array<double, size> gyro;
  for (size_t i = 0; i < gyro.size(); i++) {
    gyro[i] = raw[i] * gyro_to_degs_;
  }
  return gyro;
}

std::optional<double> MPU::ReadTemp() {
  // set device address
  if (not i2c_->SetSlaveAddress(config_.i2c_addr)) {
    return {};
  }
  // Read the two raw data registers
  const std::optional<int16_t> data =
    i2c_->ReadWord(TEMP_OUT_H, EndianByteOrder::BIG);
  if (not data.has_value()) {
    fprintf(stderr, "failed to read IMU temperature registers\n");
    return {};
  }
  // convert to real units
  const double temp = 21.0 + data.value() / TEMP_SENSITIVITY;
  return temp;
}

MpuData MPU::ReadData() {
  MpuData data;
  const auto accel_raw = ReadAccelRaw();
  if (accel_raw.has_value()) {
    data.accel = ReadAccelCalibrated(accel_raw.value());
  } else {
    printf("read accel data failed\n");
  }

  const auto gyro_raw = ReadGyroRaw();
  if (gyro_raw.has_value()) {
    data.gyro = ReadGyroCalibrated(gyro_raw.value());
  } else {
    printf("read gyro data failed\n");
  }

  const auto temp_raw = ReadTemp();
  if (temp_raw.has_value()) {
    data.temp = temp_raw.value();
  } else {
    printf("read imu thermometer failed\n");
  }

  if (config_.enable_magnetometer) {
    const auto mag_data = mag_->ReadData();
    data.mag = mag_data.calib;
    data.raw_mag = mag_data.raw;
  }

  return data;
}