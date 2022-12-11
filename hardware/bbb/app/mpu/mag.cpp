#include "mag.hpp"

#include <errno.h>
#include <string.h>

#include <algorithm>  // min max
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "logger.hpp"
#include "mpu_defs.h"

bool Mag::Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) {
  i2c_ = i2c;
  if (not i2c_->Initialized()) {
    fprintf(stderr, "ERROR: in MAG::Initialize, I2C is not initialized\n");
    return false;
  }

  // extract the factory calibration for each magnetometer axis
  ExtractFactoryCalibration();

  // // Enable i2c bypass to allow talking to magnetometer
  // if (not SetBypass(true)) {
  //   return false;
  // }

  // // Power down
  // if (not PowerOff()) {
  //   return false;
  // }

  // check who am I register
  if (not Prop()) {
    // i2c_->Lock(false);
    printf("failed to prop magnetometer\n");
    return false;
  }

  // set mode continuous sampling mode 2 (100hz) and 16 bit resolution
  if (not SetMode(MagMode::CONT_MES_2, MagBitScale::SCALE_16)) {
    return false;
  }

  // // go back to configuring the IMU, leave bypass on
  // if (not i2c_->SetSlaveAddress(MPU_BUS)) {
  //   return false;
  // }

  // // Set up master mode, IIC 400KHz
  // constexpr uint8_t master_ctrl = BIT_I2C_READ | BIT_I2C_400KHZ;
  // if (not i2c_->WriteByte(I2C_MST_CTRL, master_ctrl)) {
  //   fprintf(stderr,
  //           "ERROR: in InitMagnetometer, failed to write to I2C_MST_CTRL\n");
  //   return false;
  // }

  // // Slave 0 reads from AKM data registers
  // constexpr uint8_t slav0_add = BIT_I2C_READ | AK8963_ADDR;
  // if (not i2c_->WriteByte(I2C_SLV0_ADDR, slav0_add)) {
  //   fprintf(stderr,
  //           "ERROR: in InitMagnetometer, failed to write to
  //           I2C_SLV0_ADDR\n");
  //   return false;
  // }

  // // Compass reads start at this register
  // constexpr uint8_t slav0_reg = AK8963_ST1;
  // if (not i2c_->WriteByte(I2C_SLV0_REG, slav0_reg)) {
  //   fprintf(stderr,
  //           "ERROR: in InitMagnetometer, failed to write to I2C_SLV0_REG\n");
  //   return false;
  // }

  // // Enable slave 0, 8-byte reads
  // constexpr uint8_t num_of_bytes_to_read = 0x08;
  // constexpr uint8_t slav0_ctrl = BIT_SLAVE_EN | num_of_bytes_to_read;
  // if (not i2c_->WriteByte(I2C_SLV0_CTRL, slav0_ctrl)) {
  //   fprintf(stderr,
  //           "ERROR: in InitMagnetometer, failed to write to
  //           I2C_SLV0_CTRL\n");
  //   return false;
  // }

  // // Trigger slave 0 actions at each sample
  // constexpr uint8_t master_delay = BIT_DELAY_ES_SHADOW | BIT_S0_DELAY_EN;
  // if (not i2c_->WriteByte(I2C_MST_DELAY_CTRL, master_delay)) {
  //   fprintf(stderr,
  //           "ERROR: in InitMagnetometer, failed to write to "
  //           "I2C_MST_DELAY_CTRL\n");
  //   return false;
  // }

  // if (not SetCompassSampleRate(config_.compass_sample_rate)) {
  //   return false;
  // }
  return true;
}

bool Mag::Prop() {
  // check the who am i register
  const auto result = ReadByte(WHO_AM_I_AK8963);
  if (not result.has_value()) {
    return false;
  }

  // check result
  const uint8_t c = result.value();
  if (result.value() != WHO_AM_I_AK8963_VALUE) {
    fprintf(stderr,
            "Error in Prop: invalid who_am_i register: expected 0x%x and got "
            "0x%x\n",
            WHO_AM_I_AK8963_VALUE, result.value());
    return false;
  }

  printf("Successfully prop Magnetometer\n");
  return true;
}

// bool Mag::SetSampleRate(const uint16_t rate) {
//   constexpr uint16_t MAX_COMPASS_SAMPLE_RATE = 100;
//   constexpr uint16_t MIN_COMPASS_SAMPLE_RATE = 1;

//   uint16_t max = std::max(MAX_COMPASS_SAMPLE_RATE, config_.sample_rate);
//   uint16_t adjusted_rate = rate;
//   if (adjusted_rate > max) {
//     adjusted_rate = max;
//     printf("WARNING SetCompassSampleRate: adjust sample rate to %d\n", max);
//   } else if (adjusted_rate <= MIN_COMPASS_SAMPLE_RATE) {
//     adjusted_rate = MIN_COMPASS_SAMPLE_RATE;
//     printf("WARNING SetCompassSampleRate: adjust sample rate to %d\n",
//            MIN_COMPASS_SAMPLE_RATE);
//   }

//   const uint8_t sample_div =
//     static_cast<uint8_t>(config_.sample_rate / adjusted_rate - 1);
//   if (not i2c_->WriteByte(I2C_SLV4_CTRL, sample_div)) {
//     fprintf(stderr,
//             "ERROR SetCompassSampleRate: failed to write sample "
//             "rate\n");
//     return false;
//   }
//   return true;
// }

bool Mag::ExtractFactoryCalibration() {
  // Power down
  if (not PowerOff()) {
    return false;
  }

  // Enter Fuse ROM access mode
  if (not SetMode(MagMode::FUSE_ROM, MagBitScale::SCALE_16)) {
    return false;
  }

  // Read the xyz sensitivity adjustment values
  const std::vector<uint8_t> raw = ReadBytes(AK8963_ASAX, 3);
  if (raw.empty()) {
    return false;
  }

  // Return sensitivity adjustment values
  for (size_t i = 0; i < raw.size(); i++) {
    factory_adjust_[i] = (raw[i] - 128) / 256.0 + 1.0;
  }
  printf("%s\n", ArrayToString(" raw factory adjust: ",
                               CopyVectorToArray<uint8_t, 3>(raw))
                   .c_str());

  printf("%s\n", ArrayToString("factory adjust: ", factory_adjust_).c_str());

  // Power down magnetometer again
  return PowerOff();
}

bool Mag::PowerOff() {
  return SetMode(MagMode::POWER_DN, MagBitScale::SCALE_16);
}

bool Mag::SetMode(const MagMode mode, const MagBitScale scale) {
  const uint8_t ctrl = static_cast<uint8_t>(mode) | static_cast<uint8_t>(scale);
  if (not WriteByte(AK8963_CNTL, ctrl)) {
    return false;
  }

  MilliSleep(1);
  return true;
}

std::vector<uint8_t> Mag::ReadBytes(const uint8_t reg, const uint8_t count) {
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  if (not i2c_->SetSlaveAddress(AK8963_ADDR)) {
    return {};
  }
  // Read from register
  const std::vector<uint8_t> raw = i2c_->ReadBytes(reg, count);
  if (raw.empty()) {
    fprintf(stderr, "ERROR: in ReadBytes, failed to read register %x\n", reg);
    return {};
  }
  return raw;
}

std::optional<uint8_t> Mag::ReadByte(const uint8_t reg) {
  const auto result = ReadBytes(reg, 1);
  if (result.empty()) {
    return {};
  }
  return result[0];
}

bool Mag::WriteByte(const uint8_t reg, const uint8_t data) {
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  if (not i2c_->SetSlaveAddress(AK8963_ADDR)) {
    return false;
  }
  // write data to register
  if (not i2c_->WriteByte(reg, data)) {
    fprintf(stderr,
            "ERROR: in WriteByte, failed to write data '0x%zx' to register "
            "'0x%zx'\n",
            data, reg);
    return false;
  }
  return true;
}

bool Mag::SetBypass(const bool bypass_on) {
  // if (not i2c_->SetSlaveAddress(MPU_BUS)) {
  //   return false;
  // }

  // read current USER_CTRL configuration
  const auto current_config = i2c_->ReadByte(USER_CTRL);
  if (not current_config.has_value()) {
    fprintf(stderr,
            "ERROR in SetBypass, failed to read USER_CTRL "
            "register\n");
    return false;
  }

  // set up USER_CTRL first
  uint8_t user_config = current_config.value();
  // DONT USE FIFO_EN_BIT in DMP mode, or the MPU will generate lots of
  // unwanted interrupts
  // user_config &= ~BIT_AUX_IF_EN;

  // if (dmp_en_) {
  //   user_config |= FIFO_EN_BIT;  // enable fifo for dsp mode
  // } else {
  //   user_config &= ~FIFO_EN_BIT;  // disable fifo for non dsp mode
  // }

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

  bypass_enabled_ = bypass_on;
  return true;
}

std::optional<std::array<int16_t, 3>> Mag::ReadRaw() {
  // read the data ready bit to see if there is new data
  const std::optional<uint8_t> st1 = ReadByte(AK8963_ST1);
  if (not st1.has_value()) {
    return {};
  }
  if (not(st1.value() & MAG_DATA_READY)) {
    printf("no new magnetometer data ready, skipping read\n");
    return {};
  }
  // Read the six raw data regs into data array
  const std::vector<uint8_t> raw_bytes = ReadBytes(AK8963_XOUT_L, 6);
  const std::optional<uint8_t> st2 = ReadByte(AK8963_ST2);
  if (raw_bytes.empty() || not st2.has_value()) {
    fprintf(stderr, "ERROR: ReadRaw failed to read data register\n");
    return {};
  }
  // check if the readings saturated such as because
  // of a local field source, discard data if so
  if (st2.value() & MAGNETOMETER_SATURATION) {
    printf("WARNING: magnetometer saturated, discarding data\n");
    return {};
  }
  const auto raw_words =
    RegisterBytesToWords(raw_bytes, EndianByteOrder::LITTLE);
  return CopyVectorToArray<int16_t, 3>(raw_words);
}

std::array<int16_t, 3> AlignMagAxis(const std::array<int16_t, 3>& mag) {
  const int16_t x = mag[1];
  const int16_t y = mag[0];
  const int16_t z = static_cast<int16_t>(-mag[2]);
  return {x, y, z};
}

std::array<double, 3> Mag::ReadCalibrated(const std::array<int16_t, 3>& raw) {
  const std::array<int16_t, 3> aligned = AlignMagAxis(raw);

  // multiply by the sensitivity adjustment and convert to units of uT micro
  // Teslas. Also correct the coordinate system as someone in invensense
  // thought it would be bright idea to have the magnetometer coordinate
  // system aligned differently than the accelerometer and gyro.... -__-
  std::array<double, 3> mag;
  for (size_t i = 0; i < mag.size(); i++) {
    const double factory_cal_data =
      aligned[i] * factory_adjust_[i] * MAG_RAW_TO_uT;
    // now apply out own calibration,
    mag[i] = (factory_cal_data - offsets_[i]) * scales_[i];
  }

  return mag;
}

MagData Mag::ReadData() {
  MagData data;

  const auto mag_raw = ReadRaw();
  if (mag_raw.has_value()) {
    data.raw = mag_raw.value();
    data.calib = ReadCalibrated(data.raw);
  } else {
    printf("read mag data failed\n");
  }

  return data;
}
