#include "mag_slave.hpp"

#include <errno.h>
#include <string.h>

#include <algorithm>  // min max
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "logger.hpp"
#include "mpu_defs.h"

namespace {
std::array<double, 3> AlignMagAxis(const std::array<double, 3>& mag) {
  return {mag[0], mag[1], -mag[2]};
}
}  // namespace

bool MagSLave::Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) {
  i2c_ = i2c;
  if (not i2c_->Initialized()) {
    fprintf(stderr, "ERROR: in MAG::Initialize, I2C is not initialized\n");
    return false;
  }

  // check who am I register
  if (not Prop()) {
    // i2c_->Lock(false);
    printf("failed to prop magnetometer\n");
    return false;
  }

  // reset sensor's resisters
  if (not Reset()) {
    printf("failed to reset magnetometer\n");
    return false;
  }

  // extract the factory calibration for each magnetometer axis
  ExtractFactoryCalibration();

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
  // constexpr uint8_t master_MilliSleep = BIT_DELAY_ES_SHADOW |
  // BIT_S0_DELAY_EN; if (not i2c_->WriteByte(I2C_MST_DELAY_CTRL,
  // master_MilliSleep)) {
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

bool MagSLave::Prop() const {
  // check the who am i register
  const auto result = ReadByte(WHO_AM_I_AK8963);
  if (not result.has_value()) {
    return false;
  }

  // check result
  const uint8_t c = result.value();

  // // Set the I2C slave address of AK8963 and set for read.
  // i2c_->WriteByte(I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
  // // I2C slave 0 register address from where to begin data transfer
  // i2c_->WriteByte(I2C_SLV0_REG, WHO_AM_I_AK8963);
  // // Enable I2C and read 3 bytes
  // i2c_->WriteByte(I2C_SLV0_CTRL, 0x81);
  // MilliSleep(10);
  // // Read the x-, y-, and z-axis calibration values
  // auto c = i2c_->ReadBytes(EXT_SENS_DATA_00, 1)[0];

  if (c != WHO_AM_I_AK8963_VALUE) {
    fprintf(stderr,
            "Error in Prop: invalid who_am_i register: expected 0x%x and got "
            "0x%x\n",
            WHO_AM_I_AK8963_VALUE, c);
    return false;
  }

  printf("Successfully prop Magnetometer\n");
  return true;
}

bool MagSLave::Reset() const {
  if (not WriteByte(AK8963_CNTL2, 0x01)) {
    return false;
  }
  MilliSleep(10);
  return true;
}
// bool MagSLave::SetSampleRate(const uint16_t rate) {
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

bool MagSLave::ExtractFactoryCalibration() {
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

  const auto raw_array = CopyVectorToArray<uint8_t, 3>(raw);
  printf("%s\n", ArrayToString("Raw factory adjust: ", raw_array).c_str());
  printf("%s\n", ArrayToString("factory adjust: ", factory_adjust_).c_str());

  // Power down magnetometer again
  return PowerOff();
}

bool MagSLave::PowerOff() const {
  return SetMode(MagMode::POWER_DN, MagBitScale::SCALE_16);
}

bool MagSLave::SetMode(const MagMode mode, const MagBitScale scale) const {
  const uint8_t ctrl = static_cast<uint8_t>(mode) | static_cast<uint8_t>(scale);
  if (not WriteByte(AK8963_CNTL, ctrl)) {
    return false;
  }

  MilliSleep(1);
  return true;
}

std::vector<uint8_t> MagSLave::ReadBytes(const uint8_t reg,
                                         const uint8_t count) const {
  // prepare register data to be written
  if (not Prepare(reg, true)) {
    return {};
  }

  // Enable I2C and read
  const uint8_t number_of_bytes = 0x80 | count;
  if (not i2c_->WriteByte(I2C_SLV0_CTRL, number_of_bytes)) {
    fprintf(stderr, "ERROR: in ReadBytes, failed to enable reading: %s\n",
            strerror(errno));
    return {};
  }

  MilliSleep(10);
  // then read the response
  return i2c_->ReadBytes(EXT_SENS_DATA_00, count);
}

std::optional<uint8_t> MagSLave::ReadByte(const uint8_t reg) const {
  const auto result = ReadBytes(reg, 1);
  if (result.empty()) {
    return {};
  }
  return result[0];
}

bool MagSLave::WriteByte(const uint8_t reg, const uint8_t data) const {
  // prepare register data to be written
  if (not Prepare(reg, false)) {
    return false;
  }

  // data to be written
  if (not i2c_->WriteByte(I2C_SLV0_DO, data)) {
    fprintf(stderr,
            "ERROR: in WriteByte, failed to write %0x byte to %0x: %s\n", data,
            I2C_SLV0_DO, strerror(errno));
    return false;
  }

  // Enable I2C and write 1 byte
  const uint8_t number_of_bytes = 0x80 | 1;
  if (not i2c_->WriteByte(I2C_SLV0_CTRL, number_of_bytes)) {
    fprintf(stderr, "ERROR: in WriteByte, failed to enable writing: %s\n",
            strerror(errno));
    return false;
  }

  MilliSleep(10);
  return true;
}

bool MagSLave::Prepare(const uint8_t reg, const bool is_read) const {
  // if (not SanityCheck()) {
  //   return false;
  // }

  uint8_t address = AK8963_ADDR;
  if (is_read) {
    address |= 0x80;
  }
  // Set the I2C slave address of AK8963 and set for write.
  if (not i2c_->WriteByte(I2C_SLV0_ADDR, address)) {
    fprintf(stderr,
            "ERROR: in Prepare, failed Set the I2C slave address of AK8963: "
            "%s\n",
            strerror(errno));
    return false;
  }

  // I2C slave 0 register address from where to begin data transfer
  if (not i2c_->WriteByte(I2C_SLV0_REG, reg)) {
    fprintf(stderr, "ERROR: in SendByte, failed to write %0x byte to %0x: %s\n",
            reg, I2C_SLV0_REG, strerror(errno));
    return false;
  }

  return true;
}

std::optional<std::array<int16_t, 3>> MagSLave::ReadRaw() {
  // read the data ready bit to see if there is new data
  // const std::optional<uint8_t> st1 = ReadByte(AK8963_ST1);
  // if (not st1.has_value()) {
  //   return {};
  // }
  // if (not(st1.value() & MAG_DATA_READY)) {
  //   printf("no new magnetometer data ready, skipping read\n");
  //   return {};
  // }
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

std::array<double, 3>
MagSLave::ReadCalibrated(const std::array<int16_t, 3>& raw) {
  // multiply by the sensitivity adjustment and convert to units of uT micro
  // Teslas.
  std::array<double, 3> mag;
  for (size_t i = 0; i < mag.size(); i++) {
    const double factory_cal_data = raw[i] * factory_adjust_[i] * MAG_RAW_TO_uT;
    // now apply out own calibration,
    mag[i] = (factory_cal_data - offsets_[i]) * scales_[i];
  }

  // Correct the coordinate system as the magnetometer coordinate
  // system aligned differently than the accelerometer and gyro
  const std::array<double, 3> aligned = AlignMagAxis(mag);

  return aligned;
}

MagData MagSLave::ReadData() {
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
