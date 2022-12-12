#include "mag.hpp"

#include <errno.h>
#include <string.h>

#include <algorithm>  // min max
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "logger.hpp"
#include "mag_direct.hpp"
#include "mag_slave.hpp"
#include "mpu_defs.h"

namespace {
std::array<double, 3> AlignMagAxis(const std::array<double, 3>& mag) {
  return {mag[0], mag[1], -mag[2]};
}
}  // namespace

bool Mag::Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) {
  config_ = conf;
  switch (config_.select) {
    case MagSelect::NONE:
      connection_ = nullptr;
      break;
    case MagSelect::DIRECT:
      connection_ = std::make_shared<MagDirect>();
      break;
    case MagSelect::SLAVE:
      connection_ = std::make_shared<MagSlave>();
      break;
  }

  if (not connection_ || not connection_->Initialize(i2c, config_)) {
    printf("failed to create connection!!\n");
    return false;
  }

  // check who am I register
  if (not Prop()) {
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

bool Mag::Prop() const {
  // check the who am i register
  const auto result = connection_->ReadByte(WHO_AM_I_AK8963);
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

bool Mag::Reset() const {
  if (not connection_->WriteByte(AK8963_CNTL2, 0x01)) {
    return false;
  }
  MilliSleep(10);
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
  const std::vector<uint8_t> raw = connection_->ReadBytes(AK8963_ASAX, 3);
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

bool Mag::PowerOff() const {
  return SetMode(MagMode::POWER_DN, MagBitScale::SCALE_16);
}

bool Mag::SetMode(const MagMode mode, const MagBitScale scale) const {
  const uint8_t ctrl = static_cast<uint8_t>(mode) | static_cast<uint8_t>(scale);
  if (not connection_->WriteByte(AK8963_CNTL, ctrl)) {
    return false;
  }

  MilliSleep(1);
  return true;
}

std::optional<std::array<int16_t, 3>> Mag::ReadRaw() {
  // read the data ready bit to see if there is new data
  const std::optional<uint8_t> st1 = connection_->ReadByte(AK8963_ST1);
  if (not st1.has_value()) {
    return {};
  }
  if (not(st1.value() & MAG_DATA_READY)) {
    printf("no new magnetometer data ready, skipping read\n");
    return {};
  }
  // Read the six raw data regs into data array
  const std::vector<uint8_t> raw_bytes =
    connection_->ReadBytes(AK8963_XOUT_L, 6);
  const std::optional<uint8_t> st2 = connection_->ReadByte(AK8963_ST2);
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

std::array<double, 3> Mag::ReadCalibrated(const std::array<int16_t, 3>& raw) {
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
