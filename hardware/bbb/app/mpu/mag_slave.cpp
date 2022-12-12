#include "mag_slave.hpp"

#include <errno.h>
#include <string.h>

#include <algorithm>  // min max
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "logger.hpp"
#include "mpu_defs.h"

bool MagSlave::Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) {
  i2c_ = i2c;
  if (not i2c_ || not i2c_->Initialized()) {
    fprintf(stderr, "ERROR: in MAG::Initialize, I2C is not initialized\n");
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

std::vector<uint8_t> MagSlave::ReadBytes(const uint8_t reg,
                                         const uint8_t count) const {
  // prepare register data to be written
  if (!Prepare(reg, true) || !EnableTransmit(count)) {
    return {};
  }

  MilliSleep(10);
  // read the response
  return i2c_->ReadBytes(EXT_SENS_DATA_00, count);
}

bool MagSlave::WriteByte(const uint8_t reg, const uint8_t data) const {
  // prepare register data to be written
  constexpr uint8_t num_bytes = 0x01;
  if (!Prepare(reg, false) || !SetData(data) || !EnableTransmit(num_bytes)) {
    return false;
  }

  MilliSleep(10);
  return true;
}

bool MagSlave::Prepare(const uint8_t reg, const bool is_read) const {
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

bool MagSlave::EnableTransmit(const uint8_t count) const {
  constexpr uint8_t enable = 0x80;
  const uint8_t number_of_bytes = static_cast<uint8_t>(enable | count);
  if (not i2c_->WriteByte(I2C_SLV0_CTRL, number_of_bytes)) {
    fprintf(stderr, "ERROR: in WriteByte, failed to enable writing: %s\n",
            strerror(errno));
    return false;
  }

  return true;
}

bool MagSlave::SetData(const uint8_t data) const {
  if (not i2c_->WriteByte(I2C_SLV0_DO, data)) {
    fprintf(stderr,
            "ERROR: in WriteByte, failed to write %0x byte to %0x: %s\n", data,
            I2C_SLV0_DO, strerror(errno));
    return false;
  }
  return true;
}

// std::optional<std::array<int16_t, 3>> MagSlave::ReadRaw() {
//   // read the data ready bit to see if there is new data
//   // const std::optional<uint8_t> st1 = ReadByte(AK8963_ST1);
//   // if (not st1.has_value()) {
//   //   return {};
//   // }
//   // if (not(st1.value() & MAG_DATA_READY)) {
//   //   printf("no new magnetometer data ready, skipping read\n");
//   //   return {};
//   // }
//   // Read the six raw data regs into data array
//   const std::vector<uint8_t> raw_bytes = ReadBytes(AK8963_XOUT_L, 6);
//   const std::optional<uint8_t> st2 = ReadByte(AK8963_ST2);
//   if (raw_bytes.empty() || not st2.has_value()) {
//     fprintf(stderr, "ERROR: ReadRaw failed to read data register\n");
//     return {};
//   }
//   // check if the readings saturated such as because
//   // of a local field source, discard data if so
//   if (st2.value() & MAGNETOMETER_SATURATION) {
//     printf("WARNING: magnetometer saturated, discarding data\n");
//     return {};
//   }
//   const auto raw_words =
//     RegisterBytesToWords(raw_bytes, EndianByteOrder::LITTLE);
//   return CopyVectorToArray<int16_t, 3>(raw_words);
// }
