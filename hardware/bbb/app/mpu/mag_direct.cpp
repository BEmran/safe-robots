#include "mag_direct.hpp"

#include <errno.h>
#include <string.h>

#include <cstdio>

#include "logger.hpp"
#include "mpu_defs.h"

bool MagDirect::Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) {
  i2c_ = i2c;
  if (not i2c_ || not i2c_->Initialized()) {
    fprintf(stderr, "ERROR: in Initialize, I2C is not initialized\n");
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

std::vector<uint8_t> MagDirect::ReadBytes(const uint8_t reg,
                                          const uint8_t count) const {
  // magnetometer is actually a separate device with its
  // own address inside the mpu9250
  if (not i2c_->SetSlaveAddress(AK8963_ADDR)) {
    return {};
  }
  // Read from register
  const std::vector<uint8_t> raw = i2c_->ReadBytes(reg, count);
  if (raw.empty()) {
    fprintf(stderr,
            "ERROR: in MagDirect::ReadBytes, failed to read register %x\n",
            reg);
    return {};
  }
  return raw;
}

bool MagDirect::WriteByte(const uint8_t reg, const uint8_t data) const {
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
