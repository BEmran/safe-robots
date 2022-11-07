// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "bbb/i2c.hpp"

#include <fcntl.h>          // open O_RDWR
#include <linux/i2c-dev.h>  //for IOCTL defs
// #include <stdio.h>
// #include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

// /// @brief size of i2c buffer in bytes for writing to registers. Only
// increase
// /// if you know what you are doing.
// #define I2C_BUFFER_SIZE 128

#include "core/utils/logger_macros.hpp"
#include "sensors/common/utils.hpp"

/// @brief Maximum I2C bus identifier. Default is 5 for a total of 6 busses.
/// This can be increased by the user for special cases.
constexpr uint8_t kI2CMaxBus = 5;

I2CManager::I2CManager() {
  for (uint8_t i = 0; i < kI2CMaxBus; i++) {
    I2Cs.push_back(std::make_shared<I2C>(i));
  }
}

std::shared_ptr<I2C> I2CManager::CreateI2C(const int bus,
                                           const uint8_t devAddr) {
  // sanity check
  if (bus > I2Cs.size()) {
    SYS_LOG_WARN("Close, wrong bus number");
    return nullptr;
  }

  // initialized
  I2Cs[bus]->Initialize(devAddr);
  return I2Cs[bus];
}

void I2CManager::SetDeviceAddress(const int bus, const uint8_t devAddr) {
  // sanity check
  if (bus > I2Cs.size()) {
    SYS_LOG_WARN("SetDeviceAddress, wrong bus number");
    return;
  }
  I2Cs[bus]->SetDeviceAddress(devAddr);
}

void I2CManager::Close(const int bus) {
  // sanity check
  if (bus > I2Cs.size()) {
    SYS_LOG_WARN("Close, wrong bus number");
    return;
  }
  I2Cs[bus]->Close();
}

I2C::I2C(const int bus) : state_{I2CState(bus)} {
}

I2C::~I2C() {
  Close();
}

void I2C::Initialize(const uint8_t devAddr) {
  Configure();
  SetDeviceAddress(devAddr);
  state_.initialized = 1;
}

void I2C::Close() {
  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  close(state_.fd);
  state_.fd = -1;
  state_.devAddr = 0;
  state_.initialized = false;
}

void I2C::Configure() {
  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // open file descriptor
  char str[16];
  sprintf(str, "/dev/i2c-%d", state_.bus);
  int fd = open(str, O_RDWR);
  if (fd == -1) {
    SYS_LOG_WARN("init, failed to open /dev/i2c");
    return;
  }
  state_.fd = fd;
}

void I2C::SetDeviceAddress(const uint8_t devAddr) {
  if (state_.fd == -1) {
    SYS_LOG_WARN("SetDeviceAddress, bus not configured yet");
    return;
  }

  // if the device address is already correct, just return
  if (state_.devAddr == devAddr) {
    return;
  }

  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // if not, change it with ioctl
  if (ioctl(state_.fd, I2C_SLAVE, devAddr) < 0) {
    SYS_LOG_WARN("SetDeviceAddress, ioctl slave address change failed");
    return;
  }
  state_.devAddr = devAddr;
}

uint8_t I2C::ReadByte(const uint8_t regAddr) {
  auto data = ReadBytes(regAddr, 1);
  if (data.size() == 0) {
    return 0;
  }
  return data[0];
}

std::vector<uint8_t> I2C::ReadBytes(const uint8_t regAddr, const size_t count) {
  if (not state_.initialized) {
    SYS_LOG_WARN("ReadBytes, bus not initialized yet");
    return {};
  }

  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // write register to device
  auto ret = write(state_.fd, &regAddr, 1);
  if (ret != 1) {
    SYS_LOG_WARN("ReadBytes, failed to write to bus");
    return {};
  }

  // Read the response
  std::vector<uint8_t> data(count);
  ret = read(state_.fd, data.data(), count);
  if (ret != count) {
    SYS_LOG_WARN("ReadBytes, received unexpected bytes from device");
    return {};
  }
  return data;
}

uint16_t I2C::ReadWord(const uint8_t regAddr) {
  auto data = ReadWords(regAddr, 1);
  if (data.size() == 0) {
    return 0;
  }
  return data[0];
}

std::vector<uint16_t> I2C::ReadWords(const uint8_t regAddr,
                                     const size_t count) {
  const auto buf = ReadBytes(regAddr, count * 2);
  if (buf.size() < count * 2) {
    return {};
  }

  std::vector<uint16_t> data(count);
  // form words from bytes and put into user's data array
  for (size_t i = 0; i < count; i++) {
    const size_t idx{i * 2};
    const uint8_t msb{buf[idx]};
    const uint8_t lsb{buf[idx + 1]};
    data[i] = sensors::common::utils::ToWord({msb, lsb});
  }
  return data;
}

bool I2C::WriteByte(const uint8_t regAddr, const uint8_t data) {
  return SendBytes({regAddr, data});
}

bool I2C::WriteBytes(const uint8_t reg_addr, const std::vector<uint8_t>& data) {
  // assemble array to send, starting with the register address
  std::vector<uint8_t> write_data(data.size() + 1);
  write_data[0] = reg_addr;
  std::copy(data.begin(), data.end(), write_data.begin() + 1);

  return SendBytes(write_data);
}

bool I2C::WriteWord(const uint8_t regAddr, const uint16_t data) {
  return WriteWords(regAddr, {data});
}

bool I2C::WriteWords(const uint8_t regAddr, const std::vector<uint16_t>& data) {
  // assemble bytes to send
  std::vector<uint8_t> write_data((data.size() * 2) + 1);
  write_data[0] = regAddr;
  for (size_t i = 0; i < data.size(); i++) {
    const sensors::common::utils::Bytes bytes =
      sensors::common::utils::ToBytes(data[i]);
    const size_t idx{i * 2};
    write_data[idx + 1] = bytes.msb;
    write_data[idx + 2] = bytes.lsb;
  }
  return SendBytes(write_data);
}

bool I2C::SendByte(const uint8_t data) {
  return SendBytes({data});
}

bool I2C::SendBytes(const std::vector<uint8_t>& data) {
  // sanity check
  if (not state_.initialized) {
    SYS_LOG_WARN("SendBytes, bus not initialized yet");
    return false;
  }

  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // send the bytes
  auto ret = write(state_.fd, data.data(), data.size());

  // write should have returned the correct # bytes written
  if (ret != data.size()) {
    SYS_LOG_WARN("SendBytes, failed to write to bus");
    return false;
  }
  return true;
}

std::optional<int> I2C::GetFd() {
  if (state_.initialized) {
    SYS_LOG_WARN("[I2C]: trying to get fd but bus is not initialized yet\n");
    return {};
  }
  return {state_.fd};
}
