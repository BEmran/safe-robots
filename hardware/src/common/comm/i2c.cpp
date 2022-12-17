#include "common/comm/i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>  //for IOCTL defs
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>  // for uint8_t types etc
#include <cstdio>
#include <cstdlib>
#include <cstring>  // strerror

namespace hardware::common::comm {

I2C::I2C(const uint8_t bus, const uint8_t device_address, const bool debug)
  : CommunicationAbs(debug) {
  Initialize(bus, device_address);
}

I2C::~I2C() {
  CloseImpl();
}

bool I2C::Initialize(const uint8_t bus, const uint8_t device_address) {
  // if already initialized just set the device address
  if (IsInitialized()) {
    return SetSlaveAddress(device_address);
  }
  initialized_ = false;

  if (not Open(bus)) {
    return false;
  }
  initialized_ = true;
  return SetSlaveAddress(device_address);
}

bool I2C::Open(const uint8_t bus) {
  constexpr int size = 16;
  char filename[size];
  int len = snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
  if (len >= size) {
    fprintf(stderr, "%s: path truncated\n", filename);
    return false;
  }

  fd_ = open(filename, O_RDWR);
  if (fd_ < 0) {
    fprintf(stderr, "Error in OpenI2CDevice: Could not open file '%s': %s\n",
            filename, std::strerror(errno));
    if (errno == EACCES) {
      fprintf(stderr, "Run as root?\n");
    }
    return false;
  }
  return true;
}

bool I2C::SetSlaveAddress(const uint8_t device_address) {
  if (not IsInitialized()) {
    return false;
  }

  // if the device address is already correct, just return
  if (device_address_ == device_address) {
    return true;
  }

  // if not, change it with ioctl
  if (ioctl(fd_, I2C_SLAVE, device_address) < 0) {
    fprintf(stderr,
            "Error in SetSlaveAddress: failed to set address to 0x%02x: "
            "%s\n",
            device_address_, std::strerror(errno));
    return false;
  }

  device_address_ = device_address;
  return true;
}

void I2C::Close() {
  CloseImpl();
}

void I2C::CloseImpl() {
  if (fd_ > 0) {
    close(fd_);
  } else {
    printf("Warning: I2C already closed\n");
  }
  device_address_ = 0;
  initialized_ = false;
  fd_ = -1;
}

std::vector<uint8_t> I2C::ReadBytes(const uint8_t reg,
                                    const uint8_t count) const {
  // write register to device
  if (not SendData({reg})) {
    return {};
  }

  // read the response
  const std::vector<uint8_t> data = GetResponse(count);
  if (data.empty()) {
    return {};
  }
  return data;
}

bool I2C::WriteByte(const uint8_t reg, const uint8_t data) const {
  // write the bytes
  return SendData({reg, data});
}

bool I2C::SendData(const std::vector<uint8_t>& data) const {
  if (not IsInitialized()) {
    return false;
  }

  const size_t size = data.size();
  const ssize_t written = write(fd_, data.data(), size);
  // printf("send: fd:%d size:%d written:%d data[0]%zx\n", fd_, size, written,
  //        data[0]);
  if (written != static_cast<ssize_t>(size)) {
    fprintf(stderr, "ERROR: in SendBytes, write %ld bytes, expected %zu: %s\n",
            written, size, strerror(errno));
    return false;
  }
  return true;
}

std::vector<uint8_t> I2C::GetResponse(const size_t count) const {
  if (not IsInitialized()) {
    return {};
  }

  // then read the response
  std::vector<uint8_t> buf(count);
  const ssize_t red = read(fd_, buf.data(), count);
  // printf("received fd:%d expect:%d red:%d data[0]%zx\n", fd_, count, red,
  //        buf[0]);

  if (red != static_cast<ssize_t>(count)) {
    fprintf(stderr, "ERROR: in Read, received %ld bytes, expected %zu: %s\n",
            red, count, strerror(errno));
    return {};
  }
  return buf;
}
}  // namespace hardware::common::comm