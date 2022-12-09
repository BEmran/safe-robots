#include "i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>  //for IOCTL defs
#include <string.h>         // strerror
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>  // for uint8_t types etc
#include <cstdio>
#include <cstdlib>
#include <memory>

// preposessor macros
#define unlikely(x) __builtin_expect(!!(x), 0)
#define likely(x) __builtin_expect(!!(x), 1)

static I2C i2c[I2C_MAX_BUS + 1];

// local function
bool CheckBusRange(const int bus) {
  if (bus < 0 || bus > I2C_MAX_BUS) {
    fprintf(stderr, "ERROR: i2c bus must be between 0 & %d\n", I2C_MAX_BUS);
    return false;
  }
  return true;
}

// bool SanityCheck(int bus) {
//   if (not CheckBusRange(bus)) {
//     return false;
//   }

//   if (not i2c[bus].Initialized()) {
//     fprintf(stderr, "ERROR: in rc_i2c_write_bytes, bus not initialized
//     yet\n"); return false;
//   }

//   return true;
// }

I2C::I2C() {
}

I2C::~I2C() {
  Close();
}

bool I2C::Initialize(const int bus_num, const uint8_t device_address) {
  if (not CheckBusRange(bus_num)) {
    return false;
  }

  bus_ = bus_num;

  // if already initialized just set the device address
  if (Initialized()) {
    return SetSlaveAddress(device_address);
  }

  // lock the bus during this operation
  lock_ = true;
  initialized_ = false;

  // open file descriptor
  constexpr int size = 16;
  char filename[size];
  int len = snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus_);
  if (len >= size) {
    fprintf(stderr, "%s: path truncated\n", filename);
    return false;
  }

  fd_ = open(filename, O_RDWR);
  if (fd_ < 0) {
    fprintf(stderr, "Error in OpenI2CDevice: Could not open file '%s': %s\n",
            filename, strerror(errno));
    if (errno == EACCES) {
      fprintf(stderr, "Run as root?\n");
    }
    return false;
  }

  // return the lock state to previous state.
  lock_ = false;
  initialized_ = true;

  // set device address
  return SetSlaveAddress(device_address);
}

bool I2C::SanityCheck() const {
  if (not Initialized()) {
    fprintf(stderr, "ERROR: in SanityCheck, bus not initialized yet\n");
    return false;
  }

  return true;
}

bool I2C::SetSlaveAddress(const uint8_t device_address) {
  if (not SanityCheck()) {
    return false;
  }

  // if the device address is already correct, just return
  if (device_address_ == device_address) {
    return true;
  }

  // if not, change it with ioctl
  if (ioctl(fd_, I2C_SLAVE, device_address) < 0) {
    fprintf(stderr,
            "Error in SetSlaveAddress: failed to set address to 0x%02x: %s\n",
            device_address_, strerror(errno));
    return false;
  }

  device_address_ = device_address;
  return true;
}

bool I2C::Close() {
  close(fd_);
  device_address_ = 0;
  bus_ = 0;
  initialized_ = false;
  lock_ = false;
  return true;
}

std::optional<uint8_t> I2C::ReadByte(const uint8_t reg) const {
  const std::vector<uint8_t> data = ReadBytes(reg, 1);
  if (data.empty()) {
    return {};
  }
  return data[0];
}

std::vector<uint8_t> I2C::ReadBytes(const uint8_t reg,
                                    const size_t count) const {
  // write register to device
  if (not Write({reg})) {
    return {};
  }

  // then read the response
  const std::vector<uint8_t> data = Read(count);
  if (data.empty()) {
    return {};
  }
  return data;
}

std::optional<uint16_t> I2C::ReadWord(const uint8_t reg) const {
  const std::vector<uint16_t> data = ReadWords(reg, 1);
  if (data.empty()) {
    return {};
  }
  return data[0];
}

std::vector<uint16_t> I2C::ReadWords(const uint8_t reg,
                                     const size_t count) const {
  // then read the response
  const std::vector<uint8_t> data = ReadBytes(reg, count * 2);
  if (data.empty()) {
    return {};
  }

  // form words from bytes and put into user's data array
  std::vector<uint16_t> words(count);
  for (size_t i = 0; i < count; i++) {
    const size_t idx = i * 2;
    const uint16_t msb = static_cast<uint16_t>(data[idx]) << 8;
    const uint16_t lsb = static_cast<uint16_t>(data[idx + 1]);
    words[i] = static_cast<uint16_t>(msb | lsb);
  }

  return words;
}

bool I2C::WriteByte(const uint8_t reg, const uint8_t data) const {
  return WriteBytes(reg, {data});
}

bool I2C::WriteBytes(const uint8_t reg,
                     const std::vector<uint8_t>& data) const {
  // assemble array to send, starting with the register address
  std::vector<uint8_t> writeData(data.size() + 1);
  writeData[0] = reg;
  for (size_t i = 0; i < data.size(); i++) {
    writeData[i + 1] = data[i];
  }

  // send the bytes
  return SendBytes(writeData);
}

bool I2C::WriteWord(const uint8_t reg, const uint16_t data) const {
  // send the bytes
  return WriteWords(reg, {data});
}

bool I2C::WriteWords(const uint8_t reg,
                     const std::vector<uint16_t>& data) const {
  // assemble bytes to send from data casted as uint8_t*
  std::vector<uint8_t> writeData(data.size() * 2 + 1);
  writeData[0] = reg;
  for (size_t i = 0; i < data.size(); i++) {
    const size_t idx = i * 2 + 1;
    writeData[idx] = static_cast<uint8_t>(data[i] >> 8);
    writeData[idx + 1] = static_cast<uint8_t>(data[i] & 0xFF);
  }

  // send the bytes
  return SendBytes(writeData);
}

bool I2C::SendByte(const uint8_t data) const {
  return SendBytes({data});
}

bool I2C::SendBytes(const std::vector<uint8_t>& data) const {
  return Write(data);
}

// write register to device
bool I2C::Write(const std::vector<uint8_t>& data) const {
  if (not SanityCheck()) {
    return false;
  }

  // lock the bus during this operation
  lock_ = true;

  const size_t size = data.size();
  const ssize_t written = write(fd_, data.data(), size);
  if (written != static_cast<ssize_t>(size)) {
    fprintf(stderr, "ERROR: in Write, write %d bytes, expected %zu: %s\n",
            written, size, strerror(errno));
    // unlock after finish
    lock_ = false;
    return false;
  }
  // unlock after finish
  lock_ = false;
  return true;
}

std::vector<uint8_t> I2C::Read(const size_t count) const {
  if (not SanityCheck()) {
    return {};
  }

  // lock the bus during this operation
  lock_ = true;
  // then read the response
  std::vector<uint8_t> buf(count);
  const ssize_t red = read(fd_, buf.data(), count);
  if (red != static_cast<ssize_t>(count)) {
    fprintf(stderr, "ERROR: in Read, received %d bytes, expected %zu: %s\n",
            red, count, strerror(errno));
    // unlock after finish
    lock_ = false;
    return {};
  }
  // unlock after finish
  lock_ = false;
  return buf;
}