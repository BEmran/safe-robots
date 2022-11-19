// Copyright (C) 2022 Bara Emran - All Rights Reserved

/// @brief interface for the the Linux I2C driver

#ifndef BBB_I2C_HPP_
#define BBB_I2C_HPP_

#include <cstdint>  // for uint8_t types etc
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

class I2C;

/**
 * contains the current state of a bus. you don't need to create your own
 * instance of this, one for each bus is allocated here
 */
struct I2CState {
  bool initialized{false};
  int fd{-1};
  uint8_t devAddr{0};
  size_t bus{0};
  I2CState(const size_t bus_)
    : initialized{false}, fd{-1}, devAddr{0}, bus{bus_} {
  }
};

class I2CManager {
 public:
  I2CManager();
  std::shared_ptr<I2C> CreateI2C(const size_t bus, const uint8_t devAddr);
  void SetDeviceAddress(const size_t bus, const uint8_t devAddr);
  void Close(const size_t bus);

 private:
  std::vector<std::shared_ptr<I2C>> I2Cs;
};

class I2C {
 public:
  I2C(const size_t bus);
  ~I2C();

  void Initialize(const uint8_t devAddr);
  void SetDeviceAddress(const uint8_t devAddr);
  void Close();

  uint8_t ReadByte(const uint8_t regAddr);
  std::vector<uint8_t> ReadBytes(const uint8_t regAddr, const size_t count);
  int16_t ReadWord(const uint8_t regAddr);
  std::vector<int16_t> ReadWords(const uint8_t regAddr, const size_t count);

  bool WriteByte(const uint8_t reg_addr, const uint8_t data);
  bool WriteBytes(const uint8_t reg_addr, const std::vector<uint8_t>& data);
  bool WriteWord(const uint8_t reg_addr, const int16_t data);
  bool WriteWords(const uint8_t reg_addr, const std::vector<int16_t>& data);

  bool SendByte(const uint8_t data);
  bool SendBytes(const std::vector<uint8_t>& data);
  void Lock();
  void Unlock();
  bool Getlock();
  void SetStatus(I2CState state);
  I2CState GetStatus();
  std::optional<int> GetFd();

 protected:
  void Configure();

 private:
  std::mutex i2c_mutex_;
  I2CState state_;
};

#endif  // BBB_I2C_HPP_
