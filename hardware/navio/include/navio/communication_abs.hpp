// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef NAVIO_COMMUNICATION_ABS_HPP_
#define NAVIO_COMMUNICATION_ABS_HPP_

#include <cstdint>
#include <utility>
#include <vector>

namespace navio {
class CommunicationAbs {
 public:
  explicit CommunicationAbs(bool debug) : debug_(debug) {
  }

  virtual ~CommunicationAbs() = default;

  virtual void WriteRegister(uint8_t reg, uint8_t data) const = 0;

  virtual void WriteRegisters(
    const std::vector<std::pair<uint8_t, uint8_t>>& reg_and_data) const = 0;

  virtual uint8_t ReadRegister(uint8_t reg) const = 0;

  virtual std::vector<uint8_t> ReadRegisters(uint8_t reg,
                                             uint8_t count) const = 0;

  inline bool IsDebug() const {
    return debug_;
  }

 private:
  bool debug_;
};
}  // namespace navio
#endif  // NAVIO_COMMUNICATION_ABS_HPP_
