// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_NAVIO_INCLUDE_NAVIO_SPI_HPP_
#define HARDWARE_NAVIO_INCLUDE_NAVIO_SPI_HPP_

#include <linux/spi/spidev.h>
#include <stdint.h>

#include <string>
#include <utility>
#include <vector>

#include "navio/communication_abs.hpp"

namespace navio {
class SPI : CommunicationAbs {
 public:
  SPI(const std::string& path, const bool debug);
  ~SPI() {
  }

  int Transfer(const std::vector<uint8_t>& buff) const;

  void WriteRegister(const uint8_t reg, const uint8_t data) const override;

  void WriteRegisters(const std::vector<std::pair<uint8_t, uint8_t>>&
                        reg_and_data) const override;

  uint8_t ReadRegister(const uint8_t reg) const override;

  std::vector<uint8_t> ReadRegisters(const uint8_t reg,
                                     const uint8_t count) const override;

 protected:
  int Open() const;

  static void Close(const int fd);

 private:
  std::string path_;
};

}  // namespace navio
#endif  // HARDWARE_NAVIO_INCLUDE_NAVIO_SPI_HPP_
