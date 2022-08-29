// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef NAVIO_SPI_HPP_
#define NAVIO_SPI_HPP_

#include <linux/spi/spidev.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "navio/communication_abs.hpp"

namespace navio {
class SPI : CommunicationAbs {
 public:
  SPI(const std::string& path, bool debug);
  ~SPI() override = default;

  int Transfer(const std::vector<uint8_t>& buff) const;

  void WriteRegister(uint8_t reg, uint8_t data) const override;

  void WriteRegisters(const std::vector<std::pair<uint8_t, uint8_t>>&
                        reg_and_data) const override;

  uint8_t ReadRegister(uint8_t reg) const override;

  std::vector<uint8_t> ReadRegisters(uint8_t reg, uint8_t count) const override;

 protected:
  int Open() const;

  static void Close(int fd);

 private:
  std::string path_;
};

}  // namespace navio
#endif  // NAVIO_SPI_HPP_
