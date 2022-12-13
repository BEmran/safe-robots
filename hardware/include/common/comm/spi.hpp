// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_COMM_SPI_HPP_
#define HARDWARE_COMMON_COMM_SPI_HPP_

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common/comm/communication_abs.hpp"

namespace hardware::common::comm {

class SPI : CommunicationAbs {
 public:
  explicit SPI(std::string_view path, const bool debug = false);
  ~SPI();

  virtual bool WriteByte(const uint8_t reg, const uint8_t data) const override;
  virtual std::vector<uint8_t> ReadBytes(const uint8_t reg,
                                         const uint8_t count) const override;
  void Close() override;

 protected:
  int Open() const;

  bool Transfer(const std::vector<uint8_t>& buff) const;

 private:
  std::string path_;
};

}  // namespace hardware::common::comm
#endif  // HARDWARE_COMMON_SPI_HPP_
