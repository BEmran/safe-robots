#ifndef _MPU_SPI_HPP
#define _MPU_SPI_HPP

#include "mpu/comm_abs.hpp"

#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string>
#include <vector>

namespace spi
{

spi_ioc_transfer CreateSpiTransfer(const std::vector<uint8_t>& buf);

}  // namespace spi

class SPI : CommAbs
{
 public:
  SPI(const std::string& path, const bool debug);
  ~SPI(){}

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

#endif  //_MPU_SPI_HPP
