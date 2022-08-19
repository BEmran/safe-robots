#ifndef _MPU_SPI_HPP
#define _MPU_SPI_HPP

#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string>
#include <vector>

namespace spi
{
void PrintVec(const std::vector<uint8_t>& vec);

spi_ioc_transfer CreateSpiTransfer(const std::vector<uint8_t>& buf);

class SPI
{
 public:
  SPI(const std::string& path, const bool debug);

  int Transfer(const std::vector<uint8_t>& buff) const;

  void WriteRegister(const uint8_t reg, const uint8_t data) const;
  void WriteRegisters(const std::vector<std::pair<uint8_t, uint8_t>>& reg_and_data) const;
  
  uint8_t ReadRegister(const uint8_t reg) const;
  std::vector<uint8_t> ReadRegisters(const uint8_t reg, const uint8_t count) const;

 protected:
  int Open() const;

  static void Close(const int fd);

 private:
  std::string path_;
  bool debug_;
};
}  // namespace spi

#endif  //_MPU_SPI_HPP
