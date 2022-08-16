#ifndef _MPU_SPI_HPP
#define _MPU_SPI_HPP

#include <linux/spi/spidev.h>
#include <string>    

namespace spi
{
// void PrintDebugInfo(uint8_t* tx, uint8_t* rx, const uint32_t length);

void PrintCArrayData(uint8_t* array, const uint32_t length);

spi_ioc_transfer CreateSpiTransfer(uint8_t* tx, uint8_t* rx, const uint32_t length);

class SPI
{
 public:
  SPI(const std::string& path, const bool debug);

  int Transfer(uint8_t* buff, const uint32_t length) const;

  void WriteRegister(const uint8_t reg, const uint8_t data) const;
  
  uint8_t ReadRegister(const uint8_t reg) const;

  void ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t * dest) const;

 protected:
  int Open() const;

  static void Close(const int fd);

 private:
  std::string path_;
  bool debug_;
};
}  // namespace spi

#endif  //_MPU_SPI_HPP
