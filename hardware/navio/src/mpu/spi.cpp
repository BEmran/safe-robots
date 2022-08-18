#include "mpu/spi.hpp"

#include <unistd.h>     // close
#include <fcntl.h>      // open
#include <sys/ioctl.h>  // ioctl
#include <cstring>      // memset
#include <cstdio>       // printf
#include <linux/types.h>
#include <sys/socket.h>
#include <netdb.h>

namespace spi
{
constexpr uint32_t SpeedHz = 1000000;
constexpr uint8_t BitsPerWord = 8;
constexpr uint8_t DelayUsecs = 0;

SPI::SPI(const std::string& path, const bool debug)
  : path_(path), debug_(debug)
{
}

int SPI::Transfer(uint8_t* buff, const uint32_t length) const
{
  const int fd = Open();
  if (fd == -1)
  {
    return -1;
  }

  if (debug_)
  {
    printf("Tx: ");
    PrintCArrayData(buff, length);
  }

  spi_ioc_transfer spi_transfer = CreateSpiTransfer(buff, buff, length);
  int status = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_transfer);
  Close(fd);

  if (debug_)
  {
    printf("Rx: ");
    PrintCArrayData(buff, length);
  }

  return status;
}

void SPI::WriteRegister(const uint8_t reg, const uint8_t data) const
{
    printf("->>>>>>>>>>> WRITE reg %d: %d\n", (int)reg, (int)data);
    uint8_t buf[2] = {reg, data};
    Transfer(buf, 2);
}

uint8_t SPI::ReadRegister(const uint8_t reg) const
{
  uint8_t buffer[1] = {0};
  
  printf("->>>>>>>>>>> READ reg %d\n", (int)reg);
  ReadRegisters(reg, 1, buffer);
  return buffer[0];
}

void SPI::ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t * dest) const
{
    unsigned char buf[100] = {0};
    buf[0] = reg | 0x80;
    Transfer(buf, count + 1);

    for(uint8_t i=0; i < count; i++)
        dest[i] = buf[i + 1];

    usleep(50);
}

int SPI::Open() const
{
  int fd = ::open(path_.c_str(), O_RDWR);
  if (fd < 0)
  {
    printf("Error: Can not open SPI device: %s\n", path_.c_str());
    return -1;
  }
  return fd;
}

void SPI::Close(const int fd)
{
  if (fd < 0)
  {
    printf("Warning: spi already closed\n");
    return;
  }
  ::close(fd);
}

// void PrintDebugInfo(uint8_t* tx, uint8_t* rx, const uint32_t length)
// {
//   printf("Tx: ");
//   PrintCArrayData(tx, length);

//   printf("Rx: ");
//   PrintCArrayData(rx, length);
// }

void PrintCArrayData(uint8_t* array, const uint32_t length)
{
  for (uint i = 0; i < length; i++)
  {
    printf("[%2d]: %d\t", (int)i, (int)array[i]);
  }
  printf("\n");
}

spi_ioc_transfer CreateSpiTransfer(uint8_t* tx, uint8_t* rx, const uint32_t length)
{
  spi_ioc_transfer spi_transfer;
  memset(&spi_transfer, 0, sizeof(spi_ioc_transfer));

  // cast pointer address to long uint
  spi_transfer.tx_buf = reinterpret_cast<std::uintptr_t>(tx);
  spi_transfer.rx_buf = reinterpret_cast<std::uintptr_t>(rx);
  spi_transfer.len = length;
  spi_transfer.speed_hz = SpeedHz;
  spi_transfer.bits_per_word = BitsPerWord;
  spi_transfer.delay_usecs = DelayUsecs;
  return spi_transfer;
}
}  // namespace spi
