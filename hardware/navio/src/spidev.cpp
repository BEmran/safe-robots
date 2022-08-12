//#define _XOPEN_SOURCE 600
#include <navio/spidev.h>

#include <unistd.h>     // close
#include <fcntl.h>      // open
#include <sys/ioctl.h>  // ioctl
#include <cstring>      // memset
#include <cstdio>       // printf
#include <linux/types.h>
#include <sys/socket.h>
#include <netdb.h>

constexpr uint32_t SpeedHz = 1000000;
constexpr uint8_t BitsPerWord = 8;
constexpr uint8_t DelayUsecs = 0;

void PrintDebugInfo(uint8_t* tx, uint8_t* rx, const uint32_t length)
{
  printf("Tx: ");
  PrintCArrayData(tx, length);

  printf("Rx: ");
  PrintCArrayData(rx, length);
}

void PrintCArrayData(uint8_t* array, const uint32_t length)
{
  for (uint i = 0; i < length; i++)
  {
    printf("[%3x]: %x", i, array[i]);
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

SPIdev::SPIdev(const std::string& path) : SPIdev(path, false)
{
}

SPIdev::SPIdev(const std::string& path, const bool debug)
  : path_(path), debug_(debug)
{
}

int SPIdev::Transfer(uint8_t* tx, uint8_t* rx, const uint32_t length) const
{
  const int fd = Open();
  if (fd == -1)
  {
    return -1;
  }

  spi_ioc_transfer spi_transfer = CreateSpiTransfer(tx, rx, length);
  int status = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_transfer);
  Close(fd);

  if (debug_)
  {
    PrintDebugInfo(tx, rx, length);
  }

  return status;
}

int SPIdev::Open() const
{
  int fd = ::open(path_.c_str(), O_RDWR);
  if (fd < 0)
  {
    printf("Error: Can not open SPI device: %s\n", path_.c_str());
    return -1;
  }
  return fd;
}

void SPIdev::Close(const int fd)
{
  if (fd < 0)
  {
    printf("Warning: spi alrady closed\n");
    return;
  }
  ::close(fd);
}
