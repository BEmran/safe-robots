// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "common/comm/spi.hpp"

#include <fcntl.h>  // open
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <netdb.h>
#include <sys/ioctl.h>  // ioctl
#include <sys/socket.h>
#include <unistd.h>  // close

#include <algorithm>
#include <cstdio>   // printf
#include <cstring>  // memset
#include <iostream>
#include <memory>

namespace hardware::common::comm {
namespace {
constexpr uint32_t SpeedHz = 1000000;
constexpr uint8_t BitsPerWord = 8;
constexpr uint8_t DelayUsecs = 0;
constexpr uint8_t ReadBit = 0x80;

spi_ioc_transfer CreateSpiTransfer(const std::vector<uint8_t>& buf) {
  spi_ioc_transfer spi_transfer;
  memset(&spi_transfer, 0, sizeof(spi_ioc_transfer));

  // cast pointer address to long uint
  spi_transfer.tx_buf = reinterpret_cast<std::uintptr_t>(buf.data());
  spi_transfer.rx_buf = reinterpret_cast<std::uintptr_t>(buf.data());
  spi_transfer.len = static_cast<unsigned int>(buf.size());
  spi_transfer.speed_hz = SpeedHz;
  spi_transfer.bits_per_word = BitsPerWord;
  spi_transfer.delay_usecs = DelayUsecs;
  return spi_transfer;
}

void CloseImpl(const int fd) {
  if (fd > 0) {
    close(fd);
  } else {
    printf("Warning: SPI already closed\n");
  }
}

void PrintVec(const char* header, const std::vector<uint8_t>& vec) {
  std::cout << header;
  std::for_each(vec.begin(), vec.end(), [](const auto data) {
    std::cout << static_cast<int>(data) << "\t";
  });
  std::cout << std::endl;
}
}  // namespace

SPI::SPI(std::string_view path, const bool debug)
  : CommunicationAbs(debug), path_{path} {
}

SPI::~SPI() {
  path_ = "";
}

bool SPI::Transfer(const std::vector<uint8_t>& buff) const {
  // TODO check if we can open file only once instead of each time
  const int fd = Open();
  if (fd == -1) {
    return false;
  }

  if (IsDebug()) {
    PrintVec("Tx: ", buff);
  }

  spi_ioc_transfer spi_transfer = CreateSpiTransfer(buff);
  if (ioctl(fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0) {
    fprintf(stderr, "Error in SPI: failed to transfer data to %s: %s\n",
            path_.c_str(), std::strerror(errno));
    return false;
  }
  CloseImpl(fd);

  if (IsDebug()) {
    PrintVec("Rx: ", buff);
  }

  usleep(1000);

  return true;
}

bool SPI::WriteByte(const uint8_t reg, const uint8_t data) const {
  return Transfer({reg, data});
}

std::vector<uint8_t> SPI::ReadBytes(const uint8_t reg,
                                    const uint8_t count) const {
  const size_t size = static_cast<size_t>(count + 1);
  std::vector<uint8_t> buf(size, 0);
  buf[0] = reg | ReadBit;
  Transfer(buf);
  usleep(50);
  buf.erase(buf.begin());
  return buf;
}

int SPI::Open() const {
  const int fd = ::open(path_.c_str(), O_RDWR);
  if (fd < 0) {
    printf("Error: Can not open SPI device: %s\n", path_.c_str());
    return -1;
  }
  return fd;
}

void SPI::Close() {
  // if (fd_ > 0) {
  //   close(fd_);
  // } else {
  //   printf("Warning: SPI already closed\n");
  // }
}

}  // namespace hardware::common::comm
