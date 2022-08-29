// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "navio/spi.hpp"

#include <fcntl.h>  // open
#include <linux/types.h>
#include <netdb.h>
#include <sys/ioctl.h>  // ioctl
#include <sys/socket.h>
#include <unistd.h>  // close

#include <algorithm>
#include <cstdio>   // printf
#include <cstring>  // memset
#include <iostream>

namespace navio {
namespace {
constexpr uint32_t SpeedHz = 1000000;
constexpr uint8_t BitsPerWord = 8;
constexpr uint8_t DelayUsecs = 0;

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

void PrintVec(const char* header, const std::vector<uint8_t>& vec) {
  std::cout << header;
  std::for_each(vec.begin(), vec.end(), [](const auto data) {
    std::cout << static_cast<int>(data) << "\t";
  });
  std::cout << std::endl;
}
}  // namespace

SPI::SPI(const std::string& path, bool debug)
  : CommunicationAbs(debug), path_(path) {
}

int SPI::Transfer(const std::vector<uint8_t>& buff) const {
  const int fd = Open();
  if (fd == -1) {
    return -1;
  }

  if (IsDebug()) {
    PrintVec("Tx: ", buff);
  }

  spi_ioc_transfer spi_transfer = CreateSpiTransfer(buff);
  int status = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_transfer);
  Close(fd);

  if (IsDebug()) {
    PrintVec("Rx: ", buff);
  }

  usleep(1000);

  return status;
}

void SPI::WriteRegister(uint8_t reg, uint8_t data) const {
  Transfer({reg, data});
}

void SPI::WriteRegisters(
  const std::vector<std::pair<uint8_t, uint8_t>>& reg_and_data) const {
  std::for_each(reg_and_data.begin(), reg_and_data.end(),
                [this](const auto rd) {
                  Transfer({rd.first, rd.second});
                });
}

uint8_t SPI::ReadRegister(uint8_t reg) const {
  return ReadRegisters(reg, 1)[0];
}

std::vector<uint8_t> SPI::ReadRegisters(uint8_t reg, uint8_t count) const {
  std::vector<uint8_t> buf(count + 1, 0);
  buf[0] = reg | 0x80;
  Transfer(buf);
  usleep(50);
  buf.erase(buf.begin());
  return buf;
}

int SPI::Open() const {
  int fd = ::open(path_.c_str(), O_RDWR);
  if (fd < 0) {
    printf("Error: Can not open SPI device: %s\n", path_.c_str());
    return -1;
  }
  return fd;
}

void SPI::Close(int fd) {
  if (fd < 0) {
    printf("Warning: spi already closed\n");
    return;
  }
  ::close(fd);
}
}  // namespace navio
