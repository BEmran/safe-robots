/*
SPIDev driver code is placed under the BSD license.
Written by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
Copyright (c) 2014, Emlid Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _SPIDEV_H_
#define _SPIDEV_H_

//#define _XOPEN_SOURCE 600
#include <unistd.h>  // close
#include <fcntl.h>  // open
#include <sys/ioctl.h>  // ioctl
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstring>  // memset
#include <cstdio>	// printf
#include <string>

constexpr uint32_t SpeedHz = 1000000;
constexpr uint8_t BitsPerWord = 8;
constexpr uint8_t DelayUsecs = 0;

spi_ioc_transfer CreateSpiTransfer(uint8_t* tx, uint8_t* rx, uint32_t length)
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

class SPIdev
{
 public:
  SPIdev(const std::string& path) : SPIdev(path, false)
  {
  }

  SPIdev(const std::string& path, const bool debug) : path_(path), debug_(debug)
  {
  }

  int Transfer(uint8_t* tx, uint8_t* rx, const uint32_t length)
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

 protected:
  int Open() const
  {
    int fd = ::open(path_.c_str(), O_RDWR);
    if (fd < 0)
    {
      printf("Error: Can not open SPI device: %s\n", path_.c_str());
      return -1;
    }
    return fd;
  }

  void Close(const int fd) const
  {
    if (fd < 0)
    {
      printf("Warning: spi alrady closed\n");
      return;
    }
    ::close(fd);
  }

  void PrintDebugInfo(uint8_t* tx, uint8_t* rx, const uint32_t length) const
  {
    printf("Tx: ");
    PrintCArrayData(tx, length);

    printf("Rx: ");
    PrintCArrayData(rx, length);
  }

  void PrintCArrayData(uint8_t* array, const uint32_t length) const
  {
    for (uint i = 0; i < length; i++)
    {
      printf("[%3x]: %x", i, array[i]);
    }
    printf("\n");
  }

 private:
  std::string path_;
  bool debug_;
};

#endif  //_SPIDEV_H_
