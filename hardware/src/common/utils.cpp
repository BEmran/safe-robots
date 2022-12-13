// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "common/utils.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace hardware::common {

void MilliDelay(const uint32_t msec) {
  usleep(msec * 1000);
}

int WriteFile(const char* path, const char* fmt, ...) {
  errno = 0;

  int fd = ::open(path, O_WRONLY | O_CLOEXEC);
  if (fd == -1) {
    return -errno;
  }

  va_list args;
  va_start(args, fmt);

  int ret = ::vdprintf(fd, fmt, args);
  int errno_bkp = errno;
  ::close(fd);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }

  return ret;
}

int ReadFile(const char* path, const char* fmt, ...) {
  errno = 0;

  FILE* file = ::fopen(path, "re");
  if (!file) {
    return -errno;
  }
  va_list args;
  va_start(args, fmt);

  int ret = ::vfscanf(file, fmt, args);
  int errno_bkp = errno;
  ::fclose(file);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }
  return ret;
}
}  // namespace hardware::common
