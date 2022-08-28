#include "navio/hardware_utils.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace navio::hardware_utils {
constexpr const char* SCRIPT_PATH = "../../../check_apm.sh";

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

bool CheckApm() {
  int ret = system("ps -AT | grep -c ap-timer > /dev/null");

  if (WEXITSTATUS(ret) <= 0) {
    fprintf(stderr, "APM is running. Can't launch the example\n");
    return true;
  }

  return false;
}

int GetNavioVersion() {
  int version;
  ReadFile("/sys/firmware/devicetree/base/hat/product_id", "%x", &version);
  return version;
}

void Delay(uint32_t msec) {
  usleep(msec * 1000);
}
}  // namespace navio::hardware_utils
