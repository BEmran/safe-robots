// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "navio/utils.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstdlib>

#include "common/utils.hpp"

namespace hardware::navio {

constexpr const char* SCRIPT_PATH = "../../../check_apm.sh";

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
  hardware::common::ReadFile("/sys/firmware/devicetree/base/hat/product_id",
                             "%x", &version);
  return version;
}

}  // namespace hardware::navio
