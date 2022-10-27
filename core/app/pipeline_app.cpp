
#include <iomanip>
#include <iostream>
#include <thread>

#include "core/utils/clock.hpp"
#include "core/utils/spinner.hpp"

using core::utils::HighResolutionClock;
using core::utils::Spinner;
using core::utils::TimeStruct;

int main(int /*argc*/, char const** /*argv[]*/) {
  HighResolutionClock clock;
  Spinner spinner(2);
  while (true) {
    spinner.SpinOnce();
  }
  return 0;
}
