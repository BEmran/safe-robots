#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

#include "core/utils/spinner.hpp"

// std::chrono::system_clock::time_point ptime_;

// double UpdateTime() {
//   auto ctime = std::chrono::high_resolution_clock::now();
//   // auto dt = std::chrono::duration_cast<std::chrono::seconds>(ctime -
//   ptime_); std::chrono::duration<double> elapsed = ctime - ptime_; ptime_ =
//   ctime; return elapsed.count();
// }

// double SpinOnce() {
//   const auto dt = UpdateTime();
//   // std::cout << dt << std::endl;
//   if (dt < sampling_time_) {
//     // sleep the remaining time
//     const auto sleep_time_msec =
//       static_cast<int64_t>((sampling_time_ - dt) * 1e6);
//     // std::cout << sleep_time_msec << std::endl;
//     std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_msec));
//   }
//   // update time
//   const auto actual_dt = UpdateTime();
//   // std::cout << actual_dt << std::endl;

//   return actual_dt;
// }

// int main() {
//   spinner::Spinner spinner(1.0, true);
//   std::chrono::system_clock::time_point ctime =
//     std::chrono::high_resolution_clock::now();
//   std::chrono::system_clock::time_point ptime = ctime;
//   const double sampling_time = 0.2;
//   while (true) {
//     std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
//     auto tmp = ctime;
//     std::unique_ptr<Time>;
//     std::chrono::duration<double> actual = ctime - tmp;
//     std::chrono::duration<double> elapsed = ctime - ptime;
//     auto sleeping_time = sampling_time - elapsed.count();
//     if (sleeping_time > 0) {
//       std::this_thread::sleep_for(std::chrono::duration<double>(sleeping_time));
//     } else {
//       std::cerr << "violate sleeping time" << std::endl;
//     }
//     ptime = std::chrono::high_resolution_clock::now();
//     // std::chrono::duration<double> sleeping_time = end - start;

//     // some work
//     std::cout << " elapsed:" << elapsed.count()
//               << " sleeping: " << sleeping_time << " actual: " <<
//               actual.count()
//               << std::endl;
//     // << " sleeping_time:" << sleeping_time.count() << std::endl;
//   }

//   return 0;
// }
int main(int argc, char const* argv[]) {
  spinner::Spinner spin(10, true);
  while (true) {
    spin.SpinOnce();
  }
  return 0;
}
