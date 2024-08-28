#include <iostream>
#include <ostream>
#include <array>
#include <chrono>
#include <functional>

constexpr bool DEBUG{1};

#include "vecvec.hpp"
#include "matmat.hpp"

void testing() {
  // Vector<2> a;
  vecvec::Vector<10> a({1,2,3,4,5,6,7,8,9,0});
  a *= a;
  vecvec::Vector<10> b({1,2,3,4,5,6,7,8,9,0});
  vecvec::Vector<10> c = a * b;
  std::cout << "a: " << a << std::endl;
  std::cout << "b: " << b << std::endl;
  std::cout << "c: " << c << std::endl;
}

void test(void (*func)(void), const char* version, int iter) {
  const auto start{std::chrono::steady_clock::now()};
  for (auto i = 0; i < iter; i++) {
    func();
  }
  const auto end{std::chrono::steady_clock::now()};
  const std::chrono::duration<double> elapsed_seconds{end - start};
  std::cout << "[" << version << "] time difference " << elapsed_seconds.count()
            << " rate: " << elapsed_seconds.count() / iter << std::endl;
}

int main() {
  const int iter = 1e0;
  test(*testing, "1", iter);
  matmat::Matrix<3,3> m1(1);
  matmat::Matrix<3,3> m2({1,2,3,4,5,6,7,8,9});

  std::cout << "m1:\n" << m1 << std::endl;
  std::cout << "m2:\n" << m2 << std::endl;
  return 0;
}
