#pragma once

#include <random>
#include <algorithm>

template <typename T>
T generate_random(T vmin, T vmax) {
  static std::uniform_real_distribution<T> dis(vmin, vmax);
  static std::random_device rd;
  static std::mt19937 gen(rd());  // Standard mersenne_twister_engine
                                  // seeded with rd()
  return dis(gen);
}

template <typename T, size_t N>
std::array<T, N> generate_randoms(T vmin, T vmax) {
  std::array<T, N> arr;
  auto rand = [&vmin, &vmax] { return generate_random<T>(vmin, vmax); };
  std::generate(arr.begin(), arr.end(), rand);
  return std::move(arr);
}