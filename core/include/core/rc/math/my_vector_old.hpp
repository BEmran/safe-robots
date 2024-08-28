#pragma once

#include "my_array.hpp"
#include <memory>
#include <array>
#include <ostream>
#include <random>
#include <algorithm>
#include <cassert>
#include <initializer_list>

constexpr double max_value{std::numeric_limits<double>::max()};
constexpr double min_value{std::numeric_limits<double>::min()};

double generate_random() {
  static std::uniform_real_distribution<double> dis(-1.0, 1.0);
  static std::random_device rd;
  static std::mt19937 gen(rd());  // Standard mersenne_twister_engine
                                           // seeded with rd()
  return dis(gen);
}

template <typename T, std::size_t SIZE>
class RCVector : public Array<T, SIZE> {
 public:
  RCVector() = default;

  RCVector(std::initializer_list<T> vals) {
    std::copy(vals.begin(), vals.end(), this->begin());
  }
  // RCVector(const Array<T, SIZE>& d) : this(d) {
  // }

  // RCVector(Array<T, SIZE>&& d) : m_data(std::move(d)) {
  // }

  // RCVector operator=(const Array<T, SIZE>& d) {
  //   m_data = d;
  // }

  // RCVector operator=(Array<T, SIZE>&& d) {
  //   m_data(std::move(d));
  // }

  // T& operator[](size_t idx) {
  //   return m_data.at(idx);
  // }

  // T operator[](size_t idx) const {
  //   return m_data.at(idx);
  // }

  static RCVector zeros() {
    RCVector<T, SIZE> data;
    data.fill(0.0);
    return data;
  }

  static RCVector ones() {
    RCVector<T, SIZE> data;
    data.fill(1.0);
    return data;
  }

  static RCVector random() {
    RCVector<T, SIZE> data;
    data.randomize();
    return data;
  }

  void randomize() {
    std::generate(this->begin(), this->end(), generate_random);
  }
  // size_t size() const{
  //   return m_data.size();
  // }

  std::string print() const {
    std::string str;
    str += "[";
    std::for_each(this->cbegin(), this->cend(),
                  [&str](T d) { str += std::to_string(d) + " "; });
    str += "]";
    return str;
  }

  RCVector<T, SIZE> dot(const RCVector<T, SIZE>& other) const {
    RCVector<T, SIZE> C;
    for (auto i = 0; i < SIZE; i++) {
      C[i] = this->operator[](i) * other[i];
    }
    return C;
  }

  T mul(const RCVector<T, SIZE>& other) const {
    T sum = 0.0;
    for (auto i = 0; i < SIZE; i++) {
      sum += this->operator[](i) * other[i];
    }
    return sum;
  }
};

template <typename T, std::size_t SIZE>
RCVector<T, SIZE> dot(const RCVector<T, SIZE>& A, const RCVector<T, SIZE>& B) {
  return A.dot(B);
}

template <typename T, std::size_t SIZE>
double mul(const RCVector<T, SIZE>& A, const RCVector<T, SIZE>& B) {
  return A.mul(B);
}
