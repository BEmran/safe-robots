#pragma once

#include "my_array.hpp"
#include <memory>
#include <ostream>
#include <random>
#include <algorithm>
#include <sstream>
#include <iterator>
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

template <typename Tp, std::size_t SIZE>
class Vector : public Array<Tp, SIZE> {
 public:
  using BaseType = Array<Tp, SIZE>;

  Vector() = default;

  Vector(const BaseType& other) : BaseType(other) {
  }

  Vector(BaseType&& other) : BaseType(std::move(other)) {
  }

  static Vector zeros() {
    return std::move(BaseType(0.0));
  }

  static Vector ones() {
    return std::move(BaseType(1.0));
  }

  static Vector random() {
    BaseType arr(0.0);
    std::generate(arr.begin(), arr.end(), generate_random);
    return std::move(arr);
  }

  std::string print() const {
    std::stringstream ss;
    ss << "[";
    std::copy(this->cbegin(), this->cend(),
              std::ostream_iterator<Tp>(ss, "\n"));
    ss << "]";
    return ss.str();
  }

  Vector<Tp, SIZE> dot(const Vector<Tp, SIZE>& other) const {
    BaseType result;
    for (auto i = 0; i < SIZE; i++) {
      result.at(i) = this->at(i) * other.at(i);
    }
    return std::move(result);
  }

  Tp mul(const Vector<Tp, SIZE>& other) const {
    Tp sum = 0.0;
    for (auto i = 0; i < SIZE; i++) {
      sum += this->at(i) * other.at(i);
    }
    return sum;
  }
};

template <typename Tp, std::size_t SIZE>
Vector<Tp, SIZE> dot(const Vector<Tp, SIZE>& lhs, const Vector<Tp, SIZE>& rhs) {
  return lhs.dot(rhs);
}

template <typename Tp, std::size_t SIZE>
double mul(const Vector<Tp, SIZE>& lhs, const Vector<Tp, SIZE>& rhs) {
  return lhs.mul(rhs);
}

// template <typename Tp, typename... Up>
// Vector(Tp, Up...) -> Vector<std::enable_if_t<(std::is_same_v<Tp, Up> && ...),
// Tp>, 1 + sizeof...(Up)>;
