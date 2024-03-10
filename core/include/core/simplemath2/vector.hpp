#pragma once
#include "core/simplemath2/basic2.hpp"
#include "core/simplemath2/utility2.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>

namespace simple2 {
template <typename T, size_t SIZE>
struct Vector : public BasicVector<T, SIZE> {
  using BasicVector<T, SIZE>::data;
  Vector() noexcept : BasicVector<T, SIZE>() {
  }

  explicit Vector(T c) noexcept : BasicVector<T, SIZE>(c) {
  }

  Vector(const BasicVector<T, SIZE>& bv) noexcept : BasicVector<T, SIZE>(bv) {
  }

  inline static Vector ones() {
    return Vector(1.f);
  }

  inline static Vector zeros() {
    return Vector(0.f);
  }

  inline static Vector random(T vmin, T vmax) {
    return Vector(generate_randoms<T, 3>(vmin, vmax));
  }

  template <typename U>
  Vector& operator+=(const Vector<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < SIZE; ++idx) {
      data[idx] += static_cast<T>(other.at(idx));
    }
    return *this;
  }

  template <typename U>
  Vector& operator-=(const Vector<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < SIZE; ++idx) {
      data[idx] -= static_cast<T>(other.at(idx));
    }
    return *this;
  }

  template <typename U>
  Vector& operator*=(U s) noexcept {
    T tmp_s = static_cast<T>(s);
    for (size_t idx = 0; idx < SIZE; ++idx) {
      data[idx] *= tmp_s;
    }
    return *this;
  }

  template <typename U>
  Vector& operator*=(const Vector<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < SIZE; ++idx) {
      data[idx] *= other.at(idx);
    }
    return *this;
  }

  template <typename U>
  Vector& operator/=(U s) noexcept {
    T tmp_s = static_cast<T>(s);
    for (size_t idx = 0; idx < SIZE; ++idx) {
      data[idx] /= tmp_s;
    }
    return *this;
  }

  template <typename U>
  Vector& operator/=(const Vector<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < SIZE; ++idx) {
      data[idx] /= other.at(idx);
    }
    return *this;
  }

  T sum() const noexcept {
    T result{};
    for (size_t idx = 0; idx < SIZE; ++idx) {
      result += data[idx];
    }
    return result;
  }

  T dot(const Vector& other) const noexcept {
    T result{};
    for (size_t idx = 0; idx < SIZE; ++idx) {
      result += data[idx] * other.at(idx);
    }
  }

  T norm() noexcept {
    return std::sqrt(dot(*this));
  }

  void normalize() noexcept {
    static constexpr double EPSILON{0.00001};
    const T n = norm();
    if (n < EPSILON && n > -EPSILON) {
      std::wcerr << "WARN: norm is too small: " << n << std::endl;
      return;
    }
    *this /= n;
  }

  Vector normalized() const noexcept {
    Vector result(*this);
    result.normalize();
    return result;
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  Vector cross(const Vector& other) const noexcept {
    Vector<T, 3> tmp;
    tmp.at(0) = data[0] * other.at(2) - data[2] * other.at(1);
    tmp.at(1) = data[2] * other.at(0) - data[1] * other.at(2);
    tmp.at(2) = data[0] * other.at(1) - data[1] * other.at(0);
    return tmp;
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  Vector cross(const Vector& other) const noexcept {
    (void)other;
    return *this;
  }

  template <typename U>
  void clamp(U vmin, U vmax) noexcept {
    T tmp_vmin = static_cast<T>(vmin);
    T tmp_vmax = static_cast<T>(vmax);
    for (size_t idx = 0; idx < SIZE; ++idx) {
      data[idx] /= std::clamp(data[idx], tmp_vmin, tmp_vmax);
    }
  }

  template <typename U>
  Vector clamped(U vmin, U vmax) const noexcept {
    Vector result(*this);
    result.clamp(vmin, vmax);
    return result;
  }
};

template <typename T>
using Vector3 = Vector<T, 3>;

template <typename T>
using Vector2 = Vector<T, 2>;

}  // namespace simple2

template <typename T, size_t SIZE, typename U>
simple2::Vector<T, SIZE>
operator+(const simple2::Vector<T, SIZE>& lhs,
          const simple2::Vector<U, SIZE>& rhs) noexcept {
  simple2::Vector<T, SIZE> result(lhs);
  result += rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Vector<T, SIZE>
operator-(const simple2::Vector<T, SIZE>& lhs,
          const simple2::Vector<U, SIZE>& rhs) noexcept {
  simple2::Vector<T, SIZE> result(lhs);
  result -= rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Vector<T, SIZE>
operator*(const simple2::Vector<T, SIZE>& lhs,
          const simple2::Vector<U, SIZE>& rhs) noexcept {
  simple2::Vector<T, SIZE> result(lhs);
  result *= rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Vector<T, SIZE> operator*(const simple2::Vector<T, SIZE>& mat,
                                   U s) noexcept {
  simple2::Vector<T, SIZE> result(mat);
  result *= s;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Vector<T, SIZE> operator/(const simple2::Vector<T, SIZE>& mat,
                                   U s) noexcept {
  simple2::Vector<T, SIZE> result(mat);
  result /= s;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Vector<T, SIZE>
operator/(const simple2::Vector<T, SIZE>& lhs,
          const simple2::Vector<U, SIZE>& rhs) noexcept {
  simple2::Vector<T, SIZE> result(lhs);
  result /= rhs;
  return result;
}