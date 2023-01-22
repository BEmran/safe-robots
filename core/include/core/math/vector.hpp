// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef MY_MATH_VECTOR_HPP_
#define MY_MATH_VECTOR_HPP_

#include <assert.h>

#include <cmath>
#include <cstddef>  // size_t
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace core::math::my {

template <typename T>
class Vec {
 public:
  Vec() = default;

  Vec(std::initializer_list<T> l) : vec{l} {
  }

  Vec(const T value, const size_t length) : vec(length) {
    for (size_t i = 0; i < length; i++) {
      vec[i] = value;
    }
  }

  inline size_t Length() const {
    return vec.size();
  }

  inline bool IsEmpty() const {
    return Length() == 0;
  }

  T Norm() const {
    T norm{0};
    for (size_t i = 0; i < Length(); i++) {
      norm += vec[i] * vec[i];
    }
    return std::sqrt(norm);
  }

  void Normalize() {
    const T norm = Norm();
    for (size_t i = 0; i < Length(); i++) {
      vec[i] /= norm;
    }
  }

  T Dot(const Vec<T>& rhs) {
    T sum{};
    assert(this->Size() == rhs.Size());
    for (size_t i = 0; i < this->Size(); i++) {
      sum += this->operator[](i) * rhs[i];
    }
    return sum;
  }

  T operator[](const size_t idx) const {
    if (Length() == 0 || idx >= Length()) {
      throw std::runtime_error("wrong idx");
    }
    return vec[idx];
  }

  T& operator[](const size_t idx) {
    if (Length() == 0 || idx >= Length()) {
      throw std::runtime_error("wrong idx");
    }
    return vec[idx];
  }

 protected:
  std::vector<T> vec;
};

template <typename T>
class Vec3 : public Vec<T> {
 public:
  Vec3() : Vec3(0, 0, 0) {
  }

  Vec3(std::initializer_list<T> l) : Vec<T>(l) {
  }

  Vec3(const T x, const T y, const T z) : Vec3{{x, y, z}} {
  }

  Vec3<T> Cross(const Vec3<T>& rhs) {
    const T x = this->Y() * rhs.Z() - this->Z() * rhs.Y();
    const T y = this->Z() * rhs.X() - this->X() * rhs.Z();
    const T z = this->X() * rhs.Y() - this->Y() * rhs.X();
    return Vec3{x, y, z};
  }

  T Dot(const Vec3<T>& rhs) {
    return this->X() * rhs.X() + this->Y() * rhs.Y() + this->Z() * rhs.Z();
  }

  inline T X() const {
    return this->operator[](0);
  }

  inline T& X() {
    return this->operator[](0);
  }

  inline T Y() const {
    return this->operator[](1);
  }

  inline T& Y() {
    return this->operator[](1);
  }

  inline T Z() const {
    return this->operator[](2);
  }

  inline T& Z() {
    return this->operator[](2);
  }
};

}  // namespace core::math::my
#endif  // MY_MATH_VECTOR_HPP_