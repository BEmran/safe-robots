#pragma once
#include <cstddef>
#include "basic.hpp"
#include "utility.hpp"
#include <ostream>
#include <array>
#include <sstream>

template <typename T>
struct Mat2x2 : public MF2X2<T> {
  Mat2x2() noexcept : MF2X2<T>(0.f, 0.f, 0.f, 0.f) {
  }
  constexpr Mat2x2(T _m00, T _m01, T _m10, T _m11) noexcept
    : MF2X2<T>(_m00, _m01, _m10, _m11) {
  }

  constexpr Mat2x2(T c) noexcept : MF2X2<T>(c, c, c, c) {
  }

  // explicit Mat2x2(const std::array<T, 4>& array) noexcept
  //   : MF2X2<T>(array[0], array[1], array[2], array[3]) {
  // }

  Mat2x2(const VF2<T>& row0, const VF2<T>& row1) noexcept
    : MF2X2<T>(row0.x, row0.y, row1.x, row1.y) {
  }

  Mat2x2(const MF2X2<T>& mat) : MF2X2<T>(mat.m00, mat.m01, mat.m10, mat.m11) {
  }

  Mat2x2(const Mat2x2<T>&) = default;
  Mat2x2(Mat2x2<T>&&) = default;
  Mat2x2& operator=(const Mat2x2<T>&) = default;
  Mat2x2& operator=(Mat2x2<T>&&) = default;

  inline static Mat2x2<T> eye() {
    return Mat2x2<T>(1.f, 0.f, 0.f, 1.f);
  }

  inline static Mat2x2<T> ones() {
    return Mat2x2<T>(1.f);
  }

  inline static Mat2x2<T> zeros() {
    return Mat2x2<T>(0.f);
  }

  inline static Mat2x2<T> random(T vmin, T vmax) {
    MF2X2<T> result;
    auto rand = [&vmin, &vmax] { return generate_random<T>(vmin, vmax); };
    std::generate(std::begin(result.data), std::end(result.data), rand);
    return result;
  }

  // Comparison operators
  bool operator==(const Mat2x2<T>& other) const noexcept {
    return                       //
      this->m00 == other.m00 &&  //
      this->m01 == other.m01 &&  //
      this->m10 == other.m10 &&  //
      this->m11 == other.m11;
  }

  bool operator!=(const Mat2x2<T>& other) const noexcept {
    return not this->operator==(other);
  }

  // Assignment operators
  Mat2x2& operator=(const MF2X2<T>& other) noexcept {
    this->m00 = other.m00;
    this->m01 = other.m01;
    this->m10 = other.m10;
    this->m11 = other.m11;
    return *this;
  }

  Mat2x2& operator+=(const Mat2x2<T>& other) noexcept {
    this->m00 += other.m00;
    this->m01 += other.m01;
    this->m10 += other.m10;
    this->m11 += other.m11;
    return *this;
  }

  Mat2x2<T>& operator-=(const Mat2x2<T>& other) noexcept {
    this->m00 -= other.m00;
    this->m01 -= other.m01;
    this->m10 -= other.m10;
    this->m11 -= other.m11;
    return *this;
  }

  Mat2x2<T>& operator*=(const Mat2x2<T>& other) noexcept {
    MF2X2<T> tmp(this->row[0], this->row[1], this->row[2]);
    this->m00 = tmp.m00 * other.m00 + tmp.m01 * other.m10;
    this->m10 = tmp.m10 * other.m00 + tmp.m11 * other.m10;
    this->m01 = tmp.m00 * other.m01 + tmp.m01 * other.m11;
    this->m11 = tmp.m10 * other.m01 + tmp.m11 * other.m11;
    return *this;
  }

  Mat2x2<T> operator*(const Mat2x2<T>& other) const noexcept {
    const T tmp_m00 = this->m00 * other.m00 + this->m01 * other.m10;
    const T tmp_m10 = this->m10 * other.m00 + this->m11 * other.m10;
    const T tmp_m01 = this->m00 * other.m01 + this->m01 * other.m11;
    const T tmp_m11 = this->m10 * other.m01 + this->m11 * other.m11;
    return Mat2x2<T>(tmp_m00, tmp_m01, tmp_m10, tmp_m11);
  }

  Mat2x2<T>& operator*=(T s) noexcept {
    this->m00 *= s;
    this->m01 *= s;
    this->m10 *= s;
    this->m11 *= s;
    return *this;
  }

  Mat2x2<T>& operator/=(T s) noexcept {
    this->m00 /= s;
    this->m01 /= s;
    this->m10 /= s;
    this->m11 /= s;
    return *this;
  }

  Mat2x2<T>& operator/=(const Mat2x2<T>& other) noexcept {
    this->m00 = other.m00;
    this->m01 = other.m01;
    this->m10 = other.m10;
    this->m11 = other.m11;
    return *this;
  }

  // T sum() const noexcept {
  //   return this->m00 + this->m01 + this->m02 + this->m10 + this->m11 +
  //          this->m12 + this->m20 + this->m21 + this->m22;
  // }

  void inverse() noexcept {
    const T d = det();
    const Mat2x2 tmp(*this);
    this->m00 = tmp.m11 / d;
    this->m01 = -tmp.m01 / d;
    this->m10 = -tmp.m10 / d;
    this->m11 = tmp.m00 / d;
  }

  Mat2x2 inversed() const noexcept {
    Mat2x2 result(*this);
    result.inverse();
    return result;
  }

  void transpose() noexcept {
    std::swap(this->m01, this->m10);
  }

  Mat2x2 transposed() const noexcept {
    Mat2x2 result(*this);
    result.transpose();
    return std::move(result);;
  }

  T det() noexcept {
    return this->m00 * this->m11 - this->m01 * this->m01;
  }

  void clamp(T vmin, T vmax) noexcept {
    this->m00 = std::clamp(vmin, vmax, this->m00);
    this->m01 = std::clamp(vmin, vmax, this->m01);
    this->m10 = std::clamp(vmin, vmax, this->m11);
    this->m11 = std::clamp(vmin, vmax, this->m11);
  }

  Mat2x2 clamped(T vmin, T vmax) const noexcept {
    Mat2x2<T>result (*this);
    result.clamp();
    return std::move(result);
  }

  // Element-wise divide

  // Unary operators
  // Mat2x2<T> operator+() const noexcept {
  //   return *this;
  // }
  // Mat2x2<T> operator-() const noexcept;
};