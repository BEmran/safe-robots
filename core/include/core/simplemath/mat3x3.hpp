#pragma once
#include <cstddef>
#include "basic.hpp"
#include "utility.hpp"
#include <ostream>
#include <sstream>
#include <array>

template <typename T>
struct Mat3x3 : public MF3X3<T> {
  Mat3x3() noexcept : MF3X3<T>(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f) {
  }
  constexpr Mat3x3(T _m00, T _m01, T _m02, T _m10, T _m11, T _m12, T _m20,
                   T _m21, T _m22) noexcept
    : MF3X3<T>(_m00, _m01, _m02, _m10, _m11, _m12, _m20, _m21, _m22) {
  }

  constexpr Mat3x3(T c) noexcept : MF3X3<T>(c, c, c, c, c, c, c, c, c) {
  }

  Mat3x3(const std::array<T, 9>& array) noexcept
    : MF3X3<T>(array[0], array[1], array[2], array[3], array[4], array[5],
               array[6], array[7], array[8]) {
  }

  Mat3x3(const VF3<T>& row0, const VF3<T>& row1, const VF3<T>& row2) noexcept
    : MF3X3<T>(row0.x, row0.y, row0.z, row1.x, row1.y, row1.z, row2.x, row2.y,
               row2.z) {
  }

  Mat3x3(const MF3X3<T>& mat)
    : MF3X3<T>(mat.m00, mat.m01, mat.m02, mat.m10, mat.m11, mat.m12, mat.m20,
               mat.m21, mat.m22) {
  }
  // explicit Matrix(_In_reads_(16) const T *pArray) noexcept :

  Mat3x3(const Mat3x3<T>&) = default;
  Mat3x3(Mat3x3<T>&&) = default;
  Mat3x3& operator=(const Mat3x3<T>&) = default;
  Mat3x3& operator=(Mat3x3<T>&&) = default;

  inline static Mat3x3<T> eye() {
    return Mat3x3<T>(1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f);
  }

  inline static Mat3x3<T> ones() {
    return Mat3x3<T>(1.f);
  }

  inline static Mat3x3<T> zeros() {
    return Mat3x3<T>(0.f);
  }

  inline static Mat3x3<T> random(T vmin, T vmax) {
    return Mat3x3<T>(generate_randoms<T, 9>(vmin, vmax));
  }

  // Comparison operators
  bool operator==(const Mat3x3<T>& other) const noexcept {
    return                       //
      this->m00 == other.m00 &&  //
      this->m01 == other.m01 &&  //
      this->m02 == other.m02 &&  //
      this->m10 == other.m10 &&  //
      this->m11 == other.m11 &&  //
      this->m12 == other.m12 &&  //
      this->m20 == other.m20 &&  //
      this->m21 == other.m21 &&  //
      this->m22 == other.m22;
  }

  bool operator!=(const Mat3x3<T>& other) const noexcept {
    return not this->operator==(other);
  }

  // Assignment operators
  Mat3x3& operator=(const MF3X3<T>& other) noexcept {
    this->m00 = other.m00;
    this->m01 = other.m01;
    this->m02 = other.m02;
    this->m10 = other.m10;
    this->m11 = other.m11;
    this->m12 = other.m12;
    this->m20 = other.m20;
    this->m21 = other.m21;
    this->m22 = other.m22;
    return *this;
  }

  Mat3x3& operator+=(const Mat3x3<T>& other) noexcept {
    this->m00 += other.m00;
    this->m01 += other.m01;
    this->m02 += other.m02;
    this->m10 += other.m10;
    this->m11 += other.m11;
    this->m12 += other.m12;
    this->m20 += other.m20;
    this->m21 += other.m21;
    this->m22 += other.m22;
    return *this;
  }

  Mat3x3<T>& operator-=(const Mat3x3<T>& other) noexcept {
    this->m00 -= other.m00;
    this->m01 -= other.m01;
    this->m02 -= other.m02;
    this->m10 -= other.m10;
    this->m11 -= other.m11;
    this->m12 -= other.m12;
    this->m20 -= other.m20;
    this->m21 -= other.m21;
    this->m22 -= other.m22;
    return *this;
  }

  Mat3x3<T>& operator*=(const Mat3x3<T>& other) noexcept {
    MF3X3<T> tmp(*this);
    this->m00 = tmp.m00 * other.m00 + tmp.m01 * other.m10 + tmp.m02 * other.m20;
    this->m10 = tmp.m10 * other.m00 + tmp.m11 * other.m10 + tmp.m12 * other.m20;
    this->m20 = tmp.m20 * other.m00 + tmp.m21 * other.m10 + tmp.m22 * other.m20;
    this->m01 = tmp.m00 * other.m01 + tmp.m01 * other.m11 + tmp.m02 * other.m21;
    this->m11 = tmp.m10 * other.m01 + tmp.m11 * other.m11 + tmp.m12 * other.m21;
    this->m21 = tmp.m20 * other.m01 + tmp.m21 * other.m11 + tmp.m22 * other.m21;
    this->m02 = tmp.m00 * other.m02 + tmp.m01 * other.m12 + tmp.m02 * other.m22;
    this->m12 = tmp.m10 * other.m02 + tmp.m11 * other.m12 + tmp.m12 * other.m22;
    this->m22 = tmp.m20 * other.m02 + tmp.m21 * other.m12 + tmp.m22 * other.m22;
    return *this;
  }

  Mat3x3<T> operator*(const Mat3x3<T>& other) const noexcept {
    const T tmp_m00 =
      this->m00 * other.m00 + this->m01 * other.m10 + this->m02 * other.m20;
    const T tmp_m10 =
      this->m10 * other.m00 + this->m11 * other.m10 + this->m12 * other.m20;
    const T tmp_m20 =
      this->m20 * other.m00 + this->m21 * other.m10 + this->m22 * other.m20;
    const T tmp_m01 =
      this->m00 * other.m01 + this->m01 * other.m11 + this->m02 * other.m21;
    const T tmp_m11 =
      this->m10 * other.m01 + this->m11 * other.m11 + this->m12 * other.m21;
    const T tmp_m21 =
      this->m20 * other.m01 + this->m21 * other.m11 + this->m22 * other.m21;
    const T tmp_m02 =
      this->m00 * other.m02 + this->m01 * other.m12 + this->m02 * other.m22;
    const T tmp_m12 =
      this->m10 * other.m02 + this->m11 * other.m12 + this->m12 * other.m22;
    const T tmp_m22 =
      this->m20 * other.m02 + this->m21 * other.m12 + this->m22 * other.m22;
    return Mat3x3<T>(tmp_m00, tmp_m01, tmp_m02,  //
                     tmp_m10, tmp_m11, tmp_m12,  //
                     tmp_m20, tmp_m21, tmp_m22);
  }

  Mat3x3<T>& operator*=(T s) noexcept {
    this->m00 *= s;
    this->m01 *= s;
    this->m02 *= s;
    this->m10 *= s;
    this->m11 *= s;
    this->m12 *= s;
    this->m20 *= s;
    this->m21 *= s;
    this->m22 *= s;
    return *this;
  }

  Mat3x3<T>& operator/=(T s) noexcept {
    this->m00 /= s;
    this->m01 /= s;
    this->m02 /= s;
    this->m10 /= s;
    this->m11 /= s;
    this->m12 /= s;
    this->m20 /= s;
    this->m21 /= s;
    this->m22 /= s;
    return *this;
  }

  Mat3x3<T>& operator/=(const Mat3x3<T>& other) noexcept {
    this->m00 = other.m00;
    this->m01 = other.m01;
    this->m02 = other.m02;
    this->m10 = other.m10;
    this->m11 = other.m11;
    this->m12 = other.m12;
    this->m20 = other.m20;
    this->m21 = other.m21;
    this->m22 = other.m22;
    return *this;
  }

  // T sum() const noexcept {
  //   return this->m00 + this->m01 + this->m02 + this->m10 + this->m11 +
  //          this->m12 + this->m20 + this->m21 + this->m22;
  // }
  T det() noexcept {
    return  //
      this->m00 * (this->m11 * this->m22 - this->m21 * this->m12) +
      this->m01 * (this->m20 * this->m12 - this->m10 * this->m22) +
      this->m02 * (this->m10 * this->m21 - this->m11 * this->m21) +
      this->m10 * (this->m21 * this->m02 - this->m01 * this->m22) +
      this->m11 * (this->m00 * this->m22 - this->m20 * this->m02) +
      this->m12 * (this->m20 * this->m01 - this->m00 * this->m21) +
      this->m20 * (this->m01 * this->m12 - this->m11 * this->m02) +
      this->m21 * (this->m10 * this->m02 - this->m00 * this->m12) +
      this->m22 * (this->m00 * this->m11 - this->m01 * this->m01);
  }

  void inverse() noexcept {
    const T d = det();
    if (d == 0) {
      // warning matrix is han no inverse
      return;
    }

    transpose();
    const Mat3x3 temp(*this);
    this->m00 = (temp->m11 * temp->m22 - temp->m21 * temp->m12) / d;
    this->m01 = (temp->m20 * temp->m12 - temp->m10 * temp->m22) / d;
    this->m02 = (temp->m10 * temp->m21 - temp->m11 * temp->m21) / d;
    this->m10 = (temp->m21 * temp->m02 - temp->m01 * temp->m22) / d;
    this->m11 = (temp->m00 * temp->m22 - temp->m20 * temp->m02) / d;
    this->m12 = (temp->m20 * temp->m01 - temp->m00 * temp->m21) / d;
    this->m20 = (temp->m01 * temp->m12 - temp->m11 * temp->m02) / d;
    this->m21 = (temp->m10 * temp->m02 - temp->m00 * temp->m12) / d;
    this->m22 = (temp->m00 * temp->m11 - temp->m01 * temp->m01) / d;
  }

  Mat3x3 inversed() const noexcept {
    Mat3x3 result(*this);
    result.inverse();
    return result;
  }

  void transpose() noexcept {
    std::swap(this->m01, this->m10);
    std::swap(this->m02, this->m20);
    std::swap(this->m12, this->m21);
  }

  Mat3x3 transposed() const noexcept {
    Mat3x3 result(*this);
    result.transpose();
    return result;
  }

  // Element-wise divide

  // Unary operators
  // Mat3x3<T> operator+() const noexcept {
  //   return *this;
  // }
  // Mat3x3<T> operator-() const noexcept;
};