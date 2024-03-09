// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include <cstddef>
#include "core/simplemath/basic.hpp"
#include "core/simplemath/vector3.hpp"
#include "core/simplemath/utility.hpp"
#include <iostream>
#include <sstream>
#include <array>

template <typename T>
struct Matrix3x3 : public BasicMatrix3x3<T> {
  Matrix3x3() noexcept : BasicMatrix3x3<T>() {
  }

  explicit Matrix3x3(T c) noexcept : BasicMatrix3x3<T>(c) {
  }

  Matrix3x3(T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21,
            T m22) noexcept
    : BasicMatrix3x3<T>(m00, m01, m02, m10, m11, m12, m20, m21, m22) {
  }

  Matrix3x3(const std::array<T, 9>& array) noexcept
    : BasicMatrix3x3<T>(array[0], array[1], array[2], array[3], array[4],
                        array[5], array[6], array[7], array[8]) {
  }

  Matrix3x3(const BasicMatrix3x3<T>& bm) noexcept : BasicMatrix3x3<T>(bm) {
  }
  // Matrix3x3(const BasicVector3<T>& row0, const BasicVector3<T>& row1, const
  // BasicVector3<T>& row2) noexcept
  //   : BasicMatrix3x3<T>(row0.x, row0.y, row0.z, row1.x, row1.y, row1.z,
  //   row2.x, row2.y,
  //              row2.z) {
  // }
  // explicit Matrix(_In_reads_(16) const T *pArray) noexcept :

  // Matrix3x3(const Matrix3x3<T>&) = default;
  // Matrix3x3(Matrix3x3<T>&&) = default;
  // Matrix3x3& operator=(const Matrix3x3<T>&) = default;
  // Matrix3x3& operator=(Matrix3x3<T>&&) = default;

  inline static Matrix3x3<T> eye() {
    return Matrix3x3<T>(1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f);
  }

  inline static Matrix3x3<T> ones() {
    return Matrix3x3<T>(1.f);
  }

  inline static Matrix3x3<T> zeros() {
    return Matrix3x3<T>(0.f);
  }

  inline static Matrix3x3<T> random(T vmin, T vmax) {
    return Matrix3x3<T>(generate_randoms<T, 9>(vmin, vmax));
  }

  // // Comparison operators
  // bool operator==(const Matrix3x3<T>& other) const noexcept {
  //   return                       //
  //     this->mat[0][0] == other.m00 &&  //
  //     this->mat[0][1] == other.m01 &&  //
  //     this->mat[0][2] == other.m02 &&  //
  //     this->mat[1][0] == other.m10 &&  //
  //     this->mat[1][1] == other.m11 &&  //
  //     this->mat[1][2] == other.m12 &&  //
  //     this->mat[2][0] == other.m20 &&  //
  //     this->mat[2][1] == other.m21 &&  //
  //     this->mat[2][2] == other.m22;
  // }

  // bool operator!=(const Matrix3x3<T>& other) const noexcept {
  //   return not this->operator==(other);
  // }

  // // Assignment operators
  // Matrix3x3& operator=(const BasicMatrix3x3<T>& other) noexcept {
  //   this->mat[0][0] = other.m00;
  //   this->mat[0][1] = other.m01;
  //   this->mat[0][2] = other.m02;
  //   this->mat[1][0] = other.m10;
  //   this->mat[1][1] = other.m11;
  //   this->mat[1][2] = other.m12;
  //   this->mat[2][0] = other.m20;
  //   this->mat[2][1] = other.m21;
  //   this->mat[2][2] = other.m22;
  //   return *this;
  // }

  Matrix3x3& operator+=(const Matrix3x3<T>& other) noexcept {
    this->mat[0][0] += other(0, 0);
    this->mat[0][1] += other(0, 1);
    this->mat[0][2] += other(0, 2);
    this->mat[1][0] += other(1, 0);
    this->mat[1][1] += other(1, 1);
    this->mat[1][2] += other(1, 2);
    this->mat[2][0] += other(2, 0);
    this->mat[2][1] += other(2, 1);
    this->mat[2][2] += other(2, 2);
    return *this;
  }

  Matrix3x3<T>& operator-=(const Matrix3x3<T>& other) noexcept {
    this->mat[0][0] -= other(0, 0);
    this->mat[0][1] -= other(0, 1);
    this->mat[0][2] -= other(0, 2);
    this->mat[1][0] -= other(1, 0);
    this->mat[1][1] -= other(1, 1);
    this->mat[1][2] -= other(1, 2);
    this->mat[2][0] -= other(2, 0);
    this->mat[2][1] -= other(2, 1);
    this->mat[2][2] -= other(2, 2);
    return *this;
  }

  Matrix3x3<T>& operator*=(const Matrix3x3<T>& other) noexcept {
    BasicMatrix3x3<T> tmp(*this);
    // const T m00 = tmp(0, 0) * other(0, 0) + tmp(0, 1) * other(1, 0) + tmp(0,
    // 2) * other(2, 0);

    // const T m10 = tmp(1, 0) * other(0, 0) + tmp(1, 1) * other(1, 0) + tmp(1,
    // 2) * other(2, 0);

    // const T m20 = tmp(2, 0) * other(0, 0) + tmp(2, 1) * other(1, 0) + tmp(2,
    // 2) * other(2, 0);

    // const T m01 = tmp(0, 0) * other(0, 1) + tmp(0, 1) * other(1, 1) + tmp(0,
    // 2) * other(2, 1);

    // const T m11 = tmp(1, 0) * other(0, 1) + tmp(1, 1) * other(1, 1) + tmp(1,
    // 2) * other(2, 1);

    // const T m21 = tmp(2, 0) * other(0, 1) + tmp(2, 1) * other(1, 1) + tmp(2,
    // 2) * other(2, 1);

    // const T m02 = tmp(0, 0) * other(0, 2) + tmp(0, 1) * other(1, 2) + tmp(0,
    // 2) * other(2, 2);

    // const T m12 = tmp(1, 0) * other(0, 2) + tmp(1, 1) * other(1, 2) + tmp(1,
    // 2) * other(2, 2);

    // const T m22 = tmp(2, 0) * other(0, 2) + tmp(2, 1) * other(1, 2) + tmp(2,
    // 2) * other(2, 2);

    auto row_by_col = [&](size_t row, size_t col) -> T {
      return tmp(row, 0) * other(0, col) +  //
             tmp(row, 1) * other(1, col) +  //
             tmp(row, 2) * other(2, col);
    };

    const T m00 = row_by_col(0, 0);
    const T m10 = row_by_col(1, 0);
    const T m20 = row_by_col(2, 0);
    const T m01 = row_by_col(0, 1);
    const T m11 = row_by_col(1, 1);
    const T m21 = row_by_col(2, 1);
    const T m02 = row_by_col(0, 2);
    const T m12 = row_by_col(1, 2);
    const T m22 = row_by_col(2, 2);

    this->mat[0][0] = m00;
    this->mat[1][0] = m10;
    this->mat[2][0] = m20;
    this->mat[0][1] = m01;
    this->mat[1][1] = m11;
    this->mat[2][1] = m21;
    this->mat[0][2] = m02;
    this->mat[1][2] = m12;
    this->mat[2][2] = m22;

    return *this;
  }

  Matrix3x3<T>& operator*=(T s) noexcept {
    this->mat[0][0] *= s;
    this->mat[0][1] *= s;
    this->mat[0][2] *= s;
    this->mat[1][0] *= s;
    this->mat[1][1] *= s;
    this->mat[1][2] *= s;
    this->mat[2][0] *= s;
    this->mat[2][1] *= s;
    this->mat[2][2] *= s;
    return *this;
  }

  Matrix3x3<T>& operator/=(T s) noexcept {
    this->mat[0][0] /= s;
    this->mat[0][1] /= s;
    this->mat[0][2] /= s;
    this->mat[1][0] /= s;
    this->mat[1][1] /= s;
    this->mat[1][2] /= s;
    this->mat[2][0] /= s;
    this->mat[2][1] /= s;
    this->mat[2][2] /= s;
    return *this;
  }

  Matrix3x3<T>& operator/=(const Matrix3x3<T>& other) noexcept {
    this->operator*=(other.inversed());
    return *this;
  }

  T det() const noexcept {
    return  //
      this->mat[0][0] * (this->mat[1][1] * this->mat[2][2] -
                         this->mat[2][1] * this->mat[1][2]) +
      this->mat[0][1] * (this->mat[2][0] * this->mat[1][2] -
                         this->mat[1][0] * this->mat[2][2]) +
      this->mat[0][2] *
        (this->mat[1][0] * this->mat[2][1] - this->mat[1][1] * this->mat[2][0]);
  }

  void transpose() noexcept {
    std::swap(this->mat[0][1], this->mat[1][0]);
    std::swap(this->mat[0][2], this->mat[2][0]);
    std::swap(this->mat[1][2], this->mat[2][1]);
  }

  Matrix3x3 transposed() const noexcept {
    Matrix3x3 result(*this);
    result.transpose();
    return result;
  }

  Matrix3x3 inverse() noexcept {
    static constexpr double EPSILON{0.00001};
    const T d = det();
    if (std::abs(static_cast<double>(d)) < EPSILON) {
      std::wcerr << "WARN: det is too small: " << d << std::endl;
      return *this;  // TODO: print warning when result is false
    }
    
    // Matrix3x3 result(*this);
    // result.transpose();
    // m00 = (this->mat[1][1] * this->mat[2][2] - this->mat[1][2] * this->mat[2][1]) / d;
    // m01 = (this->mat[2][0] * this->mat[1][2] - this->mat[1][0] * this->mat[2][2]) / d;
    // m02 = (this->mat[1][0] * this->mat[2][1] - this->mat[1][1] * this->mat[2][0]) / d;
    // m10 = (this->mat[2][1] * this->mat[0][2] - this->mat[0][1] * this->mat[2][2]) / d;
    // m11 = (this->mat[0][0] * this->mat[2][2] - this->mat[2][0] * this->mat[0][2]) / d;
    // m12 = (this->mat[2][0] * this->mat[0][1] - this->mat[0][0] * this->mat[2][1]) / d;
    // m20 = (this->mat[0][1] * this->mat[1][2] - this->mat[1][1] * this->mat[0][2]) / d;
    // m21 = (this->mat[1][0] * this->mat[0][2] - this->mat[0][0] * this->mat[1][2]) / d;
    // m22 = (this->mat[0][0] * this->mat[1][1] - this->mat[0][1] * this->mat[1][0]) / d;
    // return result;
    
    return adjoint() / d;
  }

  Matrix3x3 adjoint() const noexcept {
    // Calculate the cofactor matrix and transpose matrix it at the same time.
    const T m00 = this->mat[1][1] * this->mat[2][2] - this->mat[1][2] * this->mat[2][1];
    const T m01 = this->mat[2][0] * this->mat[1][2] - this->mat[1][0] * this->mat[2][2];
    const T m02 = this->mat[1][0] * this->mat[2][1] - this->mat[1][1] * this->mat[2][0];
    const T m10 = this->mat[2][1] * this->mat[0][2] - this->mat[0][1] * this->mat[2][2];
    const T m11 = this->mat[0][0] * this->mat[2][2] - this->mat[2][0] * this->mat[0][2];
    const T m12 = this->mat[2][0] * this->mat[0][1] - this->mat[0][0] * this->mat[2][1];
    const T m20 = this->mat[0][1] * this->mat[1][2] - this->mat[1][1] * this->mat[0][2];
    const T m21 = this->mat[1][0] * this->mat[0][2] - this->mat[0][0] * this->mat[1][2];
    const T m22 = this->mat[0][0] * this->mat[1][1] - this->mat[0][1] * this->mat[1][0];
    return Matrix3x3(m00, m10, m20, m01, m11, m21, m02, m12, m22); // transposed
  }

  void clamp(T vmin, T vmax) noexcept {
    this->mat[0][0] = std::clamp(this->mat[0][0], vmin, vmax);
    this->mat[0][1] = std::clamp(this->mat[0][1], vmin, vmax);
    this->mat[0][2] = std::clamp(this->mat[0][2], vmin, vmax);
    this->mat[1][0] = std::clamp(this->mat[1][0], vmin, vmax);
    this->mat[1][1] = std::clamp(this->mat[1][1], vmin, vmax);
    this->mat[1][2] = std::clamp(this->mat[1][2], vmin, vmax);
    this->mat[2][0] = std::clamp(this->mat[2][0], vmin, vmax);
    this->mat[2][1] = std::clamp(this->mat[2][1], vmin, vmax);
    this->mat[2][2] = std::clamp(this->mat[2][2], vmin, vmax);
  }

  Matrix3x3 clamped(T vmin, T vmax) const noexcept {
    Matrix3x3<T> result(*this);
    result.clamp(vmin, vmax);
    return result;
  }

  inline Vector3<T> row(size_t idx) const noexcept {
    return Vector3<T>(this->mat[idx][0], this->mat[idx][1], this->mat[idx][2]);
  }

  inline Vector3<T> col(size_t idx) const noexcept {
    return Vector3<T>(this->mat[0][idx], this->mat[1][idx], this->mat[2][idx]);
  }
};

template <typename T>
Matrix3x3<T> operator+(const Matrix3x3<T>& lhs,
                       const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result += rhs;
  return result;
}

template <typename T>
Matrix3x3<T> operator-(const Matrix3x3<T>& lhs,
                       const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result -= rhs;
  return result;
}

template <typename T>
Matrix3x3<T> operator*(const Matrix3x3<T>& lhs,
                       const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result *= rhs;
  return result;
}

template <typename T>
Matrix3x3<T> operator*(const Matrix3x3<T>& vec, T s) noexcept {
  Matrix3x3<T> result(vec);
  result *= s;
  return result;
}

template <typename T>
inline Matrix3x3<T> operator/(const Matrix3x3<T>& vec, T s) noexcept {
  Matrix3x3<T> result(vec);
  result /= s;
  return result;
}

template <typename T>
inline Matrix3x3<T> operator/(const Matrix3x3<T>& lhs,
                              const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result /= rhs;
  return result;
}

std::ostream& operator<<(std::ostream& os, Matrix3x3<float> mat) {
  for (size_t r = 0; r < mat.rows(); ++r) {
    for (size_t c = 0; c < mat.cols(); ++c) {
      os << mat.mat[r][c] << ", ";
    }
    os << "\n";
  }
  return os;
}