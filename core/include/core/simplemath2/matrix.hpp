// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include "basic2.hpp"

#include <cstddef>
#include <iostream>

namespace simple2 {
template <typename T, size_t SIZE>
struct Matrix : public BasicSquareMatrix<T, SIZE> {
  // struct Matrix {
  using BasicSquareMatrix<T, SIZE>::data;
  using BasicSquareMatrix<T, SIZE>::mat;
  using BasicSquareMatrix<T, SIZE>::array_size;

  Matrix() noexcept : BasicSquareMatrix<T, SIZE>() {
  }

  explicit Matrix(T c) noexcept : BasicSquareMatrix<T, SIZE>(c) {
  }

  Matrix(const std::array<T, array_size>& array) noexcept
    : BasicSquareMatrix<T, SIZE>(array) {
  }

  template <typename... Args>
  Matrix(Args&&... args)
    : BasicSquareMatrix<T, SIZE>(std::forward<Args>(args)...) {
  }

  template <std::size_t S = SIZE, std::enable_if_t<S == 3 && S == SIZE, int> = 0>
 inline static Matrix<T, SIZE> eye() {
    return Matrix<T, SIZE>(0.5f);
  }

  template <std::size_t S = SIZE, std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  inline static Matrix<T, SIZE> eye() {
    return Matrix<T, SIZE>(0.5f);
  }

  inline static Matrix<T, SIZE> ones() {
    return Matrix<T, SIZE>(1.f);
  }

  inline static Matrix<T, SIZE> zeros() {
    return Matrix<T, SIZE>(0.f);
  }

  template <typename U>
  Matrix& operator+=(const Matrix<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] += static_cast<T>(other.at(idx));
    }
    return *this;
  }

  template <typename U>
  Matrix& operator-=(const Matrix<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] -= static_cast<T>(other.at(idx));
    }
    return *this;
  }

  template <typename U>
  Matrix& operator*=(U s) noexcept {
    T tmp_s = static_cast<T>(s);
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] *= tmp_s;
    }
    return *this;
  }

  template <std::size_t S = SIZE, std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  Matrix& operator*=(const Matrix<T, SIZE>& other) noexcept {
    auto row_by_col = [&](size_t row, size_t col) -> float {
      return mat[row][0] * other.mat[0][col] +  //
             mat[row][1] * other.mat[1][col] +  //
             mat[row][2] * other.mat[2][col];
    };

    T result[3][3];
    result[0][0] = row_by_col(0, 0);
    result[1][0] = row_by_col(1, 0);
    result[2][0] = row_by_col(2, 0);
    result[0][1] = row_by_col(0, 1);
    result[1][1] = row_by_col(1, 1);
    result[2][1] = row_by_col(2, 1);
    result[0][2] = row_by_col(0, 2);
    result[1][2] = row_by_col(1, 2);
    result[2][2] = row_by_col(2, 2);

    mat[0][0] = result[0][0];
    mat[0][1] = result[0][1];
    mat[0][2] = result[0][2];
    mat[1][0] = result[1][0];
    mat[1][1] = result[1][1];
    mat[1][2] = result[1][2];
    mat[2][0] = result[2][0];
    mat[2][1] = result[2][1];
    mat[2][2] = result[2][2];
    return *this;
  }

  template <std::size_t S = SIZE, std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  Matrix& operator*=(const Matrix<T, SIZE>& other) noexcept {
    (void)other;
    return *this;
  }

  template <typename U>
  Matrix& operator/=(U s) noexcept {
    T tmp_s = static_cast<T>(s);
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] /= tmp_s;
    }
    return *this;
  }

  template <typename U>
  Matrix<T, SIZE>& operator/=(const Matrix<U, SIZE>& other) noexcept {
    this->operator*=(other.inverse());
    return *this;
  }

  void transpose() noexcept {
    for (size_t r = 0; r < this->dim(); ++r) {
      for (size_t c = r + 1; c < this->dim(); ++c) {
        std::swap(mat[r][c], mat[c][r]);
      }
    }
  }

  Matrix transposed() const noexcept {
    Matrix result(*this);
    result.transpose();
    return result;
  }

  template <std::size_t S = SIZE, std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  T det() const noexcept {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) +
           mat[0][1] * (mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
  }

  template <std::size_t S = SIZE, std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  T det() const noexcept {
    return 1;
  }

  template <std::size_t S = SIZE, std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  Matrix adjoint() const noexcept {
    // Calculate the cofactor matrix and transpose matrix it at the same time.
    Matrix<T, 3> result;
    result.mat[0][0] = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
    result.mat[1][0] = mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2];
    result.mat[2][0] = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];
    result.mat[0][1] = mat[2][1] * mat[0][2] - mat[0][1] * mat[2][2];
    result.mat[1][1] = mat[0][0] * mat[2][2] - mat[2][0] * mat[0][2];
    result.mat[2][1] = mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1];
    result.mat[0][2] = mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2];
    result.mat[1][2] = mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2];
    result.mat[2][2] = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    return result;
  }

  template <std::size_t S = SIZE, std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  Matrix adjoint() const noexcept {
    return *this;
  }

  Matrix inverse() noexcept {
    static constexpr T EPSILON{static_cast<T>(0.00001)};
    const T d = det();
    if (d < EPSILON && d > -EPSILON) {
      std::wcerr << "WARN: det is too small: " << d << std::endl;
      return *this;  // TODO: print warning when result is false
    }
    Matrix result = adjoint();
    result /= d;
    return result;
  }

  template <typename U>
  void clamp(U vmin, U vmax) noexcept {
    T tmp_vmin = static_cast<T>(vmin);
    T tmp_vmax = static_cast<T>(vmax);
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] /= std::clamp(data[idx], tmp_vmin, tmp_vmax);
    }
  }

  template <typename U>
  Matrix clamped(T vmin, T vmax) const noexcept {
    Matrix result(*this);
    result.clamp(vmin, vmax);
    return result;
  }
};

// template <typename T, size_t SIZE>
// inline typename std::enable_if<std::is_eq<T, int>::value, void>::type
// typed_foo(const F& f) {
//     std::cout << ">>> messing with ints! " << f << std::endl;
// }

template <typename T>
using Matrix3 = Matrix<T, 3>;

template <typename T>
inline T det3(const Matrix<T, 3>& mat) noexcept {
  return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) +
         mat[0][1] * (mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2]) +
         mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

template <typename T>
inline void left_mul3(Matrix<T, 3>& mat, const Matrix<T, 3>& other) noexcept {
  auto row_by_col = [&](size_t row, size_t col) -> float {
    return mat[row][0] * other.mat[0][col] +  //
           mat[row][1] * other.mat[1][col] +  //
           mat[row][2] * other.mat[2][col];
  };

  T result[3][3];
  result[0][0] = row_by_col(0, 0);
  result[1][0] = row_by_col(1, 0);
  result[2][0] = row_by_col(2, 0);
  result[0][1] = row_by_col(0, 1);
  result[1][1] = row_by_col(1, 1);
  result[2][1] = row_by_col(2, 1);
  result[0][2] = row_by_col(0, 2);
  result[1][2] = row_by_col(1, 2);
  result[2][2] = row_by_col(2, 2);

  mat[0][0] = result[0][0];
  mat[0][1] = result[0][1];
  mat[0][2] = result[0][2];
  mat[1][0] = result[1][0];
  mat[1][1] = result[1][1];
  mat[1][2] = result[1][2];
  mat[2][0] = result[2][0];
  mat[2][1] = result[2][1];
  mat[2][2] = result[2][2];
}

template <typename T>
Matrix<T, 3> adjoint3(const Matrix<T, 3>& mat) noexcept {
  // Calculate the cofactor matrix and transpose matrix it at the same time.
  const T m00 = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
  const T m01 = mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2];
  const T m02 = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];
  const T m10 = mat[2][1] * mat[0][2] - mat[0][1] * mat[2][2];
  const T m11 = mat[0][0] * mat[2][2] - mat[2][0] * mat[0][2];
  const T m12 = mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1];
  const T m20 = mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2];
  const T m21 = mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2];
  const T m22 = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
  return Matrix<T, 3>(
    {m00, m10, m20, m01, m11, m21, m02, m12, m22});  // +transpose
}

// template <typename T>
// struct Matrix : public BasicSquareMatrix<T, 3> {
//   T det() const noexcept {
//     return T{};
//   }

//   Matrix& operator*=(const Matrix<T, 3>& other) noexcept {
//     return *this;
//   }

//   Matrix adjoint() const noexcept {
//     return *this;
//   }
// };

}  // namespace simple2

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator+(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result += rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator-(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result -= rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator*(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result *= rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE> operator*(const simple2::Matrix<T, SIZE>& mat,
                                   T s) noexcept {
  simple2::Matrix<T, SIZE> result(mat);
  result *= s;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE> operator/(const simple2::Matrix<T, SIZE>& mat,
                                   U s) noexcept {
  simple2::Matrix<T, SIZE> result(mat);
  result /= s;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator/(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result /= rhs;
  return result;
}