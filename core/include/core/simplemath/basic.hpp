// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include <cstddef>

template <typename T>
struct Point2D {
  T x{};
  T y{};
  Point2D() = default;
  Point2D(T _x, T _y) : x(_x), y(_y){};
};

template <typename T>
struct Point3D {
  T x{};
  T y{};
  T z{};
  Point3D() = default;
  Point3D(T _x, T _y, T _z) : x(_x), y(_y), z(_z){};
};

template <typename T>
struct Point4D {
  T x{};
  T y{};
  T z{};
  T w{};
  Point4D() = default;
  Point4D(T _x, T _y, T _z, T _w) : x(_x), y(_y), z(_z), w(_w){};
};

template <typename T>
struct BasicVector2 {
  using iterator = T*;
  using const_iterator = const T*;
  static const size_t array_size{2};
  // union {
  // Point2D<T> point;
  T data[array_size] = {};
  // };

  BasicVector2() : data{T{}, T{}} {
  }

  BasicVector2(T c) noexcept : data{c, c} {
  }

  BasicVector2(T x, T y) noexcept : data{x, y} {
  }

  inline T& at(size_t idx) noexcept {
    return data[idx];
  }

  inline T at(size_t idx) const noexcept {
    return data[idx];
  }

  inline T x() const noexcept {
    return data[0];
  }

  inline T& x() noexcept {
    return data[0];
  }

  inline T y() const noexcept {
    return data[1];
  }

  inline T& y() noexcept {
    return data[1];
  }

  constexpr inline size_t size() const noexcept {
    return array_size;
  }

  constexpr iterator begin() noexcept {
    return data;
  }

  constexpr const_iterator begin() const noexcept {
    return data;
  }

  constexpr iterator end() noexcept {
    return data + array_size;
  }

  constexpr const_iterator end() const noexcept {
    return data + array_size;
  }
};

template <typename T>
struct BasicVector3 {
  using iterator = T*;
  using const_iterator = const T*;
  static const size_t array_size{3};
  // union {
  // Point3D<T> point;
  T data[array_size] = {};
  // };

  BasicVector3() : data{T{}, T{}, T{}} {
  }

  BasicVector3(T c) noexcept : data{c, c, c} {
  }

  BasicVector3(T x, T y, T z) noexcept : data{x, y, z} {
  }

  inline T& at(size_t idx) noexcept {
    return data[idx];
  }

  inline T at(size_t idx) const noexcept {
    return data[idx];
  }

  inline T x() const noexcept {
    return data[0];
  }

  inline T& x() noexcept {
    return data[0];
  }

  inline T y() const noexcept {
    return data[1];
  }

  inline T& y() noexcept {
    return data[1];
  }

  inline T z() const noexcept {
    return data[2];
  }

  inline T& z() noexcept {
    return data[2];
  }

  constexpr inline size_t size() const noexcept {
    return array_size;
  }

  constexpr iterator begin() noexcept {
    return data;
  }

  constexpr const_iterator begin() const noexcept {
    return data;
  }

  constexpr iterator end() noexcept {
    return data + array_size;
  }

  constexpr const_iterator end() const noexcept {
    return data + array_size;
  }
};

template <typename T>
struct BasicVector4 {
  using iterator = T*;
  using const_iterator = const T*;
  static const size_t array_size{4};
  // union {
  // Point3D<T> point;
  T data[array_size] = {};
  // };

  BasicVector4() : data{T{}, T{}, T{}, T{}} {
  }

  BasicVector4(T c) noexcept : data{c, c, c, c} {
  }

  BasicVector4(T w, T x, T y, T z) noexcept : data{w, x, y, z} {
  }

  inline T& at(size_t idx) noexcept {
    return data[idx];
  }

  inline T at(size_t idx) const noexcept {
    return data[idx];
  }

  inline T w() const noexcept {
    return data[0];
  }

  inline T& w() noexcept {
    return data[0];
  }

  inline T x() const noexcept {
    return data[1];
  }

  inline T& x() noexcept {
    return data[1];
  }

  inline T y() const noexcept {
    return data[2];
  }

  inline T& y() noexcept {
    return data[2];
  }

  inline T z() const noexcept {
    return data[3];
  }

  inline T& z() noexcept {
    return data[3];
  }

  constexpr inline size_t size() const noexcept {
    return array_size;
  }

  constexpr iterator begin() noexcept {
    return data;
  }

  constexpr const_iterator begin() const noexcept {
    return data;
  }

  constexpr iterator end() noexcept {
    return data + array_size;
  }

  constexpr const_iterator end() const noexcept {
    return data + array_size;
  }
};

template <typename T>
struct MatrixElements2D {
  T m00{};
  T m01{};
  T m10{};
  T m11{};

  MatrixElements2D() = default;
  MatrixElements2D(T _m00, T _m01, T _m10, T _m11)
    : m00{_m00}, m01{_m01}, m10{_m10}, m11{_m11} {
  }
};

template <typename T>
struct MatrixElements3D {
  T m00{};
  T m01{};
  T m02{};
  T m10{};
  T m11{};
  T m12{};
  T m20{};
  T m21{};
  T m22{};
  MatrixElements3D() = default;
  MatrixElements3D(T _m00, T _m01, T _m02, T _m10, T _m11, T _m12, T _m20,
                   T _m21, T _m22) noexcept
    : m00{_m00}
    , m01{_m01}
    , m02{_m02}
    , m10{_m10}
    , m11{_m11}
    , m12{_m12}
    , m20{_m20}
    , m21{_m21}
    , m22{_m22} {
  }
};

template <typename T>
struct BasicMatrix2x2 {
  using iterator = T*;
  using const_iterator = const T*;
  static const size_t num_rows{2};
  static const size_t num_cols{2};
  static const size_t array_size{num_rows * num_cols};
  union {
    // MatrixElements2D<T> elements;
    T mat[num_rows][num_cols];
    T data[array_size] = {};
  };

  BasicMatrix2x2() : BasicMatrix2x2(T{}) {
  }

  BasicMatrix2x2(T c) noexcept : data{c, c, c, c} {
  }

  BasicMatrix2x2(T m00, T m01, T m10, T m11) noexcept
    : data{m00, m01, m10, m11} {
  }

  BasicMatrix2x2(const BasicVector2<T>& row0,
                 const BasicVector2<T>& row1) noexcept
    : BasicMatrix2x2<T>(row0.x(), row0.y(), row1.x(), row1.y()) {
  }

  inline T& at(size_t row, size_t col) noexcept {
    return data[row * num_cols + col];
  }

  inline T at(size_t row, size_t col) const noexcept {
    return data[row * num_cols + col];
  }

  inline T& at(size_t idx) noexcept {
    return data[idx];
  }

  inline T at(size_t idx) const noexcept {
    return data[idx];
  }

  constexpr inline size_t size() const noexcept {
    return array_size;
  }

  constexpr inline size_t rows() const noexcept {
    return num_rows;
  }

  constexpr inline size_t cols() const noexcept {
    return num_cols;
  }

  // iterators
  constexpr iterator begin() noexcept {
    return data;
  }

  constexpr const_iterator begin() const noexcept {
    return data;
  }

  constexpr iterator end() noexcept {
    return data + array_size;
  }

  constexpr const_iterator end() const noexcept {
    return data + array_size;
  }
};

template <typename T>
struct BasicMatrix3x3 {
  using iterator = T*;
  using const_iterator = const T*;
  static const size_t num_rows{3};
  static const size_t num_cols{3};
  static const size_t array_size{num_rows * num_cols};
  union {
    // MatrixElements3D<T> elements;
    T mat[num_rows][num_cols];
    T data[array_size] = {};
  };

  BasicMatrix3x3() : BasicMatrix3x3(T{}) {
  }

  BasicMatrix3x3(T c) noexcept : data{c, c, c, c, c, c, c, c, c} {
  }

  BasicMatrix3x3(T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21,
                 T m22) noexcept
    : data{m00, m01, m02, m10, m11, m12, m20, m21, m22} {
  }

  BasicMatrix3x3(const BasicVector2<T>& row0, const BasicVector2<T>& row1,
                 const BasicVector2<T>& row2) noexcept
    : BasicMatrix3x3<T>(row0.x(), row0.y(), row0.z(), row1.x(), row1.y(),
                        row1.z(), row2.x(), row2.y(), row2.z()) {
  }

  inline T& at(size_t row, size_t col) noexcept {
    return mat[row][col];
  }

  inline T at(size_t row, size_t col) const noexcept {
    return mat[row][col];
  }

  inline T& at(size_t idx) noexcept {
    return data[idx];
  }

  inline T at(size_t idx) const noexcept {
    return data[idx];
  }

  constexpr inline size_t size() const noexcept {
    return array_size;
  }

  constexpr inline size_t rows() const noexcept {
    return num_rows;
  }

  constexpr inline size_t cols() const noexcept {
    return num_cols;
  }

  // iterators
  constexpr iterator begin() noexcept {
    return data;
  }

  constexpr const_iterator begin() const noexcept {
    return data;
  }

  constexpr iterator end() noexcept {
    return data + array_size;
  }

  constexpr const_iterator end() const noexcept {
    return data + array_size;
  }
};