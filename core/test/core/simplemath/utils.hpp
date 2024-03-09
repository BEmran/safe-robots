// Copyright (C) 2023 Bara Emran - All Rights Reserved

#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include "core/simplemath/simplemath.hpp"
#include <ostream>
#include <string>
#include <gtest/gtest.h>

constexpr float EPSILON{1e-6f};

template <typename T>
std::ostream& operator<<(std::ostream& os, const Point2D<T>& p) {
  return os << "Point2D { "              //
            << ".x = [" << p.x << "], "  //
            << ".y = [" << p.y << "]}";  //
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Point3D<T>& p) {
  return os << "Point3D { "              //
            << ".x = [" << p.x << "], "  //
            << ".y = [" << p.y << "], "  //
            << ".z = [" << p.z << "]}";  //
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicVector2<T>& v) {
  return os << "BasicVector2 { "           //
            << ".x = [" << v.x() << "], "  //
            << ".y = [" << v.y() << "]}";  //
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicVector3<T>& v) {
  return os << "BasicVector3 { "           //
            << ".x = [" << v.x() << "], "  //
            << ".y = [" << v.y() << "], "  //
            << ".z = [" << v.z() << "]}";  //
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const MatrixElements2D<T>& e) {
  return os << "MatrixElements2D { "         //
            << ".m00 = [" << e.m00 << "], "  //
            << ".m01 = [" << e.m01 << "], "  //
            << ".m10 = [" << e.m10 << "], "  //
            << ".m11 = [" << e.m11 << "]}";
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const MatrixElements3D<T>& e) {
  return os << "MatrixElements3D {"          //
            << ".m00 = [" << e.m00 << "], "  //
            << ".m01 = [" << e.m01 << "], "  //
            << ".m02 = [" << e.m02 << "], "  //
            << ".m10 = [" << e.m10 << "], "  //
            << ".m11 = [" << e.m11 << "], "  //
            << ".m12 = [" << e.m12 << "], "  //
            << ".m20 = [" << e.m20 << "], "  //
            << ".m21 = [" << e.m21 << "], "  //
            << ".m22 = [" << e.m22 << "]}";
}

template <typename T>
std::string to_string(T* array, size_t size) {
  std::stringstream ss;
  for (size_t i = 0; i < size; ++i) {
    ss << "[" << i << "]: " << array[i];
    if (i < size - 1) {
      ss << ", ";
    }
  }
  return ss.str();
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicMatrix2x2<T>& e) {
  return os << "MatrixElements2D { .data: " << to_string(e.data, e.size())
            << "}";
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicMatrix3x3<T>& e) {
  return os << "MatrixElements3D { .data: " << to_string(e.data, e.size())
            << "}";
}
/*****************************************************************************/
[[__nodiscard__]] ::testing::AssertionResult
operator&&(::testing::AssertionResult lhs, ::testing::AssertionResult rhs) {
  if (not lhs) {
    return rhs ? lhs : lhs << rhs.message();
  } else {
    return rhs;
  }
}

template <typename T>
[[__nodiscard__]] ::testing::AssertionResult expect_near(T expect, T actual,
                                                         std::string_view msg) {
  if (std::abs(expect - actual) > EPSILON) {
    return ::testing::AssertionFailure() << "\nFailure at index [" << msg << "]"
                                         << "\nThe expect values: " << expect
                                         << "\nThe actual values: " << actual;
  }
  return ::testing::AssertionSuccess();
}

template <typename T>
[[__nodiscard__]] ::testing::AssertionResult expect_near(Point2D<T> expect,
                                                         Point2D<T> actual) {
  return expect_near(expect.x, actual.x, "x") &&  //
         expect_near(expect.y, actual.y, "y");
}

template <typename T>
[[__nodiscard__]] ::testing::AssertionResult expect_near(Point3D<T> expect,
                                                         Point3D<T> actual) {
  return expect_near(expect.x, actual.x, "x") &&  //
         expect_near(expect.y, actual.y, "y") &&  //
         expect_near(expect.z, actual.z, "z");
}

template <typename T>
[[__nodiscard__]] ::testing::AssertionResult
expect_near(MatrixElements2D<T> expect, MatrixElements2D<T> actual) {
  return expect_near(expect.m00, actual.m00, "m00") &&  //
         expect_near(expect.m01, actual.m01, "m01") &&  //
         expect_near(expect.m10, actual.m10, "m10") &&  //
         expect_near(expect.m11, actual.m11, "m11");
}

template <typename T>
[[__nodiscard__]] ::testing::AssertionResult
expect_near(MatrixElements3D<T> expect, MatrixElements3D<T> actual) {
  return expect_near(expect.m00, actual.m00, "m00") &&  //
         expect_near(expect.m01, actual.m01, "m01") &&  //
         expect_near(expect.m02, actual.m02, "m02") &&  //
         expect_near(expect.m10, actual.m10, "m10") &&  //
         expect_near(expect.m11, actual.m11, "m11") &&  //
         expect_near(expect.m12, actual.m12, "m12") &&  //
         expect_near(expect.m20, actual.m20, "m20") &&  //
         expect_near(expect.m21, actual.m21, "m21") &&  //
         expect_near(expect.m22, actual.m22, "m22");
}

template <typename T>
::testing::AssertionResult expect_near(T expect, T actual) {
  auto result = ::testing::AssertionSuccess();
  for (size_t i = 0; i < expect.size(); ++i) {
    result = result && expect_near(expect.at(i), actual.at(i), std::to_string(i));
  }
  return result;
}

#endif  // TEST_UTILS_HPP
