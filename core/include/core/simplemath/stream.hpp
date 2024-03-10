#pragma once

#include "core/simplemath/basic.hpp"
#include "core/simplemath/quaternion.hpp"
#include "core/simplemath/vector2.hpp"
#include "core/simplemath/vector3.hpp"
#include "core/simplemath/matrix2x2.hpp"
#include "core/simplemath/matrix3x3.hpp"

#include <sstream>
#include <ostream>

template <typename T>
std::ostream& operator<<(std::ostream& os, const Point2D<T>& p) {
  return os << "{x = " << p.x << ","   //
            << " y = " << p.y << "}";  //
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Point3D<T>& p) {
  return os << "{x = " << p.x << ","   //
            << " y = " << p.y << ","   //
            << " z = " << p.z << "}";  //
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const MatrixElements2D<T>& e) {
  return os << "{m00 = " << e.m00 << ","  //
            << " m01 = " << e.m01 << ","  //
            << " m10 = " << e.m10 << ","  //
            << " m11 = " << e.m11 << "}";
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const MatrixElements3D<T>& e) {
  return os << "{m00 = " << e.m00 << ","  //
            << " m01 = " << e.m01 << ","  //
            << " m02 = " << e.m02 << ","  //
            << " m10 = " << e.m10 << ","  //
            << " m11 = " << e.m11 << ","  //
            << " m12 = " << e.m12 << ","  //
            << " m20 = " << e.m20 << ","  //
            << " m21 = " << e.m21 << ","  //
            << " m22 = " << e.m22 << "}";
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicVector2<T>& vec) {
  os << "[" << vec.x() << ", " << vec.y() << "]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicVector3<T>& vec) {
  os << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicMatrix2x2<T>& mat) {
  os << "[[" << mat.at(0) << ", " << mat.at(1) << "]\n"
     << " [" << mat.at(2) << ", " << mat.at(3) << "]]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicMatrix3x3<T>& mat) {
  os << "[[" << mat.at(0) << ", " << mat.at(1) << ", " << mat.at(2) << "]\n"
     << " [" << mat.at(3) << ", " << mat.at(4) << ", " << mat.at(5) << "]\n"
     << " [" << mat.at(6) << ", " << mat.at(8) << ", " << mat.at(8) << "]]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Matrix2x2<T>& mat) {
  os << "[[" << mat.at(0) << ", " << mat.at(1) << "]\n"
     << " [" << mat.at(2) << ", " << mat.at(3) << "]]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Matrix3x3<T>& mat) {
  os << "[[" << mat.at(0) << ", " << mat.at(1) << ", " << mat.at(2) << "]\n"
     << " [" << mat.at(3) << ", " << mat.at(4) << ", " << mat.at(5) << "]\n"
     << " [" << mat.at(6) << ", " << mat.at(7) << ", " << mat.at(8) << "]]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Quaternion<T>& quat) {
  os << "{w = " << quat.scalar() << ", vec = " << quat.vec() << "}";
  return os;
}