#pragma once

#include "basic.hpp"
#include "vector.hpp"
#include "mat2x2.hpp"
#include "mat3x3.hpp"

#include <sstream>
#include <ostream>

template<typename T>
std::ostream& operator<<(std::ostream& os, const VF2<T>& vec) {
  os << "[" << vec.x << ", " << vec.y << "]";
  return os;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const VF3<T>& vec) {
  os << "[" << vec.x << ", " << vec.y << ", " << vec.z << "]";
  return os;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const MF2X2<T>& mat) {
  os << "[" << mat.row[0] << "\n " << mat.row[1] << "]";
  return os;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const MF3X3<T>& mat) {
  os << "[" << mat.row[0] << "\n " << mat.row[1] << "\n " << mat.row[2] << "]";
  return os;
}