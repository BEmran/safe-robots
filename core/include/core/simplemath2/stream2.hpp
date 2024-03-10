#pragma once

#include "core/simplemath2/basic2.hpp"
#include "core/simplemath2/matrix.hpp"

#include <sstream>
#include <ostream>

template <typename T, size_t SIZE>
std::ostream& operator<<(std::ostream& os, const simple2::BasicSquareMatrix<T, SIZE>& mat) {
  os << "[";
  for (size_t r = 0; r < mat.dim(); ++r) {
    os << "[";
    for (size_t c = 0; c < mat.dim(); ++c) {
      os << mat.at(r, c);
      if (c < mat.dim() - 1) {
        os << ",";
      }
    }
    if (r < mat.dim() - 1) {
      os << "\n";
    }
  }
  os << "]";

  return os;
}