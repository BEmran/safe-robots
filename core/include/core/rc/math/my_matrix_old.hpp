#pragma once

#include <memory>
#include "my_vector.hpp"
#include <array>
#include <cassert>
#include <ostream>

template <typename T, std::size_t ROWS, std::size_t COLS>
class RCMatrix : public RCVector<RCVector<T, COLS>, ROWS> {
 public:
  // RCVector<T, COLS>& operator[](size_t idx) {
  //   return m_data[idx];
  // }

  // RCVector<T, COLS> operator[](size_t idx) const {
  //   return m_data[idx];
  // }

  static RCMatrix zeros() {
    RCMatrix<T, COLS, ROWS> data;
    for (size_t r = 0; r < ROWS; ++r) {
      data[r].fill(0.0);
    }
    return data;
  }

  static RCMatrix ones() {
    RCMatrix<T, COLS, ROWS> data;
    for (size_t r = 0; r < ROWS; ++r) {
      data[r].fill(1.0);
    }
    return data;
  }

  static RCMatrix random() {
    RCMatrix<T, COLS, ROWS> data;
    for (size_t r = 0; r < ROWS; ++r) {
      data[r].randomize();
    }
    return data;
  }

  size_t rows() const {
    return this->size();
  }

  size_t cols() const {
    return this->at(0).size();
  }

  // auto col(size_t idx) {
  //   RCVector<std::reference_wrapper<T>, ROWS> res;
  //   for (auto r=0; r < this->size(); ++r) {
  //     res.at(idx) = this->at(r).at(idx);
  //   }
  //   return res;
  // }

  RCVector<T, ROWS> col(size_t idx) const {
    RCVector<T, ROWS> res;
    std::transform(this->cbegin(), this->cend(), res.begin(),
                   [idx](auto vec) { return vec.at(idx); });
    return res;
  }

  RCVector<T, ROWS>& row(size_t idx) {
    return this->at(idx);
  }

  RCVector<T, ROWS> row(size_t idx) const {
    return this->at(idx);
  }

  std::string print() const {
    std::string str;
    str += '[';
    for (size_t r = 0; r < ROWS; ++r) {
      if (r > 0) {
        str += " ";
      }
      str += this->at(r).print();
      if (r < ROWS - 1) {
        str += "\n";
      }
    }
    str += ']';
    return str;
  }
};

// template <std::size_t ROWS, std::size_t COLS, std::size_t COLS2>
// RCMatrix<ROWS, COLS> operator*(RCMatrix<ROWS, COLS> A, RCMatrix<COLS, COLS2>
// B) {
//   RCMatrix<ROWS, COLS2> C;
//   // go through columns of B calculating columns of C left to right
// 	for(auto c=0; c<B.cols());i++){
// 		// put column of B in sequential memory slot
// 		for(j=0;j<B.rows;j++) {
//       tmp[j]=B.d[j][i];
//     }
// 		// calculate each row in column i
// 		for(j=0;j<(A.rows);j++){
// 			C->d[j][i]=__vectorized_mult_accumulate(A.d[j],tmp,B.rows);
// 		}
// 	}
//   return RCMatrix<ROWS, COLS>::ones();
// }