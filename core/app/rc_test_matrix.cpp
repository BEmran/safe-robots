#include <stdio.h>
#include "core/rc/math/matrix.h"
#include "core/rc/math/my_vector.hpp"
#include "core/rc/math/my_matrix.hpp"
#include <ostream>

#include "core/utils/timer.hpp"
#include "core/math/math.hpp"
#include "core/simplemath/simplemath.hpp"

#define DIM 3  // dimension of matrix to test
const float AMat[9] = {1.2f, 3.4f, 5.6f, 7.8f, 9.0f, 1.9f, 2.8f, 3.7f, 4.6f};
const float BMat[9] = {4.6f, 3.7f, 2.8f, 1.9f, 9.0f, 7.8f, 5.6f, 3.4f, 1.2f};

std::ostream& operator<<(std::ostream& os, rc_matrix_t* mat) {
  os << "[[" << mat->d[0][0] << ", " << mat->d[0][1] << ", " << mat->d[0][2] << "]";
  os << "\n [" << mat->d[1][0] << ", " << mat->d[1][1] << ", " << mat->d[1][2] << "]";
  os << "\n [" << mat->d[2][0] << ", " << mat->d[2][1] << ", " << mat->d[2][2] << "]]";
  return os;
}

void rc_math() {
  rc_matrix_t A = RC_MATRIX_INITIALIZER;
  rc_matrix_t B = RC_MATRIX_INITIALIZER;
  rc_matrix_zeros(&A, DIM, DIM);
  rc_matrix_zeros(&B, DIM, DIM);
  for(size_t i=0; i<3; ++i) {
    for(size_t j=0; j<3; ++j) {
      A.d[i][j]=static_cast<double>(AMat[i * 3 + j]);
      B.d[i][j]=static_cast<double>(BMat[i * 3 + j]);
    }
  }
  rc_matrix_t C = RC_MATRIX_INITIALIZER;
  // random matrix
  // rc_matrix_zeros(&A, DIM, DIM);
  // rc_matrix_zeros(&B, DIM, DIM);
  // rc_matrix_random(&A, DIM, DIM);
  // rc_matrix_random(&B, DIM, DIM);
  rc_matrix_multiply(A, B, &C);
  // std::cout << "A:\n" << &A << std::endl;
  // std::cout << "B:\n" << &B << std::endl;
  // std::cout << "C:\n" << &C << std::endl;
}

void eigen_math() {
  core::math::Mat3 A = core::math::Mat3::Zero();
  core::math::Mat3 B = core::math::Mat3::Zero();
  A << AMat[0], AMat[1], AMat[2], AMat[3], AMat[4], AMat[5], AMat[6], AMat[7], AMat[8];
  B << BMat[0], BMat[1], BMat[2], BMat[3], BMat[4], BMat[5], BMat[6], BMat[7], BMat[8];
  core::math::Mat3 C = A * B;
  (void)C;
  // std::cout << "A:\n" << A << std::endl;
  // std::cout << "B:\n" << B << std::endl;
  // std::cout << "C:\n" << C << std::endl;
}

void my_math() {
  Matrix<double, 3, 3> A = Matrix<double, 3, 3>::zeros();
  Matrix<double, 3, 3> B = Matrix<double, 3, 3>::zeros();
  for(size_t i=0; i<3; ++i) {
    for(size_t j=0; j<3; ++j) {
      A.at(i, j)=AMat[i * 3 + j];
      B.at(i, j)=BMat[i * 3 + j];
    }
  }
  Matrix<double, 3, 3> C = A.mul(B);
  (void)C;
  // std::cout << "A:\n" << A.print() << std::endl;
  // std::cout << "B:\n" << B.print() << std::endl;
  // std::cout << "C:\n" << C.print() << std::endl;
}

void simple_math() {
  Mat3x3<float> A = Mat3x3<float>::eye();
  Mat3x3<float> B = Mat3x3<float>::eye();
  for(size_t i=0; i<9; ++i) {
      A[i]=AMat[i];
      B[i]=BMat[i];
  }
  Mat3x3<float> C = A * B;
  (void)C;
  // std::cout << "A:\n" << A << std::endl;
  // std::cout << "B:\n" << B << std::endl;
  // std::cout << "C:\n" << C << std::endl;
}

void test(void (*func)(void), const char* version) {
  const auto t0 = core::utils::TimeInSeconds();
  int imax = 10000;
  for (auto i = 0; i < imax; i++) {
    func();
  }
  const auto t1 = core::utils::TimeInSeconds();
  printf("[%s] time difference %f\n", version, t1 - t0);
}

int main() {
  test(&rc_math, "RC");
  test(&my_math, "Mine");
  test(&eigen_math, "Eigen");
  test(&simple_math, "SIMPLE");
  Mat2x2<float> A = Mat2x2<float>::random(-1, 1);

  std::cout << "A:\n" << A << std::endl;
  std::cout << "transpose:\n" << A.transposed() << std::endl;
  std::cout << "det:\n" << Vector3<float>(3.0,4.0,0.0).norm() << std::endl;


  BasicVec3<float>aa();
  BasicVec3<float>bb(BasicVec3<float>);
  BasicVec3<float>cc(11);
  BasicVec3<float>dd(BasicVec3<float>(1));
  BasicVec3<float>ee(22,33,44);
  std::cout << aa << std::endl;
  std::cout << bb << std::endl;
  std::cout << cc << std::endl;
  std::cout << dd << std::endl;
  std::cout << ee << std::endl;
  
  Vector3<float>a (1,2,3);
  const Vector3<float>b (4,5,6);
  std::cout << a << std::endl;
  std::cout << Vector3<float>(a) << std::endl;
  std::cout << Vector3<float>(b) << std::endl;
  std::cout << Vector3<float>() << std::endl;
  std::cout << Vector3<float>(7) << std::endl;
  std::cout << Vector3<float>(std::array<float,3>{7,8,9}) << std::endl;
  std::cout << Vector3<float>(BasicVec3<float>{7,8,9}) << std::endl;

  // std::cout << "transpose:\n" << A.det()<< std::endl;
  // auto c = Vector<double, 4>::ones();
  // std::cout << "ones: " << c.print() << std::endl;
  // auto d = Vector<double, 4>::zeros();
  // std::cout << "zeros: " << d.print() << std::endl;
  // // Vector a{1.1, 1.2, 1.3, 1.4};
  // // std::cout << "list: " << a.print() << std::endl;
  // Vector<double, 4> b = Vector<double, 4>::random();
  // std::cout << "random: " << b.print() << std::endl;
  // // std::cout << "dot: " << a.dot(b).print() << std::endl;
  // // std::cout << "mul: " << a.mul(b) << std::endl;

  // Matrix<double, 3, 3> A = Matrix<double, 3, 3>::ones();
  // std::cout << "ones:\n" << A.print() << std::endl;
  // Matrix<double, 3, 3> B = Matrix<double, 3, 3>::zeros();
  // std::cout << "zeros:\n" << B.print() << std::endl;

  // Matrix<double, 3, 3> C = Matrix<double, 3, 3>::random();
  // std::cout << "random:\n" << C.print() << std::endl;
  // std::cout << "random.col[0]: " << C.col(0).print() << std::endl;
  // std::cout << "random.col[1]: " << C.col(1).print() << std::endl;
  // std::cout << "random.col[2]: " << C.col(2).print() << std::endl;
  // std::cout << "random.row[0]: " << C.row(0).print() << std::endl;
  // std::cout << "random.row[1]: " << C.row(1).print() << std::endl;
  // std::cout << "random.row[2]: " << C.row(2).print() << std::endl;

  // Matrix<double, 3, 3> D = Matrix<double, 3, 3>::random();
  // std::cout << "C*D: " << C.mul(D).print() << std::endl;
  // std::cout << "C.*D: " << C.dot(D).print() << std::endl;

  // Vector<double, 3> e = Vector<double, 3>::zeros();
  // e[0] = 1.0;
  // std::cout << "C*e:\n" << C.mul(e).print() << std::endl;
  return 0;
}
