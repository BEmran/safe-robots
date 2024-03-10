#include <iostream>
#include <ostream>
#include <iostream>
#include <array>
#include <chrono>
#include <functional>
#include "core/math/math.hpp"
#include "core/simplemath/simplemath.hpp"
#include "core/simplemath2/simplemath2.hpp"

const float AMat[9] = {0.f, -3.f, -2.f, 1.f, -4.f, -2.f, -3.f, 4.f, 1.f};
const float BMat[9] = {1.f, 2.f, 3.f, 0.f, 1.f, 4.f, 5.f, 6.f, 0.f};
const float Constant{4.f};
// const float BMat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

void simple_math() {
  // Vector<2> a;
  Matrix3x3<float> a(AMat[0], AMat[1], AMat[2], AMat[3], AMat[4], AMat[5],
                     AMat[6], AMat[7], AMat[8]);
  Matrix3x3<float> b(BMat[0], BMat[1], BMat[2], BMat[3], BMat[4], BMat[5],
                     BMat[6], BMat[7], BMat[8]);
  a *= Constant;
  b += b;
  Matrix3x3<float> c = a * b;

  c.inverse();
  Quaternion<float> q1 = Quaternion<float>::from_angle_and_axis(
                           3.14f / 2.f, Vector3<float>(0.f, 1.f, 0.f))
                           .normalized();
  Quaternion<float> q2 = Quaternion<float>::from_angle_and_axis(
                           2.f * 3.14f / 3.f, Vector3<float>(1.f, 1.f, 1.f))
                           .normalized();
  Quaternion<float> q3 =
    Quaternion<float>(-0.5f, -0.5f, -0.5f, -0.5f).normalized();
  Quaternion<float> q4;
  std::cout << "q 1:\n" << q1 << std::endl;
  std::cout << "rotation 1:\n" << q1.matrix() << std::endl;
  std::cout << "q 2:\n" << q2 << std::endl;
  std::cout << "rotation 2:\n" << q2.matrix() << std::endl;
  std::cout << "q 3:\n" << q3 << std::endl;
  std::cout << "rotation 3:\n" << q3.matrix() << std::endl;
  std::cout << "q 4:\n" << q4 << std::endl;
  std::cout << "rotation 4:\n" << q4.matrix() << std::endl;

  std::cout << "rotate 1:\n"
            << rotate(q1, Vector3<float>(1.f, 2.f, 3.f)) << std::endl;
  std::cout << "rotate 2:\n"
            << rotate(q2, Vector3<float>(1.f, 2.f, 3.f)) << std::endl;
  std::cout << "rotate 3:\n"
            << rotate(q3, Vector3<float>(1.f, 2.f, 3.f)) << std::endl;
  // std::cout << "A:\n" << a << std::endl;
  // std::cout << "B:\n" << b << std::endl;
  // std::cout << "C:\n" << c << std::endl;
  // std::cout << "det:" << c.det() << std::endl;
  // std::cout << "inv:\n" << c.inverse() << std::endl;
}

void simple2_math() {
  simple2::Matrix3<float> a(AMat[0], AMat[1], AMat[2], AMat[3], AMat[4],
                            AMat[5], AMat[6], AMat[7], AMat[8]);
  simple2::Matrix3<float> b(BMat[0], BMat[1], BMat[2], BMat[3], BMat[4],
                            BMat[5], BMat[6], BMat[7], BMat[8]);
  a *= Constant;
  b += b;
  simple2::Matrix3<float> c = a * b;
  c.inverse();

  // std::cout << "A:\n" << a << std::endl;
  // std::cout << "B:\n" << b << std::endl;
  // std::cout << "C:\n" << c << std::endl;
  // std::cout << "det:" << c.det() << std::endl;
  // std::cout << "adjoint:" << c.adjoint() << std::endl;
  // std::cout << "inv:\n" << c.inverse() << std::endl;
}

void eigen_math() {
  core::math::Mat3 A;
  core::math::Mat3 B;
  A << AMat[0], AMat[1], AMat[2], AMat[3], AMat[4], AMat[5], AMat[6], AMat[7],
    AMat[8];
  B << BMat[0], BMat[1], BMat[2], BMat[3], BMat[4], BMat[5], BMat[6], BMat[7],
    BMat[8];
  A *= Constant;
  B += B;
  core::math::Mat3 C = A * B;
  C.inverse();
  Eigen::Quaternionf q1 =
    Eigen::Quaternionf(
      Eigen::AngleAxisf(3.14f / 2.f, Eigen::Vector3f(0.f, 1.f, 0.f)))
      .normalized();
  Eigen::Quaternionf q2 =
    Eigen::Quaternionf(
      Eigen::AngleAxisf(2.f * 3.14f / 3.f, Eigen::Vector3f(1.f, 1.f, 1.f)))
      .normalized();
  Eigen::Quaternionf q3 =
    Eigen::Quaternionf(-0.5f, -0.5f, -0.5f, -0.5f).normalized();
  Eigen::Quaternionf q4(1.f, 0.f, 0.f, 0.f);
  std::cout << "q 1:\n" << q1 << std::endl;
  std::cout << "rotation 1:\n" << q1.toRotationMatrix() << std::endl;
  std::cout << "q 2:\n" << q2 << std::endl;
  std::cout << "rotation 2:\n" << q2.toRotationMatrix() << std::endl;
  std::cout << "q 3:\n" << q3 << std::endl;
  std::cout << "rotation 3:\n" << q3.toRotationMatrix() << std::endl;
  std::cout << "q 4:\n" << q4 << std::endl;
  std::cout << "rotation 4:\n" << q4.toRotationMatrix() << std::endl;

  std::cout << "rotate 1:\n"
            << q1 * Eigen::Vector3f(1.f, 2.f, 3.f) << std::endl;
  std::cout << "rotate 2:\n"
            << q2 * Eigen::Vector3f(1.f, 2.f, 3.f) << std::endl;
  std::cout << "rotate 3:\n"
            << q3 * Eigen::Vector3f(1.f, 2.f, 3.f) << std::endl;

  // std::cout << "A:\n" << A << std::endl;
  // std::cout << "B:\n" << B << std::endl;
  // std::cout << "C:\n" << C << std::endl;
  // std::cout << "det:" << C.determinant() << std::endl;
  // std::cout << "inv:\n" << C.inverse() << std::endl;
}

void test(void (*func)(void), const char* version, int iter) {
  const auto start{std::chrono::steady_clock::now()};
  for (auto i = 0; i < iter; i++) {
    func();
  }
  const auto end{std::chrono::steady_clock::now()};
  const std::chrono::duration<double> elapsed_seconds{end - start};
  std::cout << "[" << version << "] time difference " << elapsed_seconds.count()
            << " rate: " << elapsed_seconds.count() / iter << std::endl;
}

int main() {
  const int iter = 1e0;
  test(*eigen_math, "eigen_math", iter);
  test(*simple_math, "simple_math", iter);
  test(*simple2_math, "simple2_math", iter);
  return 0;
}
