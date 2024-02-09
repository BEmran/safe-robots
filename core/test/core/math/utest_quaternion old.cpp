// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

#include "core/math/math.hpp"
#include "math/quaternion.h"

using core::math::Mat3;
using core::math::MATH_TYPE;
using core::math::Quat;
using core::math::Vec3;

std::string ToString(const Quat quat) {
  std::stringstream ss;
  ss << "w: " << quat.w()   //
     << " x: " << quat.x()  //
     << " y: " << quat.y()  //
     << " z: " << quat.z();
  return ss.str();
}

std::string ToString(const rc_vector_t vec) {
  if (vec.len != 4)
    return "";
  std::stringstream ss;
  ss << "w: " << vec.d[0]   //
     << " x: " << vec.d[1]  //
     << " y: " << vec.d[2]  //
     << " z: " << vec.d[3];
  return ss.str();
}

void Print(std::string_view str, const Quat quat) {
  std::cout << str << ": " << ToString(quat) << std::endl;
}

void Print(std::string_view str, const rc_vector_t quat) {
  std::cout << str << ": " << ToString(quat) << std::endl;
}

TEST(Quaternion, test) {
  Quat q1({1, 2, 3, 4});
  q1.normalize();
  Quat q2({2, 1, 0, 1});
  q2.normalize();

  size_t L = 1000000;
  Quat ans = q1 * q2;
  // Print("q1", q1);
  // Print("q2", q2);
  // Print("ans", ans);

  rc_vector_t qa = RC_VECTOR_INITIALIZER;
  rc_vector_alloc(&qa, 4);
  qa.d[0] = 1;
  qa.d[1] = 2;
  qa.d[2] = 3;
  qa.d[3] = 4;

  rc_vector_t qb = RC_VECTOR_INITIALIZER;
  rc_vector_alloc(&qb, 4);
  qb.d[0] = 2;
  qb.d[1] = 1;
  qb.d[2] = 0;
  qb.d[3] = 1;

  const double n1 = rc_quaternion_norm(qa);
  for (int i = 0; i < 4; i++)
    qa.d[i] = qa.d[i] / n1;

  const double n2 = rc_quaternion_norm(qb);
  for (int i = 0; i < 4; i++)
    qb.d[i] = qb.d[i] / n2;

  // Print("qa", qa);
  // Print("qb", qb);
  rc_vector_t qc = RC_VECTOR_INITIALIZER;
  rc_vector_alloc(&qc, 4);

  rc_quaternion_multiply(qa, qb, &qc);

  Print("qc", qc);
  rc_vector_free(&qa);
  rc_vector_free(&qb);
  rc_vector_free(&qc);
  EXPECT_TRUE(false);
}

TEST(UnitQuaternion, Constructor) {
  const float x = 1.2f;
  const float y = 3.4f;
  const float z = 5.6f;
  const float scaler = 0.7f;
  const Vec3 vec(x, y, z);
  const Quat quat = core::utils::UnitQuaternion(scaler, vec);
  // check angle
  const float gain = x / quat.x();
  EXPECT_FLOAT_EQ(scaler, gain * quat.w());
  // check axis values
  EXPECT_FLOAT_EQ(x, gain * quat.x());
  EXPECT_FLOAT_EQ(y, gain * quat.y());
  EXPECT_FLOAT_EQ(z, gain * quat.z());
  // check vector: use cross product to prove vectors are parallel
  EXPECT_TRUE(vec.cross(quat.vec()).isZero());
}

size_t SolutionIdx(const Mat3 mat) {
  std::array<float, 4> u = {mat(0, 0) + mat(1, 1) + mat(2, 2),  //
                            mat(0, 0),                          //
                            mat(1, 1),                          //
                            mat(2, 2)};
  const auto ptr = std::max_element(u.begin(), u.end());
  const size_t max_idx = std::distance(u.begin(), ptr);
}

Quat FirstSolution(const Mat3 mat) {
  const float gain = std::sqrt(1                        //
                               + mat(0, 0) * mat(0, 0)  //
                               + mat(1, 1) * mat(1, 1)  //
                               + mat(2, 2) * mat(2, 2));
  const float scaler = gain;
  const float x = (mat(2, 1) - mat(1, 2)) / gain;
  const float y = (mat(0, 2) - mat(2, 0)) / gain;
  const float z = (mat(1, 0) - mat(0, 1)) / gain;
  const Vec3 vec(x, y, z);
  return core::utils::UnitQuaternion(scaler, vec);
}

Quat SecondSolution(const Mat3 mat) {
  const float gain = std::sqrt(1                        //
                               + mat(0, 0) * mat(0, 0)  //
                               - mat(1, 1) * mat(1, 1)  //
                               - mat(2, 2) * mat(2, 2));
  const float scaler = (mat(2, 1) - mat(1, 2)) / gain;
  const float x = gain;
  const float y = (mat(0, 1) + mat(1, 0)) / gain;
  const float z = (mat(2, 0) + mat(0, 2)) / gain;
  const Vec3 vec(x, y, z);
  return core::utils::UnitQuaternion(scaler, vec);
}

Quat ThirdSolution(const Mat3 mat) {
  const float gain = std::sqrt(1                        //
                               - mat(0, 0) * mat(0, 0)  //
                               + mat(1, 1) * mat(1, 1)  //
                               - mat(2, 2) * mat(2, 2));
  const float scaler = (mat(0, 2) - mat(2, 0)) / gain;
  const float x = (mat(0, 1) + mat(1, 0)) / gain;
  const float y = gain;
  const float z = (mat(1, 2) + mat(2, 1)) / gain;
  const Vec3 vec(x, y, z);
  return core::utils::UnitQuaternion(scaler, vec);
}

Quat FourthSolution(const Mat3 mat) {
  const float gain = std::sqrt(1                        //
                               - mat(0, 0) * mat(0, 0)  //
                               - mat(1, 1) * mat(1, 1)  //
                               + mat(2, 2) * mat(2, 2));
  const float scaler = (mat(1, 0) - mat(0, 1)) / gain;
  const float x = (mat(2, 0) + mat(0, 2)) / gain;
  const float y = (mat(2, 1) + mat(1, 2)) / gain;
  const float z = gain;
  const Vec3 vec(x, y, z);
  return core::utils::UnitQuaternion(scaler, vec);
}

Quat ToQuaternion(const Mat3 mat) {
  switch (SolutionIdx(mat)) {
    case (0):
      return FirstSolution(mat);
    case (1):
      return SecondSolution(mat);
    case (2):
      return ThirdSolution(mat);
    case (3):
      return FourthSolution(mat);
    default:
  }
}

TEST(Quaternion, Multiplication) {
  Quat q1, q2;
  q1.UnitRandom();
  q2.UnitRandom();
  Vec3 a;

  const auto rotation = q1.toRotationMatrix() * q2.toRotationMatrix();
  const Quat quat = rotation.
}