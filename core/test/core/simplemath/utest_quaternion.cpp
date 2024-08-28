// Copyright (C) 2024 Bara Emran - All Rights Reserved

#include "core/simplemath/quaternion.hpp"
#include "utils.hpp"

#define START_IGNORE_SELF_ASSIGN_WARNING                                       \
  _Pragma("GCC diagnostic push")                                               \
    _Pragma("GCC diagnostic ignored \"-Wself-assign-overloaded\"")

#define STOP_IGNORE_SELF_ASSIGN_WARNING _Pragma("GCC diagnostic pop")

TEST(Quaternion, DefaultConstructor) {
  Quaternion<float> quat;
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(1, 0, 0, 0)));
}

TEST(Quaternion, ConstructorWithValues) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(w, x, y, z)));
}

TEST(Quaternion, ConstructorWithArray) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  std::array<float, 4> array = {w, x, y, z};
  Quaternion<float> quat(array);
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(w, x, y, z)));
}

TEST(Quaternion, ConstructWithAngAndVec) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(
    expect_near(Quaternion<float>(w, vec), Quaternion<float>(w, x, y, z)));
}

TEST(Quaternion, AddInplace) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  quat += quat;
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(w * 2, x * 2, y * 2, z * 2)));
}

TEST(Quaternion, Add) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  Quaternion<float> result = quat + quat;
  EXPECT_TRUE(
    expect_near(result, Quaternion<float>(w * 2, x * 2, y * 2, z * 2)));
}

TEST(Quaternion, SubtractInplace) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  START_IGNORE_SELF_ASSIGN_WARNING
  quat -= quat;
  STOP_IGNORE_SELF_ASSIGN_WARNING
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(Quaternion, Subtract) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  Quaternion<float> result = quat - quat;
  EXPECT_TRUE(expect_near(result, Quaternion<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(Quaternion, MultiplyByConstantInplace) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  const float s{4.f};
  Quaternion<float> quat(w, x, y, z);
  quat *= s;
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(w * s, x * s, y * s, z * s)));
}

TEST(Quaternion, MultiplyByConstant) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  const float s{4.f};
  Quaternion<float> quat(w, x, y, z);
  Quaternion<float> result = quat * s;
  EXPECT_TRUE(
    expect_near(result, Quaternion<float>(w * s, x * s, y * s, z * s)));
}

TEST(Quaternion, DevidByConstantInplace) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  const float s{4.f};
  Quaternion<float> quat(w, x, y, z);
  quat /= s;
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(w / s, x / s, y / s, z / s)));
}

// TEST(Quaternion, MultiplyByQuaternionInplace) {
//   const float w{0.5f};
//   const float x{1.f};
//   const float y{2.f};
//   const float z{3.f};
//   Quaternion<float> quat(w, x, y, z);
//   quat *= quat;
//   EXPECT_TRUE(expect_near(quat, Quaternion<float>(w*w, x * x, y * y, z *
//   z)));
// }

// TEST(Quaternion, MultiplyByQuaternion) {
//   const float w{0.5f};
//   const float x{1.f};
//   const float y{2.f};
//   const float z{3.f};
//   Quaternion<float> quat(w, x, y, z);
//   Quaternion<float> result = quat * quat;
//   EXPECT_TRUE(expect_near(result, Quaternion<float>(w*w, x * x, y * y, z *
//   z)));
// }

TEST(Quaternion, ToMatrix) {
  Quaternion<float> quat(0.5f, -0.5f, -0.5f, -0.5f);
  Matrix3x3<float> matrix(0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f);
  EXPECT_TRUE(expect_near(quat.matrix(), matrix));
}

TEST(Quaternion, ToVec) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  EXPECT_TRUE(expect_near(quat.vec(), Vector3<float>(x, y, z)));
}

TEST(Quaternion, RotateVector) {
  Quaternion<float> quat(0.5f, -0.5f, -0.5f, -0.5f);
  Vector3 vec(1.f, 2.f, 3.f);
  Vector3<float> result = rotate(quat, vec);
  EXPECT_TRUE(expect_near(result, quat.matrix() * vec));
}

// TEST(Quaternion, DotWithSelf) {
//   const float x{1.f};
//   const float y{2.f};
//   const float z{3.f};
//   Quaternion<float> quat(x, y, z);
//   EXPECT_FLOAT_EQ(quat.dot(quat), x * x + y * y + z * z);
// }

// TEST(Quaternion, DotWithOther) {
//   const float x{1.f};
//   const float y{2.f};
//   Quaternion<float> quat1(x);
//   Quaternion<float> quat2(y);
//   EXPECT_FLOAT_EQ(quat1.dot(quat2), x * y * 3);
// }

TEST(Quaternion, SquaredNorm) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  EXPECT_FLOAT_EQ(quat.squared_norm(), w * w + x * x + y * y + z * z);
}

TEST(Quaternion, Norm) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  EXPECT_FLOAT_EQ(quat.norm(), std::sqrt(w * w + x * x + y * y + z * z));
}

// TEST(Quaternion, Normalize) {
//   const float x{3.f};
//   const float y{4.f};
//   const float z{3.f};
//   const float n = std::sqrt(x * x + y * y + z * z);
//   Quaternion<float> quat(x, y, z);
//   quat.normalize();
//   EXPECT_TRUE(expect_near(quat, Quaternion<float>(x / n, y / n, z / n)));
// }

// TEST(Quaternion, NormalizedTest1) {
//   const float x{2.f};
//   const float y{3.f};
//   const float z{6.f};
//   const float n = std::sqrt(x * x + y * y + z * z);
//   Quaternion<float> quat(x, y, z);
//   EXPECT_TRUE(
//     expect_near(quat.normalized(), Quaternion<float>(x / n, y / n, z / n)));
// }

// TEST(Quaternion, NormalizedTest2) {
//   const float x{1.f};
//   const float y{-1.f};
//   const float z{-1.f};
//   const float inv_sqrt3 = static_cast<float>(1.0 / std::sqrt(3));
//   Quaternion<float> quat(x, y, z);
//   EXPECT_TRUE(expect_near(
//     quat.normalized(), Quaternion<float>(inv_sqrt3, -inv_sqrt3, -inv_sqrt3)));
// }

TEST(Quaternion, NormalizedWithZeroNorm) {
  const float w{0.01f};
  const float x{0.f};
  const float y{0.f};
  const float z{0.f};
  Quaternion<float> quat(w, x, y, z);
  EXPECT_TRUE(expect_near(quat.normalized(), Quaternion<float>(1, 0, 0, 0)));
}

TEST(Quaternion, Conjugate) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  quat.conjugate();
  EXPECT_TRUE(expect_near(quat, Quaternion<float>(w, -x, -y, -z)));
}

TEST(Quaternion, Conjugated) {
  const float w{0.5f};
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Quaternion<float> quat(w, x, y, z);
  EXPECT_TRUE(expect_near(quat.conjugated(), Quaternion<float>(w, -x, -y, -z)));
}
