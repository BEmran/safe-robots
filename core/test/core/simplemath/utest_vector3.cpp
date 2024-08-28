// Copyright (C) 2024 Bara Emran - All Rights Reserved

#include "core/simplemath/vector3.hpp"
#include "utils.hpp"

#define START_IGNORE_SELF_ASSIGN_WARNING                                       \
  _Pragma("GCC diagnostic push")                                               \
    _Pragma("GCC diagnostic ignored \"-Wself-assign-overloaded\"")

#define STOP_IGNORE_SELF_ASSIGN_WARNING _Pragma("GCC diagnostic pop")

TEST(Vector3, DefaultConstructor) {
  Vector3<float> vec;
  EXPECT_TRUE(expect_near(vec, Vector3<float>(0, 0, 0)));
}

TEST(Vector3, ConstructorWithSingleValue) {
  const float c{1.f};
  Vector3<float> vec(c);
  EXPECT_TRUE(expect_near(vec, Vector3<float>(c, c, c)));
}

TEST(Vector3, ConstructorWithThreeValues) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(expect_near(vec, Vector3<float>(x, y, z)));
}

TEST(Vector3, ConstructorWithArray) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  std::array<float, 3> array = {x, y, z};
  Vector3<float> vec(array);
  EXPECT_TRUE(expect_near(vec, Vector3<float>(x, y, z)));
}

TEST(Vector3, ConstructZerosVector) {
  EXPECT_TRUE(
    expect_near(Vector3<float>::zeros(), Vector3<float>(0.f, 0.f, 0.f)));
}

TEST(Vector3, ConstructOnesVector) {
  EXPECT_TRUE(
    expect_near(Vector3<float>::ones(), Vector3<float>(1.f, 1.f, 1.f)));
}

TEST(Vector3, AddInplace) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  vec += vec;
  EXPECT_TRUE(expect_near(vec, Vector3<float>(x * 2, y * 2, z * 2)));
}

TEST(Vector3, Add) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  Vector3<float> result = vec + vec;
  EXPECT_TRUE(expect_near(result, Vector3<float>(x * 2, y * 2, z * 2)));
}

TEST(Vector3, SubtractInplace) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  START_IGNORE_SELF_ASSIGN_WARNING
  vec -= vec;
  STOP_IGNORE_SELF_ASSIGN_WARNING
  EXPECT_TRUE(expect_near(vec, Vector3<float>(0.f, 0.f, 0.f)));
}

TEST(Vector3, Subtract) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  Vector3<float> result = vec - vec;
  EXPECT_TRUE(expect_near(result, Vector3<float>(0.f, 0.f, 0.f)));
}

TEST(Vector3, MultiplyByConstantInplace) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  const float s{4.f};
  Vector3<float> vec(x, y, z);
  vec *= s;
  EXPECT_TRUE(expect_near(vec, Vector3<float>(x * s, y * s, z * s)));
}

TEST(Vector3, MultiplyByConstant) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  const float s{4.f};
  Vector3<float> vec(x, y, z);
  Vector3<float> result = vec * s;
  EXPECT_TRUE(expect_near(result, Vector3<float>(x * s, y * s, z * s)));
}

TEST(Vector3, MultiplyByVectorInplace) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  vec *= vec;
  EXPECT_TRUE(expect_near(vec, Vector3<float>(x * x, y * y, z * z)));
}

TEST(Vector3, MultiplyByVector) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  Vector3<float> result = vec * vec;
  EXPECT_TRUE(expect_near(result, Vector3<float>(x * x, y * y, z * z)));
}

TEST(Vector3, DotWithSelf) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  EXPECT_FLOAT_EQ(vec.dot(vec), x * x + y * y + z * z);
}

TEST(Vector3, DotWithOther) {
  const float x{1.f};
  const float y{2.f};
  Vector3<float> vec1(x);
  Vector3<float> vec2(y);
  EXPECT_FLOAT_EQ(vec1.dot(vec2), x * y * 3);
}

TEST(Vector3, Sum) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  EXPECT_FLOAT_EQ(vec.sum(), x + y + z);
}

TEST(Vector3, Norm) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  Vector3<float> vec(x, y, z);
  EXPECT_FLOAT_EQ(vec.norm(), std::sqrt(x * x + y * y + z * z));
}

TEST(Vector3, Normalize) {
  const float x{3.f};
  const float y{4.f};
  const float z{3.f};
  const float n = std::sqrt(x * x + y * y + z * z);
  Vector3<float> vec(x, y, z);
  vec.normalize();
  EXPECT_TRUE(expect_near(vec, Vector3<float>(x / n, y / n, z / n)));
}

TEST(Vector3, NormalizedTest1) {
  const float x{2.f};
  const float y{3.f};
  const float z{6.f};
  const float n = std::sqrt(x * x + y * y + z * z);
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(
    expect_near(vec.normalized(), Vector3<float>(x / n, y / n, z / n)));
}

TEST(Vector3, NormalizedTest2) {
  const float x{1.f};
  const float y{-1.f};
  const float z{-1.f};
  const float inv_sqrt3 = static_cast<float>(1.0 / std::sqrt(3));
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(expect_near(
    vec.normalized(), Vector3<float>(inv_sqrt3, -inv_sqrt3, -inv_sqrt3)));
}

TEST(Vector3, NormalizedWithZeroNorm) {
  const float x{0.01f};
  const float y{0.f};
  const float z{0.f};
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(expect_near(vec.normalized(), Vector3<float>(1, 0, 0)));
}

TEST(Vector3, ClampOverValue) {
  const float x{3.f};
  const float y{-4.f};
  const float z{3.f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector3<float> vec(x, y, z);
  vec.clamp(vmin, vmax);
  EXPECT_TRUE(expect_near(vec, Vector3<float>(vmax, vmin, vmax)));
}

TEST(Vector3, ClampUnderValue) {
  const float x{0.5f};
  const float y{-.5f};
  const float z{0.1f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector3<float> vec(x, y, z);
  vec.clamp(vmin, vmax);
  EXPECT_TRUE(expect_near(vec, vec));
}

TEST(Vector3, ClampedOverValue) {
  const float x{2.f};
  const float y{2.f};
  const float z{3.f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(expect_near(vec.clamped(vmin, vmax),
                                 Vector3<float>(vmax, vmax, vmax)));
}

TEST(Vector3, ClampedUnderValue) {
  const float x{0.1f};
  const float y{-.2f};
  const float z{0.2f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(expect_near(vec.clamped(vmin, vmax), vec));
}

TEST(Vector3, ClampWithVector) {
  const float x{5.f};
  const float y{2.f};
  const float z{-1.f};
  const Vector3<float> vmin(-1.f, -2.f, -3.f);
  const Vector3<float> vmax(1.f);
  Vector3<float> vec(x, y, z);
  vec.clamp(vmin, vmax);
  EXPECT_TRUE(expect_near(vec, Vector3<float>(vmax.at(0), vmax.at(1), z)));
}

TEST(Vector3, ClampedWithVector) {
  const float x{0.1f};
  const float y{-.2f};
  const float z{0.2f};
  const Vector3<float> vmin(-1.f, -2.f, -3.f);
  const Vector3<float> vmax(1.f);
  Vector3<float> vec(x, y, z);
  EXPECT_TRUE(expect_near(vec.clamped(vmin, vmax), vec));
}
