// Copyright (C) 2024 Bara Emran - All Rights Reserved

#include "core/simplemath/vector2.hpp"
#include "utils.hpp"

#define START_IGNORE_SELF_ASSIGN_WARNING                                       \
  _Pragma("GCC diagnostic push")                                               \
    _Pragma("GCC diagnostic ignored \"-Wself-assign-overloaded\"")

#define STOP_IGNORE_SELF_ASSIGN_WARNING _Pragma("GCC diagnostic pop")

TEST(Vector2, DefaultConstructor) {
  Vector2<float> vec;
  EXPECT_TRUE(expect_near(vec, Vector2<float>(0, 0)));
}

TEST(Vector2, ConstructorWithSingleValue) {
  const float c{1.f};
  Vector2<float> vec(c);
  EXPECT_TRUE(expect_near(vec, Vector2<float>(c, c)));
}

TEST(Vector2, ConstructorWithThreeValues) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  EXPECT_TRUE(expect_near(vec, Vector2<float>(x, y)));
}

TEST(Vector2, ConstructorWithArray) {
  const float x{1.f};
  const float y{2.f};
  std::array<float, 2> array = {x, y};
  Vector2<float> vec(array);
  EXPECT_TRUE(expect_near(vec, Vector2<float>(x, y)));
}

TEST(Vector2, ConstructZerosVector) {
  EXPECT_TRUE(expect_near(Vector2<float>::zeros(), Vector2<float>(0.f, 0.f)));
}

TEST(Vector2, ConstructOnesVector) {
  EXPECT_TRUE(expect_near(Vector2<float>::ones(), Vector2<float>(1.f, 1.f)));
}

TEST(Vector2, AddInplace) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  vec += vec;
  EXPECT_TRUE(expect_near(vec, Vector2<float>(x * 2, y * 2)));
}

TEST(Vector2, Add) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  Vector2<float> result = vec + vec;
  EXPECT_TRUE(expect_near(result, Vector2<float>(x * 2, y * 2)));
}

TEST(Vector2, SubtractInplace) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  START_IGNORE_SELF_ASSIGN_WARNING
  vec -= vec;
  STOP_IGNORE_SELF_ASSIGN_WARNING
  EXPECT_TRUE(expect_near(vec, Vector2<float>(0.f, 0.f)));
}

TEST(Vector2, Subtract) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  Vector2<float> result = vec - vec;
  EXPECT_TRUE(expect_near(result, Vector2<float>(0.f, 0.f)));
}

TEST(Vector2, MultiplyByConstantInplace) {
  const float x{1.f};
  const float y{2.f};
  const float s{3.f};
  Vector2<float> vec(x, y);
  vec *= s;
  EXPECT_TRUE(expect_near(vec, Vector2<float>(x * s, y * s)));
}

TEST(Vector2, MultiplyByConstant) {
  const float x{1.f};
  const float y{2.f};
  const float s{3.f};
  Vector2<float> vec(x, y);
  Vector2<float> result = vec * s;
  EXPECT_TRUE(expect_near(result, Vector2<float>(x * s, y * s)));
}

TEST(Vector2, MultiplyByVectorInplace) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  vec *= vec;
  EXPECT_TRUE(expect_near(vec, Vector2<float>(x * x, y * y)));
}

TEST(Vector2, MultiplyByVector) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  Vector2<float> result = vec * vec;
  EXPECT_TRUE(expect_near(result, Vector2<float>(x * x, y * y)));
}

TEST(Vector2, DotWithSelf) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  EXPECT_FLOAT_EQ(vec.dot(vec), x * x + y * y);
}

TEST(Vector2, DotWithOther) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec1(x);
  Vector2<float> vec2(y);
  EXPECT_FLOAT_EQ(vec1.dot(vec2), x * y * 2);
}

TEST(Vector2, Sum) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  EXPECT_FLOAT_EQ(vec.sum(), x + y);
}

TEST(Vector2, Norm) {
  const float x{1.f};
  const float y{2.f};
  Vector2<float> vec(x, y);
  EXPECT_FLOAT_EQ(vec.norm(), std::sqrt(x * x + y * y));
}

TEST(Vector2, Normalize) {
  const float x{3.f};
  const float y{4.f};
  Vector2<float> vec(x, y);
  vec.normalize();
  EXPECT_TRUE(expect_near(vec, Vector2<float>(0.6f, 0.8f)));
}

TEST(Vector2, NormalizedTest1) {
  const float x{3.f};
  const float y{4.f};
  Vector2<float> vec(x, y);
  EXPECT_TRUE(expect_near(vec.normalized(), Vector2<float>(0.6f, 0.8f)));
}

TEST(Vector2, NormalizedTest2) {
  const float x{1.f};
  const float y{-1.f};
  const float inv_sqrt2 = static_cast<float>(1.0 / std::sqrt(2));
  Vector2<float> vec(x, y);
  EXPECT_TRUE(expect_near(vec.normalized(),
                                 Vector2<float>(inv_sqrt2, -inv_sqrt2)));
}

TEST(Vector2, NormalizedWithZeroNorm) {
  const float x{0.01f};
  const float y{0.f};
  Vector2<float> vec(x, y);
  EXPECT_TRUE(expect_near(vec.normalized(), Vector2<float>(1, 0)));
}

TEST(Vector2, ClampOverValue) {
  const float x{3.f};
  const float y{-4.f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector2<float> vec(x, y);
  vec.clamp(vmin, vmax);
  EXPECT_TRUE(expect_near(vec, Vector2<float>(vmax, vmin)));
}

TEST(Vector2, ClampUnderValue) {
  const float x{0.5f};
  const float y{-.5f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector2<float> vec(x, y);
  vec.clamp(vmin, vmax);
  EXPECT_TRUE(expect_near(vec, vec));
}

TEST(Vector2, ClampedOverValue) {
  const float x{2.f};
  const float y{2.f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector2<float> vec(x, y);
  EXPECT_TRUE(
    expect_near(vec.clamped(vmin, vmax), Vector2<float>(vmax, vmax)));
}

TEST(Vector2, ClampedUnderValue) {
  const float x{0.1f};
  const float y{-.2f};
  const float vmin{-1.f};
  const float vmax{1.f};
  Vector2<float> vec(x, y);
  EXPECT_TRUE(expect_near(vec.clamped(vmin, vmax), vec));
}

TEST(Vector2, ClampWithVector) {
  const float x{5.f};
  const float y{2.f};
  const Vector2<float> vmin(-1.f, -2.f);
  const Vector2<float> vmax(1.f);
  Vector2<float> vec(x, y);
  vec.clamp(vmin, vmax);
  EXPECT_TRUE(expect_near(vec, Vector2<float>(vmax.at(0), vmax.at(1))));
}

TEST(Vector2, ClampedWithVector) {
  const float x{0.1f};
  const float y{-.2f};
  const Vector2<float> vmin(-1.f, -2.f);
  const Vector2<float> vmax(1.f);
  Vector2<float> vec(x, y);
  EXPECT_TRUE(expect_near(vec.clamped(vmin, vmax), vec));
}