// Copyright (C) 2024 Bara Emran - All Rights Reserved

#include "core/simplemath/matrix2x2.hpp"
#include "utils.hpp"

#define START_IGNORE_SELF_ASSIGN_WARNING                                       \
  _Pragma("GCC diagnostic push")                                               \
    _Pragma("GCC diagnostic ignored \"-Wself-assign-overloaded\"")

#define STOP_IGNORE_SELF_ASSIGN_WARNING _Pragma("GCC diagnostic pop")

constexpr float M00{1.f};
constexpr float M01{2.f};
constexpr float M10{3.f};
constexpr float M11{4.f};
const Matrix2x2<float> MAT(M00, M01, M10, M11);

TEST(Matrix2x2, DefaultConstructor) {
  Matrix2x2<float> mat;
  EXPECT_TRUE(expect_near(mat, Matrix2x2<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(Matrix2x2, ConstructorWithSingleValue) {
  const float c{1.f};
  Matrix2x2<float> mat(c);
  EXPECT_TRUE(expect_near(mat, Matrix2x2<float>(c, c, c, c)));
}

TEST(Matrix2x2, ConstructorWithValues) {
  Matrix2x2<float> mat(M00, M01, M10, M11);
  EXPECT_TRUE(expect_near(mat, MAT));
}

TEST(Matrix2x2, ConstructorWithArray) {
  std::array<float, 4> array = {M00, M01, M10, M11};
  Matrix2x2<float> mat(array);
  EXPECT_TRUE(expect_near(mat, MAT));
}

TEST(Matrix2x2, ConstructIdentityMatrix) {
  EXPECT_TRUE(
    expect_near(Matrix2x2<float>::eye(), Matrix2x2<float>(1.f, 0.f, 0.f, 1.f)));
}

TEST(Matrix2x2, ConstructZerosMatrix) {
  EXPECT_TRUE(expect_near(Matrix2x2<float>::zeros(),
                          Matrix2x2<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(Matrix2x2, ConstructOnesMatrix) {
  EXPECT_TRUE(expect_near(Matrix2x2<float>::ones(),
                          Matrix2x2<float>(1.f, 1.f, 1.f, 1.f)));
}

TEST(Matrix2x2, AddInplace) {
  Matrix2x2<float> mat(MAT);
  mat += mat;
  EXPECT_TRUE(
    expect_near(mat, Matrix2x2<float>(M00 * 2, M01 * 2, M10 * 2, M11 * 2)));
}

TEST(Matrix2x2, Add) {
  Matrix2x2<float> result = MAT + MAT;
  EXPECT_TRUE(
    expect_near(result, Matrix2x2<float>(M00 * 2, M01 * 2, M10 * 2, M11 * 2)));
}

TEST(Matrix2x2, SubtractInplace) {
  Matrix2x2<float> mat(MAT);
  START_IGNORE_SELF_ASSIGN_WARNING
  mat -= mat;
  STOP_IGNORE_SELF_ASSIGN_WARNING
  EXPECT_TRUE(expect_near(mat, Matrix2x2<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(Matrix2x2, Subtract) {
  Matrix2x2<float> result = MAT - MAT;
  EXPECT_TRUE(expect_near(result, Matrix2x2<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(Matrix2x2, MultiplyByConstantInplace) {
  const float s{3.f};
  Matrix2x2<float> mat(MAT);
  mat *= s;
  EXPECT_TRUE(
    expect_near(mat, Matrix2x2<float>(M00 * s, M01 * s, M10 * s, M11 * s)));
}

TEST(Matrix2x2, MultiplyByConstant) {
  const float s{3.f};
  Matrix2x2<float> result = MAT * s;
  EXPECT_TRUE(
    expect_near(result, Matrix2x2<float>(M00 * s, M01 * s, M10 * s, M11 * s)));
}

TEST(Matrix2x2, MultiplyByMatrixInplace) {
  const float m00{1.f};
  const float m01{2.f};
  const float m10{1.f};
  const float m11{2.f};
  Matrix2x2<float> mat(m00, m01, m10, m11);
  mat *= mat;
  EXPECT_TRUE(expect_near(mat, Matrix2x2<float>(3, 6, 3, 6)));
}

TEST(Matrix2x2, MultiplyByMatrix) {
  const float m00{1.f};
  const float m01{2.f};
  const float m10{1.f};
  const float m11{2.f};

  Matrix2x2<float> mat(m00, m01, m10, m11);
  Matrix2x2<float> result = mat * mat;
  EXPECT_TRUE(expect_near(result, Matrix2x2<float>(3, 6, 3, 6)));
}

TEST(Matrix2x2, MultiplyByIdentityMatrix) {
  Matrix2x2<float> result = MAT * Matrix2x2<float>::eye();
  EXPECT_TRUE(expect_near(result, MAT));
}

TEST(Matrix2x2, Det) {
  EXPECT_FLOAT_EQ(MAT.det(), -2);
}

TEST(Matrix2x2, DetIdentity) {
  EXPECT_FLOAT_EQ(Matrix2x2<float>::eye().det(), 1);
}

TEST(Matrix2x2, Transpose) {
  Matrix2x2<float> mat(MAT);
  mat.transpose();
  EXPECT_TRUE(expect_near(mat, Matrix2x2<float>(M00, M10, M01, M11)));
}

TEST(Matrix2x2, Transposed) {
  EXPECT_TRUE(
    expect_near(MAT.transposed(), Matrix2x2<float>(M00, M10, M01, M11)));
}

TEST(Matrix2x2, InverseIdentity) {
  Matrix2x2<float> mat = Matrix2x2<float>::eye();
  mat.inverse();
  EXPECT_TRUE(expect_near(mat, mat));
}

TEST(Matrix2x2, Inverse) {
  const float m00{2.f};
  const float m01{1.f};
  const float m10{3.f};
  const float m11{3.f};

  Matrix2x2<float> mat(m00, m01, m10, m11);
  EXPECT_TRUE(expect_near(mat.inverse(), Matrix2x2<float>(1.f, -1.f / 3.f, -1, 2.f / 3.f)));
}

TEST(Matrix2x2, Adjoint) {
  EXPECT_TRUE(expect_near(MAT.adjoint(), Matrix2x2<float>(M11, -M01, -M10, M00)));
}

TEST(Matrix2x2, ClampOverValue) {
  Matrix2x2<float> mat(MAT);
  mat.clamp(-1, 1);
  EXPECT_TRUE(expect_near(mat, Matrix2x2<float>::ones()));
}

TEST(Matrix2x2, ClampUnderValue) {
  Matrix2x2<float> mat(MAT);
  mat.clamp(-10, 10);
  EXPECT_TRUE(expect_near(mat, mat));
}

TEST(Matrix2x2, ClampedOverValue) {
  EXPECT_TRUE(expect_near(MAT.clamped(-1, 1), Matrix2x2<float>::ones()));
}

TEST(Matrix2x2, ClampedUnderValue) {
  EXPECT_TRUE(expect_near(MAT.clamped(-10, 10), MAT));
}