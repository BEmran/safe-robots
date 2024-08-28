// Copyright (C) 2024 Bara Emran - All Rights Reserved

#include "core/simplemath/matrix3x3.hpp"
#include "utils.hpp"

#define START_IGNORE_SELF_ASSIGN_WARNING                                       \
  _Pragma("GCC diagnostic push")                                               \
    _Pragma("GCC diagnostic ignored \"-Wself-assign-overloaded\"")

#define STOP_IGNORE_SELF_ASSIGN_WARNING _Pragma("GCC diagnostic pop")

constexpr float M00{1.f};
constexpr float M01{2.f};
constexpr float M02{3.f};
constexpr float M10{4.f};
constexpr float M11{3.f};
constexpr float M12{5.f};
constexpr float M20{6.f};
constexpr float M21{7.f};
constexpr float M22{8.f};
const Matrix3x3<float> MAT(M00, M01, M02, M10, M11, M12, M20, M21, M22);

TEST(Matrix3x3, DefaultConstructor) {
  Matrix3x3<float> mat;
  EXPECT_TRUE(expect_near(
    mat, Matrix3x3<float>(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
}

TEST(Matrix3x3, ConstructorWithSingleValue) {
  const float c{1.f};
  Matrix3x3<float> mat(c);
  EXPECT_TRUE(expect_near(mat, Matrix3x3<float>(c, c, c, c, c, c, c, c, c)));
}

TEST(Matrix3x3, ConstructorWithValues) {
  EXPECT_TRUE(expect_near(
    MAT, Matrix3x3<float>(M00, M01, M02, M10, M11, M12, M20, M21, M22)));
}

TEST(Matrix3x3, ConstructorWithArray) {
  std::array<float, 9> array = {M00, M01, M02, M10, M11, M12, M20, M21, M22};
  Matrix3x3<float> mat(array);
  EXPECT_TRUE(expect_near(mat, MAT));
}

TEST(Matrix3x3, ConstructIdentityMatrix) {
  EXPECT_TRUE(
    expect_near(Matrix3x3<float>::eye(),
                Matrix3x3<float>(1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f)));
}

TEST(Matrix3x3, ConstructZerosMatrix) {
  EXPECT_TRUE(
    expect_near(Matrix3x3<float>::zeros(),
                Matrix3x3<float>(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
}

TEST(Matrix3x3, ConstructOnesMatrix) {
  EXPECT_TRUE(
    expect_near(Matrix3x3<float>::ones(),
                Matrix3x3<float>(1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f)));
}

TEST(Matrix3x3, AddInplace) {
  Matrix3x3<float> mat(MAT);
  mat += mat;
  EXPECT_TRUE(expect_near(mat, Matrix3x3<float>(M00 * 2, M01 * 2, M02 * 2,
                                                M10 * 2, M11 * 2, M12 * 2,
                                                M20 * 2, M21 * 2, M22 * 2)));
}

TEST(Matrix3x3, Add) {
  Matrix3x3<float> result = MAT + MAT;
  EXPECT_TRUE(expect_near(result, Matrix3x3<float>(M00 * 2, M01 * 2, M02 * 2,
                                                   M10 * 2, M11 * 2, M12 * 2,
                                                   M20 * 2, M21 * 2, M22 * 2)));
}

TEST(Matrix3x3, SubtractInplace) {
  Matrix3x3<float> mat(MAT);
  START_IGNORE_SELF_ASSIGN_WARNING
  mat -= mat;
  STOP_IGNORE_SELF_ASSIGN_WARNING
  EXPECT_TRUE(expect_near(mat, Matrix3x3<float>::zeros()));
}

TEST(Matrix3x3, Subtract) {
  Matrix3x3<float> result = MAT - MAT;
  EXPECT_TRUE(expect_near(result, Matrix3x3<float>::zeros()));
}

TEST(Matrix3x3, MultiplyByConstantInplace) {
  const float s{3.f};
  Matrix3x3<float> mat(MAT);
  mat *= s;
  EXPECT_TRUE(expect_near(mat, Matrix3x3<float>(M00 * s, M01 * s, M02 * s,
                                                M10 * s, M11 * s, M12 * s,
                                                M20 * s, M21 * s, M22 * s)));
}

TEST(Matrix3x3, MultiplyByConstant) {
  const float s{3.f};
  Matrix3x3<float> result = MAT * s;
  EXPECT_TRUE(expect_near(result, Matrix3x3<float>(M00 * s, M01 * s, M02 * s,
                                                   M10 * s, M11 * s, M12 * s,
                                                   M20 * s, M21 * s, M22 * s)));
}

TEST(Matrix3x3, MultiplyByMatrixInplace) {
  Matrix3x3<float> mat1(2.f, 3.f, 4.f, 3.f, 5.f, 6.f, 4.f, 5.f, 3.f);
  Matrix3x3<float> mat2(1.f, 2.f, 1.f, -1.f, 2.f, 1.f, 3.f, 2.f, 1.f);
  mat1 *= mat2;
  EXPECT_TRUE(expect_near(mat1, Matrix3x3<float>(11.f, 18.f, 9.f, 16.f, 28.f,
                                                 14.f, 8.f, 24.f, 12.f)));
}

TEST(Matrix3x3, MultiplyByMatrix) {
  Matrix3x3<float> mat1(2.f, 3.f, 4.f, 3.f, 5.f, 6.f, 4.f, 5.f, 3.f);
  Matrix3x3<float> mat2(1.f, 2.f, 1.f, -1.f, 2.f, 1.f, 3.f, 2.f, 1.f);
  Matrix3x3<float> result = mat1 * mat2;
  EXPECT_TRUE(expect_near(result, Matrix3x3<float>(11.f, 18.f, 9.f, 16.f, 28.f,
                                                   14.f, 8.f, 24.f, 12.f)));
}

TEST(Matrix3x3, MultiplyByIdentityMatrix) {
  Matrix3x3<float> result = MAT * Matrix3x3<float>::eye();
  EXPECT_TRUE(expect_near(result, MAT));
}

TEST(Matrix3x3, Det) {
  const float m00{1.f};
  const float m01{2.f};
  const float m02{3.f};
  const float m10{0.f};
  const float m11{1.f};
  const float m12{4.f};
  const float m20{5.f};
  const float m21{6.f};
  const float m22{0.f};
  Matrix3x3<float> mat(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  EXPECT_FLOAT_EQ(mat.det(), 1.f);
}

TEST(Matrix3x3, DetOfIdentity) {
  EXPECT_FLOAT_EQ(Matrix3x3<float>::eye().det(), 1.f);
}

TEST(Matrix3x3, Transpose) {
  Matrix3x3<float> mat(MAT);
  mat.transpose();
  EXPECT_TRUE(expect_near(
    mat, Matrix3x3<float>(M00, M10, M20, M01, M11, M21, M02, M12, M22)));
}

TEST(Matrix3x3, Transposed) {
  EXPECT_TRUE(
    expect_near(MAT.transposed(),
                Matrix3x3<float>(M00, M10, M20, M01, M11, M21, M02, M12, M22)));
}

TEST(Matrix3x3, InverseIdentity) {
  Matrix3x3<float> mat = Matrix3x3<float>::eye();
  mat.inverse();
  EXPECT_TRUE(expect_near(mat, mat));
}

TEST(Matrix3x3, Inverse) {

  const float m00{1.f};
  const float m01{2.f};
  const float m02{-1.f};
  const float m10{2.f};
  const float m11{1.f};
  const float m12{2.f};
  const float m20{-1.f};
  const float m21{2.f};
  const float m22{1.f};
  Matrix3x3<float> mat(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  Matrix3x3<float> expect(-3.f, -4.f, 5.f, -4.f, 0, -4.f, 5.f, -4.f, -3.f);
  expect /= -16.f;
  EXPECT_TRUE(expect_near(mat.inverse(), expect));
}

TEST(Matrix3x3, Inverse2) {
  const float m00{1.f};
  const float m01{2.f};
  const float m02{3.f};
  const float m10{0.f};
  const float m11{1.f};
  const float m12{4.f};
  const float m20{5.f};
  const float m21{6.f};
  const float m22{0.f};
  Matrix3x3<float> mat(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  Matrix3x3<float> expect(-24.f, 18.f, 5.f, 20.f, -15.f, -4.f, -5.f, 4.f, 1.f);
  expect /= 1.f;
  EXPECT_TRUE(expect_near(mat.inverse(), expect));
}

TEST(Matrix3x3, Inverse3) {
  const float m00{0.f};
  const float m01{-3.f};
  const float m02{-2.f};
  const float m10{1.f};
  const float m11{-4.f};
  const float m12{-2.f};
  const float m20{-3.f};
  const float m21{4.f};
  const float m22{1.f};
  Matrix3x3<float> mat(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  Matrix3x3<float> expect(4.f, -5.f, -2.f, 5.f, -6.f, -2.f, -8.f, 9.f, 3.f);
  EXPECT_TRUE(expect_near(mat.inverse(), expect));
}

TEST(Matrix3x3, Adjoint) {
  const float m00{1.f};
  const float m01{2.f};
  const float m02{-1.f};
  const float m10{2.f};
  const float m11{1.f};
  const float m12{2.f};
  const float m20{-1.f};
  const float m21{2.f};
  const float m22{1.f};
  Matrix3x3<float> mat(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  Matrix3x3<float> expect(-3.f, -4.f, 5.f, -4.f, 0, -4.f, 5.f, -4.f, -3.f);
  EXPECT_TRUE(expect_near(mat.adjoint(), expect));
}

TEST(Matrix3x3, Adjoint2) {
  const float m00{1.f};
  const float m01{2.f};
  const float m02{3.f};
  const float m10{0.f};
  const float m11{1.f};
  const float m12{4.f};
  const float m20{5.f};
  const float m21{6.f};
  const float m22{0.f};
  Matrix3x3<float> mat(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  Matrix3x3<float> expect(-24.f, 18.f, 5.f, 20.f, -15.f, -4.f, -5.f, 4.f, 1.f);
  EXPECT_TRUE(expect_near(mat.adjoint(), expect));
}

TEST(Matrix3x3, ClampOverValue) {
  Matrix3x3<float> mat(MAT);
  mat.clamp(-1, 1);
  EXPECT_TRUE(expect_near(mat, Matrix3x3<float>::ones()));
}

TEST(Matrix3x3, ClampUnderValue) {
  Matrix3x3<float> mat(MAT);
  mat.clamp(-10, 10);
  EXPECT_TRUE(expect_near(mat, mat));
}

TEST(Matrix3x3, ClampedOverValue) {
  EXPECT_TRUE(expect_near(MAT.clamped(-1, 1), Matrix3x3<float>::ones()));
}

TEST(Matrix3x3, ClampedUnderValue) {
  EXPECT_TRUE(expect_near(MAT.clamped(-10, 10), MAT));
}