// Copyright (C) 2023 Bara Emran - All Rights Reserved

#ifndef TEST_UTEST_BASIC_HPP
#define TEST_UTEST_BASIC_HPP

#include "core/simplemath/basic.hpp"
#include "utils.hpp"

TEST(Point2D, DefaultConstructor) {
  Point2D<float> p;
  EXPECT_TRUE(expect_near(p, Point2D<float>(0.f, 0.f)));
}

TEST(Point2D, RandomConstructor) {
  const float x{1.f};
  const float y{2.f};
  Point2D<float> p(x, y);
  EXPECT_TRUE(expect_near(Point2D<float>(x, y), p));
}

/*****************************************************************************/

TEST(Point3D, DefaultConstructor) {
  Point3D<float> p;
  EXPECT_TRUE(expect_near(p, Point3D<float>(0.f, 0.f, 0.f)));
}

TEST(Point3D, RandomConstructor) {
  const float x{1.0};
  const float y{2.0};
  const float z{3.0};
  Point3D<float> p(x, y, z);
  EXPECT_TRUE(expect_near(p, Point3D<float>(x, y, z)));
}
/*****************************************************************************/
const float X{1.f};
const float Y{2.f};
const float Z{3.f};
const BasicVector2<float> VEC2(X, Y);
const BasicVector3<float> VEC3(X, Y, Z);

TEST(BasicVector2, DefaultConstructor) {
  BasicVector2<float> vec;
  EXPECT_TRUE(expect_near(vec, BasicVector2<float>(0, 0)));
}

TEST(BasicVector2, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicVector2<float> vec(c);
  EXPECT_TRUE(expect_near(vec, BasicVector2<float>(c, c)));
}

TEST(BasicVector2, ConstructorWithXY) {
  EXPECT_TRUE(expect_near(VEC2, BasicVector2<float>(X, Y)));
}

TEST(BasicVector2, UsingSquareIndicies) {
  EXPECT_FLOAT_EQ(VEC2[0], X);
  EXPECT_FLOAT_EQ(VEC2[1], Y);
}

TEST(BasicVector2, UsingRoundIndicies) {
  EXPECT_FLOAT_EQ(VEC2(0), X);
  EXPECT_FLOAT_EQ(VEC2(1), Y);
}

TEST(BasicVector2, UsingData) {
  EXPECT_FLOAT_EQ(VEC2.data[0], X);
  EXPECT_FLOAT_EQ(VEC2.data[1], Y);
}

TEST(BasicVector2, AlternateValues) {
  BasicVector2<float> vec;
  EXPECT_TRUE(expect_near(vec, BasicVector2<float>(0.f, 0.f)));
  vec[0] = VEC2[0];
  vec[1] = VEC2[1];
  EXPECT_TRUE(expect_near(vec, VEC2));
}

TEST(BasicVector2, Begin) {
  EXPECT_EQ(VEC2.begin(), VEC2.data);
  EXPECT_FLOAT_EQ(*VEC2.begin(), X);
}

TEST(BasicVector2, End) {
  EXPECT_EQ(VEC2.end(), VEC2.data + 2);
}

/*****************************************************************************/

TEST(BasicVector3, DefaultConstructor) {
  BasicVector3<float> vec;
  EXPECT_TRUE(expect_near(vec, BasicVector3<float>(0.f, 0.f, 0.f)));
}

TEST(BasicVector3, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicVector3<float> vec(c);
  EXPECT_TRUE(expect_near(vec, BasicVector3<float>(c, c, c)));
}

TEST(BasicVector3, ConstructorWithXYZ) {
  EXPECT_TRUE(expect_near(VEC3, BasicVector3<float>(X, Y, Z)));
}

TEST(BasicVector3, UsingSquareIndicies) {
  EXPECT_FLOAT_EQ(VEC3[0], X);
  EXPECT_FLOAT_EQ(VEC3[1], Y);
  EXPECT_FLOAT_EQ(VEC3[2], Z);
}

TEST(BasicVector3, UsingRoundIndicies) {
  EXPECT_FLOAT_EQ(VEC3(0), X);
  EXPECT_FLOAT_EQ(VEC3(1), Y);
  EXPECT_FLOAT_EQ(VEC3(2), Z);
}

TEST(BasicVector3, UsingData) {
  EXPECT_FLOAT_EQ(VEC3.data[0], X);
  EXPECT_FLOAT_EQ(VEC3.data[1], Y);
  EXPECT_FLOAT_EQ(VEC3.data[2], Z);
}

TEST(BasicVector3, AssignValues) {
  BasicVector3<float> vec;
  EXPECT_TRUE(expect_near(vec, BasicVector3<float>(0.f, 0.f, 0.f)));
  vec[0] = VEC3[0];
  vec[1] = VEC3[1];
  vec[2] = VEC3[2];
  EXPECT_TRUE(expect_near(vec, VEC3));
}

TEST(BasicVector3, Begin) {
  EXPECT_EQ(VEC3.begin(), VEC3.data);
  EXPECT_FLOAT_EQ(*VEC3.begin(), X);
}

TEST(BasicVector3, End) {
  EXPECT_EQ(VEC3.end(), VEC3.data + 3);
}
/*****************************************************************************/

TEST(MatrixElements2D, DefaultConstructor) {
  MatrixElements2D<float> mat;
  EXPECT_TRUE(expect_near(mat, MatrixElements2D<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(MatrixElements2D, RandomConstructor) {
  MatrixElements2D<float> mat(1.f, 2.f, 3.f, 4.f);
  EXPECT_TRUE(expect_near(mat, MatrixElements2D<float>(1.f, 2.f, 3.f, 4.f)));
}

/*****************************************************************************/

TEST(MatrixElements3D, DefaultConstructor) {
  MatrixElements3D<float> mat;
  EXPECT_TRUE(expect_near(
    mat, MatrixElements3D<float>(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
}

TEST(MatrixElements3D, RandomConstructor) {
  MatrixElements3D<float> mat(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f);
  EXPECT_TRUE(expect_near(
    mat, MatrixElements3D<float>(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f)));
}

/*****************************************************************************/

constexpr float M00{1.f};
constexpr float M01{2.f};
constexpr float M02{3.f};
constexpr float M10{4.f};
constexpr float M11{3.f};
constexpr float M12{5.f};
constexpr float M20{6.f};
constexpr float M21{7.f};
constexpr float M22{8.f};
const BasicMatrix2x2<float> MAT2(M00, M01, M10, M11);
const BasicMatrix3x3<float> MAT3(M00, M01, M02, M10, M11, M12, M20, M21, M22);

TEST(BasicMatrix2x2, DefaultConstructor) {
  BasicMatrix2x2<float> mat;
  EXPECT_TRUE(expect_near(mat, BasicMatrix2x2<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(BasicMatrix2x2, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicMatrix2x2<float> mat(c);
  EXPECT_TRUE(expect_near(mat, BasicMatrix2x2<float>(c, c, c, c)));
}

TEST(BasicMatrix2x2, RandomConstructor) {
  EXPECT_TRUE(expect_near(MAT2, BasicMatrix2x2<float>(M00, M01, M10, M11)));
}

TEST(BasicMatrix2x2, Rows) {
  EXPECT_EQ(MAT2.rows(), 2);
}

TEST(BasicMatrix2x2, Cols) {
  EXPECT_EQ(MAT2.cols(), 2);
}

TEST(BasicMatrix2x2, Size) {
  EXPECT_EQ(MAT2.size(), 4);
}

TEST(BasicMatrix2x2, UsingSquareIndicies) {
  EXPECT_FLOAT_EQ(MAT2[0], M00);
  EXPECT_FLOAT_EQ(MAT2[1], M01);
  EXPECT_FLOAT_EQ(MAT2[2], M10);
  EXPECT_FLOAT_EQ(MAT2[3], M11);
}

TEST(BasicMatrix2x2, UsingRoundSingleIndex) {
  EXPECT_FLOAT_EQ(MAT2(0), M00);
  EXPECT_FLOAT_EQ(MAT2(1), M01);
  EXPECT_FLOAT_EQ(MAT2(2), M10);
  EXPECT_FLOAT_EQ(MAT2(3), M11);
}

TEST(BasicMatrix2x2, UsingRoundDoubleIndices) {
  EXPECT_FLOAT_EQ(MAT2(0, 0), M00);
  EXPECT_FLOAT_EQ(MAT2(0, 1), M01);
  EXPECT_FLOAT_EQ(MAT2(1, 0), M10);
  EXPECT_FLOAT_EQ(MAT2(1, 1), M11);
}

TEST(BasicMatrix2x2, UsingData) {
  EXPECT_FLOAT_EQ(MAT2.data[0], M00);
  EXPECT_FLOAT_EQ(MAT2.data[1], M01);
  EXPECT_FLOAT_EQ(MAT2.data[2], M10);
  EXPECT_FLOAT_EQ(MAT2.data[3], M11);
}

TEST(BasicMatrix2x2, UsingMat) {
  EXPECT_FLOAT_EQ(MAT2.mat[0][0], M00);
  EXPECT_FLOAT_EQ(MAT2.mat[0][1], M01);
  EXPECT_FLOAT_EQ(MAT2.mat[1][0], M10);
  EXPECT_FLOAT_EQ(MAT2.mat[1][1], M11);
}

TEST(BasicMatrix2x2, AlternateValues) {
  BasicMatrix2x2<float> mat;
  EXPECT_TRUE(expect_near(mat, BasicMatrix2x2<float>(0, 0, 0, 0)));
  for (size_t i = 0; i < mat.size(); ++i) {
    mat[i] = MAT2.data[i];
  }
  EXPECT_TRUE(expect_near(mat, MAT2));
}

TEST(BasicMatrix2x2, Begin) {
  EXPECT_EQ(MAT2.begin(), MAT2.data);
  EXPECT_FLOAT_EQ(*MAT2.begin(), X);
}

TEST(BasicMatrix2x2, End) {
  EXPECT_EQ(MAT2.end(), MAT2.data + 4);
}

/*****************************************************************************/

TEST(BasicMatrix3x3, DefaultConstructor) {
  BasicMatrix3x3<float> mat;
  EXPECT_TRUE(
    expect_near(mat, BasicMatrix3x3<float>(0, 0, 0, 0, 0, 0, 0, 0, 0)));
}

TEST(BasicMatrix3x3, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicMatrix3x3<float> mat(c);
  EXPECT_TRUE(
    expect_near(mat, BasicMatrix3x3<float>(c, c, c, c, c, c, c, c, c)));
}

TEST(BasicMatrix3x3, RandomConstructor) {
  EXPECT_TRUE(expect_near(
    MAT3, BasicMatrix3x3<float>(M00, M01, M02, M10, M11, M12, M20, M21, M22)));
}

TEST(BasicMatrix3x3, Rows) {
  EXPECT_EQ(MAT3.rows(), 3);
}

TEST(BasicMatrix3x3, Cols) {
  EXPECT_EQ(MAT3.cols(), 3);
}

TEST(BasicMatrix3x3, Size) {
  EXPECT_EQ(MAT3.size(), 9);
}

TEST(BasicMatrix3x3, UsingSquareIndicies) {
  for (size_t i = 0; i < MAT3.size(); ++i) {
    EXPECT_FLOAT_EQ(MAT3[i], MAT3.data[i]);
  }
}

TEST(BasicMatrix3x3, UsingRoundSingleIndex) {
  for (size_t i = 0; i < MAT3.size(); ++i) {
    EXPECT_FLOAT_EQ(MAT3(i), MAT3.data[i]);
  }
}

TEST(BasicMatrix3x3, UsingRoundDoubleIndices) {
  for (size_t r = 0; r < MAT3.rows(); ++r) {
    for (size_t c = 0; c < MAT3.cols(); ++c) {
      EXPECT_FLOAT_EQ(MAT3(r, c), MAT3.mat[r][c]);
    }
  }
}

TEST(BasicMatrix3x3, AssignValues) {
  BasicMatrix3x3<float> mat;
  EXPECT_TRUE(
    expect_near(mat, BasicMatrix3x3<float>(0, 0, 0, 0, 0, 0, 0, 0, 0)));
  for (size_t i = 0; i < mat.size(); ++i) {
    mat[i] = MAT3.data[i];
  }
  EXPECT_TRUE(expect_near(mat, MAT3));
}

TEST(BasicMatrix3x3, Begin) {
  EXPECT_EQ(VEC3.begin(), VEC3.data);
  EXPECT_FLOAT_EQ(*VEC3.begin(), X);
}

TEST(BasicMatrix3x3, End) {
  EXPECT_EQ(VEC3.end(), VEC3.data + 3);
}

#endif  // TEST_UTEST_BASIC_HPP

