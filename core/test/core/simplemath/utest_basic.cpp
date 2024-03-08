// Copyright (C) 2023 Bara Emran - All Rights Reserved

#ifndef TEST_UTEST_BASIC_HPP
#define TEST_UTEST_BASIC_HPP

#include "core/simplemath/basic.hpp"
#include "utils.hpp"

TEST(Point2D, DefaultConstructor) {
  Point2D<float> p2;
  EXPECT_TRUE(expect_near(p2, Point2D<float>(0.f, 0.f)));
}

TEST(Point2D, RandomConstructor) {
  const float x{1.f};
  const float y{2.f};
  Point2D<float> p2(x, y);
  EXPECT_TRUE(expect_near(p2, Point2D<float>(x, y)));
}

/*****************************************************************************/

TEST(Point3D, DefaultConstructor) {
  Point3D<float> p3;
  EXPECT_TRUE(expect_near(p3, Point3D<float>(0.f, 0.f, 0.f)));
}

TEST(Point3D, RandomConstructor) {
  const float x{1.0};
  const float y{2.0};
  const float z{3.0};
  Point3D<float> p3(x, y, z);
  EXPECT_TRUE(expect_near(p3, Point3D<float>(x, y, z)));
}
/*****************************************************************************/

TEST(BasicVector2, DefaultConstructor) {
  BasicVector2<float> bv2;
  EXPECT_TRUE(expect_near(bv2, BasicVector2<float>(0, 0)));
}

TEST(BasicVector2, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicVector2<float> bv2(c);
  EXPECT_TRUE(expect_near(bv2, BasicVector2<float>(c, c)));
}

TEST(BasicVector2, ConstructorWithXY) {
  const float x{1.f};
  const float y{2.f};
  BasicVector2<float> bv2(x, y);
  EXPECT_TRUE(expect_near(bv2, BasicVector2<float>(x, y)));
}

TEST(BasicVector2, UsingSquareIndicies) {
  BasicVector2<float> bv2(1.f, 2.f);
  for (size_t i = 0; i < bv2.size(); ++i) {
    EXPECT_FLOAT_EQ(bv2[i], bv2.data[i]);
  }
}

TEST(BasicVector2, UsingRoundIndicies) {
  BasicVector2<float> bv2(1.f, 2.f);
  for (size_t i = 0; i < bv2.size(); ++i) {
    EXPECT_FLOAT_EQ(bv2(i), bv2.data[i]);
  }
}

TEST(BasicVector2, AlternateValues) {
  BasicVector2<float> bv2;
  EXPECT_TRUE(expect_near(bv2, BasicVector2<float>(0.f, 0.f)));
  const float new_values[2] = {1.0, 2.0};
  for (size_t i = 0; i < bv2.size(); ++i) {
    bv2[i] = new_values[i];
  }
  EXPECT_TRUE(expect_near(bv2, BasicVector2<float>(1.f, 2.f)));
}

/*****************************************************************************/

TEST(BasicVector3, DefaultConstructor) {
  BasicVector3<float> bv3;
  EXPECT_TRUE(expect_near(bv3, BasicVector3<float>(0.f, 0.f, 0.f)));
}

TEST(BasicVector3, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicVector3<float> bv3(c);
  EXPECT_TRUE(expect_near(bv3, BasicVector3<float>(c, c, c)));
}

TEST(BasicVector3, ConstructorWithXYZ) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  BasicVector3<float> bv3(x, y, z);
  EXPECT_TRUE(expect_near(bv3, BasicVector3<float>(x, y, z)));
}

TEST(BasicVector3, UsingSquareIndicies) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  BasicVector3<float> bv3(x, y, z);
  for (size_t i = 0; i < bv3.size(); ++i) {
    EXPECT_FLOAT_EQ(bv3[i], bv3.data[i]);
  }
}

TEST(BasicVector3, UsingRoundIndicies) {
  const float x{1.f};
  const float y{2.f};
  const float z{3.f};
  BasicVector3<float> bv3(x, y, z);
  for (size_t i = 0; i < bv3.size(); ++i) {
    EXPECT_FLOAT_EQ(bv3(i), bv3.data[i]);
  }
}

TEST(BasicVector3, AssignValues) {
  BasicVector3<float> bv3;
  EXPECT_TRUE(expect_near(bv3, BasicVector3<float>(0.f, 0.f, 0.f)));
  const float new_values[3] = {1.0, 2.0, 3.0};
  for (size_t i = 0; i < bv3.size(); ++i) {
    bv3[i] = new_values[i];
  }
  EXPECT_TRUE(expect_near(bv3, BasicVector3<float>(1.f, 2.f, 3.f)));
}

/*****************************************************************************/

TEST(MatrixElements2D, DefaultConstructor) {
  MatrixElements2D<float> mt2;
  EXPECT_TRUE(expect_near(mt2, MatrixElements2D<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(MatrixElements2D, RandomConstructor) {
  MatrixElements2D<float> mt2(1.f, 2.f, 3.f, 4.f);
  EXPECT_TRUE(expect_near(mt2, MatrixElements2D<float>(1.f, 2.f, 3.f, 4.f)));
}

/*****************************************************************************/

TEST(MatrixElements3D, DefaultConstructor) {
  MatrixElements3D<float> mt3;
  EXPECT_TRUE(expect_near(
    mt3, MatrixElements3D<float>(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
}

TEST(MatrixElements3D, RandomConstructor) {
  MatrixElements3D<float> mt3(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f);
  EXPECT_TRUE(expect_near(
    mt3, MatrixElements3D<float>(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f)));
}

/*****************************************************************************/

TEST(BasicMatrix2x2, DefaultConstructor) {
  BasicMatrix2x2<float> bm2;
  EXPECT_TRUE(expect_near(bm2, BasicMatrix2x2<float>(0.f, 0.f, 0.f, 0.f)));
}

TEST(BasicMatrix2x2, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicMatrix2x2<float> bm2(c);
  EXPECT_TRUE(expect_near(bm2, BasicMatrix2x2<float>(c, c, c, c)));
}

TEST(BasicMatrix2x2, RandomConstructor) {
  BasicMatrix2x2<float> bm2(1.f, 2.f, 3.f, 4.f);
  EXPECT_TRUE(expect_near(bm2, BasicMatrix2x2<float>(1.f, 2.f, 3.f, 4.f)));
}

TEST(BasicMatrix2x2, UsingSquareIndicies) {
  BasicMatrix2x2<float> bm2(1.f, 2.f, 3.f, 4.f);
  for (size_t i = 0; i < bm2.size(); ++i) {
    EXPECT_FLOAT_EQ(bm2[i], bm2.data[i]);
  }
}

TEST(BasicMatrix2x2, UsingRoundSingleIndex) {
  BasicMatrix2x2<float> bm2(1.f, 2.f, 3.f, 4.f);
  for (size_t i = 0; i < bm2.size(); ++i) {
    EXPECT_FLOAT_EQ(bm2(i), bm2.data[i]);
  }
}

TEST(BasicMatrix2x2, UsingRoundDoubleIndices) {
  BasicMatrix2x2<float> bm2(1.f, 2.f, 3.f, 4.f);
  for (size_t r = 0; r < bm2.rows(); ++r) {
    for (size_t c = 0; c < bm2.cols(); ++c) {
      EXPECT_FLOAT_EQ(bm2(r, c), bm2.mat[r][c]);
    }
  }
}

TEST(BasicMatrix2x2, AlternateValues) {
  BasicMatrix2x2<float> bm2(0.f, 0.f, 0.f, 0.f);
  EXPECT_TRUE(expect_near(bm2, BasicMatrix2x2<float>(0, 0, 0, 0)));
  const float new_values[4] = {1.0, 2.0, 3.0, 4.0};
  for (size_t i = 0; i < bm2.size(); ++i) {
    bm2[i] = new_values[i];
  }
  EXPECT_TRUE(expect_near(bm2, BasicMatrix2x2<float>(1.0, 2.0, 3.0, 4.0)));
}
/*****************************************************************************/

TEST(BasicMatrix3x3, DefaultConstructor) {
  BasicMatrix3x3<float> bm3;
  EXPECT_TRUE(
    expect_near(bm3, BasicMatrix3x3<float>(0, 0, 0, 0, 0, 0, 0, 0, 0)));
}

TEST(BasicMatrix3x3, ConstructorWithSingleValue) {
  const float c{2.0};
  BasicMatrix3x3<float> bm3(c);
  EXPECT_TRUE(
    expect_near(bm3, BasicMatrix3x3<float>(c, c, c, c, c, c, c, c, c)));
}

TEST(BasicMatrix3x3, RandomConstructor) {
  BasicMatrix3x3<float> bm3(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f);

  EXPECT_TRUE(expect_near(
    bm3, BasicMatrix3x3<float>(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f)));
}

TEST(BasicMatrix3x3, UsingSquareIndicies) {
  BasicMatrix3x3<float> bm3(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f);
  for (size_t i = 0; i < bm3.size(); ++i) {
    EXPECT_FLOAT_EQ(bm3[i], bm3.data[i]);
  }
}

TEST(BasicMatrix3x3, UsingRoundSingleIndex) {
  BasicMatrix3x3<float> bm3(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f);
  for (size_t i = 0; i < bm3.size(); ++i) {
    EXPECT_FLOAT_EQ(bm3(i), bm3.data[i]);
  }
}

TEST(BasicMatrix3x3, UsingRoundDoubleIndices) {
  BasicMatrix3x3<float> bm3(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f);
  for (size_t r = 0; r < bm3.rows(); ++r) {
    for (size_t c = 0; c < bm3.cols(); ++c) {
      EXPECT_FLOAT_EQ(bm3(r, c), bm3.mat[r][c]);
    }
  }
}

TEST(BasicMatrix3x3, AssignValues) {
  BasicMatrix3x3<float> bm3;
  EXPECT_TRUE(
    expect_near(bm3, BasicMatrix3x3<float>(0, 0, 0, 0, 0, 0, 0, 0, 0)));
  const float new_values[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  for (size_t i = 0; i < bm3.size(); ++i) {
    bm3[i] = new_values[i];
  }
  EXPECT_TRUE(expect_near(
    bm3, BasicMatrix3x3<float>(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)));
}
#endif  // TEST_UTEST_BASIC_HPP
