#include "core/add.hpp"
#include "gtest/gtest.h"

TEST(blaTest, test1)
{
  // arrange
  std::vector<double> values{1, 2., 3.};
  // act
  const auto [sum, avg] = core::accumulate_vector(values);
  // assert
  EXPECT_EQ(sum, 6.0);
  EXPECT_EQ(avg, 2.0);
}