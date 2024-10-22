// A simple unit test file to build upon later.

#include <stdexcept>

#include "simple.h"

#include "gtest/gtest.h"
namespace {

// Tests Sum().

// Tests sum of positive numbers.
TEST(SumTest, Positive) {
  EXPECT_EQ(101, Sum(1, 100));
}

// Tests sum of zeros.
TEST(SumTest, Zero) { EXPECT_EQ(0, Sum(0, 0)); }

// Tests sum of negative numbers.
TEST(SumTest, Negative) {
  EXPECT_EQ(-4, Sum(-2, -2));
}

// Tests sum of positive and negative numbers.
TEST(SumTest, Both) {
  EXPECT_EQ(-4, Sum(8, -12));
}

// Tests Interpolate().

// Tests points with a positive slope.
TEST(InterpolateTest, PositiveSlope) {
  double x0 = 1;
  double y0 = 1;
  double x1 = 7;
  double y1 = 4;
  EXPECT_EQ(3, Interpolate(x0, y0, x1, y1, 5));       // interpolate
  EXPECT_EQ(3, Interpolate(x1, y1, x0, y0, 5));       // interpolate w/switched input order
  EXPECT_EQ(5, Interpolate(x0, y0, x1, y1, 9));       // extrapolate high
  EXPECT_EQ(-9.5, Interpolate(x0, y0, x1, y1, -20));  // extrapolate low
  EXPECT_EQ(y0, Interpolate(x0, y0, x1, y1, x0));     // x = x0
  EXPECT_EQ(y1, Interpolate(x0, y0, x1, y1, x1));     // x = x1

}

// Tests points with a negative slope.
TEST(InterpolateTest, NegativeSlope) {
  double x0 = -1;
  double y0 = 20;
  double x1 = 6;
  double y1 = -15;
  EXPECT_EQ(15, Interpolate(x0, y0, x1, y1, 0));      // interpolate
  EXPECT_EQ(-27.5, Interpolate(x0, y0, x1, y1, 8.5)); // extrapolate high
  EXPECT_EQ(115, Interpolate(x0, y0, x1, y1, -20));   // extrapolate low
  EXPECT_EQ(y0, Interpolate(x0, y0, x1, y1, x0));     // x = x0
  EXPECT_EQ(y1, Interpolate(x0, y0, x1, y1, x1));     // x = x1
}

// Tests points with a zero slope.
TEST(InterpolateTest, ZeroSlope) {
  double x0 = -4;
  double y0 = 9;
  double x1 = 7;
  double y1 = 9;
  EXPECT_EQ(9, Interpolate(x0, y0, x1, y1, 0));   // interpolate
  EXPECT_EQ(9, Interpolate(x0, y0, x1, y1, 8.5)); // extrapolate high
  EXPECT_EQ(9, Interpolate(x0, y0, x1, y1, -20)); // extrapolate low
  EXPECT_EQ(y0, Interpolate(x0, y0, x1, y1, x0)); // x = x0
  EXPECT_EQ(y1, Interpolate(x0, y0, x1, y1, x1)); // x = x1
}

// Tests point with an undefined slope.
TEST(InterpolateTest, UndefinedSlope) {
  double x0 = 3;
  double y0 = -1;
  double x1 = 3;
  double y1 = 12;
  EXPECT_THROW({
    try
    {
      Interpolate(x0, y0, x1, y1, 0);
    }
    catch(const std::runtime_error& e)
    {
      EXPECT_STREQ("Slope is undefined between (3, -1) and (3, 12). A y-value cannot be calculated between these points.", e.what());
      throw;
    }
  }, std::runtime_error);
}

}  // namespace
