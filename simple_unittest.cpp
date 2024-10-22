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
  PointCartesian2D p0 = {1, 1};
  PointCartesian2D p1 = {7, 4};
  EXPECT_EQ(3, Interpolate(p0, p1, 5).y);       // interpolate
  EXPECT_EQ(3, Interpolate(p1, p0, 5).y);       // interpolate w/switched input order
  EXPECT_EQ(5, Interpolate(p0, p1, 9).y);       // extrapolate high
  EXPECT_EQ(-9.5, Interpolate(p0, p1, -20).y);  // extrapolate low
  EXPECT_EQ(p0.y, Interpolate(p0, p1, p0.x).y); // x = x0
  EXPECT_EQ(p1.y, Interpolate(p0, p1, p1.x).y); // x = x1

}

// Tests points with a negative slope.
TEST(InterpolateTest, NegativeSlope) {
  PointCartesian2D p0 = {-1, 20};
  PointCartesian2D p1 = {6, -15};
  EXPECT_EQ(15, Interpolate(p0, p1, 0).y);      // interpolate
  EXPECT_EQ(15, Interpolate(p1, p0, 0).y);      // interpolate w/switched input order
  EXPECT_EQ(-27.5, Interpolate(p0, p1, 8.5).y); // extrapolate high
  EXPECT_EQ(115, Interpolate(p0, p1, -20).y);   // extrapolate low
  EXPECT_EQ(p0.y, Interpolate(p0, p1, p0.x).y); // x = x0
  EXPECT_EQ(p1.y, Interpolate(p0, p1, p1.x).y); // x = x1
}

// Tests points with a zero slope.
TEST(InterpolateTest, ZeroSlope) {
  PointCartesian2D p0 = {-4, 9};
  PointCartesian2D p1 = {7, 9};
  EXPECT_EQ(9, Interpolate(p0, p1, 0).y);       // interpolate
  EXPECT_EQ(9, Interpolate(p1, p0, 0).y);       // interpolate w/switched input order
  EXPECT_EQ(9, Interpolate(p0, p1, 8.5).y);     // extrapolate high
  EXPECT_EQ(9, Interpolate(p0, p1, -20).y);     // extrapolate low
  EXPECT_EQ(p0.y, Interpolate(p0, p1, p0.x).y); // x = x0
  EXPECT_EQ(p1.y, Interpolate(p0, p1, p1.x).y); // x = x1
}

// Tests point with an undefined slope.
TEST(InterpolateTest, UndefinedSlope) {
  PointCartesian2D p0 = {3, -1};
  PointCartesian2D p1 = {3, 12};
  EXPECT_THROW({
    try
    {
      Interpolate(p0, p1, 0);
    }
    catch(const std::runtime_error& e)
    {
      EXPECT_STREQ("Slope is undefined between (3, -1) and (3, 12). A y-value cannot be calculated between these points.", e.what());
      throw;
    }
  }, std::runtime_error);
}

}  // namespace
