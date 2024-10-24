// A simple unit test file to build upon later.

#include <cmath>
#include <stdexcept>

#include "simple.h"

#include "gtest/gtest.h"

#define NEAR_TOL 1e-10

namespace {

// ============================================================================
// Tests Sum().
// ============================================================================

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

// ============================================================================
// Tests Interpolate().
// ============================================================================

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

// ============================================================================
// Tests RadiusOfCurvature().
// ============================================================================

// Tests RadiusOfCurvature at multiple latitude values.
TEST(RadiusOfCurvatureTest, Test) {
  double a = std::sqrt(3) / 2;
  double e = 0.5;

  EXPECT_EQ(a, RadiusOfCurvature(a, e, 0));
  EXPECT_EQ(1, RadiusOfCurvature(a, e, 90));
  EXPECT_EQ(a, RadiusOfCurvature(a, e, 180));
  EXPECT_EQ(1, RadiusOfCurvature(a, e, 270));
  EXPECT_EQ(a, RadiusOfCurvature(a, e, 360));
}

// ============================================================================
// Tests ConvertLLAtoECEF().
// ============================================================================

// Test that PositionVelocityECEF is initialized to zero.
TEST(ConvertLLAtoECEFTest, TestZeroInitialization) {
  PositionVelocityECEF result;
  EXPECT_EQ(0, result.t.tv_sec);
  EXPECT_EQ(0, result.t.tv_nsec);
  EXPECT_DOUBLE_EQ(0, result.x);
  EXPECT_DOUBLE_EQ(0, result.y);
  EXPECT_DOUBLE_EQ(0, result.z);
  EXPECT_DOUBLE_EQ(0, result.v_x);
  EXPECT_DOUBLE_EQ(0, result.v_y);
  EXPECT_DOUBLE_EQ(0, result.v_z);
}

// Tests ConvertLLAtoECEF at multiple lat/long value pairs.
TEST(ConvertLLAtoECEFTest, Test) {
  // Inputs
  int sec = 1729728453;
  int nsec = 9000;
  timespec t_test {sec, nsec};
  
  double a = std::sqrt(3) / 2;
  double b = 3;
  double e = 0.5;
  double h = 1;

  double angles[] = {0, 90, 180, 270, 360};
  int len = sizeof(angles) / sizeof(angles[0]);

  // Expected values
  double a_plus_h = a + h;
  double expected_x[len][len] = {
    {a_plus_h, 0, -1 * a_plus_h, 0, a_plus_h},
    {0, 0, 0, 0, 0},
    {-1 * a_plus_h, 0, a_plus_h, 0, -1 * a_plus_h},
    { 0, 0, 0, 0, 0},
    {a_plus_h, 0, -1 * a_plus_h, 0, a_plus_h}
  };
  double expected_y[len][len] = {
    {0, a_plus_h, 0, -1 * a_plus_h, 0},
    {0, 0, 0, 0, 0},
    {0, -1 * a_plus_h, 0, a_plus_h, 0},
    {0, 0, 0, 0, 0},
    {0, a_plus_h, 0, -1 * a_plus_h, 0}
  };
  double b_sq_ovr_a_sq_plus_h = (std::pow(b, 2) / std::pow(a, 2)) + h;
  double expected_z[len] = {
    0, b_sq_ovr_a_sq_plus_h, 0, -1 * b_sq_ovr_a_sq_plus_h, 0
  };

  // Test
  for (int i = 0; i < len; i++) {
    for (int j = 0; j < len; j++) {
      PositionLLA pos_lla {t_test, angles[i], angles[j], h};
      PositionVelocityECEF result = ConvertLLAtoECEF(pos_lla, a, b, e);
      EXPECT_EQ(sec, result.t.tv_sec);
      EXPECT_EQ(nsec, result.t.tv_nsec);
      EXPECT_NEAR(expected_x[i][j], result.x, NEAR_TOL);
      EXPECT_NEAR(expected_y[i][j], result.y, NEAR_TOL);
      EXPECT_NEAR(expected_z[i], result.z, NEAR_TOL);
      EXPECT_DOUBLE_EQ(0, result.v_x);
      EXPECT_DOUBLE_EQ(0, result.v_y);
      EXPECT_DOUBLE_EQ(0, result.v_z);
    }
  }
}

}  // namespace
