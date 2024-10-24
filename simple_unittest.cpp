// A simple unit test file to build upon later.

#include <cmath>
#include <stdexcept>

#include "simple.h"

#include "gtest/gtest.h"

#define NEAR_TOL 1e-10

namespace {

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
// Tests CalculateRadiusOfCurvature().
// ============================================================================

// Tests CalculateRadiusOfCurvature at multiple latitude values.
TEST(CalculateRadiusOfCurvatureTest, Test) {
  double a = std::sqrt(3) / 2;
  double e = 0.5;

  EXPECT_EQ(a, CalculateRadiusOfCurvature(a, e, 0));
  EXPECT_EQ(1, CalculateRadiusOfCurvature(a, e, 90));
  EXPECT_EQ(a, CalculateRadiusOfCurvature(a, e, 180));
  EXPECT_EQ(1, CalculateRadiusOfCurvature(a, e, 270));
  EXPECT_EQ(a, CalculateRadiusOfCurvature(a, e, 360));
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

// ============================================================================
// Tests CalculateVelocityECEF().
// ============================================================================

TEST(CalculateVelocityECEF, Test) {
  int len = 10;
  
  // Expected values
  double expected_v_x = 0;
  double expected_v_y = 1;
  double expected_v_z[] = {
    0, -500, -250, -125, -62.5, -31.25, -15.625, -7.8125, -3.90625, -1.953125
  };

  // Inputs
  PositionVelocityECEF inputs[len];

  timespec t0 = {1000, 1000};
  inputs[0] = {t0, 5, -10, 1000};

  for (int i = 1; i < len; i++) {
    PositionVelocityECEF previous = inputs[i-1];
    timespec t_i = {previous.t.tv_sec + 1, previous.t.tv_nsec};
    inputs[i] = {t_i, previous.x, previous.y + 1, previous.z / 2};
  }

  // Get results and check
  for (int i = 1; i < len; i++) {
    EXPECT_DOUBLE_EQ(0, inputs[i].v_x);
    EXPECT_DOUBLE_EQ(0, inputs[i].v_y);
    EXPECT_DOUBLE_EQ(0, inputs[i].v_z);
    CalculateVelocityECEF(inputs[i-1], &inputs[i]);
    EXPECT_DOUBLE_EQ(expected_v_x, inputs[i].v_x);
    EXPECT_DOUBLE_EQ(expected_v_y, inputs[i].v_y);
    EXPECT_DOUBLE_EQ(expected_v_z[i], inputs[i].v_z);
  }
}

}  // namespace
