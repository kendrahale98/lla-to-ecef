/**
 * @file lla_to_ecef_unittest.cpp
 *
 * @brief Unit tests for lla_to_ecef functions, using Google Test.
 *
 * @author Kendra Hale, <kendrahale98@gmail.com>
 *
 * @date 10/25/24
 */
#include <cmath>
#include <stdexcept>

#include "./lla_to_ecef.h"

#include "gtest/gtest.h"

#define NEAR_TOL 1e-10

namespace {

// ============================================================================
// Tests interpolate_2d().
// ============================================================================

// Tests points with a positive slope.
TEST(interpolate_2dTest, PositiveSlope) {
  PointCartesian2D p0 = {1, 1};
  PointCartesian2D p1 = {7, 4};
  EXPECT_EQ(3, interpolate_2d(p0, p1, 5).y);        // interpolate
  EXPECT_EQ(3, interpolate_2d(p1, p0, 5).y);        // interp. w/switched order
  EXPECT_EQ(5, interpolate_2d(p0, p1, 9).y);        // extrapolate high
  EXPECT_EQ(-9.5, interpolate_2d(p0, p1, -20).y);   // extrapolate low
  EXPECT_EQ(p0.y, interpolate_2d(p0, p1, p0.x).y);  // x = x0
  EXPECT_EQ(p1.y, interpolate_2d(p0, p1, p1.x).y);  // x = x1
}

// Tests points with a negative slope.
TEST(interpolate_2dTest, NegativeSlope) {
  PointCartesian2D p0 = {-1, 20};
  PointCartesian2D p1 = {6, -15};
  EXPECT_EQ(15, interpolate_2d(p0, p1, 0).y);       // interpolate
  EXPECT_EQ(15, interpolate_2d(p1, p0, 0).y);       // interp. w/switched order
  EXPECT_EQ(-27.5, interpolate_2d(p0, p1, 8.5).y);  // extrapolate high
  EXPECT_EQ(115, interpolate_2d(p0, p1, -20).y);    // extrapolate low
  EXPECT_EQ(p0.y, interpolate_2d(p0, p1, p0.x).y);  // x = x0
  EXPECT_EQ(p1.y, interpolate_2d(p0, p1, p1.x).y);  // x = x1
}

// Tests points with a zero slope.
TEST(interpolate_2dTest, ZeroSlope) {
  PointCartesian2D p0 = {-4, 9};
  PointCartesian2D p1 = {7, 9};
  EXPECT_EQ(9, interpolate_2d(p0, p1, 0).y);        // interpolate
  EXPECT_EQ(9, interpolate_2d(p1, p0, 0).y);        // interp. w/switched order
  EXPECT_EQ(9, interpolate_2d(p0, p1, 8.5).y);      // extrapolate high
  EXPECT_EQ(9, interpolate_2d(p0, p1, -20).y);      // extrapolate low
  EXPECT_EQ(p0.y, interpolate_2d(p0, p1, p0.x).y);  // x = x0
  EXPECT_EQ(p1.y, interpolate_2d(p0, p1, p1.x).y);  // x = x1
}

// Tests point with an undefined slope.
TEST(interpolate_2dTest, UndefinedSlope) {
  PointCartesian2D p0 = {3, -1};
  PointCartesian2D p1 = {3, 12};
  EXPECT_THROW({
    try {
      interpolate_2d(p0, p1, 0);
    }
    catch(const std::runtime_error& e) {
      EXPECT_STREQ(
        "Slope is undefined between (3, -1) and (3, 12). A y-value cannot be calculated between these points.", e.what());
      throw;
    }
  }, std::runtime_error);
}

// ============================================================================
// Tests get_rad_of_curvature().
// ============================================================================

// Tests get_rad_of_curvature at multiple latitude values.
TEST(get_rad_of_curvatureTest, Test) {
  double a = std::sqrt(3) / 2;
  double e = 0.5;

  EXPECT_EQ(a, get_rad_of_curvature(a, e, 0));
  EXPECT_EQ(1, get_rad_of_curvature(a, e, 90));
  EXPECT_EQ(a, get_rad_of_curvature(a, e, 180));
  EXPECT_EQ(1, get_rad_of_curvature(a, e, 270));
  EXPECT_EQ(a, get_rad_of_curvature(a, e, 360));
}

// ============================================================================
// Tests lla_to_ecef_pos().
// ============================================================================

// Test that PositionVelocityECEF is initialized to zero.
TEST(lla_to_ecef_posTest, TestZeroInitialization) {
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

// Tests lla_to_ecef_pos at multiple lat/long value pairs.
TEST(lla_to_ecef_posTest, Test) {
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
      PositionVelocityECEF result = lla_to_ecef_pos(pos_lla, a, b, e);
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
// Tests get_vel_ecef().
// ============================================================================

TEST(get_vel_ecef, Test) {
  const int len = 10;

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
    get_vel_ecef(inputs[i-1], &inputs[i]);
    EXPECT_DOUBLE_EQ(expected_v_x, inputs[i].v_x);
    EXPECT_DOUBLE_EQ(expected_v_y, inputs[i].v_y);
    EXPECT_DOUBLE_EQ(expected_v_z[i], inputs[i].v_z);
  }
}

}  // namespace