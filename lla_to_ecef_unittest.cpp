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
#include "./wgs84.h"

#include "gtest/gtest.h"

#define NEAR_TOL 1e-10

namespace {

// ============================================================================
// Tests read_csv_lla().
// ============================================================================

// Test successful read.
TEST(read_csv_lla_test, TestGoodRead) {
  std::vector<PositionLLA> data = read_csv_lla("./test_data.csv");

  // Check length
  EXPECT_EQ(27, data.size());

  // Check data at a few points
  timespec ts {1532332919, 40000000};
  double lat = 53.379023557853734871514462;
  double lon = -6.709789746339143157172202;
  double alt = 50.037492251246234786776768 * KM_TO_M;
  EXPECT_TRUE(ts_is_equal(ts, data[0].t));
  EXPECT_DOUBLE_EQ(lat, data[0].latitude);
  EXPECT_DOUBLE_EQ(lon, data[0].longitude);
  EXPECT_DOUBLE_EQ(alt, data[0].altitude);

  ts.tv_sec = 1532333049;
  lat = 53.579132406588414028192346;
  lon = -11.274580378310924189122488;
  alt = 537.048475586836275397217833 * KM_TO_M;
  EXPECT_TRUE(ts_is_equal(ts, data[26].t));
  EXPECT_DOUBLE_EQ(lat, data[26].latitude);
  EXPECT_DOUBLE_EQ(lon, data[26].longitude);
  EXPECT_DOUBLE_EQ(alt, data[26].altitude);
}

// Test unsuccessful read.
TEST(read_csv_lla_test, TestBadRead) {
  EXPECT_THROW({
    try {
      std::vector<PositionLLA> data = read_csv_lla("./bad_file.csv");
    }
    catch(const std::runtime_error& e) {
      EXPECT_STREQ(
        "Could not open file ./bad_file.csv. Exiting", e.what());
      throw;
    }
  }, std::runtime_error);
}

// ============================================================================
// Tests get_ecef_vel_at_pt().
// ============================================================================

// Tests for a point out-of-range.
TEST(get_ecef_vel_at_pt_test, TestPointOutOfRange) {
  std::vector<PositionLLA> data;
  data.push_back(PositionLLA {timespec {1000, 0}, 55, 75, 1500});
  data.push_back(PositionLLA {timespec {1300, 0}, 55.4, 74, 1450});

  timespec pt {500, 0};
  std::vector<double> result;

  EXPECT_THROW({
    try {
      int result_code = get_ecef_vel_at_pt(data, pt, result, false);
    }
    catch(const std::runtime_error& e) {
      EXPECT_STREQ(
        "Point of interest is not within range of data set.", e.what());
      throw;
    }
  }, std::runtime_error);
}

// Tests for a point matching one in the set.
TEST(get_ecef_vel_at_pt_test, TestExactMatch) {
  double test_tol = 1e-6;
  
  std::vector<PositionLLA> data;
  data.push_back(PositionLLA {timespec {1000, 0}, 55, 75, 1500});
  data.push_back(PositionLLA {timespec {1300, 0}, 55.4, 74, 1450});
  data.push_back(PositionLLA {timespec {1500, 0}, 55.8, 73, 1450});
  data.push_back(PositionLLA {timespec {1600, 0}, 56, 74, 1450});
  data.push_back(PositionLLA {timespec {2000, 0}, 56.1, 75, 1450});

  // Test exact match that is not first point
  timespec pt {data[2].t.tv_sec, data[2].t.tv_nsec};
  std::vector<double> result;
  int result_code = get_ecef_vel_at_pt(data, pt, result, false);
  EXPECT_EQ(SUCCESS, result_code);
  EXPECT_NEAR(250.070838500001, result[0], test_tol);
  EXPECT_NEAR(-265.728374999999, result[1], test_tol);
  EXPECT_NEAR(125.829012000002, result[2], test_tol);

  // Test exact match that is first point
  timespec first_pt {data[0].t.tv_sec, data[0].t.tv_nsec};
  std::vector<double> result2;
  int result_code2 = get_ecef_vel_at_pt(data, first_pt, result2, false);
  EXPECT_EQ(SUCCESS, result_code2);
  EXPECT_DOUBLE_EQ(0, result2[0]);
  EXPECT_DOUBLE_EQ(0, result2[1]);
  EXPECT_DOUBLE_EQ(0, result2[2]);
}

// Tests for a point requiring interpolation.
TEST(get_ecef_vel_at_pt_test, TestNeedingInterpolation) {
  double test_tol = 1e-6;
  
  std::vector<PositionLLA> data;
  data.push_back(PositionLLA {timespec {1000, 0}, 55, 75, 1500});
  data.push_back(PositionLLA {timespec {1300, 0}, 55.4, 74, 1450});
  data.push_back(PositionLLA {timespec {1500, 0}, 55.8, 73, 1450});
  data.push_back(PositionLLA {timespec {1600, 0}, 56, 74, 1450});
  data.push_back(PositionLLA {timespec {2000, 0}, 56.1, 75, 1450});

  // Test point between first and second index
  timespec pt {1100, 0};
  std::vector<double> result;
  int result_code = get_ecef_vel_at_pt(data, pt, result, false);
  EXPECT_EQ(SUCCESS, result_code);
  EXPECT_NEAR(57.3234554444443, result[0], test_tol);
  EXPECT_NEAR(-58.1010510000003, result[1], test_tol);
  EXPECT_NEAR(28.199050777778, result[2], test_tol);

  // Test point not between first and second index
  timespec pt2 {1502, 0};
  std::vector<double> result2;
  int result_code2 = get_ecef_vel_at_pt(data, pt2, result2, false);
  EXPECT_EQ(SUCCESS, result_code2);
  EXPECT_NEAR(232.023708250001, result2[0], test_tol);
  EXPECT_NEAR(-260.396455899999, result2[1], test_tol);
  EXPECT_NEAR(125.809860020002, result2[2], test_tol);
}

// ============================================================================
// Tests find_match_or_nearest().
// ============================================================================

TEST(find_match_or_nearest_test, Test) {
  std::vector<PositionLLA> data;
  data.push_back(PositionLLA {timespec {1000, 0}, 55, 75, 1500});
  data.push_back(PositionLLA {timespec {1300, 0}, 55.4, 74, 1450});
  data.push_back(PositionLLA {timespec {1500, 0}, 55.8, 73, 1450});
  data.push_back(PositionLLA {timespec {1600, 0}, 56, 74, 1450});
  data.push_back(PositionLLA {timespec {2000, 0}, 56.1, 75, 1450});

  // Test a point with an exact match
  bool match1;
  timespec pt {data[2].t.tv_sec, data[2].t.tv_nsec};
  int result = find_match_or_nearest(data, pt, match1);
  EXPECT_TRUE(match1);
  EXPECT_EQ(2, result);

  // Test a point without an exact match
  bool match2;
  pt.tv_nsec = 9000;
  result = find_match_or_nearest(data, pt, match2);
  EXPECT_FALSE(match2);
  EXPECT_EQ(2, result);

  // Test points out of range
  bool match3;
  pt.tv_sec = 500;
  result = find_match_or_nearest(data, pt, match3);
  EXPECT_FALSE(match3);
  EXPECT_EQ(INDEX_ERR, result);

  bool match4;
  pt.tv_sec = 2500;
  result = find_match_or_nearest(data, pt, match4);
  EXPECT_FALSE(match4);
  EXPECT_EQ(INDEX_ERR, result);
}

// ============================================================================
// Tests lla_to_ecef_pos().
// ============================================================================

// Test that PositionVelocityECEF is initialized to zero.
TEST(lla_to_ecef_pos_test, TestZeroInitialization) {
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

// Tests at a point with realistic values.
TEST(lla_to_ecef_pos_test, TestRealisticInput) {
  double test_tol = 1e-4;

  PositionLLA in {
    timespec {1532334879, 40000000},
    46.2159884893476,
    -62.4089793222041,
    2089319.48641661
  };

  PositionVelocityECEF result = lla_to_ecef_pos(
    in, WGS84Params::A, WGS84Params::B, WGS84Params::E);
  EXPECT_TRUE(ts_is_equal(in.t, result.t));
  EXPECT_NEAR(2717206.9203, result.x, test_tol);
  EXPECT_NEAR(-5199521.9869, result.y, test_tol);
  EXPECT_NEAR(6090283.4011, result.z, test_tol);
  EXPECT_DOUBLE_EQ(0, result.v_x);
  EXPECT_DOUBLE_EQ(0, result.v_y);
  EXPECT_DOUBLE_EQ(0, result.v_z);
}

// Tests lla_to_ecef_pos at multiple lat/long value pairs.
TEST(lla_to_ecef_pos_test, Test) {
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
// Tests get_rad_of_curvature().
// ============================================================================

TEST(get_rad_of_curvature_test, Test) {
  double a = std::sqrt(3) / 2;
  double e = 0.5;

  EXPECT_EQ(a, get_rad_of_curvature(a, e, 0));
  EXPECT_EQ(1, get_rad_of_curvature(a, e, 90));
  EXPECT_EQ(a, get_rad_of_curvature(a, e, 180));
  EXPECT_EQ(1, get_rad_of_curvature(a, e, 270));
  EXPECT_EQ(a, get_rad_of_curvature(a, e, 360));
}

// ============================================================================
// Tests get_vel_ecef().
// ============================================================================

TEST(get_vel_ecef_test, Test) {
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

// ============================================================================
// Tests interpolate_ecef_vel().
// ============================================================================

TEST(interpolate_ecef_vel_test, Test) {
  PositionVelocityECEF before {timespec {100, 500000000}, 0, 0, 0, 1, 2, 3};
  PositionVelocityECEF after {timespec {102, 700000000}, 0, 0, 0, 4, -5, 0.006};
  timespec pt {101, 30000000};

  std::vector<double> result = interpolate_ecef_vel(before, after, pt);
  EXPECT_DOUBLE_EQ(1.7227272727272733, result[0]);
  EXPECT_DOUBLE_EQ(0.3136363636363622, result[1]);
  EXPECT_DOUBLE_EQ(2.2787181818181814, result[2]);
}

// ============================================================================
// Tests interpolate_2d().
// ============================================================================

// Tests points with a positive slope.
TEST(interpolate_2d_test, PositiveSlope) {
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
TEST(interpolate_2d_test, NegativeSlope) {
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
TEST(interpolate_2d_test, ZeroSlope) {
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
TEST(interpolate_2d_test, UndefinedSlope) {
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

}  // namespace
