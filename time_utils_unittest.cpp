/**
 * @file time_utils_unittest.cpp
 *
 * @brief Unit tests for time_utils functions, using Google Test.
 *
 * @author Kendra Hale, <kendrahale98@gmail.com>
 *
 * @date 10/25/24
 */
#include "./time_utils.h"

#include "gtest/gtest.h"

namespace {

// ============================================================================
// Tests ts_is_before().
// ============================================================================

TEST(ts_is_before, Test) {
  timespec early {1000, 1000};
  timespec late {10000, 1000};

  EXPECT_TRUE(ts_is_before(early, late));
  EXPECT_FALSE(ts_is_before(late, early));
  EXPECT_FALSE(ts_is_before(late, late));
  EXPECT_FALSE(ts_is_before(early, early));
}

// ============================================================================
// Tests ts_is_equal().
// ============================================================================

TEST(ts_is_equal, Test) {
  timespec ts1 {1000, 1000};
  timespec ts2 {10000, 1000};

  EXPECT_TRUE(ts_is_equal(ts1, ts1));
  EXPECT_TRUE(ts_is_equal(ts2, ts2));
  EXPECT_FALSE(ts_is_equal(ts1, ts2));
}

// ============================================================================
// Tests ts_to_double().
// ============================================================================

TEST(ts_to_double, Test) {
  timespec ts1 {1000, 10000000};
  timespec ts2 {10000, 0};

  EXPECT_DOUBLE_EQ(1000.01, ts_to_double(ts1));
  EXPECT_DOUBLE_EQ(10000.0, ts_to_double(ts2));
}

}  // namespace
