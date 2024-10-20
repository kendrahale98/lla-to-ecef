// A simple unit test file to build upon later.

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
}  // namespace
