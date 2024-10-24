/**
 * @file time_utils.cpp
 *
 * @brief Definitions for time utilities functions.
 *
 * @author Kendra Hale, <kendrahale98@gmail.com>
 *
 * @date 10/25/24
 */
#include "./time_utils.h"

/**
 * ts_is_before
 * @brief Checks if a given timespec precedes another.
 *
 * @param ts1   first timespec to consider
 * @param ts2   second timespec to consider
 *
 * @return true if ts1 is before ts2, false otherwise
 */
bool ts_is_before(const timespec& ts1, const timespec &ts2) {
    if (ts1.tv_sec < ts2.tv_sec) {
        return true;
    } else if (ts1.tv_sec == ts2.tv_sec) {
        return (ts1.tv_nsec < ts2.tv_nsec);
    } else {
        return false;
    }
}

/**
 * ts_is_equal
 * @brief Checks if given timespecs are equal.
 *
 * @param ts1   first timespec to consider
 * @param ts2   second timespec to consider
 *
 * @return true if ts1 and ts2 are equal, false otherwise
 */
bool ts_is_equal(const timespec& ts1, const timespec& ts2) {
    return ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec == ts2.tv_nsec;
}

/**
 * ts_to_double
 * @brief Converts a timespec to a double.
 *
 * @param ts    timespec to convert
 *
 * @return the timespec as a double
 */
double ts_to_double(const timespec& ts) {
  return ts.tv_sec + static_cast<double>(ts.tv_nsec / 1e9);
}
