/**
 * @file time_utils.h
 *
 * @brief Header for time utilities functions.
 *
 * @author Kendra Hale, <kendrahale98@gmail.com>
 *
 * @date 10/25/24
 */
#include <time.h>

#ifndef TIME_UTILS_H_
#define TIME_UTILS_H_

// Checks if a given timespec precedes another.
bool ts_is_before(const timespec& ts1, const timespec &ts2);

// Checks if given timespecs are equal.
bool ts_is_equal(const timespec& ts1, const timespec& ts2);

// Converts a timespec to a double.
double ts_to_double(const timespec& ts);

#endif  // TIME_UTILS_H_
