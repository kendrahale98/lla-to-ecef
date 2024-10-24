#include <time.h>

#ifndef TIME_UTILS_H_
#define TIME_UTILS_H_

bool ts_is_before(const timespec& ts1, const timespec &ts2);

bool ts_is_equal(const timespec& ts1, const timespec& ts2);

double ts_to_double(const timespec& ts);

#endif  // TIME_UTILS_H_