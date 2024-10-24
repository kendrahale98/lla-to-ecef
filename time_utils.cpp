#include "time_utils.h"

bool ts_is_before(const timespec& ts1, const timespec &ts2) {
    if (ts1.tv_sec < ts2.tv_sec) {
        return true;
    } else if (ts1.tv_sec == ts2.tv_sec) {
        return (ts1.tv_nsec < ts2.tv_nsec);
    } else {
        return false;
    }
}

bool ts_is_equal(const timespec& ts1, const timespec& ts2) {
    return ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec == ts2.tv_nsec;
}

double ts_to_double(const timespec& ts) {
  return ts.tv_sec + (double)ts.tv_nsec / 1e9;
}