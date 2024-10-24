// A simple program to sum numbers and report it to the console.

#include <time.h>

#include <string>
#include <vector>

#include "./time_utils.h"

#define INDEX_ERR -1
#define KM_TO_M 1000

#ifndef LLA_TO_ECEF_H_
#define LLA_TO_ECEF_H_

// Struct containing 2D Cartesian point. Assumes y(x).
struct PointCartesian2D {
    double x;
    double y;
};

// Latitude-longitude-altitude point with timespec.
struct PositionLLA {
    struct timespec t {0, 0};   // time since Unix epoch
    double latitude = 0;        // degrees
    double longitude = 0;       // degrees
    double altitude = 0;        // meters
};

// ECEF position and velocity point with timespec.
struct PositionVelocityECEF {
    struct timespec t {0, 0};   // time since Unix epoch
    double x = 0;               // meters
    double y = 0;               // meters
    double z = 0;               // meters
    double v_x = 0;             // meters/second
    double v_y = 0;             // meters/second
    double v_z = 0;             // meters/second
};

// Reads in timestamped LLA data from a CSV file.
std::vector<PositionLLA> read_csv_lla(const std::string& filename);

// Calculates ECEF velocity at a point of interest.
std::vector<double> get_ecef_vel_at_poi(
    const std::vector<PositionLLA>& lla_data,
    const timespec& point_of_interest,
    const bool& print_output
);

// Finds indices of exact match or nearest points for a point of interest.
std::vector<int> find_match_or_nearest(
    const std::vector<PositionLLA>& data,
    timespec point_of_interest
);

// Converts a position in LLA coordinates to ECEF coordinates.
PositionVelocityECEF lla_to_ecef_pos(
    PositionLLA pos_lla, double a, double b, double e
);

// Calculates radius of curvature for conversion between LLA and ECEF position.
double get_rad_of_curvature(double a, double e, double latitude);

// Calculates velocity at current point given previous point.
void get_vel_ecef(PositionVelocityECEF previous, PositionVelocityECEF* current);

// Interpolate ECEF velocity at a pt. of interest between given points.
std::vector<double> interpolate_ecef_vel(
    const PositionVelocityECEF& before,
    const PositionVelocityECEF& after,
    const timespec& point_of_interest
);

// Performs linear interpolation for y, given two known points,
PointCartesian2D interpolate_2d(
    PointCartesian2D p0, PointCartesian2D p1, double x
);

#endif  // LLA_TO_ECEF_H_
