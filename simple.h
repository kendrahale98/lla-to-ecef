// A simple program to sum numbers and report it to the console.

#include <string>
#include <vector>

#include <time.h>

#include "time_utils.h"

#define INDEX_ERR -1

#ifndef SIMPLE_H_
#define SIMPLE_H_

struct PointCartesian2D {
    double x;
    double y;
};

struct PositionLLA {
    struct timespec t {0, 0};   // time since Unix epoch
    double latitude = 0;        // degrees
    double longitude = 0;       // degrees
    double altitude = 0;        // kilometers
};

struct PositionVelocityECEF {
    struct timespec t {0, 0};   // time since Unix epoch
    double x = 0;               // kilometers
    double y = 0;               // kilometers
    double z = 0;               // kilometers
    double v_x = 0;             // kilometers/second
    double v_y = 0;             // kilometers/second
    double v_z = 0;             // kilometers/second
};

// Returns the sum of the given integers.
int Sum(int a, int b);

PointCartesian2D Interpolate(PointCartesian2D p0, PointCartesian2D p1, double x);

PositionVelocityECEF ConvertLLAtoECEF(PositionLLA pos_lla, double a, double b, double e);

void CalculateVelocityECEF(PositionVelocityECEF previous, PositionVelocityECEF* current);

double CalculateRadiusOfCurvature(double a, double e, double latitude);

// true if t1 is before t2
// bool ts_is_before(const timespec& ts1, const timespec &ts2);

// bool ts_is_equal(const timespec& ts1, const timespec& ts2);

// double ts_to_double(const timespec& ts);

std::vector<PositionLLA> ReadCSVFileLLA(const std::string& filename);

std::vector<int> SearchPoints(const std::vector<PositionLLA>& data, timespec point_of_interest);

std::vector<double> InterpolateECEFVelocities(const PositionVelocityECEF& before, const PositionVelocityECEF& after, const timespec& point_of_interest);

std::vector<double> GetVelocityAtTime(const std::vector<PositionLLA>& lla_data, const timespec& point_of_interest, const bool& print_output);

#endif  // SIMPLE_H_