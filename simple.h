// A simple program to sum numbers and report it to the console.

#include <time.h>

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

bool IsBefore(const timespec& t1, const timespec &t2);

#endif  // SIMPLE_H_